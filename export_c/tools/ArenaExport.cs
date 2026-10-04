// ArenaExport.cs - inspect / export SARPBC arena StaticMeshes (UModel .pskx) for the C game.
// Build: csc /O /out:ArenaExport.exe ArenaExport.cs
// Usage: ArenaExport dump   <StaticMesh3 dir>
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

public class Psk
{
    public string Name;
    public List<float[]> Points = new List<float[]>();
    public List<int> WPoint = new List<int>();
    public List<float[]> WUV = new List<float[]>();
    public List<int> WMat = new List<int>();
    public List<int[]> Faces = new List<int[]>();   // w0,w1,w2,mat
    public List<string> Mats = new List<string>();
    public float[] Min = { 1e30f, 1e30f, 1e30f }, Max = { -1e30f, -1e30f, -1e30f };

    public static Psk Load(string path)
    {
        var p = new Psk { Name = Path.GetFileNameWithoutExtension(path) };
        var br = new BinaryReader(File.OpenRead(path));
        long len = br.BaseStream.Length;
        while (br.BaseStream.Position + 32 <= len)
        {
            string id = Encoding.ASCII.GetString(br.ReadBytes(20)).TrimEnd('\0');
            br.ReadInt32();
            int size = br.ReadInt32(), count = br.ReadInt32();
            long start = br.BaseStream.Position;
            if (id == "PNTS0000")
                for (int i = 0; i < count; i++) p.Points.Add(new[] { br.ReadSingle(), br.ReadSingle(), br.ReadSingle() });
            else if (id == "VTXW0000")
                for (int i = 0; i < count; i++)
                {
                    int pi = size == 16 ? br.ReadUInt16() | 0 : br.ReadInt32();
                    if (size == 16) br.ReadUInt16();
                    float u = br.ReadSingle(), v = br.ReadSingle();
                    int m = br.ReadByte(); br.ReadByte(); br.ReadUInt16();
                    if (size == 16 && count > 65536) { } // not expected
                    p.WPoint.Add(pi); p.WUV.Add(new[] { u, v }); p.WMat.Add(m);
                }
            else if (id == "FACE0000" || id == "FACE3200")
                for (int i = 0; i < count; i++)
                {
                    int a, b, c;
                    if (id == "FACE3200") { a = br.ReadInt32(); b = br.ReadInt32(); c = br.ReadInt32(); }
                    else { a = br.ReadUInt16(); b = br.ReadUInt16(); c = br.ReadUInt16(); }
                    int m = br.ReadByte(); br.ReadByte(); br.ReadInt32();
                    p.Faces.Add(new[] { a, b, c, m });
                }
            else if (id == "MATT0000")
                for (int i = 0; i < count; i++)
                {
                    p.Mats.Add(Encoding.ASCII.GetString(br.ReadBytes(64)).TrimEnd('\0'));
                    br.ReadBytes(24);
                }
            br.BaseStream.Position = start + (long)size * count;
        }
        br.Close();
        foreach (var v in p.Points)
            for (int k = 0; k < 3; k++) { p.Min[k] = Math.Min(p.Min[k], v[k]); p.Max[k] = Math.Max(p.Max[k], v[k]); }
        return p;
    }
}

public static class Program
{
    // UE (x, y, z) cm, Z-up, left-handed  ->  game (x, z, y) m, Y-up. The axis swap
    // mirrors handedness, so triangle winding is flipped on output.
    static float[] ToGame(float[] v, float sx, float sy)
    {
        return new[] { v[0] * sx / 100f, v[2] / 100f, v[1] * sy / 100f };
    }

    static float[] Sub(float[] a, float[] b) { return new[] { a[0]-b[0], a[1]-b[1], a[2]-b[2] }; }
    static float[] Cross(float[] a, float[] b) { return new[] { a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0] }; }
    static float[] Norm(float[] a) { float l = (float)Math.Sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]); return l < 1e-12f ? new float[] {0,1,0} : new[] { a[0]/l, a[1]/l, a[2]/l }; }

    class Mat { public string Name, Tex; public int R, G, B, A; }

    // pskx material name -> render material
    static readonly Mat[] MatTable = {
        new Mat { Name = "Pavement01_Mat",    Tex = "Pavement01.tga", R=255, G=255, B=255, A=255 },
        new Mat { Name = "StreetLine01_Mat",  Tex = null,             R=235, G=235, B=230, A=255 },
        // SideWalk01 is PF_G8 and UModel's TGA comes out scrambled on PS3 -> flat concrete for now
        new Mat { Name = "SideWalk01_Mat",    Tex = null,             R=150, G=148, B=142, A=255 },
        new Mat { Name = "GrayTiles01_Mat",   Tex = null,             R=150, G=152, B=158, A=255 },
        new Mat { Name = "Brick01_Mat",       Tex = "Brick01.tga",    R=255, G=255, B=255, A=255 },
        new Mat { Name = "MetalGarage01_Mat", Tex = null,             R=105, G=110, B=118, A=255 },
        new Mat { Name = "material_0",        Tex = null,             R=150, G=200, B=255, A=50  },  // glass
    };

    // name, mirror mode: 0 = as-is, 1 = four quadrants (mirror X and Y), 2 = mirror Y (both ends)
    static readonly object[][] Pieces = {
        new object[] { "Pavement01", 0 }, new object[] { "Lines01", 0 }, new object[] { "Lines_PlayerStart01", 0 },
        new object[] { "SkyTrim01", 0 },  new object[] { "FactoryWalls01", 0 },
        new object[] { "SideWalk01", 1 }, new object[] { "WallBase01", 1 }, new object[] { "Wall01", 1 },
        new object[] { "WallTrim01", 1 }, new object[] { "GoalTrim01", 1 }, new object[] { "Garage01", 2 },
        new object[] { "Glass01", 1 },
    };

    // Uncompressed 32-bit BGRA TGA, top-left origin.
    static void WriteTga(string path, byte[] bgra, int w, int h)
    {
        var hdr = new byte[18];
        hdr[2] = 2; hdr[12] = (byte)w; hdr[13] = (byte)(w >> 8); hdr[14] = (byte)h; hdr[15] = (byte)(h >> 8); hdr[16] = 32; hdr[17] = 0x28;
        using (var fs = File.Create(path)) { fs.Write(hdr, 0, 18); fs.Write(bgra, 0, bgra.Length); }
    }

    static int Export(string meshDir, string texDir, string outDir, string ddsDir = null)
    {
        Directory.CreateDirectory(outDir);
        // per material: vertex list (pos, normal, uv) + indices
        var verts = new List<float>[MatTable.Length];
        var inds = new List<uint>[MatTable.Length];
        for (int i = 0; i < MatTable.Length; i++) { verts[i] = new List<float>(); inds[i] = new List<uint>(); }

        foreach (var pc in Pieces)
        {
            var p = Psk.Load(Path.Combine(meshDir, (string)pc[0] + ".pskx"));
            int mode = (int)pc[1];
            var copies = new List<float[]> { new float[] { 1, 1 } };
            if (mode == 1) { copies.Add(new float[] { -1, 1 }); copies.Add(new float[] { 1, -1 }); copies.Add(new float[] { -1, -1 }); }
            if (mode == 2) copies.Add(new float[] { 1, -1 });
            foreach (var cp in copies)
            {
                // Measured on CM_Ground01/CM_Glass01: after the axis swap, UE's original winding
                // already gives (b-a)x(c-a) normals facing the play space. Each mirror flips it.
                bool flip = (cp[0] * cp[1] < 0);
                // smooth normals per point
                var pn = new float[p.Points.Count][];
                for (int i = 0; i < pn.Length; i++) pn[i] = new float[3];
                foreach (var f in p.Faces)
                {
                    var A = ToGame(p.Points[p.WPoint[f[0]]], cp[0], cp[1]);
                    var B = ToGame(p.Points[p.WPoint[f[1]]], cp[0], cp[1]);
                    var C = ToGame(p.Points[p.WPoint[f[2]]], cp[0], cp[1]);
                    var n = flip ? Cross(Sub(C, A), Sub(B, A)) : Cross(Sub(B, A), Sub(C, A));
                    for (int k = 0; k < 3; k++) for (int j = 0; j < 3; j++) pn[p.WPoint[f[k]]][j] += n[j];
                }
                foreach (var f in p.Faces)
                {
                    int mi = Array.FindIndex(MatTable, m => m.Name == p.Mats[f[3]]);
                    if (mi < 0) { Console.WriteLine("  unmapped material {0} in {1}", p.Mats[f[3]], p.Name); continue; }
                    var order = flip ? new[] { 0, 2, 1 } : new[] { 0, 1, 2 };
                    foreach (int k in order)
                    {
                        int w = f[k];
                        var pos = ToGame(p.Points[p.WPoint[w]], cp[0], cp[1]);
                        // FactoryWalls01 has a level-wide floor at z=0, coplanar with Pavement01 (z-fighting).
                        // Sink its ground-level vertices below the pavement/sidewalk (sidewalk bottoms at -4 cm).
                        if (p.Name == "FactoryWalls01" && p.Points[p.WPoint[w]][2] <= 0.5f) pos[1] -= 0.06f;
                        // SkyTrim01 is the beam ring under the glass ceiling edge, authored around z=0 (+-80 UU).
                        // Its actor placement isn't exported; the glass underside at its x (83-85 m) is ~60.2 m, so top it there.
                        if (p.Name == "SkyTrim01") pos[1] += 59.4f;
                        var nn = Norm(pn[p.WPoint[w]]);
                        inds[mi].Add((uint)(verts[mi].Count / 8));
                        verts[mi].AddRange(new[] { pos[0], pos[1], pos[2], nn[0], nn[1], nn[2], p.WUV[w][0], p.WUV[w][1] });
                    }
                }
            }
        }

        // ---- arena.sarm (same container as the cars; submesh.material = row in arena_materials.txt)
        uint vcount = 0, icount = 0, scount = 0;
        for (int i = 0; i < MatTable.Length; i++) { vcount += (uint)(verts[i].Count / 8); icount += (uint)inds[i].Count; if (inds[i].Count > 0) scount++; }
        float[] mn = { 1e30f, 1e30f, 1e30f }, mx = { -1e30f, -1e30f, -1e30f };
        for (int i = 0; i < MatTable.Length; i++)
            for (int v = 0; v < verts[i].Count; v += 8)
                for (int k = 0; k < 3; k++) { mn[k] = Math.Min(mn[k], verts[i][v+k]); mx[k] = Math.Max(mx[k], verts[i][v+k]); }
        using (var bw = new BinaryWriter(File.Create(Path.Combine(outDir, "arena.sarm"))))
        {
            bw.Write(Encoding.ASCII.GetBytes("SARM")); bw.Write(1u); bw.Write(vcount); bw.Write(icount); bw.Write(scount); bw.Write(0u);
            foreach (var f in mn) bw.Write(f); foreach (var f in mx) bw.Write(f);
            for (int i = 0; i < 12; i++) bw.Write(0f);
            uint first = 0;
            for (int i = 0; i < MatTable.Length; i++)
            {
                if (inds[i].Count == 0) continue;
                bw.Write(first); bw.Write((uint)inds[i].Count); bw.Write((uint)i); bw.Write(0u);
                first += (uint)inds[i].Count;
            }
            for (int i = 0; i < MatTable.Length; i++)
                for (int v = 0; v < verts[i].Count; v += 8)
                {
                    for (int k = 0; k < 8; k++) bw.Write(verts[i][v+k]);
                    bw.Write(0f); bw.Write(0f); bw.Write(0f); bw.Write(1f);   // tangent unused
                }
            uint baseV = 0;
            for (int i = 0; i < MatTable.Length; i++)
            {
                foreach (var ix in inds[i]) bw.Write(ix + baseV);
                baseV += (uint)(verts[i].Count / 8);
            }
        }
        using (var sw = new StreamWriter(Path.Combine(outDir, "arena_materials.txt")))
            foreach (var m in MatTable)
                sw.WriteLine("{0} {1} {2} {3} {4} {5}", m.Name, m.Tex ?? "-", m.R, m.G, m.B, m.A);
        foreach (var m in MatTable)
        {
            if (m.Tex == null) continue;
            // UModel's TGA for PS3 DXT1 is byte-swapped garbage; its raw -dds dump decodes correctly as little-endian.
            string dds = ddsDir == null ? null : Path.Combine(ddsDir, Path.GetFileNameWithoutExtension(m.Tex) + ".dds");
            if (dds != null && File.Exists(dds))
            {
                var d = File.ReadAllBytes(dds);
                string four = Encoding.ASCII.GetString(d, 84, 4);
                int h = BitConverter.ToInt32(d, 12), w = BitConverter.ToInt32(d, 16);
                if (four == "DXT1") { WriteTga(Path.Combine(outDir, m.Tex), DecodeDxt1(d, 128, w, h, false), w, h); Console.WriteLine("{0}: decoded DXT1 {1}x{2} from dds", m.Tex, w, h); continue; }
            }
            File.Copy(Path.Combine(texDir, m.Tex), Path.Combine(outDir, m.Tex), true);
        }
        Console.WriteLine("arena.sarm: {0} verts, {1} tris, {2} submeshes, bounds [{3:0.0} {4:0.0} {5:0.0}]..[{6:0.0} {7:0.0} {8:0.0}]",
            vcount, icount / 3, scount, mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]);

        // ---- arena_col.bin: "SARC", uint count, count * 9 floats. Winding: normal = (b-a)x(c-a) faces the play space.
        var tris = new List<float[]>();
        foreach (var name in new[] { "CM_Ground01", "CM_Glass01" })
        {
            var p = Psk.Load(Path.Combine(meshDir, name + ".pskx"));
            int up = 0, down = 0, inward = 0, outward = 0;
            foreach (var f in p.Faces)
            {
                var A = ToGame(p.Points[p.WPoint[f[0]]], 1, 1);
                var B = ToGame(p.Points[p.WPoint[f[1]]], 1, 1);
                var C = ToGame(p.Points[p.WPoint[f[2]]], 1, 1);
                var n = Norm(Cross(Sub(B, A), Sub(C, A)));
                var cen = new[] { (A[0]+B[0]+C[0])/3, (A[1]+B[1]+C[1])/3, (A[2]+B[2]+C[2])/3 };
                if (cen[1] < 0.5f && Math.Abs(cen[0]) < 80 && Math.Abs(cen[2]) < 130) { if (n[1] > 0) up++; else down++; }
                var toAxis = new[] { -cen[0], 20 - cen[1], Math.Max(-130f, Math.Min(130f, cen[2])) - cen[2] };
                if (n[0]*toAxis[0] + n[1]*toAxis[1] + n[2]*toAxis[2] > 0) inward++; else outward++;
                tris.Add(new[] { A[0], A[1], A[2], B[0], B[1], B[2], C[0], C[1], C[2] });
            }
            Console.WriteLine("{0}: {1} tris  floor up/down {2}/{3}  facing arena axis in/out {4}/{5}", name, p.Faces.Count, up, down, inward, outward);
        }
        using (var bw = new BinaryWriter(File.Create(Path.Combine(outDir, "arena_col.bin"))))
        {
            bw.Write(Encoding.ASCII.GetBytes("SARC")); bw.Write((uint)tris.Count);
            foreach (var t in tris) foreach (var f in t) bw.Write(f);
        }
        return 0;
    }

    // Minimal TGA reader (type 2/10, 24/32 bpp) -> Bitmap, for previews.
    static System.Drawing.Bitmap ReadTga(string path)
    {
        var b = File.ReadAllBytes(path);
        int type = b[2], w = b[12] | (b[13] << 8), h = b[14] | (b[15] << 8), bpp = b[16] / 8;
        bool top = (b[17] & 0x20) != 0;
        int pos = 18 + b[0], n = w * h, i = 0;
        var px = new byte[n * 4];
        while (i < n)
        {
            if (type == 10)
            {
                int c = b[pos++], cnt = (c & 0x7F) + 1;
                if ((c & 0x80) != 0) { for (int k = 0; k < cnt && i < n; k++, i++) { Array.Copy(b, pos, px, i * 4, bpp); if (bpp == 3) px[i*4+3] = 255; } pos += bpp; }
                else for (int k = 0; k < cnt && i < n; k++, i++) { Array.Copy(b, pos, px, i * 4, bpp); if (bpp == 3) px[i*4+3] = 255; pos += bpp; }
            }
            else { Array.Copy(b, pos, px, i * 4, bpp); if (bpp == 3) px[i*4+3] = 255; pos += bpp; i++; }
        }
        var bmp = new System.Drawing.Bitmap(w, h, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++)
            {
                int o = ((top ? y : h - 1 - y) * w + x) * 4;
                bmp.SetPixel(x, y, System.Drawing.Color.FromArgb(255, px[o+2], px[o+1], px[o]));
            }
        return bmp;
    }

    static int Rgb565(int c, int k) { return k == 0 ? ((c >> 11) & 31) * 255 / 31 : k == 1 ? ((c >> 5) & 63) * 255 / 63 : (c & 31) * 255 / 31; }

    // Decode DXT1 blocks (8 bytes each, little-endian) into BGRA top-down.
    public static byte[] DecodeDxt1(byte[] d, int off, int w, int h, bool swap16)
    {
        var px = new byte[w * h * 4];
        int bw = w / 4, bh = h / 4;
        for (int by = 0; by < bh; by++)
            for (int bx = 0; bx < bw; bx++)
            {
                int o = off + (by * bw + bx) * 8;
                int c0, c1; uint idx;
                if (!swap16) { c0 = d[o] | (d[o+1] << 8); c1 = d[o+2] | (d[o+3] << 8); idx = (uint)(d[o+4] | (d[o+5] << 8) | (d[o+6] << 16) | (d[o+7] << 24)); }
                else { c0 = d[o+1] | (d[o] << 8); c1 = d[o+3] | (d[o+2] << 8); idx = (uint)(d[o+5] | (d[o+4] << 8) | (d[o+7] << 16) | (d[o+6] << 24)); }
                var pal = new int[4, 3];
                for (int k = 0; k < 3; k++)
                {
                    int a = Rgb565(c0, k), b = Rgb565(c1, k);
                    pal[0, k] = a; pal[1, k] = b;
                    if (c0 > c1) { pal[2, k] = (2*a + b) / 3; pal[3, k] = (a + 2*b) / 3; }
                    else { pal[2, k] = (a + b) / 2; pal[3, k] = 0; }
                }
                for (int py = 0; py < 4; py++)
                    for (int pxx = 0; pxx < 4; pxx++)
                    {
                        int sel = (int)((idx >> (2 * (py * 4 + pxx))) & 3);
                        int q = ((by * 4 + py) * w + bx * 4 + pxx) * 4;
                        px[q] = (byte)pal[sel, 2]; px[q+1] = (byte)pal[sel, 1]; px[q+2] = (byte)pal[sel, 0]; px[q+3] = 255;
                    }
            }
        return px;
    }

    static System.Drawing.Bitmap ToBitmap(byte[] bgra, int w, int h)
    {
        var bmp = new System.Drawing.Bitmap(w, h, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
        var data = bmp.LockBits(new System.Drawing.Rectangle(0, 0, w, h), System.Drawing.Imaging.ImageLockMode.WriteOnly, bmp.PixelFormat);
        System.Runtime.InteropServices.Marshal.Copy(bgra, 0, data.Scan0, bgra.Length);
        bmp.UnlockBits(data);
        return bmp;
    }

    public static int Main(string[] args)
    {
        if (args.Length >= 4 && args[0] == "dxt1png")
        {
            // dxt1png <in.dds> <out.png> <swap 0|1>
            var d = File.ReadAllBytes(args[1]);
            int h = BitConverter.ToInt32(d, 12), w = BitConverter.ToInt32(d, 16);
            ToBitmap(DecodeDxt1(d, 128, w, h, args[3] == "1"), w, h).Save(args[2], System.Drawing.Imaging.ImageFormat.Png);
            return 0;
        }
        if (args.Length >= 3 && args[0] == "unswizzle")
        {
            // unswizzle <in.tga> <out.png> : treat UModel's pixel order as PS3 Morton (Z-order) swizzle
            var src = ReadTga(args[1]);
            int w = src.Width, h = src.Height;
            var lin = new System.Drawing.Color[w * h];
            for (int y = 0; y < h; y++) for (int x = 0; x < w; x++) lin[y * w + x] = src.GetPixel(x, y);
            var dst = new byte[w * h * 4];
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                {
                    int m = 0, bit = 0;
                    for (int k = 0; (1 << k) < Math.Max(w, h); k++)
                    {
                        if ((1 << k) < w) m |= ((x >> k) & 1) << bit++;
                        if ((1 << k) < h) m |= ((y >> k) & 1) << bit++;
                    }
                    var c = lin[m];
                    int q = (y * w + x) * 4;
                    dst[q] = c.B; dst[q+1] = c.G; dst[q+2] = c.R; dst[q+3] = 255;
                }
            ToBitmap(dst, w, h).Save(args[2], System.Drawing.Imaging.ImageFormat.Png);
            return 0;
        }
        if (args.Length >= 3 && args[0] == "g8asdxt")
        {
            // g8asdxt <in.tga> <outprefix> : reinterpret UModel's G8 bytes as DXT5 / DXT1 block data
            var src = ReadTga(args[1]);
            int w = src.Width, h = src.Height;
            var raw = new byte[w * h];
            for (int y = 0; y < h; y++) for (int x = 0; x < w; x++) raw[y * w + x] = src.GetPixel(x, y).R;
            // DXT5: 16-byte blocks, colour half at +8
            var c5 = new byte[raw.Length / 2];
            for (int b = 0; b < raw.Length / 16; b++) Array.Copy(raw, b * 16 + 8, c5, b * 8, 8);
            ToBitmap(DecodeDxt1(c5, 0, w, h, false), w, h).Save(args[2] + "_dxt5.png", System.Drawing.Imaging.ImageFormat.Png);
            ToBitmap(DecodeDxt1(raw, 0, w, h * 2, false), w, h * 2).Save(args[2] + "_dxt1.png", System.Drawing.Imaging.ImageFormat.Png);
            return 0;
        }
        if (args.Length >= 3 && args[0] == "topng")
        {
            for (int i = 2; i < args.Length; i++)
            {
                var bmp = ReadTga(args[i]);
                var small = new System.Drawing.Bitmap(bmp, Math.Min(512, bmp.Width), Math.Min(512, bmp.Height));
                small.Save(Path.Combine(args[1], Path.GetFileNameWithoutExtension(args[i]) + ".png"), System.Drawing.Imaging.ImageFormat.Png);
            }
            return 0;
        }
        if (args.Length >= 5 && args[0] == "section")
        {
            // section <ueY> <xmin> <xmax> <file.pskx...> : segments where triangles cross the plane Y=ueY (UE units), within x range
            float ys = float.Parse(args[1]), x0 = float.Parse(args[2]), x1 = float.Parse(args[3]);
            for (int a = 4; a < args.Length; a++)
            {
                var p = Psk.Load(args[a]);
                var segs = new List<string>();
                foreach (var f in p.Faces)
                {
                    var V = new[] { p.Points[p.WPoint[f[0]]], p.Points[p.WPoint[f[1]]], p.Points[p.WPoint[f[2]]] };
                    var hit = new List<float[]>();
                    for (int e = 0; e < 3; e++)
                    {
                        var P = V[e]; var Q = V[(e + 1) % 3];
                        if ((P[1] - ys) * (Q[1] - ys) > 0 || P[1] == Q[1]) continue;
                        float t = (ys - P[1]) / (Q[1] - P[1]);
                        hit.Add(new[] { P[0] + t * (Q[0] - P[0]), P[2] + t * (Q[2] - P[2]) });
                    }
                    if (hit.Count < 2) continue;
                    if (Math.Max(hit[0][0], hit[1][0]) < x0 || Math.Min(hit[0][0], hit[1][0]) > x1) continue;
                    if (hit[0][0] > hit[1][0]) { var tmp = hit[0]; hit[0] = hit[1]; hit[1] = tmp; }
                    segs.Add(string.Format("  x {0,7:0}..{1,7:0}  z {2,6:0}..{3,6:0}  {4}", hit[0][0], hit[1][0], hit[0][1], hit[1][1], p.Mats[f[3]]));
                }
                segs.Sort();
                Console.WriteLine(p.Name + ":"); foreach (var s in segs) Console.WriteLine(s);
            }
            return 0;
        }
        if (args.Length >= 2 && args[0] == "lowflat")
        {
            // lowflat <file.pskx...> : horizontal faces with z < 20 cm, grouped by material and height
            for (int a = 1; a < args.Length; a++)
            {
                var p = Psk.Load(args[a]);
                var g = new SortedDictionary<string, float[]>();
                foreach (var f in p.Faces)
                {
                    var A = p.Points[p.WPoint[f[0]]]; var B = p.Points[p.WPoint[f[1]]]; var C = p.Points[p.WPoint[f[2]]];
                    if (Math.Abs(A[2]-B[2]) > 0.5f || Math.Abs(A[2]-C[2]) > 0.5f || A[2] > 20) continue;
                    string k = string.Format("{0,-14} {1} z={2:0}", p.Name, p.Mats[f[3]], A[2]);
                    float[] b; if (!g.TryGetValue(k, out b)) g[k] = b = new float[] { 1e9f, 1e9f, -1e9f, -1e9f, 0 };
                    foreach (var v in new[] { A, B, C }) { b[0] = Math.Min(b[0], v[0]); b[1] = Math.Min(b[1], v[1]); b[2] = Math.Max(b[2], v[0]); b[3] = Math.Max(b[3], v[1]); }
                    b[4]++;
                }
                foreach (var kv in g) Console.WriteLine("{0}  n{1,4}  X[{2:0},{3:0}] Y[{4:0},{5:0}]", kv.Key, kv.Value[4], kv.Value[0], kv.Value[2], kv.Value[1], kv.Value[3]);
            }
            return 0;
        }
        if (args.Length >= 4 && args[0] == "export") return Export(args[1], args[2], args[3], args.Length >= 5 ? args[4] : null);
        if (args.Length >= 2 && args[0] == "dump")
        {
            foreach (var f in Directory.GetFiles(args[1], "*.pskx"))
            {
                var p = Psk.Load(f);
                var used = new Dictionary<int, int>();
                foreach (var fc in p.Faces) used[fc[3]] = (used.ContainsKey(fc[3]) ? used[fc[3]] : 0) + 1;
                Console.WriteLine("{0,-20} v{1,6} f{2,6}  X[{3,7:0},{4,7:0}] Y[{5,7:0},{6,7:0}] Z[{7,6:0},{8,6:0}]",
                    p.Name, p.Points.Count, p.Faces.Count, p.Min[0], p.Max[0], p.Min[1], p.Max[1], p.Min[2], p.Max[2]);
                for (int i = 0; i < p.Mats.Count; i++)
                    Console.WriteLine("      mat{0} {1} ({2} faces)", i, p.Mats[i], used.ContainsKey(i) ? used[i] : 0);
            }
            return 0;
        }
        if (args.Length >= 2 && args[0] == "profile")
        {
            // profile <file.pskx> : vertex cross-sections through the centre lines
            var p = Psk.Load(args[1]);
            var xs = new SortedSet<string>(); var ys = new SortedSet<string>();
            foreach (var v in p.Points)
            {
                if (Math.Abs(v[1]) < 1500 && v[0] >= 0) xs.Add(string.Format("{0,7:0} {1,6:0}  (y {2:0})", v[0], v[2], v[1]));
                if (Math.Abs(v[0]) < 2500 && v[1] <= 0) ys.Add(string.Format("{0,7:0} {1,6:0}  (x {2:0})", v[1], v[2], v[0]));
            }
            Console.WriteLine("--- X profile (x z) near y=0"); foreach (var s in xs) Console.WriteLine(s);
            Console.WriteLine("--- Y profile (y z) near x=0"); foreach (var s in ys) Console.WriteLine(s);
            return 0;
        }
        if (args.Length >= 3 && args[0] == "render")
        {
            // render <out.png> <a.pskx> [b.pskx ...] : top view (left) + side view (right), shaded by normal
            var bmp = new System.Drawing.Bitmap(1600, 900);
            var g = System.Drawing.Graphics.FromImage(bmp);
            g.Clear(System.Drawing.Color.Black);
            float s = 860f / 40000f;
            for (int a = 2; a < args.Length; a++)
            {
                var p = Psk.Load(args[a]);
                foreach (var f in p.Faces)
                {
                    var A = p.Points[p.WPoint[f[0]]]; var B = p.Points[p.WPoint[f[1]]]; var C = p.Points[p.WPoint[f[2]]];
                    float ux = B[0]-A[0], uy = B[1]-A[1], uz = B[2]-A[2], vx = C[0]-A[0], vy = C[1]-A[1], vz = C[2]-A[2];
                    float nx = uy*vz-uz*vy, ny = uz*vx-ux*vz, nz = ux*vy-uy*vx, nl = (float)Math.Sqrt(nx*nx+ny*ny+nz*nz) + 1e-9f;
                    int sh = (int)(60 + 190 * Math.Abs(nz / nl));
                    int hue = a == 2 ? 0 : 1;
                    var col = System.Drawing.Color.FromArgb(120, hue == 0 ? sh : sh/3, hue == 0 ? sh : sh, hue == 0 ? sh/2 : sh);
                    var br = new System.Drawing.SolidBrush(col);
                    // top view: x right, y down
                    g.FillPolygon(br, new[] {
                        new System.Drawing.PointF(400 + A[0]*s, 450 + A[1]*s), new System.Drawing.PointF(400 + B[0]*s, 450 + B[1]*s),
                        new System.Drawing.PointF(400 + C[0]*s, 450 + C[1]*s) });
                    // side view: y right, z up
                    g.FillPolygon(br, new[] {
                        new System.Drawing.PointF(1200 + A[1]*s, 800 - A[2]*s), new System.Drawing.PointF(1200 + B[1]*s, 800 - B[2]*s),
                        new System.Drawing.PointF(1200 + C[1]*s, 800 - C[2]*s) });
                    br.Dispose();
                }
            }
            bmp.Save(args[1], System.Drawing.Imaging.ImageFormat.Png);
            return 0;
        }
        Console.WriteLine("usage: ArenaExport dump <dir>");
        return 1;
    }
}
