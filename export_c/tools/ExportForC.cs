// ExportForC.cs - Converts the SARPBC car GLBs + PS3 textures into a C-friendly package.
//
// Output (export_c/cars/<name>/):
//   <name>.sarm         binary mesh (see include/sarpbc_model.h for the layout)
//   body_blue.tga       baked team-paint diffuse (Blue team, *_ColorMod_Inst values)
//   body_orange.tga     baked team-paint diffuse (Orange/red team, *_ColorMod_Red values)
//   body_normal.tga     body normal map, OpenGL convention (+Y green up)
//   tire_diffuse.tga    tire diffuse (RGBA, alpha = cut-out mask)
//   tire_normal.tga     tire normal map, OpenGL convention
//   <name>.obj/.mtl     same mesh for Blender / other tools
//
// Build: csc /r:System.Web.Extensions.dll /r:System.Drawing.dll ExportForC.cs
using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Globalization;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using System.Web.Script.Serialization;

class ExportForC
{
    const string Root = @"e:\Sarpbc\exported_assets";
    const string Unc = @"e:\Sarpbc\exported_assets\uncooked_ps3";
    const string OutRoot = @"e:\Sarpbc\export_c\cars";
    static string PreviewDir = null;

    class Car
    {
        public string Name, Glb, Mask, Normal, TireDiff, TireNorm;
        public double[][] Blue, Orange; // C1, C2, C3 in linear RGB
    }

    static double[] V(double r, double g, double b) { return new[] { r, g, b }; }

    static Car[] Cars()
    {
        return new[]
        {
            new Car { Name = "backfire", Glb = "Model-T.glb",
                Mask = Unc + @"\Vehicle_Model-T\Materials\Masks_v2.png",
                Normal = Unc + @"\Vehicle_Model-T\Materials\Model-T_Normal.png",
                TireDiff = Unc + @"\Vehicle_Model-T\Materials\Model-T_RearTire_D.png",
                TireNorm = Unc + @"\Vehicle_Model-T\Materials\Model-T_RearTire_N.png",
                Blue = new[] { V(0.2195,0.2195,0.2195), V(0.9158,0.9158,0.9158), V(0.0199,0.0415,0.6388) },
                Orange = new[] { V(0.1801,0.1801,0.1801), V(0.9490,0.9490,0.9490), V(0.5732,0.0370,0.0370) } },
            new Car { Name = "octane", Glb = "RaceCar.glb",
                Mask = Unc + @"\Vehicle_RaceCar\Materials\RaceCar01_Masks_02.png",
                Normal = Unc + @"\Vehicle_RaceCar\Materials\RaceCar01_normals_02.png",
                TireDiff = null, // original Tire01_rgba is corrupt in the PS3 data
                TireNorm = Unc + @"\Vehicle_RaceCar\Materials\Tire01_normal.png",
                Blue = new[] { V(0.8277,0.8277,0.8277), V(0.2271,0.2271,0.2271), V(0.0199,0.0415,0.6388) },
                Orange = new[] { V(0.9158,0.9158,0.9158), V(0.1768,0.1768,0.1768), V(0.5732,0.0370,0.0370) } },
            new Car { Name = "scarab", Glb = "SteamPunkCar01.glb",
                Mask = Unc + @"\Vehicle_SteamPunkCar01\Materials\SteamPunkCar_D.png",
                Normal = Unc + @"\Vehicle_SteamPunkCar01\Materials\SteamPunkCar_N.png",
                TireDiff = Unc + @"\Vehicle_SteamPunkCar01\Materials\Tire01-diffuse.png",
                TireNorm = Unc + @"\Vehicle_SteamPunkCar01\Materials\Tire01-normals.png",
                Blue = new[] { V(0.0199,0.0415,0.6456), V(0.8671,0.8671,0.8671), V(0.1801,0.1801,0.1801) },
                Orange = new[] { V(0.2158,0.2158,0.2158), V(0.5732,0.0370,0.0370), V(0.9158,0.9158,0.9158) } },
            new Car { Name = "aftershock", Glb = "SpaceCar01.glb",
                Mask = Unc + @"\Vehicle_SpaceCar\Materials\SpaceCar_Masks_v2.png",
                Normal = Unc + @"\Vehicle_SpaceCar\Materials\SpaceCar_Normal.png",
                TireDiff = Unc + @"\Vehicle_SpaceCar\Materials\Tire_Diffuse_512.png",
                TireNorm = Unc + @"\Vehicle_SpaceCar\Materials\Tire_Normal.png",
                Blue = new[] { V(0.0199,0.0415,0.6456), V(0.1087,0.1087,0.1087), V(0.8592,0.8592,0.8592) },
                Orange = new[] { V(0.5732,0.0370,0.0370), V(0.1604,0.1604,0.1604), V(0.8751,0.8751,0.8751) } },
            new Car { Name = "renegade", Glb = "MonsterTruck.glb",
                Mask = Unc + @"\Vehicle_MonsterTruck\Materials\Masks_v2.png",
                Normal = Unc + @"\Vehicle_MonsterTruck\Materials\Normal.png",
                TireDiff = Unc + @"\Vehicle_MonsterTruck\Materials\MonsterTruck_Tire_Diffuse.png",
                TireNorm = Unc + @"\Vehicle_MonsterTruck\Materials\MT_Tire_Normal.png",
                Blue = new[] { V(0.8671,0.8671,0.8671), V(0.2084,0.2084,0.2084), V(0.0199,0.0415,0.6456) },
                Orange = new[] { V(0.8355,0.8355,0.8355), V(0.1734,0.1734,0.1734), V(0.5732,0.0370,0.0370) } },
            new Car { Name = "zippy", Glb = "Convertible.glb",
                Mask = Unc + @"\Vehicle_Convertible\Materials\Convertible_Masks_v2.png",
                Normal = Unc + @"\Vehicle_Convertible\Materials\Convertible_Normalsmap.png",
                TireDiff = Unc + @"\Vehicle_Convertible\Materials\Tire_Diffuse_512.png",
                TireNorm = Unc + @"\Vehicle_Convertible\Materials\Tire_Normal.png",
                Blue = new[] { V(0.2674,0.2674,0.2674), V(0.0199,0.0415,0.6388), V(0.9490,0.9490,1.0) },
                Orange = new[] { V(0.9406,0.9406,0.9406), V(0.5732,0.0370,0.0370), V(0.2158,0.2158,0.2158) } },
            new Car { Name = "marauder", Glb = "Armored01.glb",
                Mask = Unc + @"\Vehicle_Armored\Materials\Masks_v2.png",
                Normal = Unc + @"\Vehicle_Armored\Materials\Armored_Normal.png",
                TireDiff = Unc + @"\Vehicle_Armored\Materials\Tire_Diff.png",
                TireNorm = Unc + @"\Vehicle_Armored\Materials\Tire_Normal1.png",
                Blue = new[] { V(0.1768,0.1768,0.1768), V(0.0199,0.0415,0.6456), V(0.8671,0.8671,0.8671) },
                Orange = new[] { V(0.8994,0.8994,0.8994), V(0.5732,0.0370,0.0370), V(0.2158,0.2158,0.2158) } },
        };
    }

    // ---------------------------------------------------------------- GLB reading
    class Prim { public float[] Pos, Nrm, Uv, Tan; public uint[] Idx; public string Mat; }

    static double D(object o) { return Convert.ToDouble(o, CultureInfo.InvariantCulture); }
    static int I(object o) { return Convert.ToInt32(o, CultureInfo.InvariantCulture); }
    static int Get(Dictionary<string, object> d, string k, int def) { object v; return d.TryGetValue(k, out v) ? I(v) : def; }

    static float[] ReadFloats(byte[] bin, int binStart, Dictionary<string, object> json, int accIdx, int comps)
    {
        var acc = (Dictionary<string, object>)((ArrayList)json["accessors"])[accIdx];
        var bv = (Dictionary<string, object>)((ArrayList)json["bufferViews"])[I(acc["bufferView"])];
        int count = I(acc["count"]);
        int off = binStart + Get(bv, "byteOffset", 0) + Get(acc, "byteOffset", 0);
        int stride = Get(bv, "byteStride", comps * 4);
        if (I(acc["componentType"]) != 5126) throw new Exception("expected float accessor");
        var r = new float[count * comps];
        for (int i = 0; i < count; i++)
            for (int c = 0; c < comps; c++)
                r[i * comps + c] = BitConverter.ToSingle(bin, off + i * stride + c * 4);
        return r;
    }

    static uint[] ReadIndices(byte[] bin, int binStart, Dictionary<string, object> json, int accIdx)
    {
        var acc = (Dictionary<string, object>)((ArrayList)json["accessors"])[accIdx];
        var bv = (Dictionary<string, object>)((ArrayList)json["bufferViews"])[I(acc["bufferView"])];
        int count = I(acc["count"]);
        int off = binStart + Get(bv, "byteOffset", 0) + Get(acc, "byteOffset", 0);
        int type = I(acc["componentType"]);
        var r = new uint[count];
        for (int i = 0; i < count; i++)
        {
            if (type == 5123) r[i] = BitConverter.ToUInt16(bin, off + i * 2);
            else if (type == 5125) r[i] = BitConverter.ToUInt32(bin, off + i * 4);
            else if (type == 5121) r[i] = bin[off + i];
            else throw new Exception("bad index type " + type);
        }
        return r;
    }

    static List<Prim> LoadGlb(string path, float[][] wheels)
    {
        byte[] b = File.ReadAllBytes(path);
        if (BitConverter.ToUInt32(b, 0) != 0x46546C67) throw new Exception("not a GLB");
        int jsonLen = (int)BitConverter.ToUInt32(b, 12);
        string js = Encoding.UTF8.GetString(b, 20, jsonLen);
        int binStart = 20 + jsonLen + 8;
        var ser = new JavaScriptSerializer { MaxJsonLength = int.MaxValue };
        var json = (Dictionary<string, object>)ser.DeserializeObject(js);
        // DeserializeObject returns object[] for arrays; normalise to ArrayList
        Normalize(json);

        var mats = (ArrayList)json["materials"];
        var prims = new List<Prim>();
        var mesh = (Dictionary<string, object>)((ArrayList)json["meshes"])[0];
        foreach (Dictionary<string, object> p in (ArrayList)mesh["primitives"])
        {
            var a = (Dictionary<string, object>)p["attributes"];
            var pr = new Prim();
            pr.Pos = ReadFloats(b, binStart, json, I(a["POSITION"]), 3);
            pr.Nrm = ReadFloats(b, binStart, json, I(a["NORMAL"]), 3);
            pr.Uv = ReadFloats(b, binStart, json, I(a["TEXCOORD_0"]), 2);
            pr.Tan = a.ContainsKey("TANGENT") ? ReadFloats(b, binStart, json, I(a["TANGENT"]), 4) : new float[pr.Pos.Length / 3 * 4];
            pr.Idx = ReadIndices(b, binStart, json, I(p["indices"]));
            pr.Mat = (string)((Dictionary<string, object>)mats[I(p["material"])])["name"];
            prims.Add(pr);
        }

        string[] wheelNames = { "LeftFrontTire", "RightFrontTire", "LeftRearTire", "RightRearTire" };
        foreach (Dictionary<string, object> n in (ArrayList)json["nodes"])
        {
            object nm; if (!n.TryGetValue("name", out nm)) continue;
            int wi = Array.IndexOf(wheelNames, (string)nm);
            if (wi < 0 || !n.ContainsKey("translation")) continue;
            var t = (ArrayList)n["translation"];
            wheels[wi] = new[] { (float)D(t[0]), (float)D(t[1]), (float)D(t[2]) };
        }
        return prims;
    }

    static void Normalize(object o)
    {
        var d = o as Dictionary<string, object>;
        if (d != null)
        {
            foreach (var k in new List<string>(d.Keys))
            {
                if (d[k] is object[]) d[k] = new ArrayList((object[])d[k]);
                Normalize(d[k]);
            }
            return;
        }
        var l = o as ArrayList;
        if (l != null)
            for (int i = 0; i < l.Count; i++)
            {
                if (l[i] is object[]) l[i] = new ArrayList((object[])l[i]);
                Normalize(l[i]);
            }
    }

    static uint MatId(string name)
    {
        string n = name.ToLowerInvariant();
        if (n.Contains("win") || n.Contains("glass")) return 1;
        if (n.Contains("tire") || n.Contains("wheel")) return 2;
        return 0;
    }

    // ---------------------------------------------------------------- Images
    static byte[] LoadBgra(string path, out int w, out int h)
    {
        using (var src = new Bitmap(path))
        {
            w = src.Width; h = src.Height;
            var data = src.LockBits(new Rectangle(0, 0, w, h), ImageLockMode.ReadOnly, PixelFormat.Format32bppArgb);
            var px = new byte[w * h * 4];
            for (int y = 0; y < h; y++)
                Marshal.Copy(data.Scan0 + y * data.Stride, px, y * w * 4, w * 4);
            src.UnlockBits(data);
            return px; // B,G,R,A per pixel, rows top -> bottom
        }
    }

    static void WriteTga(string path, byte[] bgra, int w, int h)
    {
        using (var f = new BinaryWriter(File.Create(path)))
        {
            f.Write((byte)0); f.Write((byte)0); f.Write((byte)2);        // id len, no colormap, uncompressed truecolor
            f.Write(new byte[5]);                                       // colormap spec
            f.Write((ushort)0); f.Write((ushort)0);                     // x/y origin
            f.Write((ushort)w); f.Write((ushort)h);
            f.Write((byte)32); f.Write((byte)0x28);                     // 32bpp, 8 alpha bits, top-left origin
            f.Write(bgra);
        }
    }

    static void SavePreview(string name, byte[] bgra, int w, int h)
    {
        if (PreviewDir == null) return;
        using (var bmp = new Bitmap(w, h, PixelFormat.Format32bppArgb))
        {
            var data = bmp.LockBits(new Rectangle(0, 0, w, h), ImageLockMode.WriteOnly, PixelFormat.Format32bppArgb);
            for (int y = 0; y < h; y++) Marshal.Copy(bgra, y * w * 4, data.Scan0 + y * data.Stride, w * 4);
            bmp.UnlockBits(data);
            bmp.Save(Path.Combine(PreviewDir, name + ".png"), ImageFormat.Png);
        }
    }

    static byte LinToSrgb(double c)
    {
        c = Math.Max(0, Math.Min(1, c));
        double s = c <= 0.0031308 ? c * 12.92 : 1.055 * Math.Pow(c, 1.0 / 2.4) - 0.055;
        return (byte)Math.Round(s * 255.0);
    }

    // Same maths as the viewer: R -> C1, G -> C2, B -> C3, black -> bare chassis metal
    static byte[] BakeLivery(byte[] mask, int w, int h, double[][] c)
    {
        double[] baseCol = { 0.035, 0.038, 0.045 };
        var o = new byte[mask.Length];
        for (int i = 0; i < w * h; i++)
        {
            double r = mask[i * 4 + 2] / 255.0, g = mask[i * 4 + 1] / 255.0, b = mask[i * 4] / 255.0;
            double sum = r + g + b, painted = Math.Min(1.0, sum), inv = 1.0 / Math.Max(sum, 0.001);
            var lin = new double[3];
            for (int k = 0; k < 3; k++)
            {
                double livery = (r * c[0][k] + g * c[1][k] + b * c[2][k]) * inv;
                lin[k] = baseCol[k] + (livery - baseCol[k]) * painted;
            }
            o[i * 4 + 2] = LinToSrgb(lin[0]);
            o[i * 4 + 1] = LinToSrgb(lin[1]);
            o[i * 4 + 0] = LinToSrgb(lin[2]);
            o[i * 4 + 3] = 255;
        }
        return o;
    }

    // Unreal stores DirectX-style normal maps (green = -Y); flip green for OpenGL
    static byte[] ToGlNormal(byte[] bgra)
    {
        var o = (byte[])bgra.Clone();
        for (int i = 0; i < o.Length; i += 4) { o[i + 1] = (byte)(255 - o[i + 1]); o[i + 3] = 255; }
        return o;
    }

    // ---------------------------------------------------------------- Main
    static void Main(string[] args)
    {
        if (args.Length > 0) { PreviewDir = args[0]; Directory.CreateDirectory(PreviewDir); }
        Directory.CreateDirectory(OutRoot);
        var inv = CultureInfo.InvariantCulture;

        foreach (var car in Cars())
        {
            string dir = Path.Combine(OutRoot, car.Name);
            Directory.CreateDirectory(dir);
            var wheels = new float[4][];
            var prims = LoadGlb(Path.Combine(Root, "cars_glb", car.Glb), wheels);

            // ---- merge primitives into one vertex/index buffer with submeshes
            var verts = new List<float>();
            var idx = new List<uint>();
            var subs = new List<uint[]>();
            float[] mn = { float.MaxValue, float.MaxValue, float.MaxValue }, mx = { float.MinValue, float.MinValue, float.MinValue };
            foreach (var p in prims)
            {
                uint baseV = (uint)(verts.Count / 12);
                int n = p.Pos.Length / 3;
                for (int i = 0; i < n; i++)
                {
                    for (int k = 0; k < 3; k++) { float v = p.Pos[i * 3 + k]; verts.Add(v); mn[k] = Math.Min(mn[k], v); mx[k] = Math.Max(mx[k], v); }
                    for (int k = 0; k < 3; k++) verts.Add(p.Nrm[i * 3 + k]);
                    for (int k = 0; k < 2; k++) verts.Add(p.Uv[i * 2 + k]);
                    for (int k = 0; k < 4; k++) verts.Add(p.Tan[i * 4 + k]);
                }
                subs.Add(new uint[] { (uint)idx.Count, (uint)p.Idx.Length, MatId(p.Mat), 0 });
                foreach (var ix in p.Idx) idx.Add(ix + baseV);
            }
            uint vcount = (uint)(verts.Count / 12);

            // ---- .sarm binary
            using (var f = new BinaryWriter(File.Create(Path.Combine(dir, car.Name + ".sarm"))))
            {
                f.Write(Encoding.ASCII.GetBytes("SARM"));
                f.Write(1u);                       // version
                f.Write(vcount);
                f.Write((uint)idx.Count);
                f.Write((uint)subs.Count);
                f.Write(0u);                       // reserved
                foreach (var v in mn) f.Write(v);
                foreach (var v in mx) f.Write(v);
                for (int wi = 0; wi < 4; wi++)
                    for (int k = 0; k < 3; k++) f.Write(wheels[wi] != null ? wheels[wi][k] : 0f);
                foreach (var s in subs) foreach (var u in s) f.Write(u);
                foreach (var v in verts) f.Write(v);
                foreach (var u in idx) f.Write(u);
            }

            // ---- textures
            int w, h;
            var mask = LoadBgra(car.Mask, out w, out h);
            var blue = BakeLivery(mask, w, h, car.Blue);
            var orange = BakeLivery(mask, w, h, car.Orange);
            WriteTga(Path.Combine(dir, "body_blue.tga"), blue, w, h);
            WriteTga(Path.Combine(dir, "body_orange.tga"), orange, w, h);
            SavePreview(car.Name + "_body_blue", blue, w, h);
            SavePreview(car.Name + "_body_orange", orange, w, h);

            var nrm = LoadBgra(car.Normal, out w, out h);
            WriteTga(Path.Combine(dir, "body_normal.tga"), ToGlNormal(nrm), w, h);

            if (car.TireDiff != null)
            {
                var td = LoadBgra(car.TireDiff, out w, out h);
                WriteTga(Path.Combine(dir, "tire_diffuse.tga"), td, w, h);
            }
            else
            {
                // Solid dark rubber stand-in (sRGB ~ #1A1D22)
                var td = new byte[4 * 4 * 4];
                for (int i = 0; i < 16; i++) { td[i * 4] = 0x22; td[i * 4 + 1] = 0x1D; td[i * 4 + 2] = 0x1A; td[i * 4 + 3] = 255; }
                WriteTga(Path.Combine(dir, "tire_diffuse.tga"), td, 4, 4);
            }
            var tn = LoadBgra(car.TireNorm, out w, out h);
            WriteTga(Path.Combine(dir, "tire_normal.tga"), ToGlNormal(tn), w, h);

            // ---- OBJ + MTL
            string[] mtlNames = { "body", "glass", "tire" };
            using (var m = new StreamWriter(Path.Combine(dir, car.Name + ".mtl")))
            {
                m.WriteLine("# SARPBC " + car.Name);
                m.WriteLine("newmtl body\nKd 1 1 1\nNs 60\nmap_Kd body_blue.tga\nmap_Bump body_normal.tga\n");
                m.WriteLine("newmtl glass\nKd 0.06 0.09 0.16\nNs 200\nd 0.6\n");
                m.WriteLine("newmtl tire\nKd 1 1 1\nNs 10\nmap_Kd tire_diffuse.tga\nmap_Bump tire_normal.tga\nmap_d tire_diffuse.tga\n");
            }
            using (var o = new StreamWriter(Path.Combine(dir, car.Name + ".obj")))
            {
                o.WriteLine("# SARPBC " + car.Name + " - Y up, meters");
                o.WriteLine("mtllib " + car.Name + ".mtl");
                for (int i = 0; i < vcount; i++)
                    o.WriteLine(string.Format(inv, "v {0} {1} {2}", verts[i * 12], verts[i * 12 + 1], verts[i * 12 + 2]));
                for (int i = 0; i < vcount; i++)   // OBJ has V=0 at the bottom of the image
                    o.WriteLine(string.Format(inv, "vt {0} {1}", verts[i * 12 + 6], 1.0f - verts[i * 12 + 7]));
                for (int i = 0; i < vcount; i++)
                    o.WriteLine(string.Format(inv, "vn {0} {1} {2}", verts[i * 12 + 3], verts[i * 12 + 4], verts[i * 12 + 5]));
                foreach (var s in subs)
                {
                    o.WriteLine("usemtl " + mtlNames[s[2]]);
                    for (uint t = 0; t < s[1]; t += 3)
                    {
                        uint a = idx[(int)(s[0] + t)] + 1, b2 = idx[(int)(s[0] + t + 1)] + 1, c = idx[(int)(s[0] + t + 2)] + 1;
                        o.WriteLine("f {0}/{0}/{0} {1}/{1}/{1} {2}/{2}/{2}", a, b2, c);
                    }
                }
            }

            Console.WriteLine(string.Format(inv, "{0,-11} verts={1,5} tris={2,5} submeshes={3} size=({4:F2} x {5:F2} x {6:F2}) m",
                car.Name, vcount, idx.Count / 3, subs.Count, mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]));
        }
    }
}
