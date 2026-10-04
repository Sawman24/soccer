// PkgDump.cs - minimal UE3 (SARPBC PS3, package ver 539, big-endian) package reader.
// Decompresses a fully-compressed cooked package, then dumps the tagged default
// properties of selected exports (e.g. Default__TASimCar) so the remake can use the
// original physics values.
//
// Usage (PowerShell, no exe needed):
//   Add-Type -Path PkgDump.cs
//   [PkgDump]::Run("...\COOKEDPS3\TAGAME.XXX", "Default__TASimCar,Default__TAVehicleWheel")
//   [PkgDump]::List("...\TAGAME.XXX", "Ball")   # list exports whose name contains "Ball"
using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Text;

public static class PkgDump
{
    static byte[] img;
    static string[] names;
    class Exp { public int Class, Super, Outer, Name, NameNum, Archetype, Size, Offset; public string FullName; }
    class Imp { public int ClassName, Outer, Name; }
    static List<Exp> exps = new List<Exp>();
    static List<Imp> imps = new List<Imp>();
    static StringBuilder sb;

    // ---------------------------------------------------------------- big-endian helpers
    static int I32(byte[] b, int o) { return (b[o] << 24) | (b[o + 1] << 16) | (b[o + 2] << 8) | b[o + 3]; }
    static float F32(byte[] b, int o) { var t = new byte[] { b[o + 3], b[o + 2], b[o + 1], b[o] }; return BitConverter.ToSingle(t, 0); }

    // ---------------------------------------------------------------- LZO1X decompressor (for PS3 packages)
    // State-machine port of lzo1x_decompress (C# can't goto into loops).
    static int Lzo(byte[] src, int sp, int slen, byte[] dst, int dp)
    {
        const int LOOP = 0, FIRST_LIT = 1, MATCH = 2, MATCH_DONE = 3, MATCH_NEXT = 4;
        int ip = sp, op = dp, t = 0, m, state;
        if (src[ip] > 17)
        {
            t = src[ip++] - 17;
            if (t < 4) state = MATCH_NEXT;
            else { do dst[op++] = src[ip++]; while (--t > 0); state = FIRST_LIT; }
        }
        else state = LOOP;
        while (true)
        {
            switch (state)
            {
            case LOOP:
                t = src[ip++];
                if (t >= 16) { state = MATCH; break; }
                if (t == 0) { while (src[ip] == 0) { t += 255; ip++; } t += 15 + src[ip++]; }
                t += 3;
                do dst[op++] = src[ip++]; while (--t > 0);
                state = FIRST_LIT;
                break;
            case FIRST_LIT:
                t = src[ip++];
                if (t >= 16) { state = MATCH; break; }
                m = op - 0x0801 - (t >> 2) - (src[ip++] << 2);
                dst[op++] = dst[m++]; dst[op++] = dst[m++]; dst[op++] = dst[m];
                state = MATCH_DONE;
                break;
            case MATCH:
                if (t >= 64)
                {
                    m = op - 1 - ((t >> 2) & 7) - (src[ip++] << 3);
                    t = (t >> 5) - 1;
                }
                else if (t >= 32)
                {
                    t &= 31;
                    if (t == 0) { while (src[ip] == 0) { t += 255; ip++; } t += 31 + src[ip++]; }
                    m = op - 1 - ((src[ip] >> 2) + (src[ip + 1] << 6));
                    ip += 2;
                }
                else if (t >= 16)
                {
                    m = op - ((t & 8) << 11);
                    t &= 7;
                    if (t == 0) { while (src[ip] == 0) { t += 255; ip++; } t += 7 + src[ip++]; }
                    m -= (src[ip] >> 2) + (src[ip + 1] << 6);
                    ip += 2;
                    if (m == op) return op - dp;          // end of stream
                    m -= 0x4000;
                }
                else
                {
                    m = op - 1 - (t >> 2) - (src[ip++] << 2);
                    dst[op++] = dst[m++]; dst[op++] = dst[m];
                    state = MATCH_DONE;
                    break;
                }
                dst[op++] = dst[m++]; dst[op++] = dst[m++];
                do dst[op++] = dst[m++]; while (--t > 0);
                state = MATCH_DONE;
                break;
            case MATCH_DONE:
                t = src[ip - 2] & 3;
                state = t == 0 ? LOOP : MATCH_NEXT;
                break;
            case MATCH_NEXT:
                do dst[op++] = src[ip++]; while (--t > 0);
                t = src[ip++];
                state = MATCH;
                break;
            }
        }
    }

    static void Decompress(byte[] f, int compFlags, byte[] dst, int dstOff, int compOff)
    {
        // chunk: tag, blockSize, compSize, uncompSize, then (comp, uncomp) per block
        int blockSize = I32(f, compOff + 4), total = I32(f, compOff + 12);
        int nb = (total + blockSize - 1) / blockSize;
        int hp = compOff + 16, dp = compOff + 16 + nb * 8, op = dstOff;
        for (int i = 0; i < nb; i++)
        {
            int cs = I32(f, hp + i * 8), us = I32(f, hp + i * 8 + 4);
            if ((compFlags & 2) != 0) Lzo(f, dp, cs, dst, op);
            else
            {
                using (var ds = new DeflateStream(new MemoryStream(f, dp + 2, cs - 2), CompressionMode.Decompress))
                {
                    int got = 0; while (got < us) { int r = ds.Read(dst, op + got, us - got); if (r <= 0) break; got += r; }
                }
            }
            dp += cs; op += us;
        }
    }

    // ---------------------------------------------------------------- package loading
    static void Load(string path)
    {
        var f = File.ReadAllBytes(path);
        int p = 8;
        int headerSize = I32(f, p); p += 4;
        int fl = I32(f, p); p += 4 + fl;           // folder name (ANSI FString)
        int pkgFlags = I32(f, p); p += 4;
        int nameCount = I32(f, p), nameOff = I32(f, p + 4), expCount = I32(f, p + 8), expOff = I32(f, p + 12);
        int impCount = I32(f, p + 16), impOff = I32(f, p + 20); p += 28;   // + dependsOffset
        p += 16;                                    // guid
        int gens = I32(f, p); p += 4 + gens * 12;
        p += 8;                                     // engine, cooker version
        int compFlags = I32(f, p); p += 4;
        int chunks = I32(f, p); p += 4;
        int[,] ch = new int[chunks, 4];
        int uncompEnd = 0;
        for (int i = 0; i < chunks; i++)
        {
            for (int k = 0; k < 4; k++) ch[i, k] = I32(f, p + k * 4);
            p += 16;
            uncompEnd = Math.Max(uncompEnd, ch[i, 0] + ch[i, 1]);
        }
        Console.WriteLine("flags {0:X8} compFlags {1} chunks {2} names {3} exports {4} imports {5}", pkgFlags, compFlags, chunks, nameCount, expCount, impCount);
        if (chunks == 0) img = f;
        else
        {
            img = new byte[uncompEnd];
            Array.Copy(f, img, Math.Min(f.Length, ch[0, 0]));
            for (int i = 0; i < chunks; i++) Decompress(f, compFlags, img, ch[i, 0], ch[i, 2]);
        }
        // names
        names = new string[nameCount];
        p = nameOff;
        for (int i = 0; i < nameCount; i++)
        {
            int len = I32(img, p); p += 4;
            if (len < 0) { names[i] = Encoding.BigEndianUnicode.GetString(img, p, -len * 2 - 2); p += -len * 2; }
            else { names[i] = Encoding.ASCII.GetString(img, p, Math.Max(0, len - 1)); p += len; }
            p += 8;                                 // flags (qword)
        }
        // imports
        p = impOff;
        for (int i = 0; i < impCount; i++)
        {
            imps.Add(new Imp { ClassName = I32(img, p + 8), Outer = I32(img, p + 16), Name = I32(img, p + 20) });
            p += 28;
        }
        // exports (ver 539: has component map, net object counts, guid, package flags)
        p = expOff;
        for (int i = 0; i < expCount; i++)
        {
            var e = new Exp();
            e.Class = I32(img, p); e.Super = I32(img, p + 4); e.Outer = I32(img, p + 8);
            e.Name = I32(img, p + 12); e.NameNum = I32(img, p + 16); e.Archetype = I32(img, p + 20);
            p += 24 + 8;                            // + object flags qword
            e.Size = I32(img, p); e.Offset = I32(img, p + 4); p += 8;
            int comps = I32(img, p); p += 4 + comps * 12;
            p += 4;                                 // export flags
            int nets = I32(img, p); p += 4 + nets * 4;
            p += 16 + 4;                            // package guid + package flags
            exps.Add(e);
        }
        foreach (var e in exps) e.FullName = ObjName(e);
    }

    static string N(int idx) { return idx >= 0 && idx < names.Length ? names[idx] : "?" + idx; }
    static string ObjName(Exp e) { string s = N(e.Name) + (e.NameNum > 0 ? "_" + (e.NameNum - 1) : ""); return e.Outer > 0 ? ObjName(exps[e.Outer - 1]) + "." + s : s; }
    static string RefName(int r)
    {
        if (r > 0 && r <= exps.Count) return exps[r - 1].FullName;
        if (r < 0 && -r <= imps.Count) return N(imps[-r - 1].Name);
        return "None";
    }
    static string ClassOf(Exp e) { return RefName(e.Class); }

    // ---------------------------------------------------------------- tagged properties
    static readonly HashSet<string> RawVec = new HashSet<string> { "Vector", "Rotator", "Color", "LinearColor", "Vector2D", "Quat", "Guid", "Plane", "Matrix", "Box" };

    static int Props(int p, int end, string ind)
    {
        while (p + 8 <= end)
        {
            string name = N(I32(img, p)); p += 8;
            if (name == "None") return p;
            string type = N(I32(img, p)); p += 8;
            int size = I32(img, p), arrIdx = I32(img, p + 4); p += 8;
            string label = arrIdx > 0 ? name + "[" + arrIdx + "]" : name;
            if (type == "BoolProperty") { sb.AppendLine(ind + label + " = " + (I32(img, p) != 0)); p += 4; continue; }
            string structName = null;
            if (type == "StructProperty") { structName = N(I32(img, p)); p += 8; }
            int vs = p;
            switch (type)
            {
                case "FloatProperty": sb.AppendLine(ind + label + " = " + F32(img, p)); break;
                case "IntProperty": sb.AppendLine(ind + label + " = " + I32(img, p)); break;
                case "ByteProperty": sb.AppendLine(ind + label + " = " + (size == 8 ? N(I32(img, p)) : img[p].ToString())); break;
                case "NameProperty": sb.AppendLine(ind + label + " = " + N(I32(img, p))); break;
                case "ObjectProperty": case "ComponentProperty": case "ClassProperty":
                    sb.AppendLine(ind + label + " = " + RefName(I32(img, p))); break;
                case "StrProperty":
                    { int l = I32(img, p); sb.AppendLine(ind + label + " = \"" + (l > 0 ? Encoding.ASCII.GetString(img, p + 4, l - 1) : "") + "\""); } break;
                case "StructProperty":
                    if (RawVec.Contains(structName) || size == 12 && structName.EndsWith("Vector"))
                    {
                        var parts = new List<string>();
                        for (int k = 0; k + 4 <= size && k < 64; k += 4) parts.Add(structName == "Rotator" || structName == "Color" ? I32(img, p + k).ToString() : F32(img, p + k).ToString("0.####"));
                        sb.AppendLine(ind + label + " (" + structName + ") = (" + string.Join(", ", parts) + ")");
                    }
                    else
                    {
                        sb.AppendLine(ind + label + " (" + structName + ") {");
                        Props(p, p + size, ind + "    ");
                        sb.AppendLine(ind + "}");
                    }
                    break;
                case "ArrayProperty":
                    {
                        int n = I32(img, p);
                        sb.AppendLine(ind + label + " [" + n + "] (" + size + " bytes)");
                        // struct arrays are tagged: try to parse each element as a property list
                        int q = p + 4, elemBytes = n > 0 ? (size - 4) / n : 0;
                        if (n > 0 && n < 256 && LooksTagged(q))
                            for (int k = 0; k < n && q < p + size; k++) { sb.AppendLine(ind + "  [" + k + "] {"); q = Props(q, p + size, ind + "      "); sb.AppendLine(ind + "  }"); }
                        else if (n > 0 && elemBytes == 4 && n < 64)
                        {
                            var parts = new List<string>();
                            for (int k = 0; k < n; k++) { int v = I32(img, q + k * 4); float fv = F32(img, q + k * 4); parts.Add(Math.Abs(fv) > 1e-6 && Math.Abs(fv) < 1e7 ? fv.ToString("0.####") : RefName(v)); }
                            sb.AppendLine(ind + "    " + string.Join(", ", parts));
                        }
                    }
                    break;
                default: sb.AppendLine(ind + label + " : " + type + " (" + size + " bytes)"); break;
            }
            p = vs + size;
        }
        return p;
    }

    static bool LooksTagged(int q)
    {
        int a = I32(img, q), b = I32(img, q + 8);
        return a >= 0 && a < names.Length && b >= 0 && b < names.Length && N(b).EndsWith("Property");
    }

    public static void Run(string path, string which)
    {
        Load(path);
        sb = new StringBuilder();
        var want = new HashSet<string>(which.Split(','));
        foreach (var e in exps)
        {
            if (!want.Contains(N(e.Name)) && !want.Contains(e.FullName)) continue;
            sb.AppendLine("==== " + e.FullName + " : " + ClassOf(e) + "  (" + e.Size + " bytes)");
            int p = e.Offset + 4;                  // skip NetIndex
            for (int s = 0; s <= 256 && s + 24 < e.Size; s += 4)   // state frames etc. come first on some objects
                if (LooksTagged(e.Offset + s)) { p = e.Offset + s; break; }
            Props(p, e.Offset + e.Size, "  ");
        }
        Console.Write(sb.ToString());
    }

    public static void List(string path, string contains)
    {
        Load(path);
        foreach (var e in exps)
            if (e.FullName.IndexOf(contains, StringComparison.OrdinalIgnoreCase) >= 0)
                Console.WriteLine("{0,-60} {1,-28} {2,6}", e.FullName, ClassOf(e), e.Size);
    }
}
