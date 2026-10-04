// GlbInspect.cs - per-primitive / per-joint vertex bounds of a GLB (diagnostics for car exports)
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Web.Script.Serialization;

public static class GlbInspect
{
    static int I(object o) { return Convert.ToInt32(o); }
    static Dictionary<string, object> D(object o) { return (Dictionary<string, object>)o; }
    static object[] A(object o) { return (object[])o; }

    public static void Run(string path)
    {
        byte[] b = File.ReadAllBytes(path);
        int jl = (int)BitConverter.ToUInt32(b, 12);
        var j = D(new JavaScriptSerializer { MaxJsonLength = int.MaxValue }.DeserializeObject(Encoding.UTF8.GetString(b, 20, jl)));
        int bin = 20 + jl + 8;
        var accs = A(j["accessors"]); var bvs = A(j["bufferViews"]); var mats = A(j["materials"]);
        var skin = D(A(j["skins"])[0]); var joints = A(skin["joints"]); var nodes = A(j["nodes"]);
        Func<int, int, int, int, double> rd = (acc, i, c, unused) => {
            var a = D(accs[acc]); var bv = D(bvs[I(a["bufferView"])]);
            int off = bin + (bv.ContainsKey("byteOffset") ? I(bv["byteOffset"]) : 0) + (a.ContainsKey("byteOffset") ? I(a["byteOffset"]) : 0);
            int ct = I(a["componentType"]);
            int sz = ct == 5126 ? 4 : ct == 5123 ? 2 : 1;
            int comps = (string)a["type"] == "VEC4" ? 4 : (string)a["type"] == "VEC3" ? 3 : 2;
            int stride = bv.ContainsKey("byteStride") ? I(bv["byteStride"]) : comps * sz;
            int p = off + i * stride + c * sz;
            return ct == 5126 ? BitConverter.ToSingle(b, p) : ct == 5123 ? BitConverter.ToUInt16(b, p) : ct == 5121 ? b[p] : 0;
        };
        foreach (var po in A(D(A(j["meshes"])[0])["primitives"]))
        {
            var p = D(po); var at = D(p["attributes"]);
            int pos = I(at["POSITION"]), jn = I(at["JOINTS_0"]), wt = I(at["WEIGHTS_0"]);
            int n = I(D(accs[pos])["count"]);
            Console.WriteLine("prim mat={0} verts={1}", D(mats[I(p["material"])])["name"], n);
            var bounds = new SortedDictionary<string, double[]>();
            for (int i = 0; i < n; i++)
            {
                int best = 0; double bw = -1;
                for (int c = 0; c < 4; c++) { double w = rd(wt, i, c, 0); if (w > bw) { bw = w; best = (int)rd(jn, i, c, 0); } }
                string name = (string)D(nodes[I(joints[best])])["name"];
                double[] bb; if (!bounds.TryGetValue(name, out bb)) bounds[name] = bb = new double[] { 1e9, 1e9, 1e9, -1e9, -1e9, -1e9, 0 };
                for (int c = 0; c < 3; c++) { double v = rd(pos, i, c, 0); bb[c] = Math.Min(bb[c], v); bb[c + 3] = Math.Max(bb[c + 3], v); }
                bb[6]++;
            }
            foreach (var kv in bounds)
                Console.WriteLine("   {0,-16} n{1,5}  x[{2:0.000},{3:0.000}] y[{4:0.000},{5:0.000}] z[{6:0.000},{7:0.000}]",
                    kv.Key, kv.Value[6], kv.Value[0], kv.Value[3], kv.Value[1], kv.Value[4], kv.Value[2], kv.Value[5]);
        }
        // inverse bind matrices translation per joint
        int ibm = I(skin["inverseBindMatrices"]);
        for (int k = 0; k < joints.Length; k++)
        {
            var a = D(accs[ibm]); var bv = D(bvs[I(a["bufferView"])]);
            int off = bin + (bv.ContainsKey("byteOffset") ? I(bv["byteOffset"]) : 0) + (a.ContainsKey("byteOffset") ? I(a["byteOffset"]) : 0) + k * 64;
            var m = new float[16]; for (int q = 0; q < 16; q++) m[q] = BitConverter.ToSingle(b, off + q * 4);
            Console.WriteLine("ibm {0,-16} diag {1:0.000} {2:0.000} {3:0.000}  t {4:0.000} {5:0.000} {6:0.000}",
                D(nodes[I(joints[k])])["name"], m[0], m[5], m[10], m[12], m[13], m[14]);
        }
    }
}
