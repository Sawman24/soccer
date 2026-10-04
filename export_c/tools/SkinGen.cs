// SkinGen.cs - Procedural Car Skin Generator for SARPBC
// Generates custom skins from scratch using UV mask channels (R=C1, G=C2, B=C3)
// and bakes them into export_c/cars/<name>/body_custom.tga (+ preview PNGs).

using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Runtime.InteropServices;

class SkinGen
{
    const string Unc = @"e:\Sarpbc\exported_assets\uncooked_ps3";
    const string OutRoot = @"e:\Sarpbc\export_c\cars";

    public class CarSkin
    {
        public string CarName;
        public string MaskPath;
        public string SkinName;
        public Func<int, int, int, int, double[][]> PaletteFunc;
    }

    static double[] V(double r, double g, double b) { return new[] { r, g, b }; }

    static byte LinToSrgb(double c)
    {
        c = Math.Max(0, Math.Min(1, c));
        double s = c <= 0.0031308 ? c * 12.92 : 1.055 * Math.Pow(c, 1.0 / 2.4) - 0.055;
        return (byte)Math.Round(s * 255.0);
    }

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
            return px;
        }
    }

    static void WriteTga(string path, byte[] bgra, int w, int h)
    {
        using (var f = new BinaryWriter(File.Create(path)))
        {
            f.Write((byte)0); f.Write((byte)0); f.Write((byte)2);
            f.Write(new byte[5]);
            f.Write((ushort)0); f.Write((ushort)0);
            f.Write((ushort)w); f.Write((ushort)h);
            f.Write((byte)32); f.Write((byte)0x28); // 32bpp, top-left origin
            f.Write(bgra);
        }
    }

    static void SavePng(string path, byte[] bgra, int w, int h)
    {
        using (var bmp = new Bitmap(w, h, PixelFormat.Format32bppArgb))
        {
            var data = bmp.LockBits(new Rectangle(0, 0, w, h), ImageLockMode.WriteOnly, PixelFormat.Format32bppArgb);
            for (int y = 0; y < h; y++) Marshal.Copy(bgra, y * w * 4, data.Scan0 + y * data.Stride, w * 4);
            bmp.UnlockBits(data);
            bmp.Save(path, ImageFormat.Png);
        }
    }

    static CarSkin[] GetSkins()
    {
        return new[]
        {
            // 1. Octane: "Synthwave" (Electric Cyan, Woven Carbon, Hot Neon Magenta)
            new CarSkin {
                CarName = "octane",
                SkinName = "Synthwave",
                MaskPath = Unc + @"\Vehicle_RaceCar\Materials\RaceCar01_Masks_02.png",
                PaletteFunc = (x, y, w, h) => {
                    // Carbon weave texture on C2
                    bool weave = (((x / 2) ^ (y / 2)) & 1) != 0;
                    double carb = weave ? 0.045 : 0.025;
                    return new[] {
                        V(0.02, 0.65, 0.92),        // C1: Electric Cyan
                        V(carb, carb, carb * 1.1),  // C2: Carbon Weave
                        V(0.95, 0.05, 0.50)         // C3: Neon Magenta / Hot Pink
                    };
                }
            },

            // 2. Backfire: "Hellfire" (Obsidian Black, Deep Crimson, Fiery Amber Flame Gradient)
            new CarSkin {
                CarName = "backfire",
                SkinName = "Hellfire",
                MaskPath = Unc + @"\Vehicle_Model-T\Materials\Masks_v2.png",
                PaletteFunc = (x, y, w, h) => {
                    // Fiery gradient across vertical UV
                    double flameT = Math.Sin((x + y) * 0.08) * 0.5 + 0.5;
                    double flameR = 1.0;
                    double flameG = 0.25 + 0.55 * flameT;
                    double flameB = 0.01;
                    return new[] {
                        V(0.018, 0.018, 0.022),       // C1: Gloss Obsidian Black
                        V(0.68, 0.04, 0.02),          // C2: Ember Crimson
                        V(flameR, flameG, flameB)     // C3: Blazing Flame Gradient
                    };
                }
            },

            // 3. Scarab: "Gold Rush" (Radiant Polished Gold, Velvet Onyx, Pearl Ivory)
            new CarSkin {
                CarName = "scarab",
                SkinName = "Gold Rush",
                MaskPath = Unc + @"\Vehicle_SteamPunkCar01\Materials\SteamPunkCar_Masks_v2.png",
                PaletteFunc = (x, y, w, h) => {
                    // Subtle metallic lustre variation
                    double shimmer = 0.95 + 0.08 * Math.Sin(x * 0.15 + y * 0.1);
                    return new[] {
                        V(0.92, 0.90, 0.86),                                // C1: Pearl Ivory trim
                        V(0.02, 0.02, 0.025),                              // C2: Royal Velvet Onyx
                        V(0.98 * shimmer, 0.76 * shimmer, 0.12 * shimmer)  // C3: Radiant Polished Gold
                    };
                }
            },

            // 4. Aftershock: "Stealth Jet" (Matte Radar Navy, Tactical Grey, Neon Flight Amber)
            new CarSkin {
                CarName = "aftershock",
                SkinName = "Stealth Jet",
                MaskPath = Unc + @"\Vehicle_SpaceCar\Materials\SpaceCar_Masks_v2.png",
                PaletteFunc = (x, y, w, h) => {
                    // Micro-hex / tactical panel effect on C1
                    bool panel = ((x / 6 + y / 6) & 1) != 0;
                    double tone = panel ? 1.05 : 0.95;
                    return new[] {
                        V(0.028 * tone, 0.038 * tone, 0.065 * tone), // C1: Radar Dark Navy
                        V(0.14, 0.15, 0.18),                         // C2: Titanium Grey
                        V(1.0, 0.38, 0.02)                           // C3: Aviation Neon Amber
                    };
                }
            },

            // 5. Renegade: "Desert Camo" (Procedural Multi-tone Digital Camo, Matte Black, Military Stencil)
            new CarSkin {
                CarName = "renegade",
                SkinName = "Desert Camo",
                MaskPath = Unc + @"\Vehicle_MonsterTruck\Materials\Masks.png",
                PaletteFunc = (x, y, w, h) => {
                    // Digital camo block hashing
                    int bx = x / 14, by = y / 14;
                    int hash = Math.Abs((bx * 73 + by * 179) ^ (bx * 311 + by * 47)) % 100;
                    double[] camo;
                    if (hash < 35)      camo = V(0.68, 0.54, 0.36); // Coyote Tan
                    else if (hash < 65) camo = V(0.28, 0.32, 0.20); // Olive Drab
                    else if (hash < 85) camo = V(0.42, 0.32, 0.24); // Earth Brown
                    else                camo = V(0.12, 0.13, 0.11); // Dark Shadow
                    return new[] {
                        V(0.04, 0.04, 0.045), // C1: Tactical Black Trim
                        camo,                 // C2: Digital Camouflage Body
                        V(0.86, 0.86, 0.82)  // C3: Stencil White Decals
                    };
                }
            },

            // 6. Zippy: "Cyberpunk" (Midnight Violet, Laser Turquoise, High-tech White)
            new CarSkin {
                CarName = "zippy",
                SkinName = "Cyberpunk",
                MaskPath = Unc + @"\Vehicle_Convertible\Materials\Convertible_Masks.png",
                PaletteFunc = (x, y, w, h) => {
                    return new[] {
                        V(0.16, 0.02, 0.32),  // C1: Midnight Violet Body
                        V(0.03, 0.88, 0.75),  // C2: Laser Turquoise / Cyan Stripes
                        V(0.92, 0.95, 1.0)    // C3: High-tech White Accents
                    };
                }
            },

            // 7. Marauder: "Urban Hazard" (Battleship Slate Grey, Gunmetal, Hazard Stripes)
            new CarSkin {
                CarName = "marauder",
                SkinName = "Urban Hazard",
                MaskPath = Unc + @"\Vehicle_Armored\Materials\Armored_Masks.png",
                PaletteFunc = (x, y, w, h) => {
                    // Bold 45-degree yellow & black hazard warning stripes on C3
                    bool hazardStripe = ((x + y) / 14) % 2 == 0;
                    double[] hazardCol = hazardStripe ? V(0.96, 0.80, 0.04) : V(0.02, 0.02, 0.02);
                    return new[] {
                        V(0.04, 0.045, 0.05),// C1: Matte Gunmetal Trim
                        V(0.18, 0.20, 0.24), // C2: Battleship Slate Grey Body
                        hazardCol            // C3: Industrial Hazard Stripes
                    };
                }
            }
        };
    }

    public static void BakeSkin(CarSkin cs)
    {
        Console.WriteLine(string.Format("Baking skin '{0}' for car '{1}'...", cs.SkinName, cs.CarName));
        int w, h;
        byte[] mask = LoadBgra(cs.MaskPath, out w, out h);
        byte[] o = new byte[mask.Length];
        double[] baseChassis = { 0.035, 0.038, 0.045 };

        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                int i = y * w + x;
                double r = mask[i * 4 + 2] / 255.0; // C1
                double g = mask[i * 4 + 1] / 255.0; // C2
                double b = mask[i * 4 + 0] / 255.0; // C3
                double sum = r + g + b;
                double painted = Math.Min(1.0, sum);
                double inv = 1.0 / Math.Max(sum, 0.001);

                double[][] pal = cs.PaletteFunc(x, y, w, h);
                double[] lin = new double[3];
                for (int k = 0; k < 3; k++)
                {
                    double livery = (r * pal[0][k] + g * pal[1][k] + b * pal[2][k]) * inv;
                    lin[k] = baseChassis[k] + (livery - baseChassis[k]) * painted;
                }

                o[i * 4 + 2] = LinToSrgb(lin[0]); // R
                o[i * 4 + 1] = LinToSrgb(lin[1]); // G
                o[i * 4 + 0] = LinToSrgb(lin[2]); // B
                o[i * 4 + 3] = 255;              // A
            }
        }

        string carDir = Path.Combine(OutRoot, cs.CarName);
        Directory.CreateDirectory(carDir);
        string tgaPath = Path.Combine(carDir, "body_custom.tga");
        string pngPreview = Path.Combine(carDir, "skin_preview.png");
        WriteTga(tgaPath, o, w, h);
        SavePng(pngPreview, o, w, h);
        Console.WriteLine(string.Format("  -> Saved: {0}", tgaPath));
    }

    public static void Main(string[] args)
    {
        Console.WriteLine("=================================================");
        Console.WriteLine(" SARPBC Skin Generator");
        Console.WriteLine("=================================================");
        var skins = GetSkins();
        string filter = args.Length > 0 ? args[0].ToLowerInvariant() : "all";

        int baked = 0;
        foreach (var cs in skins)
        {
            if (filter != "all" && cs.CarName.ToLowerInvariant() != filter) continue;
            BakeSkin(cs);
            baked++;
        }
        Console.WriteLine(string.Format("Finished! Baked {0} custom skin(s).", baked));
    }
}
