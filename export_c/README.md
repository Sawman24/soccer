# SARPBC cars for C

The 7 *Supersonic Acrobatic Rocket-Powered Battle-Cars* vehicles, converted from the PS3 release into a format that plain C can load with `fread`.

```
export_c/
  include/sarpbc_model.h   single-header loader (C99, stdlib only)
  examples/example.c       loads a car + textures and prints stats
  tools/ExportForC.cs      the exporter that produced cars/ (re-runnable)
  cars/<name>/
    <name>.sarm            binary mesh
    body_blue.tga          baked team paint, Blue team
    body_orange.tga        baked team paint, Orange (red) team
    body_normal.tga        body normal map (OpenGL convention)
    tire_diffuse.tga       tire colour, alpha = cut-out mask
    tire_normal.tga        tire normal map (OpenGL convention)
    <name>.obj / .mtl      same mesh, for Blender etc.
```

| Folder | In-game name | Triangles |
|---|---|---|
| `backfire`   | Backfire   | 8,779 |
| `octane`     | Octane     | 6,548 |
| `scarab`     | Scarab     | 8,301 |
| `aftershock` | Aftershock | 7,246 |
| `renegade`   | Renegade   | 8,135 |
| `zippy`      | Zippy      | 6,244 |
| `marauder`   | Marauder   | 5,602 |

## Using it

```c
#define SARPBC_MODEL_IMPLEMENTATION
#include "sarpbc_model.h"

SarmModel car;
if (sarm_load("cars/octane/octane.sarm", &car) != SARM_OK) { /* error */ }

int w, h;
unsigned char *rgba = sarm_load_tga("cars/octane/body_blue.tga", &w, &h);

for (uint32_t i = 0; i < car.header.submesh_count; i++) {
    SarmSubmesh *s = &car.submeshes[i];
    /* bind texture for s->material, then draw
       car.indices[s->first_index .. s->first_index + s->index_count) */
}

free(rgba);
sarm_free(&car);
```

Build the example: `gcc -std=c99 -I include examples/example.c -o example` then `./example cars/octane octane`.

### OpenGL upload (if you use it)
`SarmVertex` is 48 bytes: position at offset 0, normal at 12, uv at 24, tangent at 36.

```c
glBufferData(GL_ARRAY_BUFFER, car.header.vertex_count * sizeof(SarmVertex), car.vertices, GL_STATIC_DRAW);
glBufferData(GL_ELEMENT_ARRAY_BUFFER, car.header.index_count * 4, car.indices, GL_STATIC_DRAW);
glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(SarmVertex), (void*)0);   /* pos     */
glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(SarmVertex), (void*)12);  /* normal  */
glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(SarmVertex), (void*)24);  /* uv      */
glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(SarmVertex), (void*)36);  /* tangent */
glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8_ALPHA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba); /* diffuse */
/* draw each submesh: glDrawElements(GL_TRIANGLES, s->index_count, GL_UNSIGNED_INT, (void*)(s->first_index * 4)); */
```

## Conventions

- **Axes:** right-handed, **+X forward, +Y up, +Z right**, meters. Cars are about 2 m long.
- **Winding:** counter-clockwise front faces.
- **UVs:** (0,0) = top-left of the image. `sarm_load_tga` returns rows top-to-bottom, so give the pixels to your renderer as-is and use the UVs unchanged.
- **Colour space:** diffuse TGAs are sRGB; normal maps are linear, with green = +Y (OpenGL). For DirectX-style, invert green.
- **Materials:** submesh `material` is `SARM_MAT_BODY`, `SARM_MAT_GLASS` (untextured, suggested colour `0.06, 0.09, 0.16`, alpha `0.6`) or `SARM_MAT_TIRE` (discard pixels with alpha < 0.33).
- **Wheels:** `header.wheel_pos[4]` holds the hub centres from the game skeleton, in front-left, front-right, rear-left, rear-right order. The wheels are still part of the tire submesh, not separate objects.

## Known limitations

- **Octane's tire colour texture** is corrupt in the PS3 data, so `octane/tire_diffuse.tga` is a 4×4 solid rubber colour. Its tire normal map is the real one.
- **Ambient-occlusion maps** (`*_AO`) are also corrupt on PS3 and aren't baked in, so the paint looks a little flat compared with the game.
- **Paint mask mapping:** the game files give each car's three colours (`C1`/`C2`/`C3`) but not which mask channel uses which. Red → C1, green → C2, blue → C3 is inferred, and it's the line to change in `BakeLivery()` if a car looks swapped.

## Re-running the exporter

```
cd export_c/tools
C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe /r:C:\Windows\Microsoft.NET\Framework64\v4.0.30319\System.Web.Extensions.dll /r:System.Drawing.dll ExportForC.cs
ExportForC.exe
```

These are Psyonix's copyrighted assets: fine for personal projects and learning, not for redistribution.
