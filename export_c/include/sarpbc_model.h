/*
 * sarpbc_model.h - single-header loader for the SARPBC car exports.
 * Plain C99, no dependencies beyond the C standard library.
 *
 * In exactly ONE .c file:
 *     #define SARPBC_MODEL_IMPLEMENTATION
 *     #include "sarpbc_model.h"
 * Everywhere else just #include "sarpbc_model.h".
 *
 * ---------------------------------------------------------------------------
 * .sarm file layout (little-endian, every field 4 bytes, no padding):
 *
 *   SarmHeader                      96 bytes
 *   SarmSubmesh[submesh_count]      16 bytes each
 *   SarmVertex [vertex_count]       48 bytes each
 *   uint32_t   [index_count]        triangle list, 3 indices per triangle
 *
 * Coordinate system: right-handed, +X forward, +Y up, +Z to the car's right,
 *   units = meters (a car is ~2 m long). Multiply by 100 for Unreal-style cm.
 * Winding: counter-clockwise front faces (OpenGL default glFrontFace(GL_CCW)).
 * UVs: (0,0) is the TOP-LEFT of the texture. sarm_load_tga() returns rows
 *   top-to-bottom, so you can pass the pixels straight to glTexImage2D and use
 *   the UVs unchanged (both "flips" cancel out).
 * Normal maps in the export are OpenGL convention (+Y / green points up).
 * Body/tire diffuse TGAs are sRGB; normal maps are linear.
 * ---------------------------------------------------------------------------
 */
#ifndef SARPBC_MODEL_H
#define SARPBC_MODEL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SARM_VERSION 1

/* SarmSubmesh.material */
enum {
    SARM_MAT_BODY  = 0,  /* body_blue.tga / body_orange.tga + body_normal.tga */
    SARM_MAT_GLASS = 1,  /* untextured; suggested colour (0.06, 0.09, 0.16), alpha 0.6 */
    SARM_MAT_TIRE  = 2   /* tire_diffuse.tga + tire_normal.tga; alpha < 0.33 = cut out */
};

/* Index into SarmHeader.wheel_pos */
enum {
    SARM_WHEEL_FRONT_LEFT  = 0,
    SARM_WHEEL_FRONT_RIGHT = 1,
    SARM_WHEEL_REAR_LEFT   = 2,
    SARM_WHEEL_REAR_RIGHT  = 3
};

typedef struct SarmHeader {
    char     magic[4];        /* "SARM" */
    uint32_t version;         /* SARM_VERSION */
    uint32_t vertex_count;
    uint32_t index_count;
    uint32_t submesh_count;
    uint32_t reserved;
    float    bounds_min[3];
    float    bounds_max[3];
    float    wheel_pos[4][3]; /* wheel hub centres from the game's skeleton */
} SarmHeader;

typedef struct SarmSubmesh {
    uint32_t first_index;     /* offset into the index array */
    uint32_t index_count;
    uint32_t material;        /* SARM_MAT_* */
    uint32_t reserved;
} SarmSubmesh;

typedef struct SarmVertex {
    float pos[3];
    float normal[3];
    float uv[2];
    float tangent[4];         /* xyz = tangent, w = bitangent sign */
} SarmVertex;

typedef struct SarmModel {
    SarmHeader   header;
    SarmSubmesh *submeshes;
    SarmVertex  *vertices;
    uint32_t    *indices;
} SarmModel;

/* Error codes returned by the loaders */
enum {
    SARM_OK          =  0,
    SARM_ERR_OPEN    = -1,
    SARM_ERR_READ    = -2,
    SARM_ERR_FORMAT  = -3,
    SARM_ERR_MEMORY  = -4
};

/* Load a .sarm file. On failure the model is zeroed and nothing needs freeing. */
int  sarm_load(const char *path, SarmModel *out);
void sarm_free(SarmModel *model);

/* Load an uncompressed or RLE TGA (24/32-bit). Returns malloc'd RGBA8 pixels,
 * rows top-to-bottom, or NULL on failure. Free with free(). */
unsigned char *sarm_load_tga(const char *path, int *width, int *height);

const char *sarm_error_string(int err);

#ifdef __cplusplus
}
#endif
#endif /* SARPBC_MODEL_H */

/* ======================================================================== */
#ifdef SARPBC_MODEL_IMPLEMENTATION

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef char sarm_check_header [(sizeof(SarmHeader)  == 96) ? 1 : -1];
typedef char sarm_check_submesh[(sizeof(SarmSubmesh) == 16) ? 1 : -1];
typedef char sarm_check_vertex [(sizeof(SarmVertex)  == 48) ? 1 : -1];

const char *sarm_error_string(int err)
{
    switch (err) {
    case SARM_OK:         return "ok";
    case SARM_ERR_OPEN:   return "could not open file";
    case SARM_ERR_READ:   return "unexpected end of file";
    case SARM_ERR_FORMAT: return "not a valid SARM file";
    case SARM_ERR_MEMORY: return "out of memory";
    default:              return "unknown error";
    }
}

void sarm_free(SarmModel *m)
{
    if (!m) return;
    free(m->submeshes);
    free(m->vertices);
    free(m->indices);
    memset(m, 0, sizeof(*m));
}

int sarm_load(const char *path, SarmModel *m)
{
    FILE *f;
    int err = SARM_OK;
    uint32_t i;

    memset(m, 0, sizeof(*m));
    f = fopen(path, "rb");
    if (!f) return SARM_ERR_OPEN;

    if (fread(&m->header, sizeof(SarmHeader), 1, f) != 1) { err = SARM_ERR_READ; goto fail; }
    if (memcmp(m->header.magic, "SARM", 4) != 0 || m->header.version != SARM_VERSION ||
        m->header.vertex_count == 0 || m->header.index_count % 3 != 0) {
        err = SARM_ERR_FORMAT; goto fail;
    }

    m->submeshes = (SarmSubmesh *)malloc(sizeof(SarmSubmesh) * m->header.submesh_count);
    m->vertices  = (SarmVertex  *)malloc(sizeof(SarmVertex)  * m->header.vertex_count);
    m->indices   = (uint32_t    *)malloc(sizeof(uint32_t)    * m->header.index_count);
    if (!m->submeshes || !m->vertices || !m->indices) { err = SARM_ERR_MEMORY; goto fail; }

    if (fread(m->submeshes, sizeof(SarmSubmesh), m->header.submesh_count, f) != m->header.submesh_count ||
        fread(m->vertices,  sizeof(SarmVertex),  m->header.vertex_count,  f) != m->header.vertex_count  ||
        fread(m->indices,   sizeof(uint32_t),    m->header.index_count,   f) != m->header.index_count) {
        err = SARM_ERR_READ; goto fail;
    }

    /* Validate so a corrupt file can't index out of bounds later */
    for (i = 0; i < m->header.submesh_count; i++) {
        const SarmSubmesh *s = &m->submeshes[i];
        if (s->first_index > m->header.index_count ||
            s->index_count > m->header.index_count - s->first_index) { err = SARM_ERR_FORMAT; goto fail; }
    }
    for (i = 0; i < m->header.index_count; i++)
        if (m->indices[i] >= m->header.vertex_count) { err = SARM_ERR_FORMAT; goto fail; }

    fclose(f);
    return SARM_OK;

fail:
    fclose(f);
    sarm_free(m);
    return err;
}

unsigned char *sarm_load_tga(const char *path, int *width, int *height)
{
    unsigned char hdr[18];
    unsigned char *pixels = NULL;
    FILE *f;
    int w, h, bpp, type, top_left, y;
    size_t n, count, i;

    f = fopen(path, "rb");
    if (!f) return NULL;
    if (fread(hdr, 1, 18, f) != 18) goto fail;

    type = hdr[2];
    w = hdr[12] | (hdr[13] << 8);
    h = hdr[14] | (hdr[15] << 8);
    bpp = hdr[16] / 8;
    top_left = (hdr[17] & 0x20) != 0;
    if ((type != 2 && type != 10) || hdr[1] != 0 || (bpp != 3 && bpp != 4) || w <= 0 || h <= 0) goto fail;
    if (hdr[0] && fseek(f, hdr[0], SEEK_CUR) != 0) goto fail;   /* skip image ID */

    count = (size_t)w * (size_t)h;
    pixels = (unsigned char *)malloc(count * 4);
    if (!pixels) goto fail;

    if (type == 2) {                         /* uncompressed */
        for (i = 0; i < count; i++) {
            unsigned char p[4] = {0, 0, 0, 255};
            if (fread(p, 1, (size_t)bpp, f) != (size_t)bpp) goto fail;
            pixels[i*4+0] = p[2]; pixels[i*4+1] = p[1]; pixels[i*4+2] = p[0];
            pixels[i*4+3] = bpp == 4 ? p[3] : 255;
        }
    } else {                                 /* RLE */
        i = 0;
        while (i < count) {
            int c = fgetc(f);
            unsigned char p[4] = {0, 0, 0, 255};
            if (c == EOF) goto fail;
            n = (size_t)(c & 0x7F) + 1;
            if (n > count - i) n = count - i;
            if (c & 0x80) {
                if (fread(p, 1, (size_t)bpp, f) != (size_t)bpp) goto fail;
                for (; n > 0; n--, i++) {
                    pixels[i*4+0] = p[2]; pixels[i*4+1] = p[1]; pixels[i*4+2] = p[0];
                    pixels[i*4+3] = bpp == 4 ? p[3] : 255;
                }
            } else {
                for (; n > 0; n--, i++) {
                    if (fread(p, 1, (size_t)bpp, f) != (size_t)bpp) goto fail;
                    pixels[i*4+0] = p[2]; pixels[i*4+1] = p[1]; pixels[i*4+2] = p[0];
                    pixels[i*4+3] = bpp == 4 ? p[3] : 255;
                }
            }
        }
    }

    /* Normalise to top-to-bottom row order */
    if (!top_left) {
        size_t row = (size_t)w * 4;
        unsigned char *tmp = (unsigned char *)malloc(row);
        if (!tmp) goto fail;
        for (y = 0; y < h / 2; y++) {
            unsigned char *a = pixels + (size_t)y * row;
            unsigned char *b = pixels + (size_t)(h - 1 - y) * row;
            memcpy(tmp, a, row); memcpy(a, b, row); memcpy(b, tmp, row);
        }
        free(tmp);
    }

    fclose(f);
    *width = w;
    *height = h;
    return pixels;

fail:
    free(pixels);
    fclose(f);
    return NULL;
}

#endif /* SARPBC_MODEL_IMPLEMENTATION */
