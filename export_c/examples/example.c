/*
 * example.c - load a SARPBC car and its textures, print what was loaded.
 *
 *   gcc -std=c99 -O2 -I../include example.c -o example
 *   ./example ../cars/octane octane
 *
 * MSVC:  cl /I..\include example.c
 */
#include <stdio.h>
#include <stdlib.h>

#define SARPBC_MODEL_IMPLEMENTATION
#include "sarpbc_model.h"

static const char *mat_name(uint32_t m)
{
    return m == SARM_MAT_BODY ? "body" : m == SARM_MAT_GLASS ? "glass" : m == SARM_MAT_TIRE ? "tire" : "?";
}

static void report_texture(const char *dir, const char *file)
{
    char path[512];
    int w = 0, h = 0;
    unsigned char *px;

    snprintf(path, sizeof(path), "%s/%s", dir, file);
    px = sarm_load_tga(path, &w, &h);
    if (!px) { printf("  %-18s FAILED to load\n", file); return; }
    /* first pixel, rows are top-to-bottom */
    printf("  %-18s %4d x %-4d  first pixel rgba(%d,%d,%d,%d)\n", file, w, h, px[0], px[1], px[2], px[3]);
    free(px);
}

int main(int argc, char **argv)
{
    const char *dir  = argc > 1 ? argv[1] : "../cars/octane";
    const char *name = argc > 2 ? argv[2] : "octane";
    char path[512];
    SarmModel model;
    uint32_t i;
    int err;

    snprintf(path, sizeof(path), "%s/%s.sarm", dir, name);
    err = sarm_load(path, &model);
    if (err != SARM_OK) {
        fprintf(stderr, "%s: %s\n", path, sarm_error_string(err));
        return 1;
    }

    printf("%s\n", path);
    printf("  vertices  %u\n", model.header.vertex_count);
    printf("  triangles %u\n", model.header.index_count / 3);
    printf("  size      %.2f x %.2f x %.2f m\n",
           model.header.bounds_max[0] - model.header.bounds_min[0],
           model.header.bounds_max[1] - model.header.bounds_min[1],
           model.header.bounds_max[2] - model.header.bounds_min[2]);

    for (i = 0; i < model.header.submesh_count; i++)
        printf("  submesh %u: %-5s %6u triangles\n", i, mat_name(model.submeshes[i].material),
               model.submeshes[i].index_count / 3);

    for (i = 0; i < 4; i++)
        printf("  wheel %u at (%.3f, %.3f, %.3f)\n", i,
               model.header.wheel_pos[i][0], model.header.wheel_pos[i][1], model.header.wheel_pos[i][2]);

    report_texture(dir, "body_blue.tga");
    report_texture(dir, "body_orange.tga");
    report_texture(dir, "body_normal.tga");
    report_texture(dir, "tire_diffuse.tga");
    report_texture(dir, "tire_normal.tga");

    sarm_free(&model);
    return 0;
}
