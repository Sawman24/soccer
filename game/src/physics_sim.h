#ifndef PHYSICS_SIM_H
#define PHYSICS_SIM_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define RAYMATH_STATIC_INLINE
#include "raymath.h"

#include "sarpbc_model.h"

#ifndef TraceLog
  #define LOG_INFO  0
  #define LOG_ERROR 1
  #define TraceLog(lvl, fmt, ...) do { printf("[LOG] " fmt "\n", ##__VA_ARGS__); fflush(stdout); } while(0)
#endif

#ifndef V3
  #define V3(x, y, z) ((Vector3){ (x), (y), (z) })
#endif

/* ------------------------------------------------------------------------ */
/* Tunables & Constants                                                       */
/* ------------------------------------------------------------------------ */
#define PHYS_HZ            120.0f

/* PS3-TAGame.ini: DefaultGravityZ -750 * RBPhysicsGravityScaling 1.75 */
#define GRAVITY            13.125f

/* Arena dimensions measured from the game's collision meshes */
#define ARENA_W            102.4f   /* half width  (X)                */
#define ARENA_L            144.6f   /* half length (Z) = goal line    */
#define ARENA_H            61.4f    /* top of the glass dome          */
#define RAMP_C             12.0f    /* fallback: 45-degree ramp size  */
#define GOAL_HW            20.5f    /* goal half width                */
#define GOAL_H             15.4f    /* goal height                    */
#define GOAL_D             19.0f    /* goal depth                     */
#define POST_R             0.6f
#define GRID_CELL          4.0f     /* collision grid cell size (m)   */

/* Car Dynamics */
#define CAR_IMPULSE_MASS   40.0f
#define JUMP_IMPULSE       (20000.0f / CAR_IMPULSE_MASS / 100.0f) /* TAComponents JumpImpulse  */
#define JUMP_FORCE         (1500.0f / 100.0f)                     /* JumpForce Z (as accel)    */
#define JUMP_FORCE_TIME    0.2f                                   /* JumpDelayTime             */
#define DODGE_WINDOW       1.25f                                  /* RL value                  */
#define DODGE_DEADZONE     0.35f
#define FLIP_TIME          0.65f                                  /* DoubleJumpTorqueTime      */
#define FLIP_RATE          (2.0f * PI / FLIP_TIME)
#define BOOST_ACCEL        (GRAVITY * 1.53f)
#define BOOST_MAX          3.0f                                   /* MaxBoostAmount (seconds)  */
#define BOOST_START        1.0f                                   /* StartBoostAmount          */
#define BOOST_MIN_TIME     0.25f                                  /* MinBoostTime              */
#define CAR_MAX_SPEED      (6000.0f / 100.0f)                     /* BoostSpeedCap             */
#define AIR_PUSH           (100.0f / 100.0f)                      /* DrivePushForce            */
#define AIR_T_PITCH        900.0f
#define AIR_T_YAW          700.0f
#define AIR_T_ROLL         1700.0f
#define AIR_D_PITCH        250.0f
#define AIR_D_YAW          250.0f
#define AIR_D_ROLL         200.0f
#define AIR_RESPONSE       0.15f
#define CAR_SCALE          1.2f
#define STEER_SPEED        100.0f
#define HB_LAT_SLIP        0.1f
#define HB_LONG_SLIP       0.5f
#define LAND_NO_GRIP_TIME  0.3f
#define BRAKE_ACCEL        35.0f
#define COAST_ACCEL        9.0f
#define STICKY_ACCEL       5.0f
#define LAT_GRIP           18.0f
#define LAT_GRIP_SLIDE     (LAT_GRIP * HB_LAT_SLIP)
#define SUSP_UP            0.20f
#define SUSP_DOWN          0.15f
#define SUSP_K             70.0f
#define SUSP_C             11.0f
#define LAND_ALIGN         45.0f
#define LAND_DAMP          10.0f
#define SELF_RIGHT_TIME    0.35f
#define SELF_RIGHT_LIFT    12.5f

/* Ball */
#define BALL_R             1.62f
#define BALL_MASS_RATIO    6.0f
#define BALL_RESTITUTION   0.65f
#define BALL_GROUND_FRICTION 0.12f
#define BALL_GRAVITY_SCALE 0.8f
#define BALL_HIT_SCALE     1.0f
#define BALL_DRAG          0.03f
#define BALL_MAX_SPEED     100.0f

#define MATCH_TIME         300.0f
#define KICKOFF_Z          60.0f
#define PAD_COUNT          6
#define PAD_RADIUS         3.0f
#define PAD_RESPAWN        10.0f

static const char *CAR_NAMES[] = { "octane", "backfire", "scarab", "aftershock", "renegade", "zippy", "marauder" };
#define CAR_COUNT ((int)(sizeof(CAR_NAMES) / sizeof(CAR_NAMES[0])))

/* ------------------------------------------------------------------------ */
/* Structs                                                                    */
/* ------------------------------------------------------------------------ */
typedef struct Input {
    float throttle, steer, pitch, yaw, roll;   /* -1..1 */
    int   jump, jumpPressed, boost, slide;
} Input;

typedef struct Car {
    Vector3    pos, vel, angVel;
    Quaternion rot;
    Vector3    invInertia;        /* local, per unit mass */
    Vector3    boxMin, boxMax;    /* local hitbox (model bounds) */
    float      bodyBottom;        /* local y of the body box used vs. the arena */
    Vector3    wheelLocal[4];
    float      wheelRadius;
    int        wheelContact[4];
    int        wheelsOnGround;
    float      boost, boostMinTimer;
    int        boosting;
    int        hasJumped, hasFlipped;
    float      jumpTimer, airTime, flipTimer;
    Vector2    flipDir;
    int        bodyContact;       /* chassis touched the arena this tick */
    Vector3    bodyNormal;
    float      stuckTimer;
    float      wheelBase;         /* front-rear axle distance (m) */
    float      steerAngle;        /* current front wheel angle (degrees, + = left) */
    float      landTimer;         /* tyre grip ramps back in after landing */
    float      uprightTimer;      /* recovery-jump roll in progress */
    float      wheelSpin, wheelSpinRate;   /* visual wheel roll (rad, rad/s) */
    float      wheelOfs[4];       /* visual hub offset along local up (m, + = compressed) */
    int        demolished;
    float      demoTimer;
} Car;

typedef struct Ball {
    Vector3    pos, vel, angVel;
    Quaternion rot;
} Ball;

typedef struct APlane { Vector3 n; float d; } APlane;

typedef struct ATri { Vector3 a, b, c, n; float ymin, ymax; } ATri;

/* ------------------------------------------------------------------------ */
/* Math Helpers                                                               */
/* ------------------------------------------------------------------------ */
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float signf(float v) { return v < 0.0f ? -1.0f : 1.0f; }

static inline float curve(const float *xs, const float *ys, int n, float x)
{
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n - 1]) return ys[n - 1];
    for (int i = 0; i < n - 1; i++) {
        if (x >= xs[i] && x <= xs[i + 1]) {
            float t = (x - xs[i]) / (xs[i + 1] - xs[i]);
            return ys[i] + t * (ys[i + 1] - ys[i]);
        }
    }
    return ys[n - 1];
}

static inline Vector3 car_to_world(const Car *c, Vector3 v) { return Vector3RotateByQuaternion(v, c->rot); }
static inline Vector3 car_to_local(const Car *c, Vector3 v) { return Vector3RotateByQuaternion(v, QuaternionInvert(c->rot)); }
static inline Vector3 car_fwd(const Car *c)   { return car_to_world(c, V3(1, 0, 0)); }
static inline Vector3 car_up(const Car *c)    { return car_to_world(c, V3(0, 1, 0)); }
static inline Vector3 car_right(const Car *c) { return car_to_world(c, V3(0, 0, 1)); }

static inline Vector3 car_inv_inertia(const Car *c, Vector3 t)
{
    Vector3 loc = car_to_local(c, t);
    loc = Vector3Multiply(loc, c->invInertia);
    return car_to_world(c, loc);
}

static inline Vector3 car_point_vel(const Car *c, Vector3 worldPoint)
{
    return Vector3Add(c->vel, Vector3CrossProduct(c->angVel, Vector3Subtract(worldPoint, c->pos)));
}

static inline void car_apply_impulse(Car *c, Vector3 worldPoint, Vector3 J)
{
    c->vel = Vector3Add(c->vel, J);
    c->angVel = Vector3Add(c->angVel, car_inv_inertia(c, Vector3CrossProduct(Vector3Subtract(worldPoint, c->pos), J)));
}

static inline void get_boost_pads(Vector3 *pads)
{
    pads[0] = V3(-78, 0, -(ARENA_L - 24));
    pads[1] = V3( 78, 0, -(ARENA_L - 24));
    pads[2] = V3(-78, 0,  (ARENA_L - 24));
    pads[3] = V3( 78, 0,  (ARENA_L - 24));
    pads[4] = V3(-82, 0, 0);
    pads[5] = V3( 82, 0, 0);
}

/* ------------------------------------------------------------------------ */
/* Arena Collision & Raycasting                                               */
/* ------------------------------------------------------------------------ */
static ATri *g_tris = NULL;
static int   g_triCount = 0;
static int  *g_cellStart = NULL, *g_cellItems = NULL, *g_stamp = NULL, g_stampId = 0;
static int   g_gx = 0, g_gz = 0;
static float g_gx0 = 0, g_gz0 = 0;

static int arena_planes(Vector3 p, APlane *pl)
{
    const float S2 = 0.70710678f;
    int n = 0, s;
    pl[n++] = (APlane){ V3(0,  1, 0), 0.0f };
    pl[n++] = (APlane){ V3(0, -1, 0), -ARENA_H };
    pl[n++] = (APlane){ V3(-1, 0, 0), -ARENA_W };
    pl[n++] = (APlane){ V3( 1, 0, 0), -ARENA_W };
    pl[n++] = (APlane){ V3(-S2, S2, 0), -S2 * (ARENA_W - RAMP_C) };
    pl[n++] = (APlane){ V3( S2, S2, 0), -S2 * (ARENA_W - RAMP_C) };
    for (s = -1; s <= 1; s += 2) {
        int inMouth = fabsf(p.x) < GOAL_HW && p.y < GOAL_H && p.z * s > 0.0f;
        if (!inMouth) {
            pl[n++] = (APlane){ V3(0, 0, -(float)s), -ARENA_L };
            pl[n++] = (APlane){ V3(0, S2, -(float)s * S2), -S2 * (ARENA_L - RAMP_C) };
        } else if (p.z * s > ARENA_L) {
            pl[n++] = (APlane){ V3(0, 0, -(float)s), -(ARENA_L + GOAL_D) };
            pl[n++] = (APlane){ V3(-1, 0, 0), -GOAL_HW };
            pl[n++] = (APlane){ V3( 1, 0, 0), -GOAL_HW };
            pl[n++] = (APlane){ V3(0, -1, 0), -GOAL_H };
        }
    }
    return n;
}

static int arena_mesh_load(const char *path)
{
    FILE *f = fopen(path, "rb");
    char magic[4];
    unsigned int count = 0;
    int i, x, z, pass;
    float minx = 1e30f, maxx = -1e30f, minz = 1e30f, maxz = -1e30f;
    if (!f) return 0;
    if (fread(magic, 1, 4, f) != 4 || memcmp(magic, "SARC", 4) != 0 || fread(&count, 4, 1, f) != 1 || count == 0 || count > 1000000) {
        fclose(f); return 0;
    }
    g_tris = (ATri *)calloc(count, sizeof(ATri));
    for (i = 0; i < (int)count; i++) {
        float v[9];
        ATri *t = &g_tris[i];
        if (fread(v, sizeof(float), 9, f) != 9) { fclose(f); free(g_tris); g_tris = NULL; return 0; }
        t->a = V3(v[0], v[1], v[2]); t->b = V3(v[3], v[4], v[5]); t->c = V3(v[6], v[7], v[8]);
        t->n = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(t->b, t->a), Vector3Subtract(t->c, t->a)));
        t->ymin = fminf(t->a.y, fminf(t->b.y, t->c.y));
        t->ymax = fmaxf(t->a.y, fmaxf(t->b.y, t->c.y));
        minx = fminf(minx, fminf(t->a.x, fminf(t->b.x, t->c.x))); maxx = fmaxf(maxx, fmaxf(t->a.x, fmaxf(t->b.x, t->c.x)));
        minz = fminf(minz, fminf(t->a.z, fminf(t->b.z, t->c.z))); maxz = fmaxf(maxz, fmaxf(t->a.z, fmaxf(t->b.z, t->c.z)));
    }
    fclose(f);
    g_triCount = (int)count;

    g_gx0 = minx - 1.0f; g_gz0 = minz - 1.0f;
    g_gx = (int)((maxx - g_gx0) / GRID_CELL) + 2;
    g_gz = (int)((maxz - g_gz0) / GRID_CELL) + 2;
    g_cellStart = (int *)calloc((size_t)(g_gx * g_gz + 1), sizeof(int));
    g_stamp = (int *)calloc(count, sizeof(int));
    for (pass = 0; pass < 2; pass++) {
        int *fill = NULL;
        if (pass == 1) {
            for (i = 1; i <= g_gx * g_gz; i++) g_cellStart[i] += g_cellStart[i-1];
            g_cellItems = (int *)malloc(sizeof(int) * (size_t)g_cellStart[g_gx * g_gz]);
            fill = (int *)calloc((size_t)(g_gx * g_gz), sizeof(int));
        }
        for (i = 0; i < g_triCount; i++) {
            const ATri *t = &g_tris[i];
            int x0 = (int)((fminf(t->a.x, fminf(t->b.x, t->c.x)) - g_gx0) / GRID_CELL);
            int x1 = (int)((fmaxf(t->a.x, fmaxf(t->b.x, t->c.x)) - g_gx0) / GRID_CELL);
            int z0 = (int)((fminf(t->a.z, fminf(t->b.z, t->c.z)) - g_gz0) / GRID_CELL);
            int z1 = (int)((fmaxf(t->a.z, fmaxf(t->b.z, t->c.z)) - g_gz0) / GRID_CELL);
            for (z = z0; z <= z1; z++)
                for (x = x0; x <= x1; x++) {
                    int cell = z * g_gx + x;
                    if (pass == 0) g_cellStart[cell + 1]++;
                    else g_cellItems[g_cellStart[cell] + fill[cell]++] = i;
                }
        }
        free(fill);
    }
    TraceLog(LOG_INFO, "ARENA: %d collision triangles, grid %dx%d", g_triCount, g_gx, g_gz);
    return 1;
}

static int arena_query(Vector3 mn, Vector3 mx, int *out, int maxOut)
{
    int x0, x1, z0, z1, x, z, count = 0, stamp;
    if (!g_tris) return 0;
    x0 = clampf((int)((mn.x - g_gx0) / GRID_CELL), 0, g_gx - 1);
    x1 = clampf((int)((mx.x - g_gx0) / GRID_CELL), 0, g_gx - 1);
    z0 = clampf((int)((mn.z - g_gz0) / GRID_CELL), 0, g_gz - 1);
    z1 = clampf((int)((mx.z - g_gz0) / GRID_CELL), 0, g_gz - 1);
    stamp = ++g_stampId;
    for (z = z0; z <= z1; z++)
        for (x = x0; x <= x1; x++) {
            int cell = z * g_gx + x;
            int s = g_cellStart[cell], e = g_cellStart[cell + 1], k;
            for (k = s; k < e; k++) {
                int ti = g_cellItems[k];
                if (g_stamp[ti] == stamp) continue;
                g_stamp[ti] = stamp;
                if (g_tris[ti].ymax < mn.y || g_tris[ti].ymin > mx.y) continue;
                if (count < maxOut) out[count++] = ti;
            }
        }
    return count;
}

static Vector3 closest_on_tri(Vector3 p, Vector3 a, Vector3 b, Vector3 c, int *onFace)
{
    Vector3 ab = Vector3Subtract(b, a), ac = Vector3Subtract(c, a), ap = Vector3Subtract(p, a);
    float d1 = Vector3DotProduct(ab, ap), d2 = Vector3DotProduct(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) { if (onFace) *onFace = 0; return a; }

    Vector3 bp = Vector3Subtract(p, b);
    float d3 = Vector3DotProduct(ab, bp), d4 = Vector3DotProduct(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) { if (onFace) *onFace = 0; return b; }

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        if (onFace) *onFace = 0;
        return Vector3Add(a, Vector3Scale(ab, v));
    }

    Vector3 cp = Vector3Subtract(p, c);
    float d5 = Vector3DotProduct(ab, cp), d6 = Vector3DotProduct(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) { if (onFace) *onFace = 0; return c; }

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        if (onFace) *onFace = 0;
        return Vector3Add(a, Vector3Scale(ac, w));
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        if (onFace) *onFace = 0;
        return Vector3Add(b, Vector3Scale(Vector3Subtract(c, b), w));
    }

    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom, w = vc * denom;
    if (onFace) *onFace = 1;
    return Vector3Add(a, Vector3Add(Vector3Scale(ab, v), Vector3Scale(ac, w)));
}

static int arena_mesh_sphere(Vector3 *p, float r, float reach, Vector3 *normals, int maxN)
{
    int ids[256], n, i, nc = 0;
    Vector3 rad = V3(r, r, r);
    n = arena_query(Vector3Subtract(*p, rad), Vector3Add(*p, rad), ids, 256);
    for (i = 0; i < n; i++) {
        const ATri *t = &g_tris[ids[i]];
        float s = Vector3DotProduct(t->n, Vector3Subtract(*p, t->a)), pen;
        Vector3 q, d, nrm;
        int onFace = 0;
        if (s < -reach || s > r) continue;
        q = closest_on_tri(*p, t->a, t->b, t->c, &onFace);
        d = Vector3Subtract(*p, q);
        if (s >= 0.0f) {
            float dist = Vector3Length(d);
            if (dist >= r) continue;
            nrm = dist > 1e-5f ? Vector3Scale(d, 1.0f / dist) : t->n;
            if (Vector3DotProduct(nrm, t->n) < 0.0f) continue;
            pen = r - dist;
        } else {
            if (!onFace) continue;
            nrm = t->n;
            pen = r - s;
        }
        *p = Vector3Add(*p, Vector3Scale(nrm, pen));
        if (nc < maxN) normals[nc++] = nrm;
    }
    return nc;
}

static float arena_mesh_raycast(Vector3 o, Vector3 dir, float maxT, Vector3 *nOut)
{
    int ids[256], n, i;
    Vector3 e = Vector3Add(o, Vector3Scale(dir, maxT));
    float best = maxT + 1.0f;
    n = arena_query(Vector3Subtract(Vector3Min(o, e), V3(0.1f, 0.1f, 0.1f)),
                    Vector3Add(Vector3Max(o, e), V3(0.1f, 0.1f, 0.1f)), ids, 256);
    for (i = 0; i < n; i++) {
        const ATri *t = &g_tris[ids[i]];
        Vector3 e1, e2, pv, tv, qv;
        float det, inv, u, v, tt;
        if (Vector3DotProduct(t->n, dir) >= 0.0f) continue;
        e1 = Vector3Subtract(t->b, t->a); e2 = Vector3Subtract(t->c, t->a);
        pv = Vector3CrossProduct(dir, e2);
        det = Vector3DotProduct(e1, pv);
        if (fabsf(det) < 1e-9f) continue;
        inv = 1.0f / det;
        tv = Vector3Subtract(o, t->a);
        u = Vector3DotProduct(tv, pv) * inv;
        if (u < 0.0f || u > 1.0f) continue;
        qv = Vector3CrossProduct(tv, e1);
        v = Vector3DotProduct(dir, qv) * inv;
        if (v < 0.0f || u + v > 1.0f) continue;
        tt = Vector3DotProduct(e2, qv) * inv;
        if (tt < -0.05f || tt >= best) continue;
        best = fmaxf(tt, 0.0f);
        *nOut = t->n;
    }
    return best <= maxT ? best : -1.0f;
}

static float arena_raycast(Vector3 o, Vector3 dir, float maxT, Vector3 *nOut)
{
    APlane pl[16];
    int i, n;
    float best = maxT + 1.0f;
    if (g_tris) return arena_mesh_raycast(o, dir, maxT, nOut);
    n = arena_planes(o, pl);
    for (i = 0; i < n; i++) {
        float dn = Vector3DotProduct(pl[i].n, dir), t;
        if (dn > -1e-4f) continue;
        t = (Vector3DotProduct(pl[i].n, o) - pl[i].d) / -dn;
        if (t < 0.0f) t = 0.0f;
        if (t < best) { best = t; *nOut = pl[i].n; }
    }
    return best <= maxT ? best : -1.0f;
}

static void sphere_vs_segment(Vector3 *p, Vector3 *v, float r, Vector3 a, Vector3 b, float e)
{
    Vector3 ab = Vector3Subtract(b, a);
    float t = clampf(Vector3DotProduct(Vector3Subtract(*p, a), ab) / Vector3DotProduct(ab, ab), 0, 1);
    Vector3 q = Vector3Add(a, Vector3Scale(ab, t));
    Vector3 d = Vector3Subtract(*p, q);
    float dist = Vector3Length(d), R = r + POST_R;
    if (dist < R && dist > 1e-5f) {
        Vector3 n = Vector3Scale(d, 1.0f / dist);
        float vn = Vector3DotProduct(*v, n);
        *p = Vector3Add(*p, Vector3Scale(n, R - dist));
        if (vn < 0) *v = Vector3Subtract(*v, Vector3Scale(n, vn * (1.0f + e)));
    }
}

static void sphere_vs_posts(Vector3 *p, Vector3 *v, float r, float e)
{
    int s;
    for (s = -1; s <= 1; s += 2) {
        float z = ARENA_L * s;
        if (fabsf(p->z - z) > r + POST_R + 1.0f) continue;
        sphere_vs_segment(p, v, r, V3(-GOAL_HW, 0, z), V3(-GOAL_HW, GOAL_H, z), e);
        sphere_vs_segment(p, v, r, V3( GOAL_HW, 0, z), V3( GOAL_HW, GOAL_H, z), e);
        sphere_vs_segment(p, v, r, V3(-GOAL_HW, GOAL_H, z), V3(GOAL_HW, GOAL_H, z), e);
    }
}

/* ------------------------------------------------------------------------ */
/* Car Mechanics                                                              */
/* ------------------------------------------------------------------------ */
static void car_init(Car *c, const SarmModel *m)
{
    const SarmHeader *h = &m->header;
    const float s = CAR_SCALE;
    float lx = (h->bounds_max[0] - h->bounds_min[0]) * s;
    float ly = (h->bounds_max[1] - h->bounds_min[1]) * s;
    float lz = (h->bounds_max[2] - h->bounds_min[2]) * s;
    float fx = -1e9f, rx = 1e9f;
    int i;
    memset(c, 0, sizeof(*c));
    c->boxMin = V3(h->bounds_min[0] * s, h->bounds_min[1] * s, h->bounds_min[2] * s);
    c->boxMax = V3(h->bounds_max[0] * s, h->bounds_max[1] * s, h->bounds_max[2] * s);
    for (i = 0; i < 4; i++) {
        c->wheelLocal[i] = V3(h->wheel_pos[i][0] * s, h->wheel_pos[i][1] * s, h->wheel_pos[i][2] * s);
        fx = fmaxf(fx, c->wheelLocal[i].x); rx = fminf(rx, c->wheelLocal[i].x);
    }
    c->wheelBase   = clampf(fx - rx, 1.0f, 4.0f);
    c->wheelRadius = clampf(c->wheelLocal[0].y - c->boxMin.y, 0.2f, 0.8f);
    c->bodyBottom  = c->boxMin.y + SUSP_UP + 0.05f;
    c->invInertia = V3(12.0f / (ly*ly + lz*lz), 12.0f / (lx*lx + lz*lz), 12.0f / (lx*lx + ly*ly));
}

static void car_reset(Car *c, Vector3 pos, float yaw)
{
    c->pos = pos;
    c->pos.y = -c->boxMin.y;
    c->vel = c->angVel = V3(0, 0, 0);
    c->rot = QuaternionFromAxisAngle(V3(0, 1, 0), yaw);
    c->boost = BOOST_START;
    c->boostMinTimer = 0; c->boosting = 0;
    c->hasJumped = c->hasFlipped = 0;
    c->jumpTimer = c->airTime = c->flipTimer = 0;
    c->steerAngle = c->landTimer = c->stuckTimer = 0;
    c->wheelSpinRate = 0; c->uprightTimer = 0;
    c->demolished = 0; c->demoTimer = 0.0f;
}

static inline void kickoff_spot(int slot, int team, int teamSize, Vector3 *pos, float *yaw)
{
    static const float xs[3] = { 0.0f, -22.0f, 22.0f };
    float as = team == 0 ? 1.0f : -1.0f;
    float z = slot == 0 ? KICKOFF_Z : KICKOFF_Z * 1.15f;
    (void)teamSize;
    *pos = V3(xs[slot % 3] * as, 0, -as * z);
    *yaw = team == 0 ? -PI / 2.0f : PI / 2.0f;
}

static float max_steer_angle(float speed)
{
    static const float xs[] = { 0.0f, 1000.0f, 2000.0f, 3000.0f, 3500.0f, 6000.0f };
    static const float ys[] = { 35.0f, 20.0f, 11.0f, 6.25f, 5.0f, 2.0f };
    return curve(xs, ys, 6, speed * 100.0f);
}

static float throttle_accel(float speed)
{
    static const float xs[] = { 0.0f, 24.5f, 24.7f };
    static const float ys[] = { 28.0f, 2.8f, 0.0f };
    return curve(xs, ys, 3, speed);
}

static float flip_impulse(float input, float velAlongUU)
{
    float vd = velAlongUU * signf(input), mag;
    if (vd >= 0) mag = 20000.0f + 20000.0f  * clampf(vd / 7000.0f, 0, 1);
    else         mag = 20000.0f + 100000.0f * clampf(-vd / 7000.0f, 0, 1);
    return mag * input / CAR_IMPULSE_MASS / 100.0f;
}

static float strafe_impulse(float input, float velAbsUU)
{
    float mag = 30000.0f + 20000.0f * clampf(velAbsUU / 7000.0f, 0, 1);
    return mag * input / CAR_IMPULSE_MASS / 100.0f;
}

static void car_contact_impulse(Car *c, Vector3 wp, Vector3 n)
{
    Vector3 r = Vector3Subtract(wp, c->pos), vp = car_point_vel(c, wp), vt, J, rn;
    float vn = Vector3DotProduct(vp, n), kinv, j, vtl;
    c->bodyContact = 1;
    c->bodyNormal = n;
    if (vn >= 0) return;
    rn   = Vector3CrossProduct(r, n);
    kinv = 1.0f + Vector3DotProduct(n, Vector3CrossProduct(car_inv_inertia(c, rn), r));
    j    = -vn / kinv;
    J    = Vector3Scale(n, j);
    vt   = Vector3Subtract(vp, Vector3Scale(n, vn));
    vtl  = Vector3Length(vt);
    if (vtl > 1e-4f)
        J = Vector3Add(J, Vector3Scale(vt, -fminf(vtl * 0.3f, 0.2f * j) / vtl));
    car_apply_impulse(c, wp, J);
}

static void car_collide_mesh(Car *c)
{
    float hy = 0.5f * (c->boxMax.y - c->bodyBottom);
    float r = fminf(hy, 0.45f);
    int k, m;
    for (k = 0; k < 8; k++) {
        Vector3 l = V3((k & 1) ? c->boxMax.x - r : c->boxMin.x + r,
                       (k & 2) ? c->boxMax.y - r : c->bodyBottom + r,
                       (k & 4) ? c->boxMax.z - r : c->boxMin.z + r);
        Vector3 w = Vector3Add(c->pos, car_to_world(c, l)), w2 = w, ns[8];
        int nc = arena_mesh_sphere(&w2, r, 0.7f, ns, 8);
        if (nc == 0) continue;
        c->pos = Vector3Add(c->pos, Vector3Subtract(w2, w));
        for (m = 0; m < nc; m++) car_contact_impulse(c, Vector3Subtract(w2, Vector3Scale(ns[m], r)), ns[m]);
    }
}

static void car_collide_arena(Car *c)
{
    APlane pl[16];
    int np, i, k;
    Vector3 corners[8];
    if (g_tris) { car_collide_mesh(c); return; }
    np = arena_planes(c->pos, pl);
    for (k = 0; k < 8; k++) {
        Vector3 l = V3((k & 1) ? c->boxMax.x : c->boxMin.x,
                       (k & 2) ? c->boxMax.y : c->bodyBottom,
                       (k & 4) ? c->boxMax.z : c->boxMin.z);
        corners[k] = l;
    }
    for (i = 0; i < np; i++) {
        float deepest = 0.0f; Vector3 wp = { 0 };
        for (k = 0; k < 8; k++) {
            Vector3 w = Vector3Add(c->pos, car_to_world(c, corners[k]));
            float dist = Vector3DotProduct(pl[i].n, w) - pl[i].d;
            if (dist < deepest) { deepest = dist; wp = w; }
        }
        if (deepest < 0.0f) {
            Vector3 n = pl[i].n, r, vp, vt, J, rn;
            float vn, kinv, j, vtl;
            c->bodyContact = 1;
            c->bodyNormal = n;
            c->pos = Vector3Add(c->pos, Vector3Scale(n, -deepest));
            wp = Vector3Add(wp, Vector3Scale(n, -deepest));
            r  = Vector3Subtract(wp, c->pos);
            vp = car_point_vel(c, wp);
            vn = Vector3DotProduct(vp, n);
            if (vn >= 0) continue;
            rn   = Vector3CrossProduct(r, n);
            kinv = 1.0f + Vector3DotProduct(n, Vector3CrossProduct(car_inv_inertia(c, rn), r));
            j    = -vn / kinv;
            J    = Vector3Scale(n, j);
            vt   = Vector3Subtract(vp, Vector3Scale(n, vn));
            vtl  = Vector3Length(vt);
            if (vtl > 1e-4f)
                J = Vector3Add(J, Vector3Scale(vt, -fminf(vtl * 0.3f, 0.2f * j) / vtl));
            car_apply_impulse(c, wp, J);
        }
    }
}

static void car_step(Car *c, const Input *in, float dt)
{
    Vector3 up = car_up(c), fwd = car_fwd(c), right = car_right(c);
    Vector3 acc = V3(0, -GRAVITY, 0), torque = V3(0, 0, 0), nsum = V3(0, 0, 0);
    int i, onGround;

    c->wheelsOnGround = 0;
    for (i = 0; i < 4; i++) {
        Vector3 hub = Vector3Add(c->pos, car_to_world(c, c->wheelLocal[i]));
        Vector3 anchor = Vector3Add(hub, Vector3Scale(up, SUSP_UP));
        Vector3 n = { 0, 1, 0 };
        float maxT = SUSP_UP + SUSP_DOWN + c->wheelRadius;
        float t = arena_raycast(anchor, Vector3Negate(up), maxT, &n), comp, f;
        c->wheelContact[i] = t >= 0.0f;
        {
            float ofsT = t >= 0.0f ? clampf(SUSP_UP - (t - c->wheelRadius), -SUSP_DOWN, SUSP_UP) : -0.08f;
            c->wheelOfs[i] += (ofsT - c->wheelOfs[i]) * fminf(1.0f, 30.0f * dt);
        }
        if (t < 0.0f) continue;
        c->wheelsOnGround++;
        nsum = Vector3Add(nsum, n);
        comp = SUSP_UP - (t - c->wheelRadius);
        f = SUSP_K * comp - SUSP_C * Vector3DotProduct(car_point_vel(c, anchor), up);
        if (comp > SUSP_UP) f += SUSP_K * 4.0f * (comp - SUSP_UP);   /* bump stop */
        if (f < 0.0f) f = 0.0f;
        acc    = Vector3Add(acc, Vector3Scale(up, f));
        torque = Vector3Add(torque, Vector3CrossProduct(Vector3Subtract(anchor, c->pos), Vector3Scale(up, f)));
    }
    onGround = c->wheelsOnGround >= 3;

    if (c->wheelsOnGround > 0 && c->flipTimer <= 0.0f) {
        Vector3 gn = Vector3Normalize(nsum);
        Vector3 axis = Vector3CrossProduct(up, gn);
        Vector3 wPerp = Vector3Subtract(c->angVel, Vector3Scale(up, Vector3DotProduct(c->angVel, up)));
        c->angVel = Vector3Add(c->angVel, Vector3Scale(axis, LAND_ALIGN * dt));
        c->angVel = Vector3Subtract(c->angVel, Vector3Scale(wPerp, fminf(1.0f, LAND_DAMP * dt)));
    }

    if (onGround) {
        Vector3 gn = Vector3Normalize(nsum);
        float vf = Vector3DotProduct(c->vel, fwd), vr, a = 0.0f, wUp, yawRate, grip, target, maxd;
        if (c->airTime > 0.25f) c->landTimer = LAND_NO_GRIP_TIME;
        grip = 1.0f - clampf(c->landTimer / LAND_NO_GRIP_TIME, 0.0f, 1.0f);
        c->landTimer = fmaxf(0.0f, c->landTimer - dt);

        if (fabsf(in->throttle) > 0.01f) {
            if (vf * in->throttle >= 0.0f || fabsf(vf) < 0.5f) a = throttle_accel(fabsf(vf)) * in->throttle;
            else                                               a = BRAKE_ACCEL * signf(in->throttle);
            if (in->slide) a *= HB_LONG_SLIP;
        } else if (fabsf(vf) > 0.1f) {
            a = -signf(vf) * fminf(COAST_ACCEL, fabsf(vf) / dt);
        } else {
            c->vel = Vector3Subtract(c->vel, Vector3Scale(fwd, vf));
        }
        acc = Vector3Add(acc, Vector3Scale(fwd, a));

        vr = Vector3DotProduct(c->vel, right);
        c->vel = Vector3Subtract(c->vel, Vector3Scale(right, vr * fminf(1.0f, (in->slide ? LAT_GRIP_SLIDE : LAT_GRIP) * grip * dt)));
        acc = Vector3Add(acc, Vector3Scale(gn, -STICKY_ACCEL));

        target = -in->steer * max_steer_angle(fabsf(vf));
        maxd = STEER_SPEED * dt;
        c->steerAngle += clampf(target - c->steerAngle, -maxd, maxd);
        c->steerAngle = clampf(c->steerAngle, -max_steer_angle(fabsf(vf)), max_steer_angle(fabsf(vf)));
        yawRate = vf * tanf(c->steerAngle * DEG2RAD) / c->wheelBase * (in->slide ? 1.25f : 1.0f);
        wUp = Vector3DotProduct(c->angVel, up);
        c->angVel = Vector3Add(c->angVel, Vector3Scale(up, (yawRate - wUp) * fmaxf(grip, 0.15f)));

        if (c->jumpTimer > 0.1f || !c->hasJumped) {
            c->hasJumped = c->hasFlipped = 0;
            c->jumpTimer = 0.0f;
        }
        c->airTime = 0.0f;
        c->flipTimer = 0.0f;
    } else {
        c->airTime += dt;
    }

    if (!onGround) {
        Vector3 w = car_to_local(c, c->angVel);
        float ctl = c->wheelsOnGround == 0 ? 1.0f : 0.5f;
        if (c->uprightTimer > 0.0f) {
            c->uprightTimer -= dt;
        } else if (c->flipTimer > 0.0f) {
            w.z = -c->flipDir.y * FLIP_RATE;
            w.x =  c->flipDir.x * FLIP_RATE;
            c->flipTimer -= dt;
            if (c->flipTimer <= 0.0f) { w.z = 0.0f; w.x = 0.0f; }
        } else {
            w.z += (AIR_T_PITCH *  in->pitch * ctl - AIR_D_PITCH * w.z) / (AIR_RESPONSE * AIR_D_PITCH) * dt;
            w.y += (AIR_T_YAW   * -in->yaw   * ctl - AIR_D_YAW   * w.y) / (AIR_RESPONSE * AIR_D_YAW)   * dt;
            w.x += (AIR_T_ROLL  *  in->roll  * ctl - AIR_D_ROLL  * w.x) / (AIR_RESPONSE * AIR_D_ROLL)  * dt;
        }
        c->angVel = car_to_world(c, w);
        acc = Vector3Add(acc, Vector3Scale(fwd, AIR_PUSH * in->throttle));
    }

    if (c->bodyContact && c->wheelsOnGround < 3 && Vector3DotProduct(up, c->bodyNormal) < 0.6f) {
        c->stuckTimer += dt;
    } else if (c->wheelsOnGround >= 3 || !c->bodyContact) {
        c->stuckTimer = 0.0f;
    }
    c->bodyContact = 0;

    if (in->jumpPressed) {
        if (c->stuckTimer > 0.05f) {
            Vector3 axis = Vector3CrossProduct(up, c->bodyNormal);
            float angle = acosf(clampf(Vector3DotProduct(up, c->bodyNormal), -1.0f, 1.0f));
            if (Vector3Length(axis) < 0.2f) axis = fwd;
            c->vel = Vector3Add(c->vel, Vector3Scale(c->bodyNormal, SELF_RIGHT_LIFT * SELF_RIGHT_TIME));
            c->angVel = Vector3Scale(Vector3Normalize(axis), angle / SELF_RIGHT_TIME);
            c->uprightTimer = SELF_RIGHT_TIME;
            c->hasJumped = c->hasFlipped = 1; c->jumpTimer = JUMP_FORCE_TIME;
            c->stuckTimer = 0.0f;
        } else if (onGround && !c->hasJumped) {
            c->vel = Vector3Add(c->vel, Vector3Scale(up, JUMP_IMPULSE));
            c->hasJumped = 1; c->hasFlipped = 0; c->jumpTimer = 0.0f;
        } else if (c->wheelsOnGround == 0 && !c->hasFlipped &&
                   ((c->hasJumped && c->jumpTimer < DODGE_WINDOW) || (!c->hasJumped && c->airTime < DODGE_WINDOW))) {
            float sx = in->steer, sy = -in->pitch, mag = sqrtf(sx*sx + sy*sy);
            c->hasFlipped = 1;
            if (mag > DODGE_DEADZONE) {
                Vector3 fh = V3(fwd.x, 0, fwd.z), rh;
                float nx = sx / mag, ny = sy / mag;
                if (Vector3Length(fh) < 0.1f) fh = V3(up.x, 0, up.z);
                fh = Vector3Normalize(fh);
                rh = V3(-fh.z, 0, fh.x);
                c->vel.y = 0.0f;
                c->vel = Vector3Add(c->vel, Vector3Scale(fh, flip_impulse(ny, Vector3DotProduct(c->vel, fh) * 100.0f)));
                c->vel = Vector3Add(c->vel, Vector3Scale(rh, strafe_impulse(nx, fabsf(Vector3DotProduct(c->vel, rh)) * 100.0f)));
                c->flipTimer = FLIP_TIME;
                c->flipDir = (Vector2){ nx, ny };
            } else {
                c->vel = Vector3Add(c->vel, Vector3Scale(up, JUMP_IMPULSE));
            }
        }
    }
    if (c->hasJumped) {
        if (in->jump && c->jumpTimer < JUMP_FORCE_TIME) acc = Vector3Add(acc, Vector3Scale(up, JUMP_FORCE));
        c->jumpTimer += dt;
    }

    if ((in->boost && c->boost > 0.0f) || c->boostMinTimer > 0.0f) {
        if (!c->boosting) c->boostMinTimer = BOOST_MIN_TIME;
        c->boosting = 1;
        acc = Vector3Add(acc, Vector3Scale(fwd, BOOST_ACCEL));
        c->boost = fmaxf(0.0f, c->boost - dt);
    } else {
        c->boosting = 0;
    }
    c->boostMinTimer -= dt;

    c->vel    = Vector3Add(c->vel, Vector3Scale(acc, dt));
    c->angVel = Vector3Add(c->angVel, Vector3Scale(car_inv_inertia(c, torque), dt));
    if (Vector3Length(c->vel) > CAR_MAX_SPEED) c->vel = Vector3Scale(Vector3Normalize(c->vel), CAR_MAX_SPEED);
    if (Vector3Length(c->angVel) > 12.0f)      c->angVel = Vector3Scale(Vector3Normalize(c->angVel), 12.0f);
    c->pos = Vector3Add(c->pos, Vector3Scale(c->vel, dt));
    {
        float wl = Vector3Length(c->angVel);
        if (wl > 1e-6f)
            c->rot = QuaternionNormalize(QuaternionMultiply(
                QuaternionFromAxisAngle(Vector3Scale(c->angVel, 1.0f / wl), wl * dt), c->rot));
    }
    car_collide_arena(c);

    if (c->wheelsOnGround > 0) c->wheelSpinRate = Vector3DotProduct(c->vel, car_fwd(c)) / c->wheelRadius;
    else                       c->wheelSpinRate *= expf(-0.8f * dt);
    c->wheelSpin = fmodf(c->wheelSpin - c->wheelSpinRate * dt, 2.0f * PI);
}

static inline int load_car_physics(const char *name, Car *car)
{
    SarmModel sm;
    char dir[256], path[512];
    int err;
    snprintf(dir, sizeof(dir), "export_c/cars/%s/", name);
    snprintf(path, sizeof(path), "%s%s.sarm", dir, name);
    FILE *f = fopen(path, "rb");
    if (!f) {
        snprintf(dir, sizeof(dir), "../export_c/cars/%s/", name);
        snprintf(path, sizeof(path), "%s%s.sarm", dir, name);
        f = fopen(path, "rb");
    }
    if (f) fclose(f);
    err = sarm_load(path, &sm);
    if (err != SARM_OK) {
        memset(car, 0, sizeof(*car));
        const float s = CAR_SCALE;
        car->boxMin = V3(-1.125f * s, -0.32f * s, -0.674f * s);
        car->boxMax = V3( 0.941f * s,  0.757f * s,  0.674f * s);
        car->wheelLocal[0] = V3( 0.72f * s, -0.099f * s, -0.457f * s);
        car->wheelLocal[1] = V3( 0.72f * s, -0.099f * s,  0.457f * s);
        car->wheelLocal[2] = V3(-0.72f * s, -0.08f * s,  -0.477f * s);
        car->wheelLocal[3] = V3(-0.72f * s, -0.08f * s,   0.477f * s);
        car->wheelBase   = 1.44f * s;
        car->wheelRadius = 0.221f * s;
        car->bodyBottom  = car->boxMin.y + SUSP_UP + 0.05f;
        float lx = (car->boxMax.x - car->boxMin.x);
        float ly = (car->boxMax.y - car->boxMin.y);
        float lz = (car->boxMax.z - car->boxMin.z);
        car->invInertia = V3(12.0f / (ly*ly + lz*lz), 12.0f / (lx*lx + lz*lz), 12.0f / (lx*lx + ly*ly));
        return 0;
    }
    car_init(car, &sm);
    sarm_free(&sm);
    return 1;
}

/* ------------------------------------------------------------------------ */
/* Ball Dynamics                                                              */
/* ------------------------------------------------------------------------ */
static void ball_reset(Ball *b)
{
    b->pos = V3(0, BALL_R, 0);
    b->vel = b->angVel = V3(0, 0, 0);
    b->rot = QuaternionIdentity();
}

static void ball_step(Ball *b, float dt)
{
    b->vel.y -= GRAVITY * BALL_GRAVITY_SCALE * dt;
    b->vel = Vector3Scale(b->vel, 1.0f - BALL_DRAG * dt);
    b->pos = Vector3Add(b->pos, Vector3Scale(b->vel, dt));

    float sp = Vector3Length(b->vel);
    if (sp > BALL_MAX_SPEED) b->vel = Vector3Scale(b->vel, BALL_MAX_SPEED / sp);

    if (g_tris) {
        Vector3 ns[16];
        int nc = arena_mesh_sphere(&b->pos, BALL_R, BALL_R, ns, 16);
        for (int i = 0; i < nc; i++) {
            Vector3 n = ns[i];
            float vn = Vector3DotProduct(b->vel, n);
            if (vn < 0.0f) {
                Vector3 vt = Vector3Subtract(b->vel, Vector3Scale(n, vn));
                b->vel = Vector3Add(Vector3Scale(n, -vn * BALL_RESTITUTION),
                                    Vector3Scale(vt, 1.0f - BALL_GROUND_FRICTION));
            }
        }
    } else {
        APlane pl[16];
        int np = arena_planes(b->pos, pl);
        for (int i = 0; i < np; i++) {
            float dist = Vector3DotProduct(pl[i].n, b->pos) - pl[i].d;
            if (dist < BALL_R) {
                Vector3 n = pl[i].n;
                b->pos = Vector3Add(b->pos, Vector3Scale(n, BALL_R - dist));
                float vn = Vector3DotProduct(b->vel, n);
                if (vn < 0.0f) {
                    Vector3 vt = Vector3Subtract(b->vel, Vector3Scale(n, vn));
                    b->vel = Vector3Add(Vector3Scale(n, -vn * BALL_RESTITUTION),
                                        Vector3Scale(vt, 1.0f - BALL_GROUND_FRICTION));
                }
            }
        }
        sphere_vs_posts(&b->pos, &b->vel, BALL_R, BALL_RESTITUTION);
    }

    float w = Vector3Length(b->vel) / BALL_R;
    if (w > 1e-4f) {
        Vector3 axis = Vector3Normalize(Vector3CrossProduct(V3(0, 1, 0), b->vel));
        if (Vector3Length(axis) > 0.5f) {
            b->rot = QuaternionNormalize(QuaternionMultiply(QuaternionFromAxisAngle(axis, w * dt), b->rot));
        }
    }
}

static int car_ball_collide(Car *c, Ball *b)
{
    Vector3 toB = Vector3Subtract(b->pos, c->pos);
    Vector3 loc = car_to_local(c, toB);
    Vector3 cl = V3(clampf(loc.x, c->boxMin.x, c->boxMax.x),
                    clampf(loc.y, c->boxMin.y, c->boxMax.y),
                    clampf(loc.z, c->boxMin.z, c->boxMax.z));
    Vector3 cp = car_to_world(c, cl);
    Vector3 d = Vector3Subtract(b->pos, Vector3Add(c->pos, cp));
    float dist = Vector3Length(d);
    if (dist >= BALL_R) return 0;

    Vector3 n = dist > 1e-4f ? Vector3Scale(d, 1.0f / dist) : car_up(c);
    float pen = BALL_R - dist;
    b->pos = Vector3Add(b->pos, Vector3Scale(n, pen * 0.8f));
    c->pos = Vector3Subtract(c->pos, Vector3Scale(n, pen * 0.2f));

    Vector3 vp = car_point_vel(c, Vector3Add(c->pos, cp));
    Vector3 vrel = Vector3Subtract(b->vel, vp);
    float vn = Vector3DotProduct(vrel, n);
    if (vn < 0.0f) {
        float j = -(1.0f + BALL_RESTITUTION) * vn / (1.0f + 1.0f / BALL_MASS_RATIO);
        b->vel = Vector3Add(b->vel, Vector3Scale(n, j));
        car_apply_impulse(c, Vector3Add(c->pos, cp), Vector3Scale(n, -j / BALL_MASS_RATIO));

        Vector3 fwd = car_fwd(c);
        Vector3 dir = Vector3Normalize(Vector3Add(Vector3Scale(n, 0.65f), Vector3Scale(fwd, 0.35f)));
        float relSpeed = Vector3Length(Vector3Subtract(c->vel, b->vel));
        float forwardness = fmaxf(0.0f, Vector3DotProduct(Vector3Normalize(c->vel), fwd));
        float scale = 0.45f + 0.35f * forwardness;
        b->vel = Vector3Add(b->vel, Vector3Scale(dir, relSpeed * scale * BALL_HIT_SCALE));
    }
    return 1;
}

/* ------------------------------------------------------------------------ */
/* Car-Car Collisions & Demolitions                                           */
/* ------------------------------------------------------------------------ */
static void car_sphere(const Car *c, int k, Vector3 *ctr, float *r)
{
    float hx = 0.5f * (c->boxMax.x - c->boxMin.x), cx = 0.5f * (c->boxMax.x + c->boxMin.x);
    float cy = 0.5f * (c->bodyBottom + c->boxMax.y);
    *r = fminf(0.5f * (c->boxMax.z - c->boxMin.z), hx) * 0.9f;
    *ctr = Vector3Add(c->pos, car_to_world(c, V3(cx + (k ? 1.0f : -1.0f) * fmaxf(hx - *r, 0.0f), cy, 0)));
}

static int car_car_collide(Car *a, Car *b)
{
    int i, j;
    for (i = 0; i < 2; i++) {
        for (j = 0; j < 2; j++) {
            Vector3 pa, pb, d, n;
            float ra, rb, dist, vrel;
            car_sphere(a, i, &pa, &ra);
            car_sphere(b, j, &pb, &rb);
            d = Vector3Subtract(pb, pa);
            dist = Vector3Length(d);
            if (dist >= ra + rb || dist < 1e-4f) continue;
            n = Vector3Scale(d, 1.0f / dist);
            a->pos = Vector3Subtract(a->pos, Vector3Scale(n, 0.5f * (ra + rb - dist)));
            b->pos = Vector3Add(b->pos, Vector3Scale(n, 0.5f * (ra + rb - dist)));
            vrel = Vector3DotProduct(Vector3Subtract(b->vel, a->vel), n);
            if (vrel < 0.0f) {
                float jn = -(1.0f + 0.3f) * vrel * 0.5f;
                a->vel = Vector3Subtract(a->vel, Vector3Scale(n, jn));
                b->vel = Vector3Add(b->vel, Vector3Scale(n, jn));
                if (-vrel > 8.0f) {
                    Car *victim = Vector3DotProduct(a->vel, n) > -Vector3DotProduct(b->vel, n) ? b : a;
                    victim->vel.y += 2.5f;
                }
            }
            return 1;
        }
    }
    return 0;
}

static int check_demolition(const Car *a, const Car *b, int teamA, int teamB)
{
    if (teamA == teamB) return 0;
    if (a->demolished || b->demolished) return 0;

    Vector3 toB = Vector3Subtract(b->pos, a->pos);
    float distSq = toB.x * toB.x + toB.y * toB.y + toB.z * toB.z;
    if (distSq > 3.4f * 3.4f) return 0;

    float dist = sqrtf(distSq);
    if (dist < 1e-4f) return 0;

    float speedA = Vector3Length(a->vel);
    float speedB = Vector3Length(b->vel);
    int supA = (speedA >= 37.0f);
    int supB = (speedB >= 37.0f);

    if (!supA && !supB) return 0;

    Vector3 dirToB = Vector3Scale(toB, 1.0f / dist);
    Vector3 dirToA = Vector3Scale(dirToB, -1.0f);

    Vector3 fwdA = car_fwd(a);
    Vector3 fwdB = car_fwd(b);

    float dotA = Vector3DotProduct(fwdA, dirToB);
    float dotB = Vector3DotProduct(fwdB, dirToA);

    float closingA = Vector3DotProduct(Vector3Subtract(a->vel, b->vel), dirToB);
    float closingB = Vector3DotProduct(Vector3Subtract(b->vel, a->vel), dirToA);

    int aDemosB = supA && dotA > 0.45f && closingA > 5.0f;
    int bDemosA = supB && dotB > 0.45f && closingB > 5.0f;

    if (aDemosB && bDemosA) return 3; /* Mutual demolition! Both explode! */
    if (aDemosB) return 1;            /* A demolishes B */
    if (bDemosA) return 2;            /* B demolishes A */
    return 0;
}

#endif /* PHYSICS_SIM_H */
