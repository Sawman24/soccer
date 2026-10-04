#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdint.h>

#include "net_protocol.h"
#include "net_socket.h"

#ifdef _WIN32
  #include <windows.h>
  #include <conio.h>
#else
  #include <sys/time.h>
  #include <unistd.h>
#endif

/* ------------------------------------------------------------------------ */
/* Math Types & Helpers                                                       */
/* ------------------------------------------------------------------------ */
#ifndef PI
  #define PI 3.14159265358979323846f
#endif

typedef struct Vector2 { float x, y; } Vector2;
typedef struct Vector3 { float x, y, z; } Vector3;
typedef struct Vector4 { float x, y, z, w; } Vector4;
typedef Vector4 Quaternion;

static inline Vector3 V3(float x, float y, float z) { return (Vector3){ x, y, z }; }
static inline Vector2 V2(float x, float y)          { return (Vector2){ x, y }; }
static inline Vector3 Vector3Zero(void)             { return (Vector3){ 0, 0, 0 }; }
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float signf(float v)                  { return v < 0.0f ? -1.0f : (v > 0.0f ? 1.0f : 0.0f); }

static inline Vector3 Vector3Add(Vector3 a, Vector3 b) { return V3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline Vector3 Vector3Subtract(Vector3 a, Vector3 b) { return V3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline Vector3 Vector3Scale(Vector3 v, float s) { return V3(v.x * s, v.y * s, v.z * s); }
static inline float Vector3DotProduct(Vector3 a, Vector3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline float Vector3Length(Vector3 v) { return sqrtf(Vector3DotProduct(v, v)); }
static inline Vector3 Vector3Normalize(Vector3 v) {
    float l = Vector3Length(v);
    return l > 1e-6f ? Vector3Scale(v, 1.0f / l) : Vector3Zero();
}
static inline Vector3 Vector3CrossProduct(Vector3 a, Vector3 b) {
    return V3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
static inline Vector3 Vector3Lerp(Vector3 a, Vector3 b, float t) {
    return V3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
}

static inline Quaternion QuaternionIdentity(void) { return (Quaternion){ 0.0f, 0.0f, 0.0f, 1.0f }; }
static inline Quaternion QuaternionInvert(Quaternion q) {
    float l2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    float inv = l2 > 0.0f ? 1.0f / l2 : 1.0f;
    return (Quaternion){ -q.x * inv, -q.y * inv, -q.z * inv, q.w * inv };
}
static inline Quaternion QuaternionMultiply(Quaternion q1, Quaternion q2) {
    return (Quaternion){
        q1.x * q2.w + q1.w * q2.x + q1.y * q2.z - q1.z * q2.y,
        q1.y * q2.w + q1.w * q2.y + q1.z * q2.x - q1.x * q2.z,
        q1.z * q2.w + q1.w * q2.z + q1.x * q2.y - q1.y * q2.x,
        q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
    };
}
static inline Quaternion QuaternionNormalize(Quaternion q) {
    float l = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (l < 1e-6f) return QuaternionIdentity();
    float inv = 1.0f / l;
    return (Quaternion){ q.x * inv, q.y * inv, q.z * inv, q.w * inv };
}
static inline Quaternion QuaternionFromAxisAngle(Vector3 axis, float angle) {
    Vector3 a = Vector3Normalize(axis);
    float s = sinf(angle * 0.5f);
    return (Quaternion){ a.x * s, a.y * s, a.z * s, cosf(angle * 0.5f) };
}
static inline Vector3 Vector3RotateByQuaternion(Vector3 v, Quaternion q) {
    Vector3 u = V3(q.x, q.y, q.z);
    float s = q.w;
    return Vector3Add(
        Vector3Add(Vector3Scale(u, 2.0f * Vector3DotProduct(u, v)),
                   Vector3Scale(v, s * s - Vector3DotProduct(u, u))),
        Vector3Scale(Vector3CrossProduct(u, v), 2.0f * s)
    );
}

/* ------------------------------------------------------------------------ */
/* Physics Constants & Types                                                  */
/* ------------------------------------------------------------------------ */
#define PHYS_HZ            60.0f
#define GRAVITY            13.12f
#define ARENA_W            82.0f
#define ARENA_L            102.5f
#define ARENA_H            61.4f
#define RAMP_C             12.0f
#define GOAL_HW            20.5f
#define GOAL_H             15.4f
#define GOAL_D             19.0f
#define POST_R             0.6f
#define GRID_CELL          4.0f

#define CAR_IMPULSE_MASS   40.0f
#define JUMP_IMPULSE       (20000.0f / CAR_IMPULSE_MASS / 100.0f)
#define JUMP_FORCE         (1500.0f / 100.0f)
#define JUMP_FORCE_TIME    0.2f
#define DODGE_WINDOW       1.25f
#define DODGE_DEADZONE     0.35f
#define FLIP_TIME          0.65f
#define FLIP_RATE          (2.0f * PI / FLIP_TIME)
#define BOOST_ACCEL        (GRAVITY * 1.53f)
#define BOOST_MAX          3.0f
#define BOOST_START        1.0f
#define BOOST_MIN_TIME     0.25f
#define CAR_MAX_SPEED      (6000.0f / 100.0f)
#define AIR_PUSH           (100.0f / 100.0f)

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

typedef struct Input {
    float throttle, steer, pitch, yaw, roll;
    int   jump, jumpPressed, boost, slide;
} Input;

typedef struct Car {
    Vector3    pos, vel, angVel;
    Quaternion rot;
    Vector3    invInertia;
    Vector3    boxMin, boxMax;
    float      bodyBottom;
    Vector3    wheelLocal[4];
    float      wheelRadius;
    int        wheelContact[4];
    int        wheelsOnGround;
    float      boost, boostMinTimer;
    int        boosting;
    int        hasJumped, hasFlipped;
    float      jumpTimer, airTime, flipTimer;
    Vector2    flipDir;
    int        bodyContact;
    Vector3    bodyNormal;
    float      stuckTimer;
    float      wheelBase;
    float      steerAngle;
    float      landTimer;
    float      uprightTimer;
    float      wheelSpin, wheelSpinRate;
    float      wheelOfs[4];
    int        demolished;
    float      demoTimer;
} Car;

typedef struct Ball {
    Vector3    pos, vel, angVel;
    Quaternion rot;
} Ball;

typedef struct APlane { Vector3 n; float d; } APlane;
typedef struct ATri { Vector3 a, b, c, n; float ymin, ymax; } ATri;

static ATri *g_tris = NULL;
static int   g_triCount = 0;
static int  *g_cellStart = NULL, *g_cellItems = NULL, *g_stamp = NULL, g_stampId = 0;
static int   g_gx = 0, g_gz = 0;
static float g_gx0 = 0, g_gz0 = 0;

static const char *CAR_NAMES[] = {
    "octane", "aftershock", "backfire", "breakout", "dominus",
    "marauder", "paladin", "renegade", "roadhog", "scarab",
    "takumi", "venom", "zippy"
};
#define CAR_COUNT ((int)(sizeof(CAR_NAMES) / sizeof(CAR_NAMES[0])))

/* ------------------------------------------------------------------------ */
/* Arena Mesh Loader & Collisions                                             */
/* ------------------------------------------------------------------------ */
static int arena_mesh_load(const char *path)
{
    FILE *f = fopen(path, "rb");
    char magic[4];
    unsigned int count = 0;
    int i, x, z, pass;
    float minx = 1e30f, maxx = -1e30f, minz = 1e30f, maxz = -1e30f;
    if (!f) return 0;
    if (fread(magic, 1, 4, f) != 4 || memcmp(magic, "SARC", 4) != 0 ||
        fread(&count, 4, 1, f) != 1 || count == 0 || count > 1000000) {
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
    printf("[ARENA] Loaded %d collision triangles from %s\n", g_triCount, path);
    return 1;
}

static inline Vector3 closest_pt_tri(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
{
    Vector3 ab = Vector3Subtract(b, a), ac = Vector3Subtract(c, a), ap = Vector3Subtract(p, a);
    float d1 = Vector3DotProduct(ab, ap), d2 = Vector3DotProduct(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;
    Vector3 bp = Vector3Subtract(p, b);
    float d3 = Vector3DotProduct(ab, bp), d4 = Vector3DotProduct(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        return Vector3Add(a, Vector3Scale(ab, v));
    }
    Vector3 cp = Vector3Subtract(p, c);
    float d5 = Vector3DotProduct(ab, cp), d6 = Vector3DotProduct(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        return Vector3Add(a, Vector3Scale(ac, w));
    }
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return Vector3Add(b, Vector3Scale(Vector3Subtract(c, b), w));
    }
    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom, w = vc * denom;
    return Vector3Add(a, Vector3Add(Vector3Scale(ab, v), Vector3Scale(ac, w)));
}

static int arena_mesh_sphere(Vector3 *pos, float r, float pushR, Vector3 *outNormals, int maxNormals)
{
    int count = 0;
    if (!g_tris) return 0;
    int x0 = clampf((int)((pos->x - r - g_gx0) / GRID_CELL), 0, g_gx - 1);
    int x1 = clampf((int)((pos->x + r - g_gx0) / GRID_CELL), 0, g_gx - 1);
    int z0 = clampf((int)((pos->z - r - g_gz0) / GRID_CELL), 0, g_gz - 1);
    int z1 = clampf((int)((pos->z + r - g_gz0) / GRID_CELL), 0, g_gz - 1);
    int stamp = ++g_stampId;

    for (int z = z0; z <= z1; z++) {
        for (int x = x0; x <= x1; x++) {
            int cell = z * g_gx + x;
            int start = g_cellStart[cell], end = g_cellStart[cell + 1];
            for (int k = start; k < end; k++) {
                int ti = g_cellItems[k];
                if (g_stamp[ti] == stamp) continue;
                g_stamp[ti] = stamp;
                const ATri *t = &g_tris[ti];
                if (pos->y + r < t->ymin || pos->y - r > t->ymax) continue;
                Vector3 cp = closest_pt_tri(*pos, t->a, t->b, t->c);
                Vector3 d = Vector3Subtract(*pos, cp);
                float dist = Vector3Length(d);
                if (dist < r) {
                    Vector3 n = dist > 1e-4f ? Vector3Scale(d, 1.0f / dist) : t->n;
                    if (Vector3DotProduct(n, t->n) < 0.0f) n = t->n;
                    *pos = Vector3Add(cp, Vector3Scale(n, pushR));
                    if (count < maxNormals) outNormals[count++] = n;
                }
            }
        }
    }
    return count;
}

static int arena_mesh_ray(Vector3 start, Vector3 dir, float maxDist, float *hitDist, Vector3 *hitNormal)
{
    float best = maxDist;
    int hit = 0;
    if (!g_tris) return 0;
    Vector3 end = Vector3Add(start, Vector3Scale(dir, maxDist));
    int x0 = clampf((int)((fminf(start.x, end.x) - g_gx0) / GRID_CELL), 0, g_gx - 1);
    int x1 = clampf((int)((fmaxf(start.x, end.x) - g_gx0) / GRID_CELL), 0, g_gx - 1);
    int z0 = clampf((int)((fminf(start.z, end.z) - g_gz0) / GRID_CELL), 0, g_gz - 1);
    int z1 = clampf((int)((fmaxf(start.z, end.z) - g_gz0) / GRID_CELL), 0, g_gz - 1);
    int stamp = ++g_stampId;

    for (int z = z0; z <= z1; z++) {
        for (int x = x0; x <= x1; x++) {
            int cell = z * g_gx + x;
            int s = g_cellStart[cell], e = g_cellStart[cell + 1];
            for (int k = s; k < e; k++) {
                int ti = g_cellItems[k];
                if (g_stamp[ti] == stamp) continue;
                g_stamp[ti] = stamp;
                const ATri *t = &g_tris[ti];
                Vector3 ab = Vector3Subtract(t->b, t->a), ac = Vector3Subtract(t->c, t->a);
                Vector3 pvec = Vector3CrossProduct(dir, ac);
                float det = Vector3DotProduct(ab, pvec);
                if (det < 1e-6f) continue;
                float invDet = 1.0f / det;
                Vector3 tvec = Vector3Subtract(start, t->a);
                float u = Vector3DotProduct(tvec, pvec) * invDet;
                if (u < 0.0f || u > 1.0f) continue;
                Vector3 qvec = Vector3CrossProduct(tvec, ab);
                float v = Vector3DotProduct(dir, qvec) * invDet;
                if (v < 0.0f || u + v > 1.0f) continue;
                float d = Vector3DotProduct(ac, qvec) * invDet;
                if (d >= 0.0f && d < best) {
                    best = d;
                    *hitDist = d;
                    *hitNormal = t->n;
                    hit = 1;
                }
            }
        }
    }
    return hit;
}

/* ------------------------------------------------------------------------ */
/* Car & Ball Mechanics                                                       */
/* ------------------------------------------------------------------------ */
static inline Vector3 car_to_world(const Car *c, Vector3 v) { return Vector3RotateByQuaternion(v, c->rot); }
static inline Vector3 car_to_local(const Car *c, Vector3 v) { return Vector3RotateByQuaternion(v, QuaternionInvert(c->rot)); }
static inline Vector3 car_fwd(const Car *c)   { return car_to_world(c, V3(1, 0, 0)); }
static inline Vector3 car_up(const Car *c)    { return car_to_world(c, V3(0, 1, 0)); }
static inline Vector3 car_right(const Car *c) { return car_to_world(c, V3(0, 0, 1)); }

static Vector3 car_inv_inertia(const Car *c, Vector3 t)
{
    Vector3 l = car_to_local(c, t);
    l.x *= c->invInertia.x; l.y *= c->invInertia.y; l.z *= c->invInertia.z;
    return car_to_world(c, l);
}

static Vector3 car_point_vel(const Car *c, Vector3 worldPoint)
{
    return Vector3Add(c->vel, Vector3CrossProduct(c->angVel, Vector3Subtract(worldPoint, c->pos)));
}

static void car_init_default(Car *c)
{
    memset(c, 0, sizeof(*c));
    const float s = CAR_SCALE;
    c->boxMin = V3(-1.0f * s, -0.32f * s, -0.65f * s);
    c->boxMax = V3( 1.0f * s,  0.48f * s,  0.65f * s);
    c->wheelLocal[0] = V3( 0.75f * s, -0.05f * s, -0.65f * s);
    c->wheelLocal[1] = V3( 0.75f * s, -0.05f * s,  0.65f * s);
    c->wheelLocal[2] = V3(-0.75f * s, -0.05f * s, -0.65f * s);
    c->wheelLocal[3] = V3(-0.75f * s, -0.05f * s,  0.65f * s);
    c->wheelBase   = 1.5f * s;
    c->wheelRadius = 0.32f * s;
    c->bodyBottom  = c->boxMin.y + SUSP_UP + 0.05f;
    float lx = (c->boxMax.x - c->boxMin.x);
    float ly = (c->boxMax.y - c->boxMin.y);
    float lz = (c->boxMax.z - c->boxMin.z);
    c->invInertia = V3(12.0f / (ly*ly + lz*lz), 12.0f / (lx*lx + lz*lz), 12.0f / (lx*lx + ly*ly));
}

static void car_reset(Car *c, Vector3 pos, float yaw)
{
    c->pos = pos;
    c->vel = Vector3Zero();
    c->angVel = Vector3Zero();
    c->rot = QuaternionFromAxisAngle(V3(0, 1, 0), yaw);
    c->boost = BOOST_START;
    c->boosting = 0;
    c->hasJumped = c->hasFlipped = 0;
    c->jumpTimer = c->airTime = c->flipTimer = 0.0f;
    c->wheelsOnGround = 4;
    c->steerAngle = c->landTimer = c->uprightTimer = 0.0f;
    c->wheelSpin = c->wheelSpinRate = 0.0f;
    c->demolished = 0;
    c->demoTimer = 0.0f;
}

static void car_step(Car *c, const Input *in, float dt)
{
    if (c->demolished) {
        c->demoTimer -= dt;
        if (c->demoTimer <= 0.0f) {
            c->demolished = 0;
        }
        return;
    }

    Vector3 fwd = car_fwd(c), up = car_up(c), right = car_right(c);
    Vector3 acc = V3(0, -GRAVITY, 0);
    Vector3 torque = Vector3Zero();

    /* Ground / Suspension Raycasts */
    int contacts = 0;
    Vector3 avgContactNorm = Vector3Zero();
    for (int i = 0; i < 4; i++) {
        Vector3 wpos = Vector3Add(c->pos, car_to_world(c, c->wheelLocal[i]));
        Vector3 rayDir = Vector3Scale(up, -1.0f);
        float rayLen = c->wheelRadius + SUSP_DOWN + SUSP_UP;
        float hitDist = 0.0f;
        Vector3 hitNorm = Vector3Zero();
        if (arena_mesh_ray(wpos, rayDir, rayLen, &hitDist, &hitNorm)) {
            float comp = (c->wheelRadius + SUSP_UP) - hitDist;
            if (comp > 0.0f) {
                float f = comp * SUSP_K;
                Vector3 pvel = car_point_vel(c, wpos);
                float damp = -Vector3DotProduct(pvel, hitNorm) * SUSP_C;
                Vector3 suspForce = Vector3Scale(hitNorm, fmaxf(0.0f, f + damp));
                acc = Vector3Add(acc, suspForce);
                contacts++;
                avgContactNorm = Vector3Add(avgContactNorm, hitNorm);
                c->wheelContact[i] = 1;
            } else c->wheelContact[i] = 0;
        } else c->wheelContact[i] = 0;
    }
    c->wheelsOnGround = contacts;

    /* Sticky downforce when on walls */
    if (contacts > 0) {
        avgContactNorm = Vector3Normalize(avgContactNorm);
        acc = Vector3Add(acc, Vector3Scale(avgContactNorm, -STICKY_ACCEL));
    }

    /* Drive & Steer */
    float speed = Vector3DotProduct(c->vel, fwd);
    if (contacts >= 2) {
        float driveAcc = in->throttle * (in->throttle >= 0.0f ? 28.0f : BRAKE_ACCEL);
        acc = Vector3Add(acc, Vector3Scale(fwd, driveAcc));

        /* Lateral Grip & Powerslide */
        float grip = in->slide ? LAT_GRIP_SLIDE : LAT_GRIP;
        float latSpeed = Vector3DotProduct(c->vel, right);
        acc = Vector3Add(acc, Vector3Scale(right, -latSpeed * grip));

        /* Steering yaw */
        float steerSpeed = in->steer * (speed >= 0.0f ? 1.0f : -1.0f) * 3.5f;
        torque = Vector3Add(torque, Vector3Scale(up, steerSpeed * 40.0f));
    }

    /* Boost */
    c->boosting = 0;
    if (in->boost && c->boost > 0.0f) {
        c->boost = fmaxf(0.0f, c->boost - dt);
        c->boosting = 1;
        acc = Vector3Add(acc, Vector3Scale(fwd, BOOST_ACCEL));
    }

    /* Jump & Dodge / Flip */
    if (contacts >= 2) {
        c->hasJumped = 0;
        c->hasFlipped = 0;
        c->airTime = 0.0f;
    } else {
        c->airTime += dt;
    }

    if (in->jumpPressed && contacts >= 2 && !c->hasJumped) {
        c->hasJumped = 1;
        c->jumpTimer = JUMP_FORCE_TIME;
        c->vel = Vector3Add(c->vel, Vector3Scale(up, JUMP_IMPULSE));
    } else if (in->jump && c->jumpTimer > 0.0f) {
        c->jumpTimer -= dt;
        acc = Vector3Add(acc, Vector3Scale(up, JUMP_FORCE));
    } else {
        c->jumpTimer = 0.0f;
    }

    /* Double Jump / Directional Dodge */
    if (in->jumpPressed && c->hasJumped && !c->hasFlipped && c->airTime < DODGE_WINDOW) {
        c->hasFlipped = 1;
        if (fabsf(in->pitch) > DODGE_DEADZONE || fabsf(in->steer) > DODGE_DEADZONE) {
            Vector3 dodgeDir = Vector3Normalize(Vector3Add(Vector3Scale(fwd, in->pitch), Vector3Scale(right, in->steer)));
            c->vel = Vector3Add(c->vel, Vector3Scale(dodgeDir, 8.5f));
            c->flipTimer = FLIP_TIME;
            c->flipDir = V2(in->steer, in->pitch);
        } else {
            c->vel = Vector3Add(c->vel, Vector3Scale(up, JUMP_IMPULSE * 0.9f));
        }
    }

    /* Air Roll / Pitch / Yaw */
    if (contacts == 0) {
        if (c->flipTimer > 0.0f) {
            c->flipTimer -= dt;
            torque = Vector3Add(torque, Vector3Scale(right, c->flipDir.y * FLIP_RATE * 80.0f));
            torque = Vector3Add(torque, Vector3Scale(fwd, -c->flipDir.x * FLIP_RATE * 80.0f));
        } else {
            torque = Vector3Add(torque, Vector3Scale(right, in->pitch * AIR_T_PITCH));
            torque = Vector3Add(torque, Vector3Scale(up, in->yaw * AIR_T_YAW));
            torque = Vector3Add(torque, Vector3Scale(fwd, -in->roll * AIR_T_ROLL));
        }
    }

    /* Integrate Velocity & Position */
    c->vel = Vector3Add(c->vel, Vector3Scale(acc, dt));
    c->angVel = Vector3Add(c->angVel, Vector3Scale(car_inv_inertia(c, torque), dt));

    /* Speed Caps */
    if (Vector3Length(c->vel) > CAR_MAX_SPEED) c->vel = Vector3Scale(Vector3Normalize(c->vel), CAR_MAX_SPEED);
    if (Vector3Length(c->angVel) > 12.0f)      c->angVel = Vector3Scale(Vector3Normalize(c->angVel), 12.0f);

    c->pos = Vector3Add(c->pos, Vector3Scale(c->vel, dt));
    float wl = Vector3Length(c->angVel);
    if (wl > 1e-6f) {
        c->rot = QuaternionNormalize(QuaternionMultiply(
            QuaternionFromAxisAngle(Vector3Scale(c->angVel, 1.0f / wl), wl * dt), c->rot));
    }

    /* Arena Collision & Pushback */
    Vector3 normals[16];
    arena_mesh_sphere(&c->pos, c->wheelRadius + 0.3f, c->wheelRadius + 0.3f, normals, 16);
}

static void ball_reset(Ball *b)
{
    b->pos = V3(0, BALL_R, 0);
    b->vel = b->angVel = Vector3Zero();
    b->rot = QuaternionIdentity();
}

static void ball_step(Ball *b, float dt)
{
    b->vel.y -= GRAVITY * BALL_GRAVITY_SCALE * dt;
    b->vel = Vector3Scale(b->vel, 1.0f - BALL_DRAG * dt);
    float sp = Vector3Length(b->vel);
    if (sp > BALL_MAX_SPEED) b->vel = Vector3Scale(b->vel, BALL_MAX_SPEED / sp);
    b->pos = Vector3Add(b->pos, Vector3Scale(b->vel, dt));

    Vector3 ns[16];
    int nc = arena_mesh_sphere(&b->pos, BALL_R, BALL_R, ns, 16);
    for (int i = 0; i < nc; i++) {
        Vector3 n = ns[i];
        float vn = Vector3DotProduct(b->vel, n);
        if (vn < 0.0f) {
            Vector3 vt = Vector3Subtract(b->vel, Vector3Scale(n, vn));
            float vtl = Vector3Length(vt), drop = BALL_GROUND_FRICTION * (1.0f + BALL_RESTITUTION) * -vn;
            b->vel = Vector3Subtract(b->vel, Vector3Scale(n, vn * (1.0f + BALL_RESTITUTION)));
            if (vtl > 1e-4f) b->vel = Vector3Subtract(b->vel, Vector3Scale(vt, fminf(drop, vtl) / vtl));
            if (-vn < 1.0f) b->vel = Vector3Subtract(b->vel, Vector3Scale(n, Vector3DotProduct(b->vel, n)));
        }
        b->angVel = Vector3Scale(Vector3CrossProduct(n, b->vel), 1.0f / BALL_R);
    }

    float wl = Vector3Length(b->angVel);
    if (wl > 1e-6f) {
        b->rot = QuaternionNormalize(QuaternionMultiply(
            QuaternionFromAxisAngle(Vector3Scale(b->angVel, 1.0f / wl), wl * dt), b->rot));
    }
}

static int car_ball_collide(Car *c, Ball *b)
{
    Vector3 lp = car_to_local(c, Vector3Subtract(b->pos, c->pos));
    Vector3 q  = V3(clampf(lp.x, c->boxMin.x, c->boxMax.x),
                    clampf(lp.y, c->boxMin.y, c->boxMax.y),
                    clampf(lp.z, c->boxMin.z, c->boxMax.z));
    Vector3 dl = Vector3Subtract(lp, q), n, cp, rel;
    float dist = Vector3Length(dl), vn;
    if (dist >= BALL_R) return 0;
    n  = dist > 1e-4f ? car_to_world(c, Vector3Scale(dl, 1.0f / dist)) : car_up(c);
    cp = Vector3Add(c->pos, car_to_world(c, q));
    b->pos = Vector3Add(b->pos, Vector3Scale(n, BALL_R - dist));
    rel = Vector3Subtract(b->vel, car_point_vel(c, cp));
    vn  = Vector3DotProduct(rel, n);
    if (vn < 0.0f) {
        float j = -(1.0f + 0.2f) * vn / (1.0f + 1.0f / BALL_MASS_RATIO);
        b->vel = Vector3Add(b->vel, Vector3Scale(n, j));
        c->vel = Vector3Subtract(c->vel, Vector3Scale(n, j / BALL_MASS_RATIO));
        Vector3 dir = Vector3Normalize(Vector3Subtract(b->pos, c->pos));
        float relSpeed = fminf(Vector3Length(rel), 80.0f);
        b->vel = Vector3Add(b->vel, Vector3Scale(dir, relSpeed * 0.65f * BALL_HIT_SCALE));
    }
    return 1;
}

static int check_demolition(const Car *a, const Car *b, int teamA, int teamB)
{
    if (teamA == teamB) return 0;
    if (a->demolished || b->demolished) return 0;
    Vector3 toB = Vector3Subtract(b->pos, a->pos);
    float distSq = Vector3DotProduct(toB, toB);
    if (distSq > 3.4f * 3.4f) return 0;
    float dist = sqrtf(distSq);
    if (dist < 1e-4f) return 0;

    int supA = (Vector3Length(a->vel) >= 37.0f);
    int supB = (Vector3Length(b->vel) >= 37.0f);
    if (!supA && !supB) return 0;

    Vector3 dirToB = Vector3Scale(toB, 1.0f / dist);
    Vector3 dirToA = Vector3Scale(dirToB, -1.0f);
    float dotA = Vector3DotProduct(car_fwd(a), dirToB);
    float dotB = Vector3DotProduct(car_fwd(b), dirToA);
    float closingA = Vector3DotProduct(Vector3Subtract(a->vel, b->vel), dirToB);
    float closingB = Vector3DotProduct(Vector3Subtract(b->vel, a->vel), dirToA);

    int aDemosB = supA && dotA > 0.45f && closingA > 5.0f;
    int bDemosA = supB && dotB > 0.45f && closingB > 5.0f;
    if (aDemosB && bDemosA) return 3;
    if (aDemosB) return 1;
    if (bDemosA) return 2;
    return 0;
}

/* ------------------------------------------------------------------------ */
/* Server Client Session                                                     */
/* ------------------------------------------------------------------------ */
typedef struct ClientSession {
    int      active;
    NetAddr  addr;
    uint32_t last_seen_ms;
    uint32_t last_input_tick;
    uint8_t  player_id;
    uint8_t  team;
    uint8_t  car_model;
    uint8_t  skin;
    char     name[24];
    Input    latest_input;
} ClientSession;

/* High-res time in milliseconds */
static uint32_t get_time_ms(void)
{
#ifdef _WIN32
    static LARGE_INTEGER freq;
    static int init = 0;
    if (!init) { QueryPerformanceFrequency(&freq); init = 1; }
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return (uint32_t)((now.QuadPart * 1000) / freq.QuadPart);
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint32_t)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
#endif
}

/* ------------------------------------------------------------------------ */
/* Main Server Loop                                                           */
/* ------------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    int port = SARP_DEFAULT_PORT;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) port = atoi(argv[++i]);
    }

    printf("\n============================================================\n");
    printf("        SARPBC DEDICATED SERVER (60 Hz FIXED TICK)\n");
    printf("============================================================\n");
    printf("[SERVER] Initializing network subsystem...\n");
    if (!net_init()) {
        fprintf(stderr, "[ERROR] Failed to initialize network!\n");
        return 1;
    }

    NetSocket sock = net_socket_create(port, 1);
    if (sock == NET_INVALID_SOCKET) {
        fprintf(stderr, "[ERROR] Could not bind UDP port %d!\n", port);
        net_shutdown();
        return 1;
    }
    printf("[SERVER] Listening on UDP port %d (Non-blocking)\n", port);

    /* Load Arena Collision */
    const char *col_paths[] = {
        "../export_c/arena/arena_col.bin",
        "export_c/arena/arena_col.bin",
        "assets/arena/arena_col.bin",
        "arena_col.bin"
    };
    int arena_ok = 0;
    for (int i = 0; i < 4; i++) {
        if (arena_mesh_load(col_paths[i])) { arena_ok = 1; break; }
    }
    if (!arena_ok) {
        printf("[WARNING] Collision mesh arena_col.bin not found! Using fallback bounds.\n");
    }

    /* Initialize World & Players */
    Car cars[SARP_MAX_CLIENTS];
    ClientSession clients[SARP_MAX_CLIENTS];
    memset(clients, 0, sizeof(clients));
    for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
        car_init_default(&cars[i]);
    }

    Ball ball;
    ball_reset(&ball);

    Vector3 pads[PAD_COUNT] = {
        V3(-78, 0, -(ARENA_L - 24)), V3(78, 0, -(ARENA_L - 24)),
        V3(-78, 0,  (ARENA_L - 24)), V3(78, 0,  (ARENA_L - 24)),
        V3(-82, 0, 0),               V3(82, 0, 0)
    };
    float padTimer[PAD_COUNT] = { 0 };

    uint32_t server_tick = 0;
    uint8_t game_state = 1; /* 1 = PLAY */
    float match_time = MATCH_TIME;
    uint8_t score_blue = 0, score_orange = 0;

    printf("[SERVER] Match ready! Waiting for players to join...\n");
    printf("[SERVER] Available console commands: status, reset, quit\n\n");
    fflush(stdout);

    uint32_t last_tick_time = get_time_ms();
    const uint32_t tick_interval_ms = 16; /* ~60 Hz */
    int running = 1;

    char recv_buf[2048];
    NetAddr sender;

    while (running) {
        uint32_t now_ms = get_time_ms();

        /* --- 1. Receive & Process Incoming UDP Packets --- */
        int bytes = 0;
        while ((bytes = net_socket_recv(sock, &sender, recv_buf, sizeof(recv_buf))) > 0) {
            if (bytes < (int)sizeof(NetHeader)) continue;
            NetHeader *hdr = (NetHeader *)recv_buf;
            if (hdr->magic != SARP_NET_MAGIC || hdr->version != SARP_NET_VERSION) continue;

            if (hdr->type == PKT_PING) {
                PktPing *ping = (PktPing *)recv_buf;
                PktPong pong;
                pong.header.magic = SARP_NET_MAGIC;
                pong.header.version = SARP_NET_VERSION;
                pong.header.type = PKT_PONG;
                pong.header.player_id = 0xFF;
                pong.header.seq = ++server_tick;
                pong.header.ack = hdr->seq;
                pong.client_time_ms = ping->client_time_ms;
                pong.server_time_ms = now_ms;
                net_socket_send(sock, &sender, &pong, sizeof(pong));
            } else if (hdr->type == PKT_JOIN_REQ) {
                PktJoinReq *req = (PktJoinReq *)recv_buf;
                /* Find existing or allocate slot */
                int slot = -1;
                for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                    if (clients[i].active && net_addr_equal(&clients[i].addr, &sender)) {
                        slot = i; break;
                    }
                }
                if (slot < 0) {
                    for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                        if (!clients[i].active) { slot = i; break; }
                    }
                }

                PktJoinAck ack;
                memset(&ack, 0, sizeof(ack));
                ack.header.magic = SARP_NET_MAGIC;
                ack.header.version = SARP_NET_VERSION;
                ack.header.type = PKT_JOIN_ACK;
                ack.header.player_id = 0xFF;

                if (slot >= 0) {
                    ClientSession *cs = &clients[slot];
                    cs->active = 1;
                    cs->addr = sender;
                    cs->last_seen_ms = now_ms;
                    cs->player_id = (uint8_t)slot;
                    cs->car_model = req->car_model < CAR_COUNT ? req->car_model : 0;
                    cs->skin = req->skin;
                    cs->team = (req->pref_team <= 1) ? req->pref_team : (uint8_t)(slot % 2);
                    memcpy(cs->name, req->player_name, sizeof(cs->name) - 1);
                    cs->name[sizeof(cs->name) - 1] = 0;

                    /* Kickoff position */
                    float kz = cs->team == 0 ? -KICKOFF_Z : KICKOFF_Z;
                    float kyaw = cs->team == 0 ? -PI / 2.0f : PI / 2.0f;
                    float kx = ((slot / 2) - 1) * 12.0f;
                    car_reset(&cars[slot], V3(kx, 0.4f, kz), kyaw);

                    ack.accepted = 1;
                    ack.assigned_id = (uint8_t)slot;
                    ack.assigned_team = cs->team;
                    ack.car_model = cs->car_model;
                    ack.skin = cs->skin;
                    ack.tick_rate = 60;
                    snprintf(ack.server_name, sizeof(ack.server_name), "SARPBC Official #1");

                    char addr_str[64];
                    net_addr_to_string(&sender, addr_str, sizeof(addr_str));
                    printf("[JOIN] Slot %d (%s) connected from %s (Team %s, Car: %s)\n",
                           slot, cs->name, addr_str, cs->team == 0 ? "BLUE" : "ORANGE", CAR_NAMES[cs->car_model]);
                    fflush(stdout);
                } else {
                    ack.accepted = 0;
                }
                net_socket_send(sock, &sender, &ack, sizeof(ack));
            } else if (hdr->type == PKT_CLIENT_INPUT) {
                PktClientInput *cin = (PktClientInput *)recv_buf;
                int pid = hdr->player_id;
                if (pid >= 0 && pid < SARP_MAX_CLIENTS && clients[pid].active &&
                    net_addr_equal(&clients[pid].addr, &sender)) {
                    ClientSession *cs = &clients[pid];
                    cs->last_seen_ms = now_ms;
                    cs->last_input_tick = cin->client_tick;
                    cs->latest_input.throttle    = cin->input.throttle;
                    cs->latest_input.steer       = cin->input.steer;
                    cs->latest_input.pitch       = cin->input.pitch;
                    cs->latest_input.yaw         = cin->input.yaw;
                    cs->latest_input.roll        = cin->input.roll;
                    cs->latest_input.jump        = cin->input.jump;
                    cs->latest_input.jumpPressed = cin->input.jumpPressed;
                    cs->latest_input.boost       = cin->input.boost;
                    cs->latest_input.slide       = cin->input.slide;
                }
            } else if (hdr->type == PKT_DISCONNECT) {
                int pid = hdr->player_id;
                if (pid >= 0 && pid < SARP_MAX_CLIENTS && clients[pid].active) {
                    printf("[LEAVE] Slot %d (%s) disconnected\n", pid, clients[pid].name);
                    fflush(stdout);
                    clients[pid].active = 0;
                }
            }
        }

        /* --- 2. Check Client Timeouts (5 seconds) --- */
        for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
            if (clients[i].active && now_ms - clients[i].last_seen_ms > 5000) {
                printf("[TIMEOUT] Slot %d (%s) timed out\n", i, clients[i].name);
                fflush(stdout);
                clients[i].active = 0;
            }
        }

        /* --- 3. Step Authoritative 60 Hz Physics Tick --- */
        if (now_ms - last_tick_time >= tick_interval_ms) {
            float dt = 1.0f / PHYS_HZ;
            server_tick++;
            last_tick_time = now_ms;

            /* Step Active Cars */
            for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                if (clients[i].active) {
                    car_step(&cars[i], &clients[i].latest_input, dt);
                    clients[i].latest_input.jumpPressed = 0;
                }
            }

            /* Step Ball */
            ball_step(&ball, dt);

            /* Car-Ball Collisions */
            for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                if (clients[i].active && !cars[i].demolished) {
                    car_ball_collide(&cars[i], &ball);
                }
            }

            /* Car-Car Collisions & Demolitions */
            for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                if (!clients[i].active || cars[i].demolished) continue;
                for (int j = i + 1; j < SARP_MAX_CLIENTS; j++) {
                    if (!clients[j].active || cars[j].demolished) continue;
                    int demo = check_demolition(&cars[i], &cars[j], clients[i].team, clients[j].team);
                    if (demo == 1) {
                        cars[j].demolished = 1; cars[j].demoTimer = 3.0f;
                        printf("[DEMO] %s demolished %s!\n", clients[i].name, clients[j].name);
                        fflush(stdout);
                    } else if (demo == 2) {
                        cars[i].demolished = 1; cars[i].demoTimer = 3.0f;
                        printf("[DEMO] %s demolished %s!\n", clients[j].name, clients[i].name);
                        fflush(stdout);
                    } else if (demo == 3) {
                        cars[i].demolished = 1; cars[i].demoTimer = 3.0f;
                        cars[j].demolished = 1; cars[j].demoTimer = 3.0f;
                        printf("[DEMO] Mutual demolition between %s and %s!\n", clients[i].name, clients[j].name);
                        fflush(stdout);
                    }
                }
            }

            /* Boost Pads */
            for (int p = 0; p < PAD_COUNT; p++) {
                if (padTimer[p] > 0.0f) {
                    padTimer[p] -= dt;
                } else {
                    for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                        if (!clients[i].active || cars[i].demolished) continue;
                        float dx = cars[i].pos.x - pads[p].x, dz = cars[i].pos.z - pads[p].z;
                        if (dx*dx + dz*dz < PAD_RADIUS * PAD_RADIUS && cars[i].pos.y < 4.0f) {
                            cars[i].boost = BOOST_MAX;
                            padTimer[p] = PAD_RESPAWN;
                        }
                    }
                }
            }

            /* Goal Check */
            if (fabsf(ball.pos.z) > ARENA_L + BALL_R) {
                int scorer = ball.pos.z > 0 ? 0 : 1; /* 0 = Blue scored on Orange net, 1 = Orange scored */
                if (scorer == 0) score_blue++; else score_orange++;
                printf("[GOAL!] %s scored! (Blue %d - %d Orange)\n",
                       scorer == 0 ? "BLUE TEAM" : "ORANGE TEAM", score_blue, score_orange);
                fflush(stdout);
                ball_reset(&ball);
                for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                    if (clients[i].active) {
                        float kz = clients[i].team == 0 ? -KICKOFF_Z : KICKOFF_Z;
                        float kyaw = clients[i].team == 0 ? -PI / 2.0f : PI / 2.0f;
                        float kx = ((i / 2) - 1) * 12.0f;
                        car_reset(&cars[i], V3(kx, 0.4f, kz), kyaw);
                    }
                }
            }

            /* Match Timer Countdown */
            if (match_time > 0.0f) match_time -= dt;

            /* --- 4. Broadcast Authoritative Snapshot to All Clients --- */
            PktServerState state_pkt;
            memset(&state_pkt, 0, sizeof(state_pkt));
            state_pkt.header.magic = SARP_NET_MAGIC;
            state_pkt.header.version = SARP_NET_VERSION;
            state_pkt.header.type = PKT_SERVER_STATE;
            state_pkt.header.player_id = 0xFF;
            state_pkt.header.seq = server_tick;
            state_pkt.server_tick = server_tick;
            state_pkt.game_state = game_state;
            state_pkt.match_time = match_time;
            state_pkt.score_blue = score_blue;
            state_pkt.score_orange = score_orange;
            state_pkt.car_count = SARP_MAX_CLIENTS;

            /* Ball state */
            state_pkt.ball.pos = (NetVec3){ ball.pos.x, ball.pos.y, ball.pos.z };
            state_pkt.ball.vel = (NetVec3){ ball.vel.x, ball.vel.y, ball.vel.z };
            state_pkt.ball.angVel = (NetVec3){ ball.angVel.x, ball.angVel.y, ball.angVel.z };
            state_pkt.ball.rot = (NetQuat){ ball.rot.x, ball.rot.y, ball.rot.z, ball.rot.w };

            /* Car states */
            for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                NetCarState *ncs = &state_pkt.cars[i];
                ncs->active = clients[i].active ? 1 : 0;
                ncs->player_id = (uint8_t)i;
                ncs->team = clients[i].team;
                ncs->car_model = clients[i].car_model;
                ncs->skin = clients[i].skin;
                strncpy(ncs->player_name, clients[i].name, sizeof(ncs->player_name) - 1);
                ncs->demolished = cars[i].demolished ? 1 : 0;
                ncs->wheels_on_ground = (uint8_t)cars[i].wheelsOnGround;
                ncs->supersonic = (Vector3Length(cars[i].vel) >= 37.0f) ? 1 : 0;
                ncs->boost = cars[i].boost;
                ncs->pos = (NetVec3){ cars[i].pos.x, cars[i].pos.y, cars[i].pos.z };
                ncs->vel = (NetVec3){ cars[i].vel.x, cars[i].vel.y, cars[i].vel.z };
                ncs->angVel = (NetVec3){ cars[i].angVel.x, cars[i].angVel.y, cars[i].angVel.z };
                ncs->rot = (NetQuat){ cars[i].rot.x, cars[i].rot.y, cars[i].rot.z, cars[i].rot.w };
                ncs->steerAngle = cars[i].steerAngle;
                ncs->wheelSpin = cars[i].wheelSpin;
            }

            for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                if (clients[i].active) {
                    state_pkt.header.ack = clients[i].last_input_tick;
                    net_socket_send(sock, &clients[i].addr, &state_pkt, sizeof(state_pkt));
                }
            }
        }

#ifdef _WIN32
        /* Console input check (non-blocking) */
        if (_kbhit()) {
            char cmd[64] = { 0 };
            if (fgets(cmd, sizeof(cmd), stdin)) {
                cmd[strcspn(cmd, "\r\n")] = 0;
                if (strcmp(cmd, "quit") == 0 || strcmp(cmd, "exit") == 0) {
                    running = 0;
                } else if (strcmp(cmd, "status") == 0) {
                    printf("\n--- SERVER STATUS (Tick %u | Score: Blue %d - %d Orange) ---\n",
                           server_tick, score_blue, score_orange);
                    int count = 0;
                    for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                        if (clients[i].active) {
                            char astr[64];
                            net_addr_to_string(&clients[i].addr, astr, sizeof(astr));
                            printf("  Slot %d: [%s] Name: '%s' | Car: %s | %s\n",
                                   i, clients[i].team == 0 ? "BLUE" : "ORANGE", clients[i].name,
                                   CAR_NAMES[clients[i].car_model], astr);
                            count++;
                        }
                    }
                    if (count == 0) printf("  (No players connected)\n");
                    printf("-----------------------------------------------------------\n\n");
                } else if (strcmp(cmd, "reset") == 0) {
                    ball_reset(&ball);
                    score_blue = score_orange = 0;
                    match_time = MATCH_TIME;
                    printf("[SERVER] Match reset.\n");
                }
            }
        }
        Sleep(1);
#else
        usleep(1000);
#endif
    }

    printf("[SERVER] Shutting down dedicated server...\n");
    net_socket_close(sock);
    net_shutdown();
    if (g_tris) free(g_tris);
    if (g_cellStart) free(g_cellStart);
    if (g_cellItems) free(g_cellItems);
    if (g_stamp) free(g_stamp);
    printf("[SERVER] Goodbye!\n");
    return 0;
}
