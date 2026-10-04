/*
 * SARPBC-C  --  a from-scratch C99 + raylib re-creation of the Soccar mode of
 * "Supersonic Acrobatic Rocket-Powered Battle-Cars" (Psyonix, 2008).
 *
 * Milestone 1: one car, one ball, one arena. Driving, jump / double jump /
 * dodge, boost, air control, ball physics, goals, score and match timer.
 *
 * The engine code is original. The car model/textures are loaded at runtime
 * from YOUR OWN extracted game files (export_c/cars/<name>/); nothing from
 * the game is compiled into the executable.
 *
 * World frame: right-handed, +Y up, metres. The field runs along Z
 * (blue goal at -Z, orange goal at +Z), its width along X.
 * Car local frame (same as the .sarm export): +X forward, +Y up, +Z right.
 *
 * Unreal units: 1 uu = 1 cm, so config values are divided by 100.
 * SARPBC works at roughly 1.75x Rocket League scale (the Octane is 207 uu
 * long here vs 118 uu in RL), so wherever a value is NOT in the PS3 configs
 * we use the RL value * 1.75. Those places are marked "RLx1.75".
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#define SARPBC_MODEL_IMPLEMENTATION
#include "sarpbc_model.h"
#undef SARPBC_MODEL_IMPLEMENTATION

#include "net_client.h"
#include "physics_sim.h"

static const Vector3 LIGHT_DIR = { -0.35f, -0.85f, 0.4f };

typedef struct CarRender {
    Mesh     mesh[16];
    int      material[16];
    int      count;
    Material matBody, matGlass, matTire;
    Texture2D texBlue, texOrange, texCustom;
    /* wheels split out of the body at load time so they can spin/steer/bounce;
     * vertices are relative to whub (mesh space, unscaled) */
    Mesh     wmesh[4][3];
    int      wvalid[4][3], wanim[4], wfront[4];
    Vector3  whub[4];
} CarRender;

typedef enum { ST_COUNTDOWN, ST_PLAY, ST_GOAL, ST_OVER } GameState;

/* ------------------------------------------------------------------------ */
/* Bots                                                                       */
/* ------------------------------------------------------------------------ */
#define MAX_CARS 8
#define PRED_DT  (1.0f / 30.0f)
#define PRED_N   150                  /* 5 s of ball prediction */

#define BOT_ACT_NONE   0
#define BOT_ACT_JUMP   1              /* single jump for low-mid balls */
#define BOT_ACT_DJUMP  2              /* double jump */
#define BOT_ACT_DODGE  3              /* directional dodge / dash in any direction */
#define BOT_ACT_AERIAL 4              /* rocket-boost flight in 3D */

typedef struct Bot {
    int     action;             /* BOT_ACT_* */
    float   actionT;            /* timer within current action */
    float   stuckT, reverseT;
    Vector2 dodgeDir;           /* (steer, pitch) direction for dodge */
    float   kickJitter;         /* random kickoff offset */
    float   recoverT;           /* cooldown between recovery jumps */
    int     role;               /* 0 attack, 1 defend, 2 support */
    float   dashCooldown;       /* delay between speed-flip dashes */
    Vector3 aerialTarget;       /* 3D target point in air */
} Bot;

static const char *SKILL_NAMES[] = { "Rookie", "Pro", "All-Star" };
static const char *MODE_NAMES[]  = { "Free play", "1 v 1", "2 v 2", "3 v 3" };

/* Shared world knowledge, refreshed once per physics tick by bot_world_update(). */
static Vector3 g_pred[PRED_N], g_predVel[PRED_N];   /* ball path, sample i = i*PRED_DT after g_predAge ago */
static float   g_predAge = 0.0f;
static int     g_predCalls = 0;
static const Vector3 *g_botPads = NULL;
static const float   *g_botPadTimer = NULL;
static int            g_botPadCount = 0;

static float flat_dist(Vector3 a, Vector3 b) { float dx = a.x - b.x, dz = a.z - b.z; return sqrtf(dx*dx + dz*dz); }
static Vector3 flat_dir(Vector3 from, Vector3 to)
{
    Vector3 d = V3(to.x - from.x, 0, to.z - from.z);
    float l = Vector3Length(d);
    return l > 1e-4f ? Vector3Scale(d, 1.0f / l) : V3(0, 0, 1);
}

/* Steering helper: + = target is to the car's left. */
static float bot_angle_to(const Car *c, Vector3 target)
{
    Vector3 l = car_to_local(c, Vector3Subtract(target, c->pos));
    return atan2f(-l.z, l.x);
}

/* Re-simulate the ball's path when it was hit (velocity no longer matches the
 * prediction) and otherwise every few ticks. */
static void bot_world_update(const Ball *b, float dt, const Vector3 *pads, const float *padTimer, int npads)
{
    int i, k;
    Ball s = *b;
    g_botPads = pads; g_botPadTimer = padTimer; g_botPadCount = npads;
    g_predAge += dt;
    k = (int)(g_predAge / PRED_DT + 0.5f);
    if (g_predCalls++ > 0 && k < PRED_N - 30 && (g_predCalls % 6) != 0 &&
        Vector3Distance(g_predVel[k], b->vel) < 1.5f)
        return;
    g_predAge = 0.0f;
    for (i = 0; i < PRED_N; i++) {
        g_pred[i] = s.pos; g_predVel[i] = s.vel;
        ball_step(&s, PRED_DT * 0.5f);
        ball_step(&s, PRED_DT * 0.5f);
    }
}

static Vector3 pred_at(float t)
{
    float f = (t + g_predAge) / PRED_DT;
    int i = (int)f;
    if (i < 0) return g_pred[0];
    if (i >= PRED_N - 1) return g_pred[PRED_N - 1];
    return Vector3Lerp(g_pred[i], g_pred[i + 1], f - (float)i);
}

/* Straight-line distance the car can cover after i*PRED_DT (throttle + boost). */
static void drive_cover(const Car *c, int useBoost, float *cov)
{
    float v = fmaxf(0.0f, Vector3DotProduct(c->vel, car_fwd(c))), d = 0.0f, bst = useBoost ? c->boost : 0.0f;
    int i;
    for (i = 0; i < PRED_N; i++) {
        float a = throttle_accel(v);
        cov[i] = d;
        if (bst > 0.0f) { a += BOOST_ACCEL; bst -= PRED_DT; }
        v = fminf(v + a * PRED_DT, CAR_MAX_SPEED);
        d += v * PRED_DT;
    }
}

typedef struct Intercept {
    float   t;
    Vector3 p;
    int     ok;
    int     isAerial;   /* 1 if high ball reachable by rocket boost flight */
    int     isWall;     /* 1 if ball is on the arena wall / ramp */
} Intercept;

/* Earliest moment the car can get to the predicted ball (ground, wall, or aerial). */
static Intercept find_intercept(const Car *c, int useBoost, float maxH, int skill)
{
    float cov[PRED_N];
    Intercept r = { PRED_N * PRED_DT, g_pred[PRED_N - 1], 0, 0, 0 };
    int i;
    drive_cover(c, useBoost, cov);
    for (i = 0; i < PRED_N; i++) {
        float t = (float)i * PRED_DT - g_predAge, d, tt;
        Vector3 p;
        int k, onWall, canAerial;
        if (t < 0.05f) continue;
        p = pred_at(t);
        if (p.y > maxH) continue;
        onWall = (fabsf(p.x) > 60.0f || fabsf(p.z) > 113.0f) && p.y > 2.0f;
        canAerial = (p.y > 4.2f && !onWall && ((skill == 1 && p.y <= 6.5f) || skill >= 2) && c->boost >= 0.40f);

        if (canAerial) {
            /* 3D aerial rocket flight reach check */
            float dist3d = Vector3Distance(c->pos, p) - BALL_R;
            float maxBoostT = fminf(t, c->boost);
            float flightDist = fmaxf(0.0f, Vector3Length(c->vel)) * t + 0.5f * BOOST_ACCEL * maxBoostT * maxBoostT;
            if (dist3d <= flightDist + 4.0f && t < 2.5f) {
                r.t = t; r.p = p; r.ok = 1; r.isAerial = 1;
                return r;
            }
        } else if (onWall) {
            /* Wall ride intercept */
            float dist3d = Vector3Distance(c->pos, p) - (BALL_R + 1.0f);
            tt = t - fabsf(bot_angle_to(c, p)) * 0.25f;
            k  = (int)(tt / PRED_DT);
            if (dist3d <= 0.0f || (k >= 0 && cov[k < PRED_N ? k : PRED_N - 1] >= dist3d)) {
                r.t = t; r.p = p; r.ok = 1; r.isWall = 1;
                return r;
            }
        } else {
            /* Ground/low intercept */
            d  = flat_dist(c->pos, p) - (BALL_R + 1.0f);
            tt = t - fabsf(bot_angle_to(c, p)) * 0.30f;
            k  = (int)(tt / PRED_DT);
            if (d <= 0.0f || (k >= 0 && cov[k < PRED_N ? k : PRED_N - 1] >= d)) {
                r.t = t; r.p = p; r.ok = 1;
                return r;
            }
        }
    }
    return r;
}

/* Intelligent boost pad routing: finds the best pad that adds minimal detour to target */
static int best_pad(const Car *c, Vector3 target, float maxDetour)
{
    int i, best = -1;
    float bestScore = 1e9f;
    for (i = 0; i < g_botPadCount; i++) {
        float dToPad, dPadToTgt, directDist, detour, score;
        if (g_botPadTimer[i] > 0.5f) continue; /* must be active or about to respawn */
        dToPad = flat_dist(c->pos, g_botPads[i]);
        dPadToTgt = flat_dist(g_botPads[i], target);
        directDist = flat_dist(c->pos, target);
        detour = (dToPad + dPadToTgt) - directDist;
        if (detour < maxDetour && dToPad < 65.0f) {
            score = detour + dToPad * 0.35f;
            if (score < bestScore) {
                bestScore = score;
                best = i;
            }
        }
    }
    return best;
}

/* team: 0 = blue (attacks +Z), 1 = orange (attacks -Z) */
static Input bot_think(int self, Car *cars, const int *team, int n, const Ball *b, Bot *bt, int skill, float dt)
{
    Car *c = &cars[self];
    Input in;
    const float as = team[self] == 0 ? 1.0f : -1.0f;
    const Vector3 goal = V3(0, 0, as * ARENA_L), own = V3(0, 0, -as * ARENA_L);
    const float maxH = skill == 0 ? 3.0f : skill == 1 ? 8.5f : 20.0f;
    float speed = Vector3Length(c->vel), vf = Vector3DotProduct(c->vel, car_fwd(c));
    float bestT = 1e9f, defBest = 1e9f, arriveT = 0.0f, ang, dTarget, wUp, align = 0.0f;
    int onGround = c->wheelsOnGround >= 3, i, attacker = self, defender = -1, shooting = 0, danger = 0;
    int kickoff = fabsf(b->pos.x) < 0.01f && fabsf(b->pos.z) < 0.01f && Vector3Length(b->vel) < 0.1f;
    Intercept me = find_intercept(c, skill >= 1, maxH, skill);
    Vector3 target, I, dir = V3(0, 0, as);
    memset(&in, 0, sizeof(in));

    bt->dashCooldown -= dt;

    /* if a high ball can drop to a clean bumper height very soon, wait unless an aerial or wall ride is better */
    if (me.ok && me.p.y > 2.8f && !me.isWall && (skill == 0 || c->boost < 0.35f)) {
        Intercept low = find_intercept(c, skill >= 1, 2.8f, skill);
        if (low.ok && low.t < me.t + 1.2f) me = low;
    }
    I = me.p;
    if (kickoff) {
        if (bt->kickJitter == 0.0f) bt->kickJitter = (float)GetRandomValue(-100, 100) / 100.0f + 0.001f;
        I.x += bt->kickJitter * 0.9f;
    }

    /* is the ball about to go into our net? */
    for (i = 0; i < (int)(3.0f / PRED_DT); i++)
        if (g_pred[i].z * as < -(ARENA_L - 1.0f) && fabsf(g_pred[i].x) < GOAL_HW + 2.5f) { danger = 1; break; }

    /* --- roles: fastest to the ball attacks (with hysteresis), nearest our goal defends --- */
    for (i = 0; i < n; i++) {
        Intercept it;
        float tq;
        if (team[i] != team[self]) continue;
        it = (i == self) ? me : find_intercept(&cars[i], cars[i].boost > 0.0f, (i == self ? maxH : 8.5f), skill);
        tq = it.t;
        if ((it.p.z - cars[i].pos.z) * as < -1.0f && !danger) tq += 1.2f;
        if (i == self && bt->role == 0) tq -= 0.35f;   /* hysteresis keeps attacker focused */
        if (tq < bestT) { bestT = tq; attacker = i; }
    }
    for (i = 0; i < n; i++) {
        float dg;
        if (team[i] != team[self] || i == attacker) continue;
        dg = flat_dist(cars[i].pos, own);
        if (dg < defBest) { defBest = dg; defender = i; }
    }
    bt->role = (self == attacker) ? 0 : ((self == defender) ? 1 : 2);
    if (danger && (self == defender || me.t < bestT + 1.2f)) bt->role = 0;   /* everyone scrambles to save dangerous balls */

    /* --- choose target position in 3D ----------------------------------------- */
    if (bt->role == 0) {
        if (me.isWall) {
            /* Wall attack: drive directly to the ball on the wall */
            target = I;
            arriveT = me.ok ? me.t : 0.0f;
            shooting = 1;
        } else if (me.isAerial && skill >= 1 && c->boost >= 0.35f) {
            /* Aerial attack: rocket flight directly to aerial interception point */
            target = I;
            arriveT = me.t;
            shooting = 1;
        } else {
            /* Ground/low shot: aim for opponent goal corners */
            float cornerX = (I.x > 0.0f ? 1.0f : -1.0f) * (GOAL_HW - 2.5f);
            Vector3 aim = V3(cornerX, 1.6f, goal.z);
            Vector3 carToI = flat_dir(c->pos, I);
            dir = flat_dir(I, aim);
            if (danger || (I.z - own.z) * as < 40.0f) {
                /* Defensive clear: hit hard away from own goal */
                Vector3 away = flat_dir(own, I);
                dir = Vector3Normalize(Vector3Add(Vector3Scale(away, 0.7f), Vector3Scale(dir, 0.3f)));
            }
            align = Vector3DotProduct(carToI, dir);
            if (align >= -0.2f) {
                /* Aggressive attack directly through the ball */
                Vector3 contact = Vector3Subtract(I, Vector3Scale(dir, BALL_R + 0.8f));
                target = contact;
                arriveT = me.ok ? me.t : 0.0f;
                shooting = 1;
            } else {
                /* Behind the ball: cut in aggressively or tight loop */
                float d = flat_dist(c->pos, I);
                if (d < 12.0f) {
                    target = Vector3Subtract(I, Vector3Scale(carToI, BALL_R + 0.5f));
                    shooting = 1; arriveT = me.t;
                } else {
                    float side = c->pos.x > I.x ? 1.0f : -1.0f;
                    target = V3(I.x + side * 6.0f, 0, I.z - as * 6.0f);
                    shooting = 0;
                }
            }
        }
        /* Boost routing: if low on boost and not in an immediate shot, route via active pad */
        if (!kickoff && skill >= 1 && c->boost < 1.6f && !me.isAerial && !me.isWall) {
            int p = best_pad(c, target, (c->boost < 0.6f ? 28.0f : 14.0f));
            if (p >= 0) target = g_botPads[p];
        }
    } else if (bt->role == 1) {
        /* Goalkeeper / Last Man: defend goal, challenge when ball enters our half */
        if (danger || (b->pos.z - own.z) * as < 55.0f) {
            target = I;
            shooting = 1;
            arriveT = me.ok ? me.t : 0.0f;
        } else {
            Vector3 toBall = flat_dir(own, b->pos);
            target = Vector3Add(own, Vector3Scale(toBall, 22.0f));
            if (skill >= 1 && c->boost < 1.8f) {
                int p = best_pad(c, target, 35.0f);
                if (p >= 0) target = g_botPads[p];
            }
        }
    } else {
        /* Support / Second Man: shadow play closely (18-24 m), pounce on rebounds */
        target = V3(b->pos.x * 0.65f, 0, b->pos.z - as * 20.0f);
        if (skill >= 1 && c->boost < 2.0f) {
            int p = best_pad(c, target, 35.0f);
            if (p >= 0) target = g_botPads[p];
        }
    }

    /* Arena boundary clamping: allow full height and full wall bounds */
    if (me.isWall || c->pos.y > 2.0f || fabsf(c->pos.x) > 60.0f) {
        target.x = clampf(target.x, -ARENA_W + 1.2f, ARENA_W - 1.2f);
        target.z = clampf(target.z, -ARENA_L - GOAL_D + 2.0f, ARENA_L + GOAL_D - 2.0f);
        target.y = clampf(target.y, 0.3f, ARENA_H - 2.0f);
    } else {
        target.x = clampf(target.x, -ARENA_W + 3.0f, ARENA_W - 3.0f);
        target.z = clampf(target.z, -ARENA_L - 4.0f, ARENA_L + 4.0f);
    }

    /* --- drive & steering ----------------------------------------------------- */
    ang = bot_angle_to(c, target);
    dTarget = (target.y > 2.0f || c->pos.y > 2.0f) ? Vector3Distance(c->pos, target) : flat_dist(c->pos, target);
    wUp = Vector3DotProduct(c->angVel, car_up(c));
    in.steer = clampf(-(ang - wUp * 0.12f) * (skill == 0 ? 2.5f : 4.5f), -1.0f, 1.0f);
    in.throttle = skill == 0 ? 0.85f : 1.0f;

    /* 50/50 challenge: if an opponent is also rushing the ball, challenge aggressively! */
    int challenge5050 = 0;
    for (i = 0; i < n; i++) {
        if (team[i] != team[self] && Vector3Distance(cars[i].pos, b->pos) < 18.0f &&
            Vector3Distance(c->pos, b->pos) < 22.0f) {
            challenge5050 = 1;
            break;
        }
    }

    /* --- powerslide to dramatically decrease turn radius on sharp turns or cutbacks --- */
    if (onGround && skill >= 1) {
        float turnMag = fabsf(ang);
        int onWall = (car_up(c).y < 0.85f);
        /* Powerslide when needing to turn sharply:
         * At high speeds (> 16 m/s): steer angle is limited by MaxSteerAngleCurve, so turns > 26 deg require slide
         * At mid speeds (> 8 m/s): turns > 38 deg require slide
         * At lower speeds (> 2.5 m/s): turns > 52 deg whip around tightly */
        float slideReq = (speed > 16.0f) ? 0.45f : (speed > 8.0f ? 0.65f : 0.90f);
        if (onWall) slideReq = 1.15f;   /* preserve tyre adhesion on steep walls */
        if (turnMag > slideReq && speed > 2.5f) {
            in.slide = 1;
            in.steer = -signf(ang);   /* full steering lock while sliding */
            /* Feather throttle on severe cutbacks (> 110 deg) to let rear whip without washing wide */
            if (turnMag > 1.9f && speed > 13.0f) in.throttle = 0.5f;
        }
    }

    /* stuck against wall or post: reverse out */
    if (onGround && speed < 1.5f && in.throttle > 0.0f) bt->stuckT += dt; else bt->stuckT = 0.0f;
    if (bt->stuckT > 0.8f) { bt->reverseT = 0.6f; bt->stuckT = 0.0f; }
    if (bt->reverseT > 0.0f) { bt->reverseT -= dt; in.throttle = -1.0f; in.steer = -in.steer; in.boost = 0; in.slide = 0; }

    /* --- boost usage ---------------------------------------------------------- */
    if (skill >= 1) {
        if (kickoff) {
            in.throttle = 1.0f;
            in.boost = (fabsf(ang) < 0.45f && c->boost > 0.0f);
        } else if (challenge5050 && bt->role == 0) {
            in.throttle = 1.0f;
            if (fabsf(ang) < 0.50f && c->boost > 0.0f) in.boost = 1;
        } else if (me.isWall && c->pos.y < target.y - 2.0f && fabsf(ang) < 0.50f && c->boost > 0.15f) {
            /* Aggressive boost climb up the wall to beat gravity */
            in.boost = 1;
        } else if (shooting && arriveT > 0.0f) {
            float want = dTarget / fmaxf(arriveT, 0.05f);
            if (want < vf - 6.0f) in.throttle = 0.0f;   /* light tap, never full brake */
            if (fabsf(ang) < 0.45f && c->boost > 0.0f && (want > vf + 2.0f || dTarget > 14.0f))
                in.boost = 1;
        } else if (onGround && fabsf(ang) < 0.40f && c->boost > 0.0f && dTarget > 18.0f &&
                   (bt->role == 0 || danger || c->boost > 1.0f)) {
            in.boost = 1;
        }
        /* stop boosting if already at supersonic max speed */
        if (speed > CAR_MAX_SPEED - 2.0f) in.boost = 0;
    }

    /* --- action triggers: dashing in all directions, jumps, and aerials ------- */
    if (bt->action == BOT_ACT_NONE && (onGround || c->pos.y > 2.0f) && skill >= 1 && bt->dashCooldown <= 0.0f && c->landTimer <= 0.0f) {
        float dI = Vector3Distance(c->pos, I), angI = bot_angle_to(c, I), h = I.y;
        int onWallCar = (car_up(c).y < 0.85f && c->wheelsOnGround >= 3);
        int canStrike = (challenge5050 || danger || me.isWall || (shooting && align > 0.40f));
        float jumpReachDist = fmaxf(2.8f, vf * 0.22f + 0.6f);

        if (me.isAerial && skill >= 1 && c->boost >= 0.40f && fabsf(angI) < 0.40f && !onWallCar && dI < 28.0f && me.t < 1.8f) {
            /* Rocket flight / aerial in 3D: only for genuinely high balls when aligned and close */
            bt->action = BOT_ACT_AERIAL;
            bt->actionT = 0.0f;
            bt->aerialTarget = me.p;
            bt->dashCooldown = 2.5f;
        } else if (onWallCar && me.isWall && dI < 4.8f) {
            /* On wall: strike dodge directly into the wall ball */
            Vector3 toB = car_to_local(c, Vector3Subtract(I, c->pos));
            float mag = sqrtf(toB.x * toB.x + toB.z * toB.z);
            float nx = mag > 0.1f ? clampf(toB.z / mag, -1.0f, 1.0f) : 0.0f;
            float ny = mag > 0.1f ? clampf(toB.x / mag, -1.0f, 1.0f) : 1.0f;
            bt->action = BOT_ACT_DODGE;
            bt->actionT = 0.0f;
            bt->dodgeDir = (Vector2){ nx, -ny };
            bt->dashCooldown = 2.0f;
        } else if (onGround && (bt->role == 0 || danger) && h > 3.2f && h <= 4.8f && fabsf(angI) < 0.25f && dI < jumpReachDist && !me.isWall) {
            /* Single jump for mid balls: timed to hit ball at jump apex! (balls <= 3.2m hit cleanly on ground) */
            bt->action = BOT_ACT_JUMP;
            bt->actionT = 0.0f;
            bt->dashCooldown = 1.8f;
        } else if (onGround && (bt->role == 0 || danger) && h > 4.8f && h <= 7.2f && skill >= 2 && fabsf(angI) < 0.22f && dI < jumpReachDist && !me.isWall) {
            /* Double jump: timed to hit ball at double-jump apex! */
            bt->action = BOT_ACT_DJUMP;
            bt->actionT = 0.0f;
            bt->dashCooldown = 2.2f;
        } else if (onGround && bt->role == 0 && canStrike && dI < 4.4f &&
                   (h <= 3.2f || me.isWall) && speed > 8.0f && fabsf(angI) < 0.40f) {
            /* Directional strike dodge directly into the ball at contact */
            Vector3 toB = car_to_local(c, Vector3Subtract(I, c->pos));
            float mag = sqrtf(toB.x * toB.x + toB.z * toB.z);
            float nx = mag > 0.1f ? clampf(toB.z / mag, -1.0f, 1.0f) : 0.0f;
            float ny = mag > 0.1f ? clampf(toB.x / mag, -1.0f, 1.0f) : 1.0f;
            bt->action = BOT_ACT_DODGE;
            bt->actionT = 0.0f;
            bt->dodgeDir = (Vector2){ nx, -ny };
            bt->dashCooldown = 2.5f;
        } else if (onGround && bt->role == 0 && dI < 4.0f && fabsf(angI) > 0.85f && fabsf(angI) < 1.9f && speed > 9.0f) {
            /* Side flip: lateral cut-off strike when ball crosses beside the car */
            bt->action = BOT_ACT_DODGE;
            bt->actionT = 0.0f;
            bt->dodgeDir = (Vector2){ angI > 0 ? -1.0f : 1.0f, -0.3f };
            bt->dashCooldown = 3.0f;
        } else if (onGround && (bt->role == 1 || danger) && dTarget > 22.0f && fabsf(ang) > 2.5f && speed < 8.0f) {
            /* Backward flip to rapidly retreat toward own half */
            bt->action = BOT_ACT_DODGE;
            bt->actionT = 0.0f;
            bt->dodgeDir = (Vector2){ 0.0f, 1.0f };
            bt->dashCooldown = 4.0f;
        } else if (onGround && vf > 10.0f && dTarget > 35.0f && fabsf(ang) < 0.25f && c->boost < 0.35f && speed < 35.0f) {
            /* Downfield speed-flip dash only when empty on boost */
            bt->action = BOT_ACT_DODGE;
            bt->actionT = 0.0f;
            bt->dodgeDir = (Vector2){ clampf(-ang * 1.5f, -0.6f, 0.6f), -1.0f };
            bt->dashCooldown = 5.0f;
        }
    }

    /* --- execute current action ----------------------------------------------- */
    if (bt->action == BOT_ACT_JUMP) {
        float t = bt->actionT;
        in.slide = 0;
        in.jump = t < 0.20f;
        in.jumpPressed = (t == 0.0f);
        bt->actionT += dt;
        if (bt->actionT > 1.0f || (t > 0.35f && onGround)) {
            bt->action = BOT_ACT_NONE;
            bt->dashCooldown = 1.2f;
        }
    } else if (bt->action == BOT_ACT_DJUMP) {
        float t = bt->actionT;
        in.slide = 0;
        in.jump = t < 0.20f;
        if (t == 0.0f) {
            in.jumpPressed = 1;
        } else if (t >= 0.22f && !c->hasFlipped && c->wheelsOnGround == 0) {
            in.jumpPressed = 1;
            in.jump = 1;
            in.pitch = 0.0f;
            in.steer = 0.0f;
        }
        bt->actionT += dt;
        if (bt->actionT > 1.5f || (t > 0.40f && onGround)) {
            bt->action = BOT_ACT_NONE;
            bt->dashCooldown = 1.5f;
        }
    } else if (bt->action == BOT_ACT_DODGE) {
        float t = bt->actionT;
        in.slide = 0;
        in.jump = t < 0.06f;
        if (!c->hasFlipped) {
            if (t == 0.0f) {
                in.jumpPressed = 1;
            } else if (t >= 0.08f && c->wheelsOnGround == 0) {
                in.jumpPressed = 1;
                in.pitch = bt->dodgeDir.y;
                in.steer = bt->dodgeDir.x;
            }
        }
        bt->actionT += dt;
        if (bt->actionT > 0.85f || (t > 0.35f && onGround)) {
            bt->action = BOT_ACT_NONE;
            bt->dashCooldown = 1.5f;
        }
    } else if (bt->action == BOT_ACT_AERIAL) {
        /* Rocket boost flight in 3D */
        float t = bt->actionT;
        Vector3 toTgt = Vector3Subtract(bt->aerialTarget, c->pos);
        float dist3d = Vector3Length(toTgt);
        float tRem = fmaxf(0.1f, me.t - t);
        Vector3 vDes = Vector3Add(Vector3Scale(toTgt, 1.0f / tRem), V3(0, 0.5f * GRAVITY * tRem, 0));
        Vector3 dV = Vector3Subtract(vDes, c->vel);
        Vector3 tDir = Vector3Length(dV) > 0.1f ? Vector3Normalize(dV) : V3(0, 1, 0);
        Vector3 L = car_to_local(c, tDir);
        Vector3 w = car_to_local(c, c->angVel);
        Vector3 u = car_to_local(c, V3(0, 1, 0));
        float pitchErr, yawErr;
        int isHigh = (bt->aerialTarget.y > 5.5f);

        in.slide = 0;
        /* Fast aerial launch:
         * 1st jump off ground at t < 0.20s.
         * For high balls (> 5.5m), 2nd jump double-tap with neutral stick at t in [0.10s, 0.15s]
         * so it adds pure vertical impulse without triggering a backflip. */
        in.jump = (t < 0.20f);
        if (t == 0.0f) {
            in.jumpPressed = 1;
        } else if (isHigh && !c->hasFlipped && t >= 0.10f && t < 0.16f && c->wheelsOnGround == 0) {
            in.jumpPressed = 1;
            in.pitch = 0.0f;
            in.steer = 0.0f;
        }

        /* Vector flight attitude: pitch, yaw, roll to point nose along thrust vector */
        if (!in.jumpPressed) {
            pitchErr = atan2f(L.y, L.x);
            yawErr   = atan2f(L.z, L.x);
            in.pitch = clampf(pitchErr * 4.2f - w.z * 0.24f, -1.0f, 1.0f);
            in.yaw   = clampf(yawErr * 4.2f + w.y * 0.24f, -1.0f, 1.0f);
            in.roll  = clampf(u.z * 2.8f - w.x * 0.20f, -1.0f, 1.0f);
        }

        /* Burn rocket boost when nose aligns with flight path */
        if (L.x > 0.35f && c->boost > 0.0f && t > 0.06f) in.boost = 1;

        /* Aerial strike when reaching ball: if flip still available, dodge through ball */
        if (dist3d < 4.0f) {
            if (!c->hasFlipped) {
                in.jumpPressed = 1;
                in.pitch = -1.0f;
            }
            bt->action = BOT_ACT_NONE;
            bt->dashCooldown = 1.5f;
        }
        bt->actionT += dt;
        if (bt->actionT > 3.0f || c->boost <= 0.0f || (t > 0.40f && onGround)) {
            bt->action = BOT_ACT_NONE;
            bt->dashCooldown = 1.5f;
        }
    }

    /* --- air recovery: land on wheels, nose towards target when not flying ---- */
    if (!onGround && c->flipTimer <= 0.0f && bt->action != BOT_ACT_DODGE && bt->action != BOT_ACT_AERIAL) {
        Vector3 u = car_to_local(c, V3(0, 1, 0)), w = car_to_local(c, c->angVel);
        in.pitch = clampf(-u.x * 3.0f - w.z * 0.35f, -1.0f, 1.0f);
        in.roll  = (u.y < 0.0f && fabsf(u.z) < 0.3f) ? 1.0f : clampf(u.z * 3.0f - w.x * 0.3f, -1.0f, 1.0f);
        in.yaw   = clampf(-ang * 1.5f, -1.0f, 1.0f);
    }

    /* stuck on roof or side: jump to self-right */
    bt->recoverT -= dt;
    if (c->stuckTimer > 0.15f && bt->recoverT <= 0.0f) {
        in.jumpPressed = 1; in.jump = 1; bt->recoverT = 0.5f; bt->action = BOT_ACT_NONE;
    }

    return in;
}


/* ------------------------------------------------------------------------ */
/* Settings (settings.ini next to the exe)                                    */
/* ------------------------------------------------------------------------ */
static const char *CAR_SKIN_NAMES[] = {
    "Synthwave",     /* octane */
    "Hellfire",      /* backfire */
    "Gold Rush",     /* scarab */
    "Stealth Jet",   /* aftershock */
    "Desert Camo",   /* renegade */
    "Cyberpunk",     /* zippy */
    "Urban Hazard"   /* marauder */
};
static const float MATCH_LENGTHS[] = { 180.0f, 300.0f, 600.0f, 0.0f };   /* 0 = unlimited */
static const char *MATCH_NAMES[]   = { "3 min", "5 min", "10 min", "Unlimited" };

typedef struct Settings {
    float camDist, camHeight, fov;      /* metres, height/distance ratio, vertical degrees */
    int   boostFov, matchIdx, invertPitch, fullscreen, showFps, showHints, car;
    int   mode, botSkill;               /* MODE_NAMES / SKILL_NAMES index */
    int   shadows, bloom;               /* graphics toggles */
    int   skin;                         /* 0 = Team, 1 = Custom */
} Settings;
static Settings g_set = { 4.6f, 0.44f, 75.0f, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1 };

static void settings_path(char *out, size_t n) {
    if (FileExists("settings.ini")) snprintf(out, n, "settings.ini");
    else snprintf(out, n, "%ssettings.ini", GetApplicationDirectory());
}

static void settings_load(void)
{
    char path[600], line[160], key[64], sv[64];
    int i;
    FILE *f;
    settings_path(path, sizeof(path));
    if (!(f = fopen(path, "r"))) return;
    while (fgets(line, sizeof(line), f)) {
        float v;
        if (sscanf(line, " %63[^= ] = %63s", key, sv) != 2) continue;
        v = (float)atof(sv);
        if      (!strcmp(key, "camera_distance")) g_set.camDist     = clampf(v, 3.0f, 14.0f);
        else if (!strcmp(key, "camera_height"))   g_set.camHeight   = clampf(v, 0.1f, 0.8f);
        else if (!strcmp(key, "fov"))             g_set.fov         = clampf(v, 45.0f, 90.0f);
        else if (!strcmp(key, "boost_fov"))       g_set.boostFov    = v != 0;
        else if (!strcmp(key, "match_length"))    g_set.matchIdx    = (int)clampf(v, 0, 3);
        else if (!strcmp(key, "invert_pitch"))    g_set.invertPitch = v != 0;
        else if (!strcmp(key, "fullscreen"))      g_set.fullscreen  = v != 0;
        else if (!strcmp(key, "show_fps"))        g_set.showFps     = v != 0;
        else if (!strcmp(key, "show_hints"))      g_set.showHints   = v != 0;
        else if (!strcmp(key, "mode"))            g_set.mode        = (int)clampf(v, 0, 3);
        else if (!strcmp(key, "bot_skill"))       g_set.botSkill    = (int)clampf(v, 0, 2);
        else if (!strcmp(key, "shadows"))         g_set.shadows     = v != 0;
        else if (!strcmp(key, "bloom"))           g_set.bloom       = v != 0;
        else if (!strcmp(key, "skin"))            g_set.skin        = (int)clampf(v, 0, 1);
        else if (!strcmp(key, "car"))
            for (i = 0; i < CAR_COUNT; i++) if (!strcmp(sv, CAR_NAMES[i])) g_set.car = i;
    }
    fclose(f);
}

static void settings_save(void)
{
    char path[600];
    FILE *f;
    settings_path(path, sizeof(path));
    if (!(f = fopen(path, "w"))) return;
    fprintf(f, "camera_distance=%.2f\ncamera_height=%.2f\nfov=%.0f\nboost_fov=%d\nmatch_length=%d\n"
               "invert_pitch=%d\nfullscreen=%d\nshow_fps=%d\nshow_hints=%d\ncar=%s\nmode=%d\nbot_skill=%d\n"
               "shadows=%d\nbloom=%d\nskin=%d\n",
            g_set.camDist, g_set.camHeight, g_set.fov, g_set.boostFov, g_set.matchIdx,
            g_set.invertPitch, g_set.fullscreen, g_set.showFps, g_set.showHints, CAR_NAMES[g_set.car],
            g_set.mode, g_set.botSkill, g_set.shadows, g_set.bloom, g_set.skin);
    fclose(f);
}

/* ------------------------------------------------------------------------ */
/* Input                                                                      */
/* ------------------------------------------------------------------------ */
static Input read_input(void)
{
    static int trigSeen[2] = { 0, 0 };
    Input in;
    float kx, ky, roll;
    memset(&in, 0, sizeof(in));

    kx = (float)(IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) - (float)(IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT));
    ky = (float)(IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN))  - (float)(IsKeyDown(KEY_W) || IsKeyDown(KEY_UP));
    roll = (float)IsKeyDown(KEY_E) - (float)IsKeyDown(KEY_Q);
    in.throttle    = -ky;
    in.steer       = kx;
    in.pitch       = ky;
    in.jump        = IsKeyDown(KEY_SPACE) || IsMouseButtonDown(MOUSE_BUTTON_LEFT);
    in.jumpPressed = IsKeyPressed(KEY_SPACE) || IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
    in.boost       = IsKeyDown(KEY_LEFT_SHIFT) || IsMouseButtonDown(MOUSE_BUTTON_RIGHT);
    in.slide       = IsKeyDown(KEY_LEFT_CONTROL);

    if (IsGamepadAvailable(0)) {
        float gx = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X);
        float gy = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y);
        float rt = GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_TRIGGER);
        float lt = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_TRIGGER);
        /* triggers can report 0 until first touched; only trust them once seen at rest (-1) */
        if (rt < -0.5f) trigSeen[0] = 1;
        if (lt < -0.5f) trigSeen[1] = 1;
        rt = trigSeen[0] ? (rt + 1.0f) * 0.5f : 0.0f;
        lt = trigSeen[1] ? (lt + 1.0f) * 0.5f : 0.0f;
        if (fabsf(gx) < 0.12f) gx = 0;
        if (fabsf(gy) < 0.12f) gy = 0;
        if (gx != 0) in.steer = gx;
        if (gy != 0) in.pitch = gy;
        if (rt > 0.05f || lt > 0.05f) in.throttle = rt - lt;
        in.jump        |= IsGamepadButtonDown(0, GAMEPAD_BUTTON_RIGHT_FACE_DOWN);
        in.jumpPressed |= IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_FACE_DOWN);
        in.boost       |= IsGamepadButtonDown(0, GAMEPAD_BUTTON_RIGHT_FACE_RIGHT);
        in.slide       |= IsGamepadButtonDown(0, GAMEPAD_BUTTON_RIGHT_FACE_LEFT);
        if (IsGamepadButtonDown(0, GAMEPAD_BUTTON_LEFT_TRIGGER_1))  roll -= 1.0f;
        if (IsGamepadButtonDown(0, GAMEPAD_BUTTON_RIGHT_TRIGGER_1)) roll += 1.0f;
    }

    if (g_set.invertPitch) in.pitch = -in.pitch;

    /* powerslide button doubles as air roll (steer -> roll) */
    if (in.slide) { in.roll = in.steer; in.yaw = 0.0f; }
    else          { in.yaw = in.steer; }
    if (roll != 0.0f) in.roll = clampf(roll, -1, 1);
    return in;
}

/* ------------------------------------------------------------------------ */
/* Rendering                                                                  */
/* ------------------------------------------------------------------------ */
/* ---- lighting + post-processing ------------------------------------------
 * scene -> RGBA16F HDR target (linear light: sun + sky ambient + specular +
 * fresnel sky reflection + point lights + 4096^2 PCF shadow map + fog), then
 * bloom mip chain -> ACES tonemap / grade / vignette -> FXAA -> backbuffer.  */
#define MAX_PT_LIGHTS 16
#define SHADOW_RES    4096
#define BLOOM_MIPS    6
#define SHADOW_NEAR_RES  2048
#define SHADOW_NEAR_SIZE 56.0f      /* metres covered by the sharp cascade around the player */
static const Vector3 SUN_DIR = { -0.38f, -0.78f, 0.50f };   /* crisp daytime stadium sun */

/* shared by the lit + sky shaders: authentic UE3 daytime sky, linear HDR */
#define SKY_GLSL \
    "uniform vec3 sunDir;\n" \
    "vec3 skyColor(vec3 d){\n" \
    "  float y = d.y;\n" \
    "  vec3 zen = vec3(0.08, 0.26, 0.68), mid = vec3(0.32, 0.54, 0.84), hor = vec3(0.72, 0.82, 0.92), c;\n" \
    "  if (y >= 0.0) c = mix(mix(hor, mid, smoothstep(0.0, 0.22, y)), zen, smoothstep(0.16, 0.85, y));\n" \
    "  else c = mix(hor * 0.65, vec3(0.14, 0.16, 0.20), clamp(-y * 5.0, 0.0, 1.0));\n" \
    "  float s = max(dot(d, -sunDir), 0.0);\n" \
    "  c += vec3(1.70, 1.45, 1.10) * (pow(s, 1800.0) * 60.0 + pow(s, 32.0) * 1.5 + pow(s, 6.0) * 0.35);\n" \
    "  float az = atan(d.z, d.x);\n" \
    "  if (y > 0.02) {\n" \
    "    vec2 uv = d.xz / (y + 0.10) * 0.50;\n" \
    "    float n1 = sin(uv.x * 2.0 + cos(uv.y * 1.5)) * 0.5 + 0.5;\n" \
    "    float n2 = sin(uv.x * 4.6 - uv.y * 3.8 + n1 * 3.2) * 0.5 + 0.5;\n" \
    "    float n3 = sin(uv.x * 9.5 + uv.y * 8.2) * 0.5 + 0.5;\n" \
    "    float cloud = smoothstep(0.48, 0.78, n1 * 0.55 + n2 * 0.32 + n3 * 0.13) * smoothstep(0.02, 0.15, y);\n" \
    "    vec3 cloudCol = mix(vec3(0.65, 0.70, 0.82), vec3(1.45, 1.40, 1.30), pow(max(dot(normalize(vec3(d.x, 0.35, d.z)), -sunDir), 0.0), 3.0) * 0.8 + 0.25);\n" \
    "    c = mix(c, cloudCol, cloud * 0.90);\n" \
    "  }\n" \
    "  float towerAngle = abs(fract((az + 0.785398) / 1.570796) - 0.5) * 1.570796;\n" \
    "  float inTowerY = smoothstep(0.12, 0.20, y) * smoothstep(0.42, 0.30, y);\n" \
    "  float towerLight = pow(max(1.0 - towerAngle * 10.0, 0.0), 3.5) * inTowerY;\n" \
    "  c += vec3(2.2, 2.3, 2.6) * (towerLight * 16.0);\n" \
    "  float mast = smoothstep(0.015, 0.005, towerAngle) * smoothstep(0.05, 0.12, y) * smoothstep(0.38, 0.30, y);\n" \
    "  c = mix(c, vec3(0.12, 0.14, 0.18), mast * 0.85);\n" \
    "  float beam = pow(max(1.0 - towerAngle * 4.5, 0.0), 4.0) * smoothstep(0.02, 0.22, y) * smoothstep(0.42, 0.26, y);\n" \
    "  c += vec3(0.6, 0.7, 0.9) * beam * 0.8;\n" \
    "  float mElev = 0.075 + 0.038 * sin(az * 3.0 + 1.2) + 0.020 * sin(az * 7.0 - 0.5) + 0.012 * cos(az * 13.0);\n" \
    "  if (y < mElev && y > 0.0) {\n" \
    "    float mFog = clamp(y / mElev, 0.0, 1.0);\n" \
    "    vec3 mCol = mix(vec3(0.20, 0.26, 0.36), hor * 0.82, mFog * 0.65);\n" \
    "    c = mix(c, mCol, smoothstep(mElev + 0.008, mElev - 0.002, y));\n" \
    "  }\n" \
    "  float sRim = smoothstep(0.085, 0.075, y) * smoothstep(0.045, 0.060, y);\n" \
    "  c = mix(c, vec3(0.16, 0.20, 0.25), sRim * 0.85);\n" \
    "  float rimLights = sin(az * 45.0) > 0.5 ? 1.0 : 0.0;\n" \
    "  c += vec3(0.9, 0.7, 0.4) * sRim * rimLights * 1.2;\n" \
    "  return c; }\n"

static const char *LIT_VS =
    "#version 330\n"
    "in vec3 vertexPosition; in vec2 vertexTexCoord; in vec3 vertexNormal;\n"
    "uniform mat4 mvp; uniform mat4 matModel; uniform mat4 matNormal;\n"
    "out vec2 fragUV; out vec3 fragN; out vec3 fragPos;\n"
    "void main(){ fragUV = vertexTexCoord;\n"
    "  fragPos = (matModel * vec4(vertexPosition, 1.0)).xyz;\n"
    "  fragN = normalize((matNormal * vec4(vertexNormal, 0.0)).xyz);\n"
    "  gl_Position = mvp * vec4(vertexPosition, 1.0); }\n";

static const char *LIT_FS =
    "#version 330\n"
    "in vec2 fragUV; in vec3 fragN; in vec3 fragPos; out vec4 finalColor;\n"
    "uniform sampler2D texture0; uniform vec4 colDiffuse;\n"
    "uniform sampler2DShadow shadowMap, shadowNear; uniform mat4 lightVP, lightVPN; uniform int shadowOn;\n"
    "uniform float shadowBias, shadowBiasN, shadowTexel, shadowTexelN;\n"
    "uniform vec3 sunCol; uniform vec3 viewPos; uniform vec4 matParams;\n"   /* spec, shininess, reflection, emissive */
    "uniform vec4 ptPos[16]; uniform vec4 ptCol[16]; uniform int ptCount;\n"
    "uniform vec3 fogCol; uniform float fogDensity;\n"
    SKY_GLSL
    /* 3x3 taps of hardware 2x2 bilinear compares = smooth 4x4-ish PCF */
    "float pcf(sampler2DShadow m, vec3 c, float t){ float s = 0.0;\n"
    "  for (int y = -1; y <= 1; y++) for (int x = -1; x <= 1; x++) s += texture(m, vec3(c.xy + vec2(x, y) * t, c.z));\n"
    "  return s / 9.0; }\n"
    "float shadowTerm(vec3 p, vec3 n, float ndl){\n"
    "  if (shadowOn == 0) return 1.0;\n"
    "  float slope = 1.0 - ndl, far = 1.0, near = 1.0, e;\n"
    "  vec3 c = (lightVPN * vec4(p + n * (0.015 + 0.05 * slope), 1.0)).xyz * 0.5 + 0.5;\n"
    "  bool inNear;\n"
    "  e = max(abs(c.x - 0.5), abs(c.y - 0.5)) * 2.0;\n"
    "  inNear = e < 0.95 && c.z < 1.0;\n"
    "  if (inNear) { near = pcf(shadowNear, vec3(c.xy, c.z - shadowBiasN), shadowTexelN); if (e < 0.8) return near; }\n"
    "  c = (lightVP * vec4(p + n * (0.05 + 0.15 * slope), 1.0)).xyz * 0.5 + 0.5;\n"
    "  if (c.x > 0.0 && c.y > 0.0 && c.x < 1.0 && c.y < 1.0 && c.z < 1.0) far = pcf(shadowMap, vec3(c.xy, c.z - shadowBias), shadowTexel);\n"
    "  return inNear ? mix(near, far, smoothstep(0.8, 0.95, e)) : far; }\n"
    "void main(){ vec4 t = texture(texture0, fragUV);\n"
    "  if (t.a < 0.33) discard;\n"
    "  vec4 base = t * colDiffuse;\n"
    "  vec3 alb = pow(base.rgb, vec3(2.2));\n"
    "  vec3 n = normalize(gl_FrontFacing ? fragN : -fragN);\n"
    "  vec3 v = normalize(viewPos - fragPos), L = -sunDir;\n"
    "  float spec = matParams.x, shin = matParams.y, nv = max(dot(n, v), 0.0);\n"
    "  float ndl = max(dot(n, L), 0.0);\n"
    "  float sh = ndl > 0.0 ? shadowTerm(fragPos, n, ndl) : 0.0;\n"
    "  float norm = (shin + 8.0) / 25.13;\n"
    "  vec3 amb = mix(vec3(0.32, 0.30, 0.28), vec3(0.48, 0.54, 0.65), n.y * 0.5 + 0.5);\n"
    "  vec3 col = alb * amb;\n"
    "  col += sunCol * sh * ndl * (alb + spec * norm * pow(max(dot(n, normalize(L + v)), 0.0), shin));\n"
    "  for (int i = 0; i < ptCount; i++) {\n"
    "    vec3 d = ptPos[i].xyz - fragPos; float dist = length(d);\n"
    "    float a = clamp(1.0 - dist / ptPos[i].w, 0.0, 1.0); a *= a;\n"
    "    if (a <= 0.0) continue;\n"
    "    vec3 l = d / max(dist, 0.001); float nl = max(dot(n, l), 0.0);\n"
    "    col += ptCol[i].rgb * a * nl * (alb + spec * norm * pow(max(dot(n, normalize(l + v)), 0.0), shin));\n"
    "  }\n"
    "  float F = matParams.z * mix(0.04, 0.45, pow(1.0 - nv, 4.0));\n"
    "  col += skyColor(reflect(-v, n)) * F * (0.4 + 0.6 * max(sh, 0.35));\n"
    "  col += alb * matParams.w;\n"
    "  float alpha = base.a < 1.0 ? clamp(base.a * 0.7 + F * 0.25, 0.0, 1.0) : 1.0;\n"
    "  float fog = 1.0 - exp(-length(viewPos - fragPos) * fogDensity);\n"
    "  finalColor = vec4(mix(col, fogCol, fog), alpha); }\n";

static const char *DEPTH_VS =
    "#version 330\n"
    "in vec3 vertexPosition; in vec2 vertexTexCoord; uniform mat4 mvp; out vec2 fragUV;\n"
    "void main(){ fragUV = vertexTexCoord; gl_Position = mvp * vec4(vertexPosition, 1.0); }\n";
static const char *DEPTH_FS =
    "#version 330\n"
    "in vec2 fragUV; uniform sampler2D texture0; out vec4 finalColor;\n"
    "void main(){ if (texture(texture0, fragUV).a < 0.33) discard; finalColor = vec4(1.0); }\n";

static const char *SKY_VS =
    "#version 330\n"
    "in vec3 vertexPosition; uniform mat4 mvp; out vec3 dir;\n"
    "void main(){ dir = vertexPosition; gl_Position = mvp * vec4(vertexPosition, 1.0); }\n";
static const char *SKY_FS =
    "#version 330\n"
    "in vec3 dir; out vec4 finalColor;\n"
    SKY_GLSL
    "void main(){ finalColor = vec4(skyColor(normalize(dir)), 1.0); }\n";

/* rlgl batch geometry (pads, flame, nets): vertex colour in sRGB, scaled into HDR so it blooms */
static const char *EMIS_FS =
    "#version 330\n"
    "in vec2 fragTexCoord; in vec4 fragColor; out vec4 finalColor;\n"
    "uniform sampler2D texture0; uniform float intensity;\n"
    "void main(){ vec4 c = fragColor * texture(texture0, fragTexCoord);\n"
    "  finalColor = vec4(pow(c.rgb, vec3(2.2)) * intensity, c.a); }\n";

/* bloom: 4-tap bilinear downsample (first pass = soft-threshold prefilter) */
static const char *DOWN_FS =
    "#version 330\n"
    "in vec2 fragTexCoord; out vec4 finalColor;\n"
    "uniform sampler2D texture0; uniform vec2 texel; uniform int prefilter; uniform float threshold;\n"
    "void main(){ vec2 uv = fragTexCoord;\n"
    "  vec3 c = texture(texture0, uv).rgb * 0.5\n"
    "    + (texture(texture0, uv + texel * vec2(-1.0, -1.0)).rgb + texture(texture0, uv + texel * vec2(1.0, -1.0)).rgb\n"
    "     + texture(texture0, uv + texel * vec2(-1.0,  1.0)).rgb + texture(texture0, uv + texel * vec2(1.0,  1.0)).rgb) * 0.125;\n"
    "  if (prefilter == 1) { c = min(c, vec3(40.0));\n"
    "    float br = max(c.r, max(c.g, c.b)), knee = threshold * 0.5;\n"
    "    float soft = clamp(br - threshold + knee, 0.0, 2.0 * knee); soft = soft * soft / (4.0 * knee + 1e-4);\n"
    "    c *= max(soft, br - threshold) / max(br, 1e-4); }\n"
    "  finalColor = vec4(c, 1.0); }\n";

/* bloom: 9-tap tent upsample, blended additively onto the next larger mip */
static const char *UP_FS =
    "#version 330\n"
    "in vec2 fragTexCoord; out vec4 finalColor;\n"
    "uniform sampler2D texture0; uniform vec2 texel;\n"
    "void main(){ vec2 uv = fragTexCoord, o = texel;\n"
    "  vec3 c = texture(texture0, uv).rgb * 4.0;\n"
    "  c += (texture(texture0, uv + vec2(-o.x, 0.0)).rgb + texture(texture0, uv + vec2(o.x, 0.0)).rgb\n"
    "      + texture(texture0, uv + vec2(0.0, -o.y)).rgb + texture(texture0, uv + vec2(0.0, o.y)).rgb) * 2.0;\n"
    "  c += texture(texture0, uv - o).rgb + texture(texture0, uv + o).rgb\n"
    "     + texture(texture0, uv + vec2(o.x, -o.y)).rgb + texture(texture0, uv + vec2(-o.x, o.y)).rgb;\n"
    "  finalColor = vec4(c / 16.0, 1.0); }\n";

static const char *COMP_FS =
    "#version 330\n"
    "in vec2 fragTexCoord; out vec4 finalColor;\n"
    "uniform sampler2D texture0; uniform sampler2D bloomTex;\n"
    "uniform float exposure, bloomStr, vignette, aberration;\n"
    "void main(){ vec2 uv = fragTexCoord, dc = uv - 0.5;\n"
    "  vec3 h;\n"
    "  if (aberration > 0.0) {\n"
    "    vec2 dir = dc * aberration;\n"
    "    h  = texture(texture0, uv).rgb * 0.45;\n"
    "    h += texture(texture0, uv - dir * 0.6).rgb * 0.33;\n"
    "    h += texture(texture0, uv - dir * 1.2).rgb * 0.22;\n"
    "  } else {\n"
    "    h = texture(texture0, uv).rgb;\n"
    "  }\n"
    "  if (bloomStr > 0.0) h += texture(bloomTex, uv).rgb * bloomStr;\n"
    "  vec3 x = h * exposure;\n"
    "  vec3 c = (x * (1.0 + x * 0.11)) / (1.0 + x);\n"       /* UE3 extended Reinhard tonemap */
    "  float l = dot(c, vec3(0.2126, 0.7152, 0.0722));\n"
    "  c = max(mix(vec3(l), c, 1.08), 0.0);\n"                /* clean SARPBC arcade saturation */
    "  if (vignette > 0.0) c *= 1.0 - vignette * pow(length(dc * vec2(1.0, 0.85)) * 1.3, 2.0);\n"
    "  c = pow(clamp(c, 0.0, 1.0), vec3(1.0 / 2.2));\n"       /* DisplayGamma=2.2 */
    "  finalColor = vec4(c, dot(c, vec3(0.299, 0.587, 0.114))); }\n";             /* luma in alpha for FXAA */

static const char *FXAA_FS =
    "#version 330\n"
    "in vec2 fragTexCoord; out vec4 finalColor;\n"
    "uniform sampler2D texture0; uniform vec2 texel;\n"
    "void main(){ vec2 uv = fragTexCoord;\n"
    "  float nw = texture(texture0, uv + vec2(-1.0, -1.0) * texel).a, ne = texture(texture0, uv + vec2(1.0, -1.0) * texel).a;\n"
    "  float sw = texture(texture0, uv + vec2(-1.0,  1.0) * texel).a, se = texture(texture0, uv + vec2(1.0,  1.0) * texel).a;\n"
    "  vec4 m = texture(texture0, uv);\n"
    "  float lmin = min(m.a, min(min(nw, ne), min(sw, se))), lmax = max(m.a, max(max(nw, ne), max(sw, se)));\n"
    "  if (lmax - lmin < max(0.0312, lmax * 0.125)) { finalColor = vec4(m.rgb, 1.0); return; }\n"
    "  vec2 dir = vec2(-((nw + ne) - (sw + se)), (nw + sw) - (ne + se));\n"
    "  float red = max((nw + ne + sw + se) * 0.03125, 1.0 / 128.0);\n"
    "  dir = clamp(dir / (min(abs(dir.x), abs(dir.y)) + red), vec2(-8.0), vec2(8.0)) * texel;\n"
    "  vec3 a = 0.5 * (texture(texture0, uv - dir / 6.0).rgb + texture(texture0, uv + dir / 6.0).rgb);\n"
    "  vec3 b = a * 0.5 + 0.25 * (texture(texture0, uv - dir * 0.5).rgb + texture(texture0, uv + dir * 0.5).rgb);\n"
    "  float lb = dot(b, vec3(0.299, 0.587, 0.114));\n"
    "  finalColor = vec4((lb < lmin || lb > lmax) ? a : b, 1.0); }\n";

#ifdef _WIN32
__declspec(dllimport) void __stdcall glDrawBuffer(unsigned int mode);   /* GL 1.1, exported by opengl32 */
__declspec(dllimport) void __stdcall glReadBuffer(unsigned int mode);
__declspec(dllimport) void __stdcall glTexParameteri(unsigned int target, unsigned int pname, int param);
#endif

typedef struct Gfx {
    Shader lit, depth, sky, emis, down, up, comp, fxaa;
    int lMat, lView, lLightVP, lLightVPN, lShadowOn, lPtPos, lPtCol, lPtCount;
    int eInt, dTexel, dPre, dThr, uTexel, cBloom, cExp, cBloomStr, cVig, cAb, fTexel;
    RenderTexture2D shadow, shadowN, hdr, ldr, bloom[BLOOM_MIPS];
    int w, h;
    float depthRange, zNear, zFar;
    Matrix lightView, lightProj, lightProjN;
    Mesh skyMesh;
    Material skyMat;
    Vector4 ptPos[MAX_PT_LIGHTS], ptCol[MAX_PT_LIGHTS];
    int ptCount;
} Gfx;
static Gfx G;

static RenderTexture2D gfx_make_rt(int w, int h, int format, int depth)
{
    RenderTexture2D rt = { 0 };
    rt.id = rlLoadFramebuffer();
    rlEnableFramebuffer(rt.id);
    rt.texture.id = rlLoadTexture(NULL, w, h, format, 1);
    rt.texture.width = w; rt.texture.height = h; rt.texture.format = format; rt.texture.mipmaps = 1;
    rlFramebufferAttach(rt.id, rt.texture.id, RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_TEXTURE2D, 0);
    if (depth) {
        rt.depth.id = rlLoadTextureDepth(w, h, true);
        rt.depth.width = w; rt.depth.height = h;
        rlFramebufferAttach(rt.id, rt.depth.id, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_RENDERBUFFER, 0);
    }
    if (!rlFramebufferComplete(rt.id)) TraceLog(LOG_WARNING, "GFX: render target %dx%d incomplete", w, h);
    rlDisableFramebuffer();
    SetTextureFilter(rt.texture, TEXTURE_FILTER_BILINEAR);
    SetTextureWrap(rt.texture, TEXTURE_WRAP_CLAMP);
    return rt;
}

static void gfx_free_targets(void)
{
    int i;
    if (G.hdr.id) UnloadRenderTexture(G.hdr);
    if (G.ldr.id) UnloadRenderTexture(G.ldr);
    for (i = 0; i < BLOOM_MIPS; i++) if (G.bloom[i].id) UnloadRenderTexture(G.bloom[i]);
    G.hdr.id = G.ldr.id = 0;
    for (i = 0; i < BLOOM_MIPS; i++) G.bloom[i].id = 0;
}

/* (re)create the screen-sized targets when the window size changes */
static void gfx_resize(void)
{
    int w = GetRenderWidth(), h = GetRenderHeight(), i;
    if (w <= 0 || h <= 0 || (w == G.w && h == G.h)) return;
    gfx_free_targets();
    G.w = w; G.h = h;
    G.hdr = gfx_make_rt(w, h, PIXELFORMAT_UNCOMPRESSED_R16G16B16A16, 1);
    G.ldr = gfx_make_rt(w, h, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8, 0);
    for (i = 0; i < BLOOM_MIPS; i++) {
        int bw = w >> (i + 1), bh = h >> (i + 1);
        G.bloom[i] = gfx_make_rt(bw < 1 ? 1 : bw, bh < 1 ? 1 : bh, PIXELFORMAT_UNCOMPRESSED_R16G16B16A16, 0);
    }
}

static Shader gfx_shader(const char *vs, const char *fs)
{
    Shader s = LoadShaderFromMemory(vs, fs);
    if (!IsShaderValid(s)) TraceLog(LOG_WARNING, "GFX: shader failed to compile");
    return s;
}

/* depth-only FBO; the texture is set up for sampler2DShadow (compare mode + linear = 2x2 PCF in hardware) */
static RenderTexture2D gfx_make_shadow(int res)
{
    RenderTexture2D rt = { 0 };
    rt.id = rlLoadFramebuffer();
    rt.texture.id = rlLoadTextureDepth(res, res, false);
    rt.texture.width = rt.texture.height = res;
    rt.texture.format = PIXELFORMAT_UNCOMPRESSED_R32; rt.texture.mipmaps = 1;
    rt.depth = rt.texture;
    rlFramebufferAttach(rt.id, rt.texture.id, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
#ifdef _WIN32
    rlEnableFramebuffer(rt.id);
    glDrawBuffer(0); glReadBuffer(0);    /* GL_NONE: no colour attachment */
    rlDisableFramebuffer();
    rlEnableTexture(rt.texture.id);
    glTexParameteri(0x0DE1 /*GL_TEXTURE_2D*/, 0x884C /*GL_TEXTURE_COMPARE_MODE*/, 0x884E /*GL_COMPARE_REF_TO_TEXTURE*/);
    glTexParameteri(0x0DE1, 0x884D /*GL_TEXTURE_COMPARE_FUNC*/, 0x0203 /*GL_LEQUAL*/);
    glTexParameteri(0x0DE1, 0x2801 /*MIN_FILTER*/, 0x2601 /*GL_LINEAR*/);
    glTexParameteri(0x0DE1, 0x2800 /*MAG_FILTER*/, 0x2601);
    glTexParameteri(0x0DE1, 0x2802 /*WRAP_S*/, 0x812F /*CLAMP_TO_EDGE*/);
    glTexParameteri(0x0DE1, 0x2803 /*WRAP_T*/, 0x812F);
    rlDisableTexture();
#endif
    if (!rlFramebufferComplete(rt.id)) TraceLog(LOG_WARNING, "GFX: shadow framebuffer %d incomplete", res);
    return rt;
}

static void gfx_init(void)
{
    Vector3 sun = Vector3Normalize(SUN_DIR);
    Vector3 sunCol = { 1.55f, 1.45f, 1.30f };   /* authentic warm UE3 directional sun */
    Vector3 fogCol = { 0.68f, 0.75f, 0.85f };   /* atmospheric blue horizon haze */
    Vector3 lo = { 1e9f, 1e9f, 1e9f }, hi = { -1e9f, -1e9f, -1e9f }, ctr = { 0, 10, 0 };
    float fogD = 0.0007f, bias, texel = 1.0f / SHADOW_RES, thr = 0.85f;
    int slot = 10, k;
    memset(&G, 0, sizeof(G));

    G.lit = gfx_shader(LIT_VS, LIT_FS);
    G.lit.locs[SHADER_LOC_MATRIX_MODEL]  = GetShaderLocation(G.lit, "matModel");
    G.lit.locs[SHADER_LOC_MATRIX_NORMAL] = GetShaderLocation(G.lit, "matNormal");
    G.lMat      = GetShaderLocation(G.lit, "matParams");
    G.lView     = GetShaderLocation(G.lit, "viewPos");
    G.lLightVP  = GetShaderLocation(G.lit, "lightVP");
    G.lLightVPN = GetShaderLocation(G.lit, "lightVPN");
    G.lShadowOn = GetShaderLocation(G.lit, "shadowOn");
    G.lPtPos    = GetShaderLocation(G.lit, "ptPos");
    G.lPtCol    = GetShaderLocation(G.lit, "ptCol");
    G.lPtCount  = GetShaderLocation(G.lit, "ptCount");
    SetShaderValue(G.lit, GetShaderLocation(G.lit, "sunDir"), &sun, SHADER_UNIFORM_VEC3);
    SetShaderValue(G.lit, GetShaderLocation(G.lit, "sunCol"), &sunCol, SHADER_UNIFORM_VEC3);
    SetShaderValue(G.lit, GetShaderLocation(G.lit, "fogCol"), &fogCol, SHADER_UNIFORM_VEC3);
    SetShaderValue(G.lit, GetShaderLocation(G.lit, "fogDensity"), &fogD, SHADER_UNIFORM_FLOAT);
    SetShaderValue(G.lit, GetShaderLocation(G.lit, "shadowMap"), &slot, SHADER_UNIFORM_INT);
    SetShaderValue(G.lit, GetShaderLocation(G.lit, "shadowTexel"), &texel, SHADER_UNIFORM_FLOAT);

    G.depth = gfx_shader(DEPTH_VS, DEPTH_FS);
    G.sky   = gfx_shader(SKY_VS, SKY_FS);
    SetShaderValue(G.sky, GetShaderLocation(G.sky, "sunDir"), &sun, SHADER_UNIFORM_VEC3);
    G.emis  = gfx_shader(NULL, EMIS_FS);   G.eInt = GetShaderLocation(G.emis, "intensity");
    G.down  = gfx_shader(NULL, DOWN_FS);
    G.dTexel = GetShaderLocation(G.down, "texel"); G.dPre = GetShaderLocation(G.down, "prefilter");
    G.dThr   = GetShaderLocation(G.down, "threshold");
    SetShaderValue(G.down, G.dThr, &thr, SHADER_UNIFORM_FLOAT);
    G.up    = gfx_shader(NULL, UP_FS);     G.uTexel = GetShaderLocation(G.up, "texel");
    G.comp  = gfx_shader(NULL, COMP_FS);
    G.cBloom = GetShaderLocation(G.comp, "bloomTex"); G.cExp = GetShaderLocation(G.comp, "exposure");
    G.cBloomStr = GetShaderLocation(G.comp, "bloomStr"); G.cVig = GetShaderLocation(G.comp, "vignette");
    G.cAb = GetShaderLocation(G.comp, "aberration");
    G.fxaa  = gfx_shader(NULL, FXAA_FS);   G.fTexel = GetShaderLocation(G.fxaa, "texel");

    G.skyMesh = GenMeshSphere(900.0f, 24, 48);
    G.skyMat = LoadMaterialDefault();
    G.skyMat.shader = G.sky;

    /* shadow maps: depth-only FBOs sampled with hardware depth compare + bilinear (PCF) */
    G.shadow  = gfx_make_shadow(SHADOW_RES);
    G.shadowN = gfx_make_shadow(SHADOW_NEAR_RES);

    /* far cascade: fitted once to the whole arena; the near one follows the player (gfx_shadow_begin) */
    G.lightView = MatrixLookAt(Vector3Subtract(ctr, Vector3Scale(sun, 400.0f)), ctr, V3(0, 1, 0));
    for (k = 0; k < 8; k++) {
        Vector3 p = V3(k & 1 ? ARENA_W + 4 : -ARENA_W - 4, k & 2 ? ARENA_H + 2 : -2.0f,
                       k & 4 ? ARENA_L + GOAL_D + 4 : -ARENA_L - GOAL_D - 4);
        p = Vector3Transform(p, G.lightView);
        lo = Vector3Min(lo, p); hi = Vector3Max(hi, p);
    }
    G.zNear = -hi.z - 5.0f; G.zFar = -lo.z + 5.0f;
    G.lightProj = MatrixOrtho(lo.x, hi.x, lo.y, hi.y, G.zNear, G.zFar);
    G.lightProjN = G.lightProj;
    G.depthRange = (hi.z - lo.z) + 10.0f;
    {
        Matrix vp = MatrixMultiply(G.lightView, G.lightProj);
        float biasN = 0.025f / G.depthRange, texelN = 1.0f / SHADOW_NEAR_RES;
        int slotN = 11;
        SetShaderValueMatrix(G.lit, G.lLightVP, vp);
        SetShaderValueMatrix(G.lit, G.lLightVPN, vp);
        SetShaderValue(G.lit, GetShaderLocation(G.lit, "shadowNear"), &slotN, SHADER_UNIFORM_INT);
        SetShaderValue(G.lit, GetShaderLocation(G.lit, "shadowBiasN"), &biasN, SHADER_UNIFORM_FLOAT);
        SetShaderValue(G.lit, GetShaderLocation(G.lit, "shadowTexelN"), &texelN, SHADER_UNIFORM_FLOAT);
    }
    bias = 0.06f / G.depthRange;   /* ~6 cm in [0,1] depth units */
    SetShaderValue(G.lit, GetShaderLocation(G.lit, "shadowBias"), &bias, SHADER_UNIFORM_FLOAT);
    TraceLog(LOG_INFO, "GFX: shadows: far %.0f x %.0f m (%.1f cm/texel), near %.0f m (%.1f cm/texel)", hi.x - lo.x, hi.y - lo.y,
             100.0f * (hi.x - lo.x) / SHADOW_RES, SHADOW_NEAR_SIZE, 100.0f * SHADOW_NEAR_SIZE / SHADOW_NEAR_RES);
    gfx_resize();
}

static void gfx_unload(void)
{
    gfx_free_targets();
    rlUnloadFramebuffer(G.shadow.id);
    rlUnloadTexture(G.shadow.texture.id);
    rlUnloadFramebuffer(G.shadowN.id);
    rlUnloadTexture(G.shadowN.texture.id);
    UnloadMesh(G.skyMesh);
    MemFree(G.skyMat.maps);
    UnloadShader(G.lit); UnloadShader(G.depth); UnloadShader(G.sky); UnloadShader(G.emis);
    UnloadShader(G.down); UnloadShader(G.up); UnloadShader(G.comp); UnloadShader(G.fxaa);
}

/* per-draw material response: specular strength, shininess, sky reflection, emissive */
static void gfx_mat(float spec, float shin, float refl, float emis)
{
    float v[4] = { spec, shin, refl, emis };
    SetShaderValue(G.lit, G.lMat, v, SHADER_UNIFORM_VEC4);
}

static void gfx_light(Vector3 p, float radius, Vector3 col)
{
    if (G.ptCount >= MAX_PT_LIGHTS) return;
    G.ptPos[G.ptCount] = (Vector4){ p.x, p.y, p.z, radius };
    G.ptCol[G.ptCount] = (Vector4){ col.x, col.y, col.z, 0.0f };
    G.ptCount++;
}

static void gfx_emissive_begin(float intensity)
{
    SetShaderValue(G.emis, G.eInt, &intensity, SHADER_UNIFORM_FLOAT);
    BeginShaderMode(G.emis);
}

/* shadow pass for one cascade (0 = whole arena, 1 = sharp map around `focus`);
 * caller draws casters with G.depth as the override shader */
static void gfx_shadow_begin(int cascade, Vector3 focus)
{
    rlActiveTextureSlot(10); rlDisableTexture();          /* don't sample what we write */
    rlActiveTextureSlot(11); rlDisableTexture(); rlActiveTextureSlot(0);
    if (cascade == 1) {
        /* centre in light space, snapped to whole texels so edges don't crawl as the player moves */
        Vector3 c = Vector3Transform(focus, G.lightView);
        float h = SHADOW_NEAR_SIZE * 0.5f, tx = SHADOW_NEAR_SIZE / SHADOW_NEAR_RES;
        Matrix vp;
        c.x = floorf(c.x / tx) * tx; c.y = floorf(c.y / tx) * tx;
        G.lightProjN = MatrixOrtho(c.x - h, c.x + h, c.y - h, c.y + h, G.zNear, G.zFar);   /* same depth range = same bias units */
        vp = MatrixMultiply(G.lightView, G.lightProjN);
        SetShaderValueMatrix(G.lit, G.lLightVPN, vp);
    }
    BeginTextureMode(cascade == 1 ? G.shadowN : G.shadow);
    rlClearScreenBuffers();
    rlSetMatrixProjection(cascade == 1 ? G.lightProjN : G.lightProj);
    rlSetMatrixModelview(G.lightView);
    rlEnableDepthTest();
}

static void gfx_shadow_end(void)
{
    rlDisableDepthTest();
    EndTextureMode();
}

static void gfx_scene_begin(Camera3D cam, int shadows)
{
    int on = shadows;
    SetShaderValue(G.lit, G.lShadowOn, &on, SHADER_UNIFORM_INT);
    SetShaderValue(G.lit, G.lView, &cam.position, SHADER_UNIFORM_VEC3);
    SetShaderValue(G.lit, G.lPtCount, &G.ptCount, SHADER_UNIFORM_INT);
    if (G.ptCount > 0) {
        SetShaderValueV(G.lit, G.lPtPos, G.ptPos, SHADER_UNIFORM_VEC4, G.ptCount);
        SetShaderValueV(G.lit, G.lPtCol, G.ptCol, SHADER_UNIFORM_VEC4, G.ptCount);
    }
    rlActiveTextureSlot(10); rlEnableTexture(G.shadow.texture.id);
    rlActiveTextureSlot(11); rlEnableTexture(G.shadowN.texture.id); rlActiveTextureSlot(0);
    BeginTextureMode(G.hdr);
    ClearBackground(BLACK);
    BeginMode3D(cam);
    rlDisableDepthMask(); rlDisableBackfaceCulling();
    DrawMesh(G.skyMesh, G.skyMat, MatrixTranslate(cam.position.x, cam.position.y, cam.position.z));
    rlEnableDepthMask(); rlEnableBackfaceCulling();
}

static void gfx_scene_end(void)
{
    EndMode3D();
    EndTextureMode();
}

/* full-screen pass; extraLoc >= 0 binds a second sampler (must happen after the shader switch,
 * which flushes the batch and resets the extra texture slots) */
static void gfx_blit2(RenderTexture2D *dst, Shader sh, Texture2D src, int extraLoc, Texture2D extra)
{
    if (dst) BeginTextureMode(*dst);
    rlDisableColorBlend();                       /* straight copy: alpha carries data (FXAA luma) */
    BeginShaderMode(sh);
    if (extraLoc >= 0) SetShaderValueTexture(sh, extraLoc, extra);
    DrawTexturePro(src, (Rectangle){ 0, 0, (float)src.width, -(float)src.height },
                   (Rectangle){ 0, 0, (float)(dst ? dst->texture.width : GetScreenWidth()),
                                      (float)(dst ? dst->texture.height : GetScreenHeight()) },
                   (Vector2){ 0, 0 }, 0.0f, WHITE);
    EndShaderMode();                             /* flushes the batch before blending comes back */
    rlEnableColorBlend();
    if (dst) EndTextureMode();
}

static void gfx_blit(RenderTexture2D *dst, Shader sh, Texture2D src)
{
    Texture2D none = { 0 };
    gfx_blit2(dst, sh, src, -1, none);
}

/* HDR -> bloom -> tonemapped LDR target; boost (0..1) adds radial motion streak */
static void gfx_post(int bloomOn, float boost)
{
    float exposure = 1.0f, bloomStr = bloomOn ? 0.25f : 0.0f, vig = 0.04f, ab = 0.022f * boost;
    int i;
    if (bloomOn) {
        for (i = 0; i < BLOOM_MIPS; i++) {
            Texture2D src = i == 0 ? G.hdr.texture : G.bloom[i - 1].texture;
            Vector2 tx = { 1.0f / src.width, 1.0f / src.height };
            int pre = i == 0;
            SetShaderValue(G.down, G.dTexel, &tx, SHADER_UNIFORM_VEC2);
            SetShaderValue(G.down, G.dPre, &pre, SHADER_UNIFORM_INT);
            gfx_blit(&G.bloom[i], G.down, src);
        }
        for (i = BLOOM_MIPS - 1; i > 0; i--) {
            Vector2 tx = { 1.0f / G.bloom[i].texture.width, 1.0f / G.bloom[i].texture.height };
            SetShaderValue(G.up, G.uTexel, &tx, SHADER_UNIFORM_VEC2);
            BeginTextureMode(G.bloom[i - 1]);
            BeginBlendMode(BLEND_ADDITIVE);
            BeginShaderMode(G.up);
            DrawTexturePro(G.bloom[i].texture, (Rectangle){ 0, 0, (float)G.bloom[i].texture.width, -(float)G.bloom[i].texture.height },
                           (Rectangle){ 0, 0, (float)G.bloom[i - 1].texture.width, (float)G.bloom[i - 1].texture.height },
                           (Vector2){ 0, 0 }, 0.0f, WHITE);
            EndShaderMode();
            EndBlendMode();
            EndTextureMode();
        }
    }
    SetShaderValue(G.comp, G.cExp, &exposure, SHADER_UNIFORM_FLOAT);
    SetShaderValue(G.comp, G.cBloomStr, &bloomStr, SHADER_UNIFORM_FLOAT);
    SetShaderValue(G.comp, G.cVig, &vig, SHADER_UNIFORM_FLOAT);
    SetShaderValue(G.comp, G.cAb, &ab, SHADER_UNIFORM_FLOAT);
    gfx_blit2(&G.ldr, G.comp, G.hdr.texture, G.cBloom, G.bloom[0].texture);
}

/* LDR target -> backbuffer through FXAA (call inside BeginDrawing) */
static void gfx_present(void)
{
    Vector2 tx = { 1.0f / G.w, 1.0f / G.h };
    SetShaderValue(G.fxaa, G.fTexel, &tx, SHADER_UNIFORM_VEC2);
    gfx_blit(NULL, G.fxaa, G.ldr.texture);
}

static Texture2D load_tga_texture(const char *path, int opaque)
{
    Texture2D t = { 0 };
    int w, h;
    unsigned char *px = sarm_load_tga(path, &w, &h);
    if (px) {
        Image img = { px, w, h, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
        if (opaque) { int k; for (k = 0; k < w * h; k++) px[k * 4 + 3] = 255; }
        t = LoadTextureFromImage(img);
        GenTextureMipmaps(&t);
        SetTextureFilter(t, TEXTURE_FILTER_TRILINEAR);
        free(px);
    } else {
        TraceLog(LOG_WARNING, "could not load %s", path);
    }
    return t;
}

/* de-indexed raylib mesh from a list of SARM indices, positions relative to origin */
static Mesh mesh_from_indices(const SarmModel *m, const uint32_t *idx, int count, Vector3 origin)
{
    Mesh mesh = { 0 };
    int i;
    mesh.vertexCount   = count;
    mesh.triangleCount = count / 3;
    mesh.vertices  = (float *)MemAlloc(sizeof(float) * 3 * count);
    mesh.normals   = (float *)MemAlloc(sizeof(float) * 3 * count);
    mesh.texcoords = (float *)MemAlloc(sizeof(float) * 2 * count);
    for (i = 0; i < count; i++) {
        const SarmVertex *v = &m->vertices[idx[i]];
        mesh.vertices[i*3+0] = v->pos[0] - origin.x;
        mesh.vertices[i*3+1] = v->pos[1] - origin.y;
        mesh.vertices[i*3+2] = v->pos[2] - origin.z;
        memcpy(&mesh.normals[i*3], v->normal, sizeof(float) * 3);
        memcpy(&mesh.texcoords[i*2], v->uv, sizeof(float) * 2);
    }
    UploadMesh(&mesh, false);
    return mesh;
}

static int nearest_wheel(const SarmModel *m, const float *p)
{
    int w, best = 0;
    float bd = 1e9f;
    for (w = 0; w < 4; w++) {
        float dx = p[0] - m->header.wheel_pos[w][0], dy = p[1] - m->header.wheel_pos[w][1], dz = p[2] - m->header.wheel_pos[w][2];
        float d = dx*dx + dy*dy + dz*dz;
        if (d < bd) { bd = d; best = w; }
    }
    return best;
}

static int car_render_load(CarRender *cr, const SarmModel *m, const char *dir, Shader lit)
{
    uint32_t s, i;
    int w, k;
    char path[1024];
    Texture2D tTire;
    float bmin[4][3], bmax[4][3], R[4], halfW[4];
    uint32_t *wl[4][3];
    int wn[4][3];
    memset(cr, 0, sizeof(*cr));

    /* 1. wheel extents from the tyre geometry (each tyre vertex -> nearest hub) */
    for (w = 0; w < 4; w++) for (k = 0; k < 3; k++) { bmin[w][k] = 1e9f; bmax[w][k] = -1e9f; }
    for (s = 0; s < m->header.submesh_count; s++) {
        const SarmSubmesh *sm = &m->submeshes[s];
        if (sm->material != SARM_MAT_TIRE) continue;
        for (i = 0; i < sm->index_count; i++) {
            const float *p = m->vertices[m->indices[sm->first_index + i]].pos;
            w = nearest_wheel(m, p);
            for (k = 0; k < 3; k++) { bmin[w][k] = fminf(bmin[w][k], p[k]); bmax[w][k] = fmaxf(bmax[w][k], p[k]); }
        }
    }
    for (w = 0; w < 4; w++) {
        if (bmax[w][0] < bmin[w][0]) { cr->wanim[w] = 0; R[w] = 0; continue; }
        cr->wanim[w]  = 1;
        cr->whub[w]   = V3(0.5f * (bmin[w][0] + bmax[w][0]), 0.5f * (bmin[w][1] + bmax[w][1]), 0.5f * (bmin[w][2] + bmax[w][2]));
        R[w]          = 0.5f * fmaxf(bmax[w][0] - bmin[w][0], bmax[w][1] - bmin[w][1]);
        halfW[w]      = 0.5f * (bmax[w][2] - bmin[w][2]);
        cr->wfront[w] = cr->whub[w].x > 0.0f;
    }

    /* 2. split triangles: tyres + anything fully inside a wheel cylinder (rims, hubs) */
    for (w = 0; w < 4; w++) for (k = 0; k < 3; k++) {
        wl[w][k] = (uint32_t *)MemAlloc(sizeof(uint32_t) * m->header.index_count);
        wn[w][k] = 0;
    }
    for (s = 0; s < m->header.submesh_count && cr->count < 16; s++) {
        const SarmSubmesh *sm = &m->submeshes[s];
        uint32_t *body = (uint32_t *)MemAlloc(sizeof(uint32_t) * (sm->index_count + 3));
        int nb = 0, mat = (int)sm->material;
        if (mat < 0 || mat > 2) mat = SARM_MAT_BODY;
        for (i = 0; i + 2 < sm->index_count; i += 3) {
            const uint32_t *tri = &m->indices[sm->first_index + i];
            int dest = -1, v;
            if (mat == SARM_MAT_TIRE) {
                float c[3];
                for (k = 0; k < 3; k++) c[k] = (m->vertices[tri[0]].pos[k] + m->vertices[tri[1]].pos[k] + m->vertices[tri[2]].pos[k]) / 3.0f;
                dest = nearest_wheel(m, c);
            } else {
                for (w = 0; w < 4 && dest < 0; w++) {
                    int inside = cr->wanim[w];
                    for (v = 0; v < 3 && inside; v++) {
                        const float *p = m->vertices[tri[v]].pos;
                        float dx = p[0] - cr->whub[w].x, dy = p[1] - cr->whub[w].y, dz = p[2] - cr->whub[w].z;
                        inside = dx*dx + dy*dy < R[w] * R[w] * 0.96f && fabsf(dz) < halfW[w] * 1.15f;
                    }
                    if (inside) dest = w;
                }
            }
            if (dest >= 0 && cr->wanim[dest]) {
                for (v = 0; v < 3; v++) wl[dest][mat][wn[dest][mat]++] = tri[v];
            } else {
                for (v = 0; v < 3; v++) body[nb++] = tri[v];
            }
        }
        if (nb > 0) {
            cr->mesh[cr->count] = mesh_from_indices(m, body, nb, V3(0, 0, 0));
            cr->material[cr->count] = mat;
            cr->count++;
        }
        MemFree(body);
    }
    for (w = 0; w < 4; w++) for (k = 0; k < 3; k++) {
        if (wn[w][k] > 0) { cr->wmesh[w][k] = mesh_from_indices(m, wl[w][k], wn[w][k], cr->whub[w]); cr->wvalid[w][k] = 1; }
        MemFree(wl[w][k]);
    }

    snprintf(path, sizeof(path), "%sbody_blue.tga", dir);    cr->texBlue   = load_tga_texture(path, 0);
    snprintf(path, sizeof(path), "%sbody_orange.tga", dir);  cr->texOrange = load_tga_texture(path, 0);
    snprintf(path, sizeof(path), "%sbody_custom.tga", dir);  cr->texCustom = load_tga_texture(path, 0);
    snprintf(path, sizeof(path), "%stire_diffuse.tga", dir); tTire = load_tga_texture(path, 1);   /* alpha = rim mask, not opacity */

    cr->matBody  = LoadMaterialDefault(); cr->matBody.shader  = lit;
    cr->matGlass = LoadMaterialDefault(); cr->matGlass.shader = lit;
    cr->matTire  = LoadMaterialDefault(); cr->matTire.shader  = lit;
    if (cr->texBlue.id) cr->matBody.maps[MATERIAL_MAP_DIFFUSE].texture = cr->texBlue;
    if (tTire.id) cr->matTire.maps[MATERIAL_MAP_DIFFUSE].texture = tTire;
    cr->matGlass.maps[MATERIAL_MAP_DIFFUSE].color = (Color){ 15, 23, 41, 153 };
    return cr->count > 0;
}

/* team 0 = blue livery, 1 = orange; ovr = depth shader for the shadow pass (NULL = lit) */
static void car_mat(const CarRender *cr, int kind, const Shader *ovr, Material *out)
{
    *out = kind == SARM_MAT_TIRE ? cr->matTire : kind == SARM_MAT_GLASS ? cr->matGlass : cr->matBody;
    if (ovr) { out->shader = *ovr; return; }
    if (kind == SARM_MAT_TIRE)       gfx_mat(0.12f, 12.0f, 0.04f, 0.0f);
    else if (kind == SARM_MAT_GLASS) gfx_mat(0.90f, 160.0f, 0.70f, 0.0f);
    else                             gfx_mat(0.60f, 60.0f, 0.28f, 0.0f);   /* glossy car paint */
}

static void car_render_draw(CarRender *cr, const Car *c, int team, int skin, const Shader *ovr)
{
    Matrix xf = MatrixMultiply(MatrixMultiply(MatrixScale(CAR_SCALE, CAR_SCALE, CAR_SCALE), QuaternionToMatrix(c->rot)),
                               MatrixTranslate(c->pos.x, c->pos.y, c->pos.z));
    int pass, i, w;
    Material m;
    Texture2D body = cr->texBlue;
    if (skin == 1 && cr->texCustom.id)        body = cr->texCustom;
    else if (team == 1 && cr->texOrange.id)   body = cr->texOrange;
    if (body.id) cr->matBody.maps[MATERIAL_MAP_DIFFUSE].texture = body;
    rlDisableBackfaceCulling();                  /* thin fender/flap planes are single-sided in the source meshes */
    for (pass = 0; pass < 2; pass++) {          /* opaque first, glass second */
        for (i = 0; i < cr->count; i++) {
            int glass = cr->material[i] == SARM_MAT_GLASS;
            if (glass != pass) continue;
            car_mat(cr, cr->material[i], ovr, &m);
            DrawMesh(cr->mesh[i], m, xf);
        }
        /* wheels: roll, steer (fronts), then follow the suspension */
        for (w = 0; w < 4; w++) {
            Matrix wx;
            float steer = cr->wfront[w] ? c->steerAngle * DEG2RAD : 0.0f;
            if (!cr->wanim[w]) continue;
            wx = MatrixMultiply(MatrixRotateZ(c->wheelSpin), MatrixRotateY(steer));
            wx = MatrixMultiply(wx, MatrixTranslate(cr->whub[w].x, cr->whub[w].y + c->wheelOfs[w] / CAR_SCALE, cr->whub[w].z));
            wx = MatrixMultiply(wx, xf);
            for (i = 0; i < 3; i++) {
                if (!cr->wvalid[w][i] || (i == SARM_MAT_GLASS) != pass) continue;
                car_mat(cr, i, ovr, &m);
                DrawMesh(cr->wmesh[w][i], m, wx);
            }
        }
    }
    rlEnableBackfaceCulling();
}

/* free GPU meshes/textures; materials only own their maps array (the shader is shared) */
static void car_render_unload(CarRender *cr)
{
    Material *ms[3];
    int i, k;
    ms[0] = &cr->matBody; ms[1] = &cr->matGlass; ms[2] = &cr->matTire;
    for (i = 0; i < cr->count; i++) UnloadMesh(cr->mesh[i]);
    for (i = 0; i < 4; i++) for (k = 0; k < 3; k++) if (cr->wvalid[i][k]) UnloadMesh(cr->wmesh[i][k]);
    if (cr->texBlue.id)   UnloadTexture(cr->texBlue);
    if (cr->texOrange.id) UnloadTexture(cr->texOrange);
    if (cr->texCustom.id) UnloadTexture(cr->texCustom);
    for (i = 0; i < 3; i++) {
        if (!ms[i]->maps) continue;
        if (i != 0 && ms[i]->maps[MATERIAL_MAP_DIFFUSE].texture.id != rlGetTextureIdDefault())
            UnloadTexture(ms[i]->maps[MATERIAL_MAP_DIFFUSE].texture);
        MemFree(ms[i]->maps);
    }
    memset(cr, 0, sizeof(*cr));
}

/* ---- arena visual mesh (export_c/arena/arena.sarm + arena_materials.txt) ---- */
typedef struct ArenaRender {
    Mesh     mesh[16];
    Material mat[16];
    int      translucent[16];
    float    par[16][4];
    int      count;
} ArenaRender;

static int arena_render_load(ArenaRender *ar, const char *dir, Shader lit)
{
    char path[1024], line[256], names[16][64], texs[16][64];
    int cols[16][4], nmat = 0;
    uint32_t s, i;
    SarmModel m;
    FILE *f;
    memset(ar, 0, sizeof(*ar));
    snprintf(path, sizeof(path), "%sarena_materials.txt", dir);
    f = fopen(path, "r");
    if (!f) return 0;
    while (nmat < 16 && fgets(line, sizeof(line), f))
        if (sscanf(line, "%63s %63s %d %d %d %d", names[nmat], texs[nmat],
                   &cols[nmat][0], &cols[nmat][1], &cols[nmat][2], &cols[nmat][3]) == 6) nmat++;
    fclose(f);
    snprintf(path, sizeof(path), "%sarena.sarm", dir);
    if (sarm_load(path, &m) != SARM_OK) return 0;

    for (s = 0; s < m.header.submesh_count && ar->count < 16; s++) {
        const SarmSubmesh *sm = &m.submeshes[s];
        Mesh mesh = { 0 };
        Material mat;
        int mi = (int)sm->material;
        if (sm->index_count == 0 || mi >= nmat) continue;
        mesh.vertexCount   = (int)sm->index_count;
        mesh.triangleCount = (int)sm->index_count / 3;
        mesh.vertices  = (float *)MemAlloc(sizeof(float) * 3 * sm->index_count);
        mesh.normals   = (float *)MemAlloc(sizeof(float) * 3 * sm->index_count);
        mesh.texcoords = (float *)MemAlloc(sizeof(float) * 2 * sm->index_count);
        for (i = 0; i < sm->index_count; i++) {
            const SarmVertex *v = &m.vertices[m.indices[sm->first_index + i]];
            memcpy(&mesh.vertices[i*3], v->pos, sizeof(float) * 3);
            memcpy(&mesh.normals[i*3], v->normal, sizeof(float) * 3);
            memcpy(&mesh.texcoords[i*2], v->uv, sizeof(float) * 2);
        }
        UploadMesh(&mesh, false);
        mat = LoadMaterialDefault();
        mat.shader = lit;
        mat.maps[MATERIAL_MAP_DIFFUSE].color = (Color){ (unsigned char)cols[mi][0], (unsigned char)cols[mi][1],
                                                        (unsigned char)cols[mi][2], (unsigned char)cols[mi][3] };
        if (strcmp(texs[mi], "-") != 0) {
            Texture2D t;
            snprintf(path, sizeof(path), "%s%s", dir, texs[mi]);
            t = load_tga_texture(path, 0);
            if (t.id) {
                SetTextureWrap(t, TEXTURE_WRAP_REPEAT);
                rlTextureParameters(t.id, RL_TEXTURE_FILTER_ANISOTROPIC, 8);
                mat.maps[MATERIAL_MAP_DIFFUSE].texture = t;
            }
        }
        ar->mesh[ar->count] = mesh;
        ar->mat[ar->count] = mat;
        ar->translucent[ar->count] = cols[mi][3] < 255;
        {   /* spec, shininess, sky reflection, emissive -- picked per surface type */
            const char *nm = names[mi];
            float p[4] = { 0.15f, 16.0f, 0.10f, 0.0f };
            if      (strstr(nm, "Pavement"))   { p[0] = 0.22f; p[1] = 24.0f;  p[2] = 0.18f; }
            else if (strstr(nm, "StreetLine")) { p[0] = 0.25f; p[1] = 24.0f;  p[2] = 0.18f; }
            else if (strstr(nm, "SideWalk"))   { p[0] = 0.12f; p[1] = 14.0f;  p[2] = 0.10f; }
            else if (strstr(nm, "GrayTiles"))  { p[0] = 0.30f; p[1] = 40.0f;  p[2] = 0.25f; }
            else if (strstr(nm, "Brick"))      { p[0] = 0.05f; p[1] = 8.0f;   p[2] = 0.02f; }
            else if (strstr(nm, "Metal"))      { p[0] = 0.60f; p[1] = 48.0f;  p[2] = 0.45f; }
            if (ar->translucent[ar->count])    { p[0] = 0.60f; p[1] = 200.0f; p[2] = 0.12f; }   /* dome: subtle, mostly see-through */
            memcpy(ar->par[ar->count], p, sizeof(p));
        }
        ar->count++;
    }
    TraceLog(LOG_INFO, "ARENA: %u render tris in %d materials", m.header.index_count / 3, ar->count);
    sarm_free(&m);
    return ar->count > 0;
}

/* ovr = depth shader for the shadow pass (glass doesn't cast) */
static void arena_render_draw(const ArenaRender *ar, int translucentPass, const Shader *ovr)
{
    int i;
    if (translucentPass) { rlDisableDepthMask(); rlDisableBackfaceCulling(); }
    for (i = 0; i < ar->count; i++) {
        Material m = ar->mat[i];
        if (ar->translucent[i] != translucentPass) continue;
        if (ovr) m.shader = *ovr;
        else     gfx_mat(ar->par[i][0], ar->par[i][1], ar->par[i][2], ar->par[i][3]);
        DrawMesh(ar->mesh[i], m, MatrixIdentity());
    }
    if (translucentPass) { rlEnableDepthMask(); rlEnableBackfaceCulling(); }
}

/* team-coloured "net" across each goal mouth so you can tell the ends apart */
static void draw_goal_nets(void)
{
    int s;
    rlDisableBackfaceCulling();
    rlDisableDepthMask();
    rlBegin(RL_QUADS);
    for (s = -1; s <= 1; s += 2) {
        float z = (ARENA_L + 1.0f) * s;
        Color c = s < 0 ? (Color){ 40, 110, 255, 60 } : (Color){ 255, 130, 30, 60 };
        rlColor4ub(c.r, c.g, c.b, c.a);
        rlVertex3f(-GOAL_HW, 0.05f, z); rlVertex3f(GOAL_HW, 0.05f, z);
        rlVertex3f(GOAL_HW, GOAL_H, z); rlVertex3f(-GOAL_HW, GOAL_H, z);
    }
    rlEnd();
    rlDrawRenderBatchActive();   /* state changes below don't flush the batch themselves */
    rlEnableDepthMask();
    rlEnableBackfaceCulling();
}

static Color shade(Color c, Vector3 n)
{
    float l = 0.5f + 0.5f * fmaxf(0.0f, Vector3DotProduct(n, Vector3Negate(Vector3Normalize(LIGHT_DIR))));
    return (Color){ (unsigned char)(c.r * l), (unsigned char)(c.g * l), (unsigned char)(c.b * l), c.a };
}

static void quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Color col)
{
    rlColor4ub(col.r, col.g, col.b, col.a);
    rlVertex3f(a.x, a.y, a.z); rlVertex3f(b.x, b.y, b.z);
    rlVertex3f(c.x, c.y, c.z); rlVertex3f(d.x, d.y, d.z);
}

static void draw_arena_shell(void)
{
    const float W = ARENA_W, L = ARENA_L, H = ARENA_H, C = RAMP_C, HW = GOAL_HW, GH = GOAL_H;
    const float S2 = 0.70710678f;
    Color ramp = { 70, 78, 92, 255 }, wall = { 52, 58, 72, 255 };
    int s;
    rlDisableBackfaceCulling();
    rlBegin(RL_QUADS);
    for (s = -1; s <= 1; s += 2) {
        float fs = (float)s;
        /* side ramp + wall (x = s*W) */
        quad(V3(fs*(W-C), 0, -(L-C)), V3(fs*(W-C), 0, L-C), V3(fs*W, C, L), V3(fs*W, C, -L),
             shade(ramp, V3(-fs*S2, S2, 0)));
        quad(V3(fs*W, C, -L), V3(fs*W, C, L), V3(fs*W, H, L), V3(fs*W, H, -L), shade(wall, V3(-fs, 0, 0)));
        /* end ramps, split around the goal mouth (z = s*L) */
        quad(V3(-(W-C), 0, fs*(L-C)), V3(-HW, 0, fs*(L-C)), V3(-HW, C, fs*L), V3(-W, C, fs*L),
             shade(ramp, V3(0, S2, -fs*S2)));
        quad(V3(HW, 0, fs*(L-C)), V3(W-C, 0, fs*(L-C)), V3(W, C, fs*L), V3(HW, C, fs*L),
             shade(ramp, V3(0, S2, -fs*S2)));
        quad(V3(-HW, 0, fs*(L-C)), V3(-HW, C, fs*L), V3(-HW, 0, fs*L), V3(-HW, 0, fs*L), shade(ramp, V3(1, 0, 0)));
        quad(V3( HW, 0, fs*(L-C)), V3( HW, C, fs*L), V3( HW, 0, fs*L), V3( HW, 0, fs*L), shade(ramp, V3(-1, 0, 0)));
        /* end wall pieces */
        quad(V3(-W, C, fs*L), V3(-HW, C, fs*L), V3(-HW, H, fs*L), V3(-W, H, fs*L), shade(wall, V3(0, 0, -fs)));
        quad(V3(HW, C, fs*L), V3(W, C, fs*L), V3(W, H, fs*L), V3(HW, H, fs*L), shade(wall, V3(0, 0, -fs)));
        quad(V3(-HW, GH, fs*L), V3(HW, GH, fs*L), V3(HW, H, fs*L), V3(-HW, H, fs*L), shade(wall, V3(0, 0, -fs)));
        quad(V3(-HW, C, fs*L), V3(-HW, GH, fs*L), V3(-HW, GH, fs*L), V3(-HW, C, fs*L), wall);
    }
    rlEnd();

    /* wall grid lines for depth perception */
    rlBegin(RL_LINES);
    rlColor4ub(90, 100, 120, 255);
    {
        float z, y;
        for (s = -1; s <= 1; s += 2) {
            for (z = -L; z <= L + 0.1f; z += 16.0f) { rlVertex3f(s*W, C, z); rlVertex3f(s*W, H, z); }
            for (y = C; y <= H + 0.1f; y += 10.0f) { rlVertex3f(s*W, y, -L); rlVertex3f(s*W, y, L); }
        }
    }
    rlEnd();
    rlDrawRenderBatchActive();   /* culling state changes don't flush the batch themselves */
    rlEnableBackfaceCulling();
}

static void draw_goals(void)
{
    const float L = ARENA_L, HW = GOAL_HW, GH = GOAL_H, D = GOAL_D;
    int s;
    for (s = -1; s <= 1; s += 2) {
        float fs = (float)s;
        Color team = s < 0 ? (Color){ 40, 110, 255, 255 } : (Color){ 255, 130, 30, 255 };
        Color glow = team; glow.a = 70;
        DrawCylinderEx(V3(-HW, 0, fs*L), V3(-HW, GH, fs*L), POST_R, POST_R, 10, RAYWHITE);
        DrawCylinderEx(V3( HW, 0, fs*L), V3( HW, GH, fs*L), POST_R, POST_R, 10, RAYWHITE);
        DrawCylinderEx(V3(-HW, GH, fs*L), V3(HW, GH, fs*L), POST_R, POST_R, 10, RAYWHITE);
        rlDisableBackfaceCulling();
        rlDisableDepthMask();
        rlBegin(RL_QUADS);
        quad(V3(-HW, 0, fs*(L+D)), V3(HW, 0, fs*(L+D)), V3(HW, GH, fs*(L+D)), V3(-HW, GH, fs*(L+D)), glow);
        quad(V3(-HW, 0, fs*L), V3(-HW, 0, fs*(L+D)), V3(-HW, GH, fs*(L+D)), V3(-HW, GH, fs*L), glow);
        quad(V3( HW, 0, fs*L), V3( HW, 0, fs*(L+D)), V3( HW, GH, fs*(L+D)), V3( HW, GH, fs*L), glow);
        quad(V3(-HW, GH, fs*L), V3(HW, GH, fs*L), V3(HW, GH, fs*(L+D)), V3(-HW, GH, fs*(L+D)), glow);
        rlEnd();
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
        rlEnableBackfaceCulling();
    }
}

static Texture2D make_field_texture(void)
{
    const float W = ARENA_W, LT = ARENA_L + GOAL_D;
    const int iw = 1024, ih = (int)(1024.0f * LT / W);
    Image img = GenImageColor(iw, ih, (Color){ 46, 110, 52, 255 });
    float ppm = iw / (2.0f * W);                       /* pixels per metre */
    int i, lw = (int)fmaxf(2.0f, 0.5f * ppm);
    Color line = (Color){ 235, 235, 235, 255 };
#define PX(x) ((int)(((x) + W) * ppm))
#define PZ(z) ((int)(((z) + LT) * ppm))
    for (i = 0; i < 18; i++)                           /* mowing stripes */
        if (i & 1) ImageDrawRectangle(&img, 0, PZ(-ARENA_L + i * (2*ARENA_L/18)), iw,
                                      (int)(2*ARENA_L/18 * ppm), (Color){ 52, 122, 58, 255 });
    ImageDrawRectangle(&img, 0, PZ(-ARENA_L - GOAL_D), iw, (int)(GOAL_D * ppm), (Color){ 60, 60, 66, 255 });
    ImageDrawRectangle(&img, 0, PZ(ARENA_L), iw, (int)(GOAL_D * ppm) + 2, (Color){ 60, 60, 66, 255 });
    ImageDrawRectangle(&img, 0, PZ(0) - lw/2, iw, lw, line);                         /* halfway   */
    for (i = 0; i < lw; i++) ImageDrawCircleLines(&img, PX(0), PZ(0), (int)(18 * ppm) - i, line);
    ImageDrawRectangle(&img, PX(-GOAL_HW*2), PZ(-ARENA_L), lw, (int)(24*ppm), line); /* boxes */
    ImageDrawRectangle(&img, PX( GOAL_HW*2), PZ(-ARENA_L), lw, (int)(24*ppm), line);
    ImageDrawRectangle(&img, PX(-GOAL_HW*2), PZ(-ARENA_L + 24), (int)(GOAL_HW*4*ppm) + lw, lw, line);
    ImageDrawRectangle(&img, PX(-GOAL_HW*2), PZ(ARENA_L - 24), lw, (int)(24*ppm), line);
    ImageDrawRectangle(&img, PX( GOAL_HW*2), PZ(ARENA_L - 24), lw, (int)(24*ppm), line);
    ImageDrawRectangle(&img, PX(-GOAL_HW*2), PZ(ARENA_L - 24), (int)(GOAL_HW*4*ppm) + lw, lw, line);
    ImageDrawRectangle(&img, PX(-GOAL_HW), PZ(-ARENA_L) - lw/2, (int)(GOAL_HW*2*ppm), lw, (Color){ 80, 150, 255, 255 });
    ImageDrawRectangle(&img, PX(-GOAL_HW), PZ( ARENA_L) - lw/2, (int)(GOAL_HW*2*ppm), lw, (Color){ 255, 150, 60, 255 });
#undef PX
#undef PZ
    {
        Texture2D t = LoadTextureFromImage(img);
        GenTextureMipmaps(&t);
        SetTextureFilter(t, TEXTURE_FILTER_TRILINEAR);
        UnloadImage(img);
        return t;
    }
}

/* ------------------------------------------------------------------------ */
/* 3D Visual Particle System                                                */
/* ------------------------------------------------------------------------ */
#define MAX_PARTICLES 2048

typedef struct {
    Vector3 pos;
    Vector3 vel;
    Color   colorStart;
    Color   colorEnd;
    float   sizeStart;
    float   sizeEnd;
    float   life;
    float   maxLife;
    float   drag;
    float   gravity;
    int     additive;    /* 1 = additive glow/bloom, 0 = alpha smoke */
    int     active;
} Particle;

static Particle  g_particles[MAX_PARTICLES];
static int       g_particleHead = 0;
static Texture2D g_particleTex;

static float p_randf(void)
{
    return (float)rand() / (float)RAND_MAX;
}

static float p_range(float min, float max)
{
    return min + p_randf() * (max - min);
}

static Vector3 p_randv3(float min, float max)
{
    return V3(p_range(min, max), p_range(min, max), p_range(min, max));
}

static Vector3 p_rand_dir(void)
{
    float z = p_range(-1.0f, 1.0f);
    float a = p_range(0.0f, 2.0f * PI);
    float r = sqrtf(fmaxf(0.0f, 1.0f - z * z));
    return V3(r * cosf(a), z, r * sinf(a));
}

static Texture2D make_particle_texture(void)
{
    int size = 64;
    Image img = GenImageColor(size, size, BLANK);
    Color *pixels = (Color *)img.data;
    float center = (size - 1) * 0.5f;
    float radius = center;
    int y, x;
    for (y = 0; y < size; y++) {
        for (x = 0; x < size; x++) {
            float dx = (x - center) / radius;
            float dy = (y - center) / radius;
            float r2 = dx * dx + dy * dy;
            if (r2 < 1.0f) {
                float a = expf(-r2 * 3.5f) * (1.0f - r2);
                unsigned char val = (unsigned char)clampf(a * 255.0f, 0.0f, 255.0f);
                pixels[y * size + x] = (Color){ 255, 255, 255, val };
            } else {
                pixels[y * size + x] = (Color){ 0, 0, 0, 0 };
            }
        }
    }
    {
        Texture2D tex = LoadTextureFromImage(img);
        GenTextureMipmaps(&tex);
        SetTextureFilter(tex, TEXTURE_FILTER_BILINEAR);
        UnloadImage(img);
        return tex;
    }
}

static Particle *particle_spawn(Vector3 pos, Vector3 vel, Color c0, Color c1,
                                float s0, float s1, float life, float drag,
                                float gravity, int additive)
{
    int i;
    for (i = 0; i < MAX_PARTICLES; i++) {
        int idx = (g_particleHead + i) % MAX_PARTICLES;
        if (!g_particles[idx].active) {
            g_particleHead = (idx + 1) % MAX_PARTICLES;
            Particle *p = &g_particles[idx];
            p->pos = pos;
            p->vel = vel;
            p->colorStart = c0;
            p->colorEnd = c1;
            p->sizeStart = s0;
            p->sizeEnd = s1;
            p->life = 0.0f;
            p->maxLife = life;
            p->drag = drag;
            p->gravity = gravity;
            p->additive = additive;
            p->active = 1;
            return p;
        }
    }
    /* Pool full: reuse oldest */
    Particle *p = &g_particles[g_particleHead];
    g_particleHead = (g_particleHead + 1) % MAX_PARTICLES;
    p->pos = pos;
    p->vel = vel;
    p->colorStart = c0;
    p->colorEnd = c1;
    p->sizeStart = s0;
    p->sizeEnd = s1;
    p->life = 0.0f;
    p->maxLife = life;
    p->drag = drag;
    p->gravity = gravity;
    p->additive = additive;
    p->active = 1;
    return p;
}

static void particles_update(float dt)
{
    int i;
    for (i = 0; i < MAX_PARTICLES; i++) {
        Particle *p = &g_particles[i];
        if (!p->active) continue;

        p->life += dt;
        if (p->life >= p->maxLife) {
            p->active = 0;
            continue;
        }

        p->vel.y -= p->gravity * dt;
        float d = 1.0f - p->drag * dt;
        if (d < 0.0f) d = 0.0f;
        p->vel = Vector3Scale(p->vel, d);
        p->pos = Vector3Add(p->pos, Vector3Scale(p->vel, dt));

        /* Pitch collision bounce for sparks & debris */
        if (p->pos.y < 0.05f) {
            p->pos.y = 0.05f;
            p->vel.y = -p->vel.y * 0.35f;
            p->vel.x *= 0.70f;
            p->vel.z *= 0.70f;
        }
    }
}

static void particles_draw_pass(Camera3D cam, Texture2D tex, int passAdditive)
{
    int i;
    Vector3 fwd, camUp, camRight;
    float hs, r, g, b, a, t, sz;
    Color col;

    if (!tex.id) return;

    fwd = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
    camUp = cam.up;
    camRight = Vector3Normalize(Vector3CrossProduct(fwd, camUp));
    camUp = Vector3CrossProduct(camRight, fwd);

    rlSetTexture(tex.id);
    rlBegin(RL_QUADS);

    for (i = 0; i < MAX_PARTICLES; i++) {
        Particle *p = &g_particles[i];
        if (!p->active || p->additive != passAdditive) continue;

        t = p->life / p->maxLife;
        sz = Lerp(p->sizeStart, p->sizeEnd, t);
        hs = sz * 0.5f;

        r = Lerp((float)p->colorStart.r, (float)p->colorEnd.r, t);
        g = Lerp((float)p->colorStart.g, (float)p->colorEnd.g, t);
        b = Lerp((float)p->colorStart.b, (float)p->colorEnd.b, t);
        a = Lerp((float)p->colorStart.a, (float)p->colorEnd.a, t);
        if (a < 1.0f) continue;

        col = (Color){ (unsigned char)clampf(r, 0.0f, 255.0f),
                       (unsigned char)clampf(g, 0.0f, 255.0f),
                       (unsigned char)clampf(b, 0.0f, 255.0f),
                       (unsigned char)clampf(a, 0.0f, 255.0f) };

        Vector3 rx = Vector3Scale(camRight, hs);
        Vector3 uy = Vector3Scale(camUp, hs);

        Vector3 p0 = Vector3Subtract(Vector3Subtract(p->pos, rx), uy);
        Vector3 p1 = Vector3Subtract(Vector3Add(p->pos, rx), uy);
        Vector3 p2 = Vector3Add(Vector3Add(p->pos, rx), uy);
        Vector3 p3 = Vector3Add(Vector3Subtract(p->pos, rx), uy);

        rlColor4ub(col.r, col.g, col.b, col.a);
        rlTexCoord2f(0.0f, 0.0f); rlVertex3f(p0.x, p0.y, p0.z);
        rlTexCoord2f(1.0f, 0.0f); rlVertex3f(p1.x, p1.y, p1.z);
        rlTexCoord2f(1.0f, 1.0f); rlVertex3f(p2.x, p2.y, p2.z);
        rlTexCoord2f(0.0f, 1.0f); rlVertex3f(p3.x, p3.y, p3.z);
    }

    rlEnd();
    rlSetTexture(0);
}

/* Emitters */
static void particles_boost_emit(const Car *c, int isBlue)
{
    Vector3 back = Vector3Add(c->pos, car_to_world(c, V3(c->boxMin.x - 0.25f, 0.14f, 0.0f)));
    Vector3 bk = Vector3Scale(car_fwd(c), -1.0f);
    int k;

    /* Plume particles: expanding, slowing plasma balls */
    for (k = 0; k < 2; k++) {
        Vector3 p = Vector3Add(back, p_randv3(-0.05f, 0.05f));
        Vector3 vel = Vector3Add(Vector3Scale(c->vel, 0.35f),
                                Vector3Scale(bk, p_range(16.0f, 26.0f)));
        vel = Vector3Add(vel, p_randv3(-1.8f, 1.8f));

        Color c0 = isBlue ? (Color){ 130, 220, 255, 230 } : (Color){ 255, 210, 80, 240 };
        Color c1 = isBlue ? (Color){ 20, 70, 220, 0 }     : (Color){ 230, 45, 10, 0 };
        float s0 = p_range(0.20f, 0.32f);
        float s1 = p_range(0.50f, 0.85f);
        float life = p_range(0.16f, 0.28f);

        particle_spawn(p, vel, c0, c1, s0, s1, life, 3.8f, -0.8f, 1);
    }

    /* Boost sparks: high speed jittery incandescent embers */
    if (p_randf() < 0.65f) {
        Vector3 p = Vector3Add(back, p_randv3(-0.04f, 0.04f));
        Vector3 vel = Vector3Add(Vector3Scale(c->vel, 0.50f),
                                Vector3Scale(bk, p_range(22.0f, 38.0f)));
        vel = Vector3Add(vel, p_randv3(-4.5f, 4.5f));

        Color c0 = (Color){ 255, 255, 230, 255 };
        Color c1 = isBlue ? (Color){ 80, 180, 255, 0 } : (Color){ 255, 120, 25, 0 };
        float s0 = p_range(0.08f, 0.14f);
        float s1 = 0.02f;
        float life = p_range(0.22f, 0.42f);

        particle_spawn(p, vel, c0, c1, s0, s1, life, 1.8f, 12.0f, 1);
    }
}

static void particles_drift_emit(const Car *c, float latSpeed, int sliding)
{
    if (c->wheelsOnGround < 2) return;
    float speed = Vector3Length(c->vel);
    if (latSpeed < 2.5f && (!sliding || speed < 4.5f)) return;

    Vector3 wl = Vector3Add(c->pos, car_to_world(c, c->wheelLocal[2]));
    Vector3 wr = Vector3Add(c->pos, car_to_world(c, c->wheelLocal[3]));
    wl.y = fmaxf(0.06f, wl.y);
    wr.y = fmaxf(0.06f, wr.y);

    Vector3 wheels[2] = { wl, wr };
    int w;
    for (w = 0; w < 2; w++) {
        if (p_randf() < 0.75f) {
            Vector3 p = Vector3Add(wheels[w], p_randv3(-0.1f, 0.1f));
            Vector3 vel = Vector3Add(Vector3Scale(c->vel, 0.12f),
                                    V3(p_range(-0.5f, 0.5f), p_range(0.6f, 1.8f), p_range(-0.5f, 0.5f)));
            Color c0 = (Color){ 200, 205, 215, (unsigned char)clampf(latSpeed * 22.0f + 60.0f, 60.0f, 160.0f) };
            Color c1 = (Color){ 160, 165, 175, 0 };
            float s0 = p_range(0.25f, 0.40f);
            float s1 = p_range(0.85f, 1.35f);
            float life = p_range(0.35f, 0.65f);

            particle_spawn(p, vel, c0, c1, s0, s1, life, 2.2f, -1.2f, 0);
        }

        if (latSpeed > 5.5f && p_randf() < 0.5f) {
            Vector3 sp = wheels[w];
            Vector3 svel = Vector3Add(Vector3Scale(c->vel, 0.4f),
                                     V3(p_range(-3.5f, 3.5f), p_range(1.5f, 5.0f), p_range(-3.5f, 3.5f)));
            Color sc0 = (Color){ 255, 220, 80, 255 };
            Color sc1 = (Color){ 255, 60, 10, 0 };
            particle_spawn(sp, svel, sc0, sc1, 0.08f, 0.02f, p_range(0.15f, 0.30f), 1.5f, 14.0f, 1);
        }
    }
}

static void particles_supersonic_emit(const Car *c)
{
    float speed = Vector3Length(c->vel);
    if (speed < 38.0f) return;

    Vector3 wl = Vector3Add(c->pos, car_to_world(c, c->wheelLocal[2]));
    Vector3 wr = Vector3Add(c->pos, car_to_world(c, c->wheelLocal[3]));
    Vector3 wheels[2] = { wl, wr };
    int w;
    for (w = 0; w < 2; w++) {
        if (p_randf() < 0.8f) {
            Vector3 p = Vector3Add(wheels[w], p_randv3(-0.06f, 0.06f));
            Vector3 vel = Vector3Scale(c->vel, 0.08f);
            Color c0 = (Color){ 200, 245, 255, 210 };
            Color c1 = (Color){ 70, 150, 255, 0 };
            float s0 = p_range(0.35f, 0.50f);
            float s1 = 0.08f;
            particle_spawn(p, vel, c0, c1, s0, s1, 0.22f, 0.5f, 0.0f, 1);
        }
    }
}

static void particles_impact_burst(Vector3 pos, Vector3 normal, float relSpeed)
{
    int numSparks = (int)clampf(relSpeed * 3.0f, 16.0f, 50.0f);
    int k;

    /* Central shockwave flash */
    for (k = 0; k < 2; k++) {
        Vector3 p = Vector3Add(pos, p_randv3(-0.1f, 0.1f));
        Vector3 vel = Vector3Scale(normal, p_range(1.0f, 3.0f));
        Color c0 = (Color){ 255, 255, 255, 255 };
        Color c1 = (Color){ 255, 180, 80, 0 };
        particle_spawn(p, vel, c0, c1, 0.4f, 1.8f, 0.12f, 2.0f, 0.0f, 1);
    }

    /* High speed sparks */
    for (k = 0; k < numSparks; k++) {
        Vector3 dir = Vector3Normalize(Vector3Add(Vector3Scale(normal, p_range(0.6f, 1.4f)), p_rand_dir()));
        float spd = p_range(12.0f, 32.0f) * fminf(relSpeed / 15.0f, 1.6f);
        Vector3 vel = Vector3Scale(dir, spd);
        Color c0 = (Color){ 255, 245, 180, 255 };
        Color c1 = (Color){ 255, 90, 15, 0 };
        float s0 = p_range(0.10f, 0.18f);
        float s1 = 0.02f;
        float life = p_range(0.25f, 0.55f);
        particle_spawn(pos, vel, c0, c1, s0, s1, life, 2.2f, 13.0f, 1);
    }

    /* Dust puff */
    for (k = 0; k < 6; k++) {
        Vector3 vel = Vector3Add(Vector3Scale(normal, p_range(1.0f, 4.0f)), p_randv3(-2.0f, 2.0f));
        Color c0 = (Color){ 190, 195, 205, 120 };
        Color c1 = (Color){ 140, 145, 155, 0 };
        particle_spawn(pos, vel, c0, c1, 0.35f, 1.10f, 0.35f, 3.0f, -0.5f, 0);
    }
}

static void particles_pad_pickup(Vector3 padPos)
{
    int k;
    Vector3 center = V3(padPos.x, 0.25f, padPos.z);

    /* Shock ring bursting upward and outward */
    for (k = 0; k < 28; k++) {
        float angle = (float)k / 28.0f * 2.0f * PI + p_range(-0.1f, 0.1f);
        float spd = p_range(6.0f, 14.0f);
        Vector3 pos = Vector3Add(center, V3(cosf(angle) * 0.8f, 0.1f, sinf(angle) * 0.8f));
        Vector3 vel = V3(cosf(angle) * spd, p_range(5.0f, 12.0f), sinf(angle) * spd);

        Color c0 = (Color){ 255, 215, 60, 255 };
        Color c1 = (Color){ 255, 100, 15, 0 };
        float s0 = p_range(0.20f, 0.35f);
        float s1 = p_range(0.40f, 0.70f);
        float life = p_range(0.40f, 0.65f);
        particle_spawn(pos, vel, c0, c1, s0, s1, life, 2.5f, 6.0f, 1);
    }

    /* Rising central golden spark fountain */
    for (k = 0; k < 16; k++) {
        Vector3 p = Vector3Add(center, p_randv3(-0.4f, 0.4f));
        Vector3 vel = V3(p_range(-1.8f, 1.8f), p_range(12.0f, 22.0f), p_range(-1.8f, 1.8f));
        Color c0 = (Color){ 255, 255, 210, 255 };
        Color c1 = (Color){ 255, 140, 20, 0 };
        particle_spawn(p, vel, c0, c1, 0.16f, 0.03f, p_range(0.45f, 0.75f), 2.2f, 12.0f, 1);
    }
}

static void particles_goal_explosion(Vector3 goalPos, int scorerTeam)
{
    int k;
    int isBlue = (scorerTeam == 1);

    /* 1. Core expanding shockwave flash */
    for (k = 0; k < 6; k++) {
        Vector3 p = Vector3Add(goalPos, p_randv3(-0.4f, 0.4f));
        Color c0 = (Color){ 255, 255, 255, 255 };
        Color c1 = isBlue ? (Color){ 100, 200, 255, 0 } : (Color){ 255, 160, 35, 0 };
        particle_spawn(p, V3(0, 0, 0), c0, c1, 2.5f, 18.0f, 0.28f, 0.8f, 0.0f, 1);
    }

    /* 2. Expanding horizontal shockwave ring */
    for (k = 0; k < 28; k++) {
        float a = (float)k / 28.0f * 2.0f * PI;
        Vector3 dir = V3(cosf(a), p_range(-0.1f, 0.1f), sinf(a));
        float spd = p_range(28.0f, 48.0f);
        Vector3 p = Vector3Add(goalPos, Vector3Scale(dir, 1.2f));
        Vector3 vel = Vector3Scale(dir, spd);
        Color c0 = (Color){ 255, 255, 255, 240 };
        Color c1 = isBlue ? (Color){ 40, 140, 255, 0 } : (Color){ 255, 100, 15, 0 };
        particle_spawn(p, vel, c0, c1, 1.4f, 7.5f, 0.45f, 2.0f, 0.0f, 1);
    }

    /* 3. Massive expanding fireball / plasma clouds */
    for (k = 0; k < 90; k++) {
        Vector3 dir = p_rand_dir();
        if (goalPos.z > 0.0f) dir.z = -fabsf(dir.z) * 1.5f;
        else                  dir.z =  fabsf(dir.z) * 1.5f;
        dir = Vector3Normalize(dir);

        float spd = p_range(16.0f, 52.0f);
        Vector3 vel = Vector3Scale(dir, spd);
        Color c0 = isBlue ? (Color){ 190, 245, 255, 255 } : (Color){ 255, 235, 130, 255 };
        Color c1 = isBlue ? (Color){ 15, 50, 230, 0 }      : (Color){ 220, 25, 5, 0 };
        float s0 = p_range(2.0f, 3.8f);
        float s1 = p_range(9.0f, 16.0f);
        float life = p_range(0.70f, 1.45f);
        particle_spawn(goalPos, vel, c0, c1, s0, s1, life, 2.4f, -3.0f, 1);
    }

    /* 4. Blazing high-speed fireworks / incandescent sparks */
    for (k = 0; k < 160; k++) {
        Vector3 dir = p_rand_dir();
        if (goalPos.z > 0.0f) dir.z = -fabsf(dir.z) * 1.4f;
        else                  dir.z =  fabsf(dir.z) * 1.4f;
        dir = Vector3Normalize(dir);

        float spd = p_range(35.0f, 95.0f);
        Vector3 vel = Vector3Scale(dir, spd);
        Color c0 = (Color){ 255, 255, 245, 255 };
        Color c1 = isBlue ? (Color){ 80, 200, 255, 0 } : (Color){ 255, 150, 30, 0 };
        float s0 = p_range(0.24f, 0.45f);
        float s1 = 0.04f;
        float life = p_range(1.0f, 2.2f);
        particle_spawn(goalPos, vel, c0, c1, s0, s1, life, 1.4f, 14.0f, 1);
    }

    /* 5. Billowing heavy smoke clouds */
    for (k = 0; k < 45; k++) {
        Vector3 dir = p_rand_dir();
        dir.y = fabsf(dir.y) * 1.2f + 0.4f;
        if (goalPos.z > 0.0f) dir.z = -fabsf(dir.z);
        else                  dir.z =  fabsf(dir.z);
        Vector3 vel = Vector3Scale(Vector3Normalize(dir), p_range(8.0f, 26.0f));
        Color c0 = (Color){ 100, 100, 110, 160 };
        Color c1 = (Color){ 40, 40, 45, 0 };
        float s0 = p_range(2.0f, 3.5f);
        float s1 = p_range(7.0f, 12.0f);
        float life = p_range(1.4f, 2.4f);
        particle_spawn(goalPos, vel, c0, c1, s0, s1, life, 1.8f, -2.0f, 0);
    }
}

static void particles_demolition_explosion(Vector3 pos)
{
    int k;

    /* 1. Core incandescent flash */
    for (k = 0; k < 4; k++) {
        Vector3 p = Vector3Add(pos, p_randv3(-0.2f, 0.2f));
        Color c0 = (Color){ 255, 255, 255, 255 };
        Color c1 = (Color){ 255, 180, 50, 0 };
        particle_spawn(p, V3(0, 0, 0), c0, c1, 1.8f, 9.0f, 0.20f, 1.2f, 0.0f, 1);
    }

    /* 2. Expanding shockwave ring */
    for (k = 0; k < 20; k++) {
        float a = (float)k / 20.0f * 2.0f * PI;
        Vector3 dir = V3(cosf(a), p_range(-0.1f, 0.1f), sinf(a));
        float spd = p_range(18.0f, 32.0f);
        Vector3 p = Vector3Add(pos, Vector3Scale(dir, 0.8f));
        Vector3 vel = Vector3Scale(dir, spd);
        Color c0 = (Color){ 255, 240, 160, 240 };
        Color c1 = (Color){ 255, 80, 10, 0 };
        particle_spawn(p, vel, c0, c1, 0.8f, 3.8f, 0.35f, 2.5f, 0.0f, 1);
    }

    /* 3. Fiery blast clouds */
    for (k = 0; k < 50; k++) {
        Vector3 dir = p_rand_dir();
        float spd = p_range(10.0f, 34.0f);
        Vector3 vel = Vector3Scale(dir, spd);
        Color c0 = (Color){ 255, 230, 90, 250 };
        Color c1 = (Color){ 210, 30, 5, 0 };
        float s0 = p_range(1.2f, 2.2f);
        float s1 = p_range(3.8f, 6.5f);
        float life = p_range(0.50f, 1.05f);
        particle_spawn(pos, vel, c0, c1, s0, s1, life, 3.0f, -2.0f, 1);
    }

    /* 4. High-velocity shrapnel sparks & glowing debris */
    for (k = 0; k < 90; k++) {
        Vector3 dir = p_rand_dir();
        float spd = p_range(25.0f, 65.0f);
        Vector3 vel = Vector3Scale(dir, spd);
        Color c0 = (Color){ 255, 255, 220, 255 };
        Color c1 = (Color){ 255, 110, 20, 0 };
        float s0 = p_range(0.18f, 0.32f);
        float s1 = 0.03f;
        float life = p_range(0.65f, 1.40f);
        particle_spawn(pos, vel, c0, c1, s0, s1, life, 1.8f, 16.0f, 1);
    }

    /* 5. Billowing black smoke plume */
    for (k = 0; k < 25; k++) {
        Vector3 dir = p_rand_dir();
        dir.y = fabsf(dir.y) + 0.5f;
        Vector3 vel = Vector3Scale(Vector3Normalize(dir), p_range(5.0f, 18.0f));
        Color c0 = (Color){ 80, 80, 85, 160 };
        Color c1 = (Color){ 30, 30, 35, 0 };
        float s0 = p_range(1.2f, 2.4f);
        float s1 = p_range(4.0f, 7.0f);
        float life = p_range(1.0f, 1.8f);
        particle_spawn(pos, vel, c0, c1, s0, s1, life, 2.2f, -1.8f, 0);
    }
}

/* Check if car A demolishes car B upon contact */
static void demolish_car(int victimIdx, int attackerIdx, Car *cars, const int *carModel,
                         float *demoBannerTimer, char *demoBannerText, int textLen, Color *bannerColor, float *camShake)
{
    Car *vic = &cars[victimIdx];
    vic->demolished = 1;
    vic->demoTimer = 3.0f;
    vic->vel = V3(0, 0, 0);
    vic->angVel = V3(0, 0, 0);
    vic->boosting = 0;

    particles_demolition_explosion(vic->pos);
    *camShake = 0.65f;

    if (victimIdx == 0) {
        snprintf(demoBannerText, textLen, "DEMOLISHED BY %s!",
                 attackerIdx >= 0 ? CAR_NAMES[carModel[attackerIdx] < 0 ? 0 : carModel[attackerIdx]] : "OPPONENT");
        *bannerColor = (Color){ 255, 60, 40, 255 };
        *demoBannerTimer = 3.0f;
    } else if (attackerIdx == 0) {
        snprintf(demoBannerText, textLen, "DEMOLITION! [%s]",
                 CAR_NAMES[carModel[victimIdx] < 0 ? 0 : carModel[victimIdx]]);
        *bannerColor = (Color){ 255, 185, 45, 255 };
        *demoBannerTimer = 2.5f;
    }
}

/* ------------------------------------------------------------------------ */
/* Headless physics self-test (sarpbc.exe --test)                             */
/* ------------------------------------------------------------------------ */
static void sim(Car *c, Input in, float seconds, int jumpAtStart)
{
    int n = (int)(seconds * PHYS_HZ), k;
    for (k = 0; k < n; k++) {
        Input t = in;
        t.jumpPressed = (k == 0 && jumpAtStart);
        car_step(c, &t, 1.0f / PHYS_HZ);
    }
}

static void run_physics_test(Car *c)
{
    Input in;
    Ball b;
    float maxY, y0, t;
    int k, touched = 0;

    car_reset(c, V3(0, 0, 0), 0.0f);
    memset(&in, 0, sizeof(in));
    sim(c, in, 2.0f, 0);
    printf("rest:   y=%.3f  wheels=%d  (box bottom %.3f, wheel r %.3f)\n",
           c->pos.y, c->wheelsOnGround, c->boxMin.y, c->wheelRadius);
    y0 = c->pos.y;

    in.jump = 1; maxY = y0;
    for (k = 0; k < (int)(2.0f * PHYS_HZ); k++) {
        Input s = in; s.jumpPressed = k == 0;
        car_step(c, &s, 1.0f / PHYS_HZ);
        if (c->pos.y > maxY) maxY = c->pos.y;
    }
    printf("jump (held):    apex %.2f m\n", maxY - y0);
    memset(&in, 0, sizeof(in)); sim(c, in, 2.0f, 0);

    in.jump = 1; maxY = y0;
    for (k = 0; k < (int)(3.0f * PHYS_HZ); k++) {
        Input s = in; s.jumpPressed = (k == 0 || k == (int)(0.25f * PHYS_HZ));
        if (k == 1) s.jump = 1;
        car_step(c, &s, 1.0f / PHYS_HZ);
        if (c->pos.y > maxY) maxY = c->pos.y;
    }
    printf("double jump:    apex %.2f m\n", maxY - y0);
    memset(&in, 0, sizeof(in)); sim(c, in, 2.0f, 0);

    car_reset(c, V3(0, 0, -100), -PI / 2.0f);
    memset(&in, 0, sizeof(in)); sim(c, in, 1.0f, 0);
    in.throttle = 1;
    sim(c, in, 1.0f, 0); printf("throttle 1s:    %.1f m/s\n", Vector3Length(c->vel));
    sim(c, in, 4.0f, 0); printf("throttle 5s:    %.1f m/s (no boost)\n", Vector3Length(c->vel));
    in.boost = 1;
    sim(c, in, 3.0f, 0); printf("boost 3s:       %.1f m/s, boost left %.2f\n", Vector3Length(c->vel), c->boost);

    car_reset(c, V3(0, 0, 0), 0.0f);
    memset(&in, 0, sizeof(in)); sim(c, in, 1.0f, 0);
    in.throttle = 1; sim(c, in, 1.5f, 0);
    {
        float sp = Vector3Length(c->vel), w;
        in.steer = 1; sim(c, in, 1.0f, 0);
        w = fabsf(c->angVel.y);
        printf("full lock @%.1f m/s: yaw %.2f rad/s, radius %.1f m (wheelbase %.2f m)\n", sp, w, w > 0 ? Vector3Length(c->vel) / w : 0, c->wheelBase);
        in.slide = 1; sim(c, in, 1.0f, 0);
        printf("  + powerslide 1s: yaw %.2f rad/s, speed %.1f m/s, sideways %.1f m/s\n",
               fabsf(c->angVel.y), Vector3Length(c->vel), fabsf(Vector3DotProduct(c->vel, car_right(c))));
    }

    /* forward dodge from a standstill */
    car_reset(c, V3(0, 0, 0), 0.0f);
    memset(&in, 0, sizeof(in)); sim(c, in, 1.0f, 0);
    sim(c, in, 0.15f, 1);
    in.pitch = -1;
    {
        Input s = in; s.jumpPressed = 1;
        car_step(c, &s, 1.0f / PHYS_HZ);
    }
    printf("fwd dodge:      horizontal %.1f m/s\n", sqrtf(c->vel.x*c->vel.x + c->vel.z*c->vel.z));
    memset(&in, 0, sizeof(in)); sim(c, in, 2.0f, 0);
    printf("  after landing: up.y %.2f wheels %d\n", car_up(c).y, c->wheelsOnGround);

    car_reset(c, V3(0, 0, 0), 0.0f);
    c->pos.y = 3.0f; c->rot = QuaternionFromAxisAngle(V3(1, 0, 0), 1.2f);
    memset(&in, 0, sizeof(in)); sim(c, in, 2.0f, 0);
    printf("tilted 69deg drop: up.y %.2f wheels %d\n", car_up(c).y, c->wheelsOnGround);

    car_reset(c, V3(0, 0, 0), 0.0f);
    c->pos.y = 2.0f; c->rot = QuaternionFromAxisAngle(V3(1, 0, 0), PI);
    memset(&in, 0, sizeof(in)); sim(c, in, 4.0f, 0);
    printf("roof drop:      up.y %.2f wheels %d (should stay on the roof)\n", car_up(c).y, c->wheelsOnGround);
    {
        Input s = in; s.jumpPressed = 1; s.jump = 1;
        car_step(c, &s, 1.0f / PHYS_HZ);
        sim(c, in, 2.0f, 0);
        printf("  + jump:       up.y %.2f wheels %d (should be back on its wheels)\n", car_up(c).y, c->wheelsOnGround);
    }
    car_reset(c, V3(0, 0, 0), 0.0f);
    c->pos.y = 2.0f; c->rot = QuaternionFromAxisAngle(V3(1, 0, 0), PI / 2.0f);
    memset(&in, 0, sizeof(in)); sim(c, in, 3.0f, 0);
    printf("side drop:      up.y %.2f wheels %d", car_up(c).y, c->wheelsOnGround);
    {
        Input s = in; s.jumpPressed = 1; s.jump = 1;
        car_step(c, &s, 1.0f / PHYS_HZ);
        sim(c, in, 2.0f, 0);
        printf("  -> jump: up.y %.2f wheels %d\n", car_up(c).y, c->wheelsOnGround);
    }

    car_reset(c, V3(0, 0, 0), 0.0f);
    c->pos.y = 8.0f; c->rot = QuaternionFromAxisAngle(V3(0, 0, 1), PI / 2.0f);   /* nose straight up */
    memset(&in, 0, sizeof(in)); in.boost = 1; c->boost = BOOST_MAX;
    sim(c, in, 1.0f, 0);
    printf("boost nose-up 1s: vy %.1f m/s (positive = can fly)\n", c->vel.y);

    /* drive into a resting ball at full throttle */
    car_reset(c, V3(0, 0, -30), -PI / 2.0f);
    ball_reset(&b);
    memset(&in, 0, sizeof(in)); in.throttle = 1;
    for (t = 0; t < 4.0f && !touched; t += 1.0f / PHYS_HZ) {
        car_step(c, &in, 1.0f / PHYS_HZ);
        ball_step(&b, 1.0f / PHYS_HZ);
        touched = car_ball_collide(c, &b);
    }
    for (k = 0; k < 4; k++) { ball_step(&b, 1.0f / PHYS_HZ); car_ball_collide(c, &b); }
    printf("ball hit:       touched=%d car %.1f m/s -> ball %.1f m/s\n", touched, Vector3Length(c->vel), Vector3Length(b.vel));

    /* drive at the side wall: should climb the ramp onto the wall */
    car_reset(c, V3(60, 0, 0), 0.0f);                 /* facing +X */
    memset(&in, 0, sizeof(in)); sim(c, in, 0.5f, 0);
    in.throttle = 1;
    maxY = 0;
    for (k = 0; k < (int)(6.0f * PHYS_HZ); k++) {
        car_step(c, &in, 1.0f / PHYS_HZ);
        if (c->pos.y > maxY) maxY = c->pos.y;
    }
    printf("wall drive:     max height %.1f m, now (%.1f, %.1f, %.1f) wheels %d up(%.2f %.2f %.2f)\n",
           maxY, c->pos.x, c->pos.y, c->pos.z, c->wheelsOnGround, car_up(c).x, car_up(c).y, car_up(c).z);

    /* ball rolled into the orange goal */
    ball_reset(&b); b.pos = V3(0, BALL_R, 120); b.vel = V3(0, 0, 30);
    for (k = 0; k < (int)(3.0f * PHYS_HZ); k++) ball_step(&b, 1.0f / PHYS_HZ);
    printf("ball into goal: z %.1f (goal line %.1f) y %.1f\n", b.pos.z, ARENA_L, b.pos.y);

    /* ball fired at the side glass */
    ball_reset(&b); b.pos = V3(50, 20, 0); b.vel = V3(40, 0, 0);
    for (k = 0; k < (int)(2.0f * PHYS_HZ); k++) ball_step(&b, 1.0f / PHYS_HZ);
    printf("ball off wall:  x %.1f vx %.1f (should be heading back, x < 102)\n", b.pos.x, b.vel.x);

    /* ball dropped from high: must stay inside the dome */
    ball_reset(&b); b.pos = V3(0, 50, 0); b.vel = V3(0, 30, 0);
    for (k = 0; k < (int)(3.0f * PHYS_HZ); k++) ball_step(&b, 1.0f / PHYS_HZ);
    printf("ball at ceiling: y %.1f vy %.1f\n", b.pos.y, b.vel.y);

    /* bots ------------------------------------------------------------------ */
    {
        static const float pxz[6][2] = { { -78, -(ARENA_L - 24) }, { 78, -(ARENA_L - 24) }, { -78, ARENA_L - 24 },
                                         { 78, ARENA_L - 24 }, { -82, 0 }, { 82, 0 } };
        Vector3 tp[6];
        float tpt[6];
        int skill, trial;
        for (k = 0; k < 6; k++) tp[k] = V3(pxz[k][0], 0, pxz[k][1]);

        /* shooting drill: one blue bot, empty net, ball dropped at 8 spots; time to score */
        for (skill = 0; skill < 3; skill++) {
            int scored = 0, own = 0;
            float total = 0.0f;
            for (trial = 0; trial < 8; trial++) {
                static const float spots[8][3] = { { 0, 1.62f, 40 }, { 40, 1.62f, 20 }, { -60, 6, 60 }, { 70, 1.62f, 100 },
                                                   { -30, 10, -20 }, { 0, 1.62f, -60 }, { 90, 3, -10 }, { -20, 1.62f, 125 } };
                Car cs[1];
                Bot bs[1];
                int tm[1] = { 0 };
                float t;
                cs[0] = *c; memset(bs, 0, sizeof(bs)); memset(tpt, 0, sizeof(tpt));
                car_reset(&cs[0], V3(0, 0, -KICKOFF_Z), -PI / 2.0f);
                ball_reset(&b); b.pos = V3(spots[trial][0], spots[trial][1], spots[trial][2]);
                g_predCalls = 0;
                for (t = 0; t < 30.0f; t += 1.0f / PHYS_HZ) {
                    Input bi;
                    bot_world_update(&b, 1.0f / PHYS_HZ, tp, tpt, 6);
                    bi = bot_think(0, cs, tm, 1, &b, &bs[0], skill, 1.0f / PHYS_HZ);
                    car_step(&cs[0], &bi, 1.0f / PHYS_HZ);
                    ball_step(&b, 1.0f / PHYS_HZ);
                    car_ball_collide(&cs[0], &b);
                    for (k = 0; k < 6; k++) {
                        tpt[k] -= 1.0f / PHYS_HZ;
                        if (tpt[k] <= 0 && flat_dist(cs[0].pos, tp[k]) < PAD_RADIUS) { cs[0].boost = BOOST_MAX; tpt[k] = PAD_RESPAWN; }
                    }
                    if (fabsf(b.pos.z) > ARENA_L + BALL_R) break;
                }
                if (b.pos.z > ARENA_L) { scored++; total += t; } else if (b.pos.z < -ARENA_L) own++;
                if (getenv("SARPBC_BOTTRACE"))
                    printf("   %s trial %d: %s t=%.1f ball(%.0f,%.0f,%.0f v%.0f) car(%.0f,%.0f,%.0f v%.0f) boost %.1f\n", SKILL_NAMES[skill], trial,
                           b.pos.z > ARENA_L ? "GOAL" : "miss", t, b.pos.x, b.pos.y, b.pos.z, Vector3Length(b.vel),
                           cs[0].pos.x, cs[0].pos.y, cs[0].pos.z, Vector3Length(cs[0].vel), cs[0].boost);
            }
            printf("bot drill %-8s: scored %d/8 (own goals %d), avg %.1f s\n", SKILL_NAMES[skill], scored, own, scored ? total / scored : 0.0f);
        }

        /* matches: 2 v 2 at each skill, then All-Star vs Rookie and Pro vs Rookie */
        for (skill = 0; skill < 5; skill++) {
            Car cs[4];
            Bot bs[4];
            int tm[4] = { 0, 0, 1, 1 }, goals[2] = { 0, 0 }, touches[4] = { 0 }, jumps = 0, m;
            int sk[2];
            sk[0] = skill < 3 ? skill : (skill == 3 ? 2 : 1);
            sk[1] = skill < 3 ? skill : 0;
            for (m = 0; m < 4; m++) cs[m] = *c;
#define BOT_KICKOFF() do { for (m = 0; m < 4; m++) { Vector3 p_; float y_; kickoff_spot(m % 2, tm[m], 2, &p_, &y_); car_reset(&cs[m], p_, y_); } \
                           memset(bs, 0, sizeof(bs)); memset(tpt, 0, sizeof(tpt)); ball_reset(&b); g_predCalls = 0; } while (0)
            BOT_KICKOFF();
            for (k = 0; k < (int)(180.0f * PHYS_HZ); k++) {
                int q;
                bot_world_update(&b, 1.0f / PHYS_HZ, tp, tpt, 6);
                for (m = 0; m < 4; m++) {
                    Input bi = bot_think(m, cs, tm, 4, &b, &bs[m], sk[tm[m]], 1.0f / PHYS_HZ);
                    jumps += bi.jumpPressed;
                    car_step(&cs[m], &bi, 1.0f / PHYS_HZ);
                }
                ball_step(&b, 1.0f / PHYS_HZ);
                for (m = 0; m < 4; m++) touches[m] += car_ball_collide(&cs[m], &b);
                for (m = 0; m < 4; m++) { int n2; for (n2 = m + 1; n2 < 4; n2++) car_car_collide(&cs[m], &cs[n2]); }
                for (q = 0; q < 6; q++) {
                    tpt[q] -= 1.0f / PHYS_HZ;
                    for (m = 0; m < 4 && tpt[q] <= 0; m++)
                        if (flat_dist(cs[m].pos, tp[q]) < PAD_RADIUS && cs[m].pos.y < 4) { cs[m].boost = BOOST_MAX; tpt[q] = PAD_RESPAWN; }
                }
                if (fabsf(b.pos.z) > ARENA_L + BALL_R) { goals[b.pos.z > 0 ? 0 : 1]++; BOT_KICKOFF(); }
                if (getenv("SARPBC_BOTTRACE") && skill == atoi(getenv("SARPBC_BOTTRACE")) && k % (int)(2.0f * PHYS_HZ) == 0)
                    printf("  t=%3.0f ball(%6.1f %5.1f %6.1f) | b0(%.0f,%.0f,%.0f v%.0f r%d) b1(%.0f,%.0f,%.0f v%.0f r%d) o0(%.0f,%.0f,%.0f v%.0f r%d)\n",
                           k / PHYS_HZ, b.pos.x, b.pos.y, b.pos.z,
                           cs[0].pos.x, cs[0].pos.y, cs[0].pos.z, Vector3Length(cs[0].vel), bs[0].role,
                           cs[1].pos.x, cs[1].pos.y, cs[1].pos.z, Vector3Length(cs[1].vel), bs[1].role,
                           cs[2].pos.x, cs[2].pos.y, cs[2].pos.z, Vector3Length(cs[2].vel), bs[2].role);
            }
#undef BOT_KICKOFF
            printf("bots 2v2 %-8s vs %-8s 3 min: blue %d - %d orange, touch ticks %d/%d/%d/%d, jumps %d\n",
                   SKILL_NAMES[sk[0]], SKILL_NAMES[sk[1]], goals[0], goals[1], touches[0], touches[1], touches[2], touches[3], jumps);
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Car loading (startup + menu car select)                                    */
/* ------------------------------------------------------------------------ */
/* looks next to the exe, then in ../export_c; fills dir (with trailing slash) */
static int load_car(const char *name, Car *car, CarRender *cr, Shader lit, int render)
{
    char dir[512], path[1024];
    SarmModel sm;
    int err;
    snprintf(dir, sizeof(dir), "%sassets/cars/%s/", GetApplicationDirectory(), name);
    snprintf(path, sizeof(path), "%s%s.sarm", dir, name);
    if (!FileExists(path)) {
        snprintf(dir, sizeof(dir), "%s../export_c/cars/%s/", GetApplicationDirectory(), name);
        snprintf(path, sizeof(path), "%s%s.sarm", dir, name);
    }
    if (!FileExists(path)) {
        snprintf(dir, sizeof(dir), "assets/cars/%s/", name);
        snprintf(path, sizeof(path), "%s%s.sarm", dir, name);
    }
    if (!FileExists(path)) {
        snprintf(dir, sizeof(dir), "../export_c/cars/%s/", name);
        snprintf(path, sizeof(path), "%s%s.sarm", dir, name);
    }
    if (!FileExists(path)) {
        snprintf(dir, sizeof(dir), "export_c/cars/%s/", name);
        snprintf(path, sizeof(path), "%s%s.sarm", dir, name);
    }
    err = sarm_load(path, &sm);
    if (err != SARM_OK) {
        TraceLog(LOG_ERROR, "sarm_load(%s): %s", path, sarm_error_string(err));
        return 0;
    }
    car_init(car, &sm);
    if (render) { car_render_unload(cr); car_render_load(cr, &sm, dir, lit); }
    sarm_free(&sm);
    return 1;
}

/* ------------------------------------------------------------------------ */
/* Menus                                                                      */
/* ------------------------------------------------------------------------ */
typedef enum { SCR_MENU, SCR_ONLINE_JOIN, SCR_SETTINGS, SCR_GAME, SCR_PAUSE } Screen;

typedef struct Nav { int up, down, left, right, ok, back; } Nav;

static Nav read_nav(void)
{
    static int stickHeld = 0;
    Nav n;
    memset(&n, 0, sizeof(n));
    n.up    = IsKeyPressed(KEY_UP)    || IsKeyPressed(KEY_W);
    n.down  = IsKeyPressed(KEY_DOWN)  || IsKeyPressed(KEY_S);
    n.left  = IsKeyPressed(KEY_LEFT)  || IsKeyPressed(KEY_A);
    n.right = IsKeyPressed(KEY_RIGHT) || IsKeyPressed(KEY_D);
    n.ok    = IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER) || IsKeyPressed(KEY_SPACE);
    n.back  = IsKeyPressed(KEY_ESCAPE) || IsKeyPressed(KEY_BACKSPACE);
    if (IsGamepadAvailable(0)) {
        float gx = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X);
        float gy = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y);
        int dir = gy < -0.6f ? 1 : gy > 0.6f ? 2 : gx < -0.6f ? 3 : gx > 0.6f ? 4 : 0;
        if (dir && dir != stickHeld) { n.up |= dir == 1; n.down |= dir == 2; n.left |= dir == 3; n.right |= dir == 4; }
        stickHeld = dir;
        n.up    |= IsGamepadButtonPressed(0, GAMEPAD_BUTTON_LEFT_FACE_UP);
        n.down  |= IsGamepadButtonPressed(0, GAMEPAD_BUTTON_LEFT_FACE_DOWN);
        n.left  |= IsGamepadButtonPressed(0, GAMEPAD_BUTTON_LEFT_FACE_LEFT);
        n.right |= IsGamepadButtonPressed(0, GAMEPAD_BUTTON_LEFT_FACE_RIGHT);
        n.ok    |= IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_FACE_DOWN);
        n.back  |= IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_FACE_RIGHT);
    }
    return n;
}

typedef struct MenuItem { const char *label; char value[48]; } MenuItem;

/* Procedural authentic SARPBC high-tech soccer ball texture (1024x512 equirectangular) */
static Texture2D make_tech_ball_texture(void)
{
    const int w = 1024, h = 1024;
    unsigned char *px = (unsigned char *)MemAlloc(w * h * 4);
    if (!px) return (Texture2D){ 0 };

    /* 12 pentagon centers (icosahedron vertices) */
    Vector3 pent[12];
    float phi = (1.0f + sqrtf(5.0f)) * 0.5f;
    int idx = 0;
    for (int s1 = -1; s1 <= 1; s1 += 2) {
        for (int s2 = -1; s2 <= 1; s2 += 2) {
            pent[idx++] = Vector3Normalize(V3(0.0f, (float)s1, (float)s2 * phi));
            pent[idx++] = Vector3Normalize(V3((float)s1, (float)s2 * phi, 0.0f));
            pent[idx++] = Vector3Normalize(V3((float)s1 * phi, 0.0f, (float)s2));
        }
    }

    /* 20 hexagon centers (icosahedron face centers) */
    Vector3 hex[20];
    idx = 0;
    for (int i = 0; i < 12; i++) {
        for (int j = i + 1; j < 12; j++) {
            float d = Vector3DotProduct(pent[i], pent[j]);
            if (d > 0.4f && d < 0.6f) {
                for (int k = j + 1; k < 12; k++) {
                    float d2 = Vector3DotProduct(pent[i], pent[k]);
                    float d3 = Vector3DotProduct(pent[j], pent[k]);
                    if (d2 > 0.4f && d2 < 0.6f && d3 > 0.4f && d3 < 0.6f) {
                        Vector3 c = Vector3Normalize(V3(pent[i].x + pent[j].x + pent[k].x,
                                                       pent[i].y + pent[j].y + pent[k].y,
                                                       pent[i].z + pent[j].z + pent[k].z));
                        int dup = 0;
                        for (int m = 0; m < idx; m++) {
                            if (Vector3DotProduct(hex[m], c) > 0.99f) { dup = 1; break; }
                        }
                        if (!dup && idx < 20) hex[idx++] = c;
                    }
                }
            }
        }
    }

    const float seam_w  = 0.016f;
    const float glow_w  = 0.046f;
    const float bevel_w = 0.082f;

    for (int y = 0; y < h; y++) {
        float v = ((float)y + 0.5f) / (float)h;
        float phiA = v * 2.0f * PI;
        float sinP = sinf(phiA), cosP = cosf(phiA);
        for (int x = 0; x < w; x++) {
            float u = ((float)x + 0.5f) / (float)w;
            float theta = u * PI;
            float sinT = sinf(theta), cosT = cosf(theta);
            Vector3 p = V3(sinT * cosP, sinT * sinP, cosT);

            float bdot1 = -2.0f, bdot2 = -2.0f;
            int is_pent1 = 0;
            int p_idx1 = 0;

            for (int i = 0; i < 12; i++) {
                float d = Vector3DotProduct(p, pent[i]);
                if (d > bdot1) {
                    bdot2 = bdot1;
                    bdot1 = d; is_pent1 = 1; p_idx1 = i;
                } else if (d > bdot2) {
                    bdot2 = d;
                }
            }
            for (int j = 0; j < 20; j++) {
                float d = Vector3DotProduct(p, hex[j]);
                if (d > bdot1) {
                    bdot2 = bdot1;
                    bdot1 = d; is_pent1 = 0;
                } else if (d > bdot2) {
                    bdot2 = d;
                }
            }

            float ang1 = acosf(fminf(1.0f, fmaxf(-1.0f, bdot1)));
            float ang2 = acosf(fminf(1.0f, fmaxf(-1.0f, bdot2)));
            float edge_dist = (ang2 - ang1) * 0.5f;

            float r = 0, g = 0, b = 0;

            if (edge_dist < seam_w) {
                /* Deep dark recessed panel seam */
                float groove = edge_dist / seam_w;
                r = 18.0f + 14.0f * groove;
                g = 20.0f + 15.0f * groove;
                b = 26.0f + 18.0f * groove;
            } else if (edge_dist < glow_w) {
                /* Glowing neon energy circuitry tracks */
                float t = (edge_dist - seam_w) / (glow_w - seam_w);
                float glow = sinf(t * PI);
                if (is_pent1 && (p_idx1 & 1)) {
                    /* Electric Amber/Orange energy seam */
                    r = 255.0f * glow + 180.0f * (1.0f - glow);
                    g = 140.0f * glow + 70.0f * (1.0f - glow);
                    b = 30.0f * glow;
                } else {
                    /* Electric Cyan / Turquoise energy seam */
                    r = 40.0f + 200.0f * powf(glow, 2.5f);
                    g = 175.0f + 80.0f * glow;
                    b = 255.0f * glow + 210.0f * (1.0f - glow);
                }
            } else {
                /* Panel interior plate */
                float bevel = edge_dist < bevel_w ? (edge_dist - glow_w) / (bevel_w - glow_w) : 1.0f;
                bevel = 0.58f + 0.42f * bevel;

                int pat = ((int)(x * 3) ^ (int)(y * 3)) & 3;
                float micro = 1.0f - 0.05f * (float)pat;

                if (is_pent1) {
                    /* Carbon-fiber / stealth composite pentagon with glowing center emblem */
                    if (ang1 < 0.155f && ang1 > 0.105f) {
                        float ring_glow = 1.0f - fabsf(ang1 - 0.130f) / 0.025f;
                        if (p_idx1 & 1) {
                            r = 255.0f; g = 140.0f + 80.0f * ring_glow; b = 30.0f;
                        } else {
                            r = 40.0f + 180.0f * ring_glow; g = 190.0f + 65.0f * ring_glow; b = 255.0f;
                        }
                    } else if (ang1 <= 0.105f) {
                        r = 46.0f; g = 52.0f; b = 64.0f;
                    } else {
                        r = 40.0f * bevel * micro;
                        g = 44.0f * bevel * micro;
                        b = 54.0f * bevel * micro;
                    }
                } else {
                    /* Brushed titanium / silver hexagon plate */
                    float base_col = 205.0f * bevel * micro + 22.0f * cosf(ang1 * 8.0f);
                    r = fminf(255.0f, base_col);
                    g = fminf(255.0f, base_col * 1.02f);
                    b = fminf(255.0f, base_col * 1.06f);
                }
            }

            int pidx = (y * w + x) * 4;
            px[pidx + 0] = (unsigned char)fminf(255.0f, fmaxf(0.0f, r));
            px[pidx + 1] = (unsigned char)fminf(255.0f, fmaxf(0.0f, g));
            px[pidx + 2] = (unsigned char)fminf(255.0f, fmaxf(0.0f, b));
            px[pidx + 3] = 255;
        }
    }

    Image img = { px, w, h, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
    ExportImage(img, "tech_ball.png");
    Texture2D tex = LoadTextureFromImage(img);
    GenTextureMipmaps(&tex);
    SetTextureFilter(tex, TEXTURE_FILTER_TRILINEAR);
    MemFree(px);
    TraceLog(LOG_INFO, "BALL: generated high-def authentic tech ball texture (1024x512)");
    return tex;
}

/* Modern Stadium Broadcast Scoreboard */
static void draw_hud_scoreboard(int sw, int scoreBlue, int scoreOrange, float matchTime, float matchLen, GameState state)
{
    int mins = (int)ceilf(fmaxf(0.0f, matchTime)) / 60, secs = (int)ceilf(fmaxf(0.0f, matchTime)) % 60;
    const char *clock = matchLen > 0.0f ? TextFormat("%d:%02d", mins, secs) : "FREE";
    if (state == ST_OVER && scoreBlue == scoreOrange) clock = "+0:00";

    int totalW = 380, totalH = 54;
    int x0 = sw / 2 - totalW / 2, y0 = 12;

    /* Soft drop shadow */
    DrawRectangleRounded((Rectangle){ (float)x0 + 2, (float)y0 + 3, (float)totalW, (float)totalH }, 0.28f, 8, (Color){ 0, 0, 0, 140 });

    /* Main container: dark frosted glass */
    DrawRectangleRounded((Rectangle){ (float)x0, (float)y0, (float)totalW, (float)totalH }, 0.28f, 8, (Color){ 12, 16, 26, 235 });
    DrawRectangleRoundedLinesEx((Rectangle){ (float)x0, (float)y0, (float)totalW, (float)totalH }, 0.28f, 8, 2.0f, (Color){ 45, 65, 95, 200 });

    /* Blue Team section (left) */
    Rectangle blueBadge = { (float)x0 + 6, (float)y0 + 6, 110, (float)totalH - 12 };
    DrawRectangleRounded(blueBadge, 0.25f, 6, (Color){ 24, 72, 165, 240 });
    DrawRectangleRoundedLinesEx(blueBadge, 0.25f, 6, 1.5f, (Color){ 65, 145, 255, 255 });
    DrawText("BLUE", (int)blueBadge.x + 12, (int)blueBadge.y + 13, 16, (Color){ 180, 215, 255, 255 });
    const char *bScoreStr = TextFormat("%d", scoreBlue);
    DrawText(bScoreStr, (int)(blueBadge.x + blueBadge.width - 16 - MeasureText(bScoreStr, 32)), (int)blueBadge.y + 5, 32, (Color){ 240, 248, 255, 255 });

    /* Orange Team section (right) */
    Rectangle orangeBadge = { (float)(x0 + totalW - 116), (float)y0 + 6, 110, (float)totalH - 12 };
    DrawRectangleRounded(orangeBadge, 0.25f, 6, (Color){ 195, 75, 18, 240 });
    DrawRectangleRoundedLinesEx(orangeBadge, 0.25f, 6, 1.5f, (Color){ 255, 135, 40, 255 });
    const char *oScoreStr = TextFormat("%d", scoreOrange);
    DrawText(oScoreStr, (int)orangeBadge.x + 16, (int)orangeBadge.y + 5, 32, (Color){ 255, 248, 240, 255 });
    DrawText("ORANGE", (int)(orangeBadge.x + orangeBadge.width - 12 - MeasureText("ORANGE", 16)), (int)orangeBadge.y + 13, 16, (Color){ 255, 215, 180, 255 });

    /* Center clock pod */
    Rectangle clockPod = { (float)x0 + 124, (float)y0 + 8, (float)totalW - 248, (float)totalH - 16 };
    DrawRectangleRounded(clockPod, 0.25f, 6, (Color){ 6, 8, 14, 255 });
    DrawRectangleRoundedLinesEx(clockPod, 0.25f, 6, 1.0f, (Color){ 32, 44, 64, 200 });

    Color clockCol = RAYWHITE;
    if (matchTime < 30.0f && matchLen > 0.0f) {
        float pulse = 0.5f + 0.5f * sinf((float)GetTime() * 8.0f);
        clockCol = ColorAlpha((Color){ 255, 180, 60, 255 }, 0.7f + 0.3f * pulse);
    }
    int cw = MeasureText(clock, 26);
    DrawText(clock, (int)(clockPod.x + clockPod.width/2 - cw/2), (int)clockPod.y + 6, 26, clockCol);
}

/* Curved Radial Boost Gauge & Speedometer */
static void draw_hud_boost_and_speed(int sw, int sh, float boost, float speed, int boosting)
{
    float bPct = fminf(1.0f, fmaxf(0.0f, boost / BOOST_MAX));
    int bInt = (int)(bPct * 100.0f + 0.5f);
    int isSupersonic = speed >= 37.0f;

    /* Boost radial gauge at bottom right */
    Vector2 center = { (float)(sw - 95), (float)(sh - 95) };
    float rOut = 62.0f, rIn = 48.0f;

    /* Outer drop shadow */
    DrawCircle((int)center.x + 2, (int)center.y + 3, rOut + 2, (Color){ 0, 0, 0, 120 });

    /* Background ring track */
    DrawRing(center, rIn, rOut, 40.0f, 320.0f, 48, (Color){ 18, 24, 34, 230 });
    DrawRingLines(center, rIn, rOut, 40.0f, 320.0f, 48, (Color){ 40, 55, 80, 180 });

    /* Active energy fill arc */
    float fillAngle = 40.0f + bPct * 280.0f;
    Color arcCol;
    if (isSupersonic) arcCol = (Color){ 40, 230, 255, 255 };
    else if (boosting) arcCol = (Color){ 255, 235, 60, 255 };
    else if (bPct > 0.33f) arcCol = (Color){ 255, 145, 25, 255 };
    else arcCol = (Color){ 255, 65, 25, 255 };

    if (bPct > 0.005f) {
        DrawRing(center, rIn + 1.0f, rOut - 1.0f, 40.0f, fillAngle, 48, arcCol);
    }

    /* Inner pod */
    DrawCircle((int)center.x, (int)center.y, rIn - 1.0f, (Color){ 10, 14, 22, 245 });
    DrawCircleLines((int)center.x, (int)center.y, rIn - 1.0f, (Color){ 35, 48, 70, 200 });

    /* Boost number readout */
    const char *bNumStr = TextFormat("%d", bInt);
    int bnw = MeasureText(bNumStr, 34);
    Color numCol = isSupersonic ? (Color){ 120, 240, 255, 255 } : boosting ? (Color){ 255, 255, 180, 255 } : RAYWHITE;
    DrawText(bNumStr, (int)center.x - bnw / 2, (int)center.y - 20, 34, numCol);
    DrawText("BOOST", (int)center.x - MeasureText("BOOST", 11) / 2, (int)center.y + 14, 11, (Color){ 170, 185, 205, 220 });

    /* Speedometer badge above gauge */
    float kmh = speed * 3.6f;
    int spX = sw - 165, spY = sh - 185, spW = 140, spH = 36;
    DrawRectangleRounded((Rectangle){ (float)spX + 2, (float)spY + 2, (float)spW, (float)spH }, 0.35f, 6, (Color){ 0, 0, 0, 110 });
    DrawRectangleRounded((Rectangle){ (float)spX, (float)spY, (float)spW, (float)spH }, 0.35f, 6, (Color){ 12, 16, 26, 225 });
    DrawRectangleRoundedLinesEx((Rectangle){ (float)spX, (float)spY, (float)spW, (float)spH }, 0.35f, 6, 1.5f, (Color){ 45, 65, 95, 190 });

    const char *kmhNum = TextFormat("%.0f", kmh);
    int spnw = MeasureText(kmhNum, 22);
    Color spCol = isSupersonic ? (Color){ 40, 230, 255, 255 } : kmh > 100.0f ? (Color){ 255, 165, 40, 255 } : RAYWHITE;
    DrawText(kmhNum, spX + 16, spY + 7, 22, spCol);
    DrawText("KM/H", spX + 20 + spnw, spY + 12, 13, (Color){ 160, 185, 210, 220 });

    /* Speed mini-bar indicator */
    float spRatio = fminf(1.0f, speed / 40.0f);
    DrawRectangle(spX + 16, spY + spH - 6, spW - 32, 3, (Color){ 25, 32, 45, 255 });
    DrawRectangle(spX + 16, spY + spH - 6, (int)((spW - 32) * spRatio), 3, spCol);

    /* Supersonic Overdrive Alert Banner */
    if (isSupersonic) {
        float flash = 0.6f + 0.4f * sinf((float)GetTime() * 12.0f);
        int sbW = 150, sbH = 26;
        int sbX = sw - 170, sbY = sh - 222;
        DrawRectangleRounded((Rectangle){ (float)sbX, (float)sbY, (float)sbW, (float)sbH }, 0.35f, 6, ColorAlpha((Color){ 0, 140, 240, 255 }, flash * 0.9f));
        DrawRectangleRoundedLinesEx((Rectangle){ (float)sbX, (float)sbY, (float)sbW, (float)sbH }, 0.35f, 6, 1.5f, ColorAlpha((Color){ 100, 230, 255, 255 }, flash));
        const char *ssText = "SUPERSONIC";
        int sstw = MeasureText(ssText, 14);
        DrawText(ssText, sbX + sbW/2 - sstw/2, sbY + 6, 14, (Color){ 255, 255, 255, 255 });
    }
}

/* Tactical HUD Badges (Ball Cam, Player Card, Controls) */
static void draw_hud_tactical_badges(int sw, int sh, int ballCam, const char *carName, const char *modeName, const char *skillName, int showFps, int showHints)
{
    (void)sw;
    /* Player Car Info Card (Top Left) */
    int cardW = 210, cardH = 48;
    DrawRectangleRounded((Rectangle){ 18, 14, (float)cardW, (float)cardH }, 0.28f, 6, (Color){ 12, 16, 26, 215 });
    DrawRectangleRoundedLinesEx((Rectangle){ 18, 14, (float)cardW, (float)cardH }, 0.28f, 6, 1.5f, (Color){ 45, 65, 95, 180 });
    DrawRectangleRounded((Rectangle){ 23, 19, 4, (float)cardH - 10 }, 0.4f, 4, (Color){ 55, 145, 255, 255 });
    DrawText(carName, 34, 19, 18, RAYWHITE);
    DrawText(TextFormat("%s  -  %s", modeName, skillName), 34, 40, 13, (Color){ 175, 190, 215, 220 });

    if (showFps) {
        int fps = GetFPS();
        const char *fpsStr = TextFormat("%d FPS", fps);
        int fpsW = MeasureText(fpsStr, 14) + 16;
        DrawRectangleRounded((Rectangle){ 18, 68, (float)fpsW, 22 }, 0.35f, 4, (Color){ 10, 14, 20, 200 });
        DrawText(fpsStr, 26, 72, 14, (Color){ 70, 235, 130, 240 });
    }

    /* Ball Cam Pill Badge (Bottom Left) */
    int bcW = 145, bcH = 32;
    int bcX = 18, bcY = sh - (showHints ? 66 : 46);
    DrawRectangleRounded((Rectangle){ (float)bcX + 2, (float)bcY + 2, (float)bcW, (float)bcH }, 0.35f, 6, (Color){ 0, 0, 0, 110 });
    if (ballCam) {
        DrawRectangleRounded((Rectangle){ (float)bcX, (float)bcY, (float)bcW, (float)bcH }, 0.35f, 6, (Color){ 16, 42, 85, 230 });
        DrawRectangleRoundedLinesEx((Rectangle){ (float)bcX, (float)bcY, (float)bcW, (float)bcH }, 0.35f, 6, 1.5f, (Color){ 45, 175, 255, 255 });
        DrawCircle(bcX + 16, bcY + bcH/2, 5, (Color){ 45, 210, 255, 255 });
        DrawText("BALL CAM", bcX + 28, bcY + 8, 15, RAYWHITE);
        DrawText("ON", bcX + 112, bcY + 9, 13, (Color){ 45, 210, 255, 255 });
    } else {
        DrawRectangleRounded((Rectangle){ (float)bcX, (float)bcY, (float)bcW, (float)bcH }, 0.35f, 6, (Color){ 18, 22, 30, 200 });
        DrawRectangleRoundedLinesEx((Rectangle){ (float)bcX, (float)bcY, (float)bcW, (float)bcH }, 0.35f, 6, 1.2f, (Color){ 50, 60, 75, 180 });
        DrawCircleLines(bcX + 16, bcY + bcH/2, 4.5f, (Color){ 160, 170, 185, 220 });
        DrawText("CAR CAM", bcX + 28, bcY + 8, 15, (Color){ 180, 190, 205, 220 });
    }

    /* Keybind hint bar */
    if (showHints) {
        DrawText("W/S Drive   A/D Steer   SPACE Jump/Dodge   SHIFT Boost   CTRL Slide/Roll   C Ball Cam   ESC Menu",
                 18, sh - 26, 15, (Color){ 175, 185, 200, 190 });
    }
}

/* Draws a modernized frosted-glass vertical menu card */
static void menu_run(const char *title, MenuItem *it, int n, int *sel, Nav nav, int *act, int *adj)
{
    static Vector2 lastMouse = { -1, -1 };
    const int rowH = n > 11 ? 40 : 48;
    const int cardW = 540;
    const int cardH = n * rowH + (title[0] ? 100 : 50);
    const int fs = n > 11 ? 22 : 25;
    int sw = GetScreenWidth(), sh = GetScreenHeight();
    int cardX = sw / 2 - cardW / 2;
    int cardY = sh / 2 - cardH / 2 + (title[0] ? 15 : 45);
    int startY = cardY + (title[0] ? 80 : 25);
    Vector2 m = GetMousePosition();
    int moved = m.x != lastMouse.x || m.y != lastMouse.y;
    lastMouse = m;

    *act = -1; *adj = 0;
    if (nav.up)    *sel = (*sel + n - 1) % n;
    if (nav.down)  *sel = (*sel + 1) % n;
    if (nav.left)  *adj = -1;
    if (nav.right) *adj = 1;
    if (nav.ok) { if (it[*sel].value[0]) *adj = 1; else *act = *sel; }

    /* Screen dim overlay */
    DrawRectangle(0, 0, sw, sh, (Color){ 0, 0, 0, 130 });

    /* Card background shadow */
    DrawRectangleRounded((Rectangle){ (float)cardX + 4, (float)cardY + 6, (float)cardW, (float)cardH }, 0.08f, 6, (Color){ 0, 0, 0, 160 });

    /* Frosted glass card */
    DrawRectangleRounded((Rectangle){ (float)cardX, (float)cardY, (float)cardW, (float)cardH }, 0.08f, 6, (Color){ 12, 16, 26, 235 });
    DrawRectangleRoundedLinesEx((Rectangle){ (float)cardX, (float)cardY, (float)cardW, (float)cardH }, 0.08f, 6, 2.0f, (Color){ 45, 75, 120, 220 });

    /* Title at top of card */
    if (title[0]) {
        int tw = MeasureText(title, 36);
        DrawText(title, cardX + cardW / 2 - tw / 2, cardY + 24, 36, (Color){ 255, 185, 50, 255 });
        DrawRectangle(cardX + 50, cardY + 66, cardW - 100, 2, (Color){ 255, 160, 40, 160 });
    }

    /* Menu items */
    for (int i = 0; i < n; i++) {
        Rectangle r = { (float)(cardX + 24), (float)(startY + i * rowH), (float)(cardW - 48), (float)(rowH - 6) };
        int hover = CheckCollisionPointRec(m, r);
        if (hover && moved) *sel = i;
        if (hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            *sel = i;
            if (it[i].value[0]) *adj = 1; else *act = i;
        }
        if (hover && IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && it[i].value[0]) {
            *sel = i; *adj = -1;
        }
        int on = (i == *sel);

        if (on) {
            DrawRectangleRounded(r, 0.28f, 6, (Color){ 255, 140, 25, 240 });
            DrawRectangleRoundedLinesEx(r, 0.28f, 6, 1.5f, (Color){ 255, 210, 100, 255 });
            DrawText(">", (int)r.x + 14, (int)r.y + (rowH - 6)/2 - fs/2 + 2, fs, BLACK);
            DrawText(it[i].label, (int)r.x + 36, (int)r.y + (rowH - 6)/2 - fs/2 + 2, fs, BLACK);
        } else {
            DrawRectangleRounded(r, 0.28f, 6, hover ? (Color){ 32, 42, 60, 220 } : (Color){ 18, 24, 36, 180 });
            DrawRectangleRoundedLinesEx(r, 0.28f, 6, 1.0f, (Color){ 38, 52, 75, 160 });
            DrawText(it[i].label, (int)r.x + 24, (int)r.y + (rowH - 6)/2 - fs/2 + 2, fs, (Color){ 220, 228, 240, 255 });
        }

        if (it[i].value[0]) {
            const char *valStr = TextFormat("<  %s  >", it[i].value);
            int vw = MeasureText(valStr, fs - 2);
            DrawText(valStr, (int)(r.x + r.width - 18 - vw), (int)r.y + (rowH - 6)/2 - (fs - 2)/2 + 2, fs - 2,
                     on ? (Color){ 20, 20, 20, 255 } : (Color){ 255, 180, 80, 255 });
        }
    }

    /* Footer navigation hints */
    DrawText("ENTER / A  Select    < / >  Adjust    ESC / B  Back",
             sw / 2 - MeasureText("ENTER / A  Select    < / >  Adjust    ESC / B  Back", 16) / 2,
             sh - 36, 16, (Color){ 180, 195, 215, 220 });
}

/* ------------------------------------------------------------------------ */
/* Online Multiplayer HUD & Scoreboard                                       */
/* ------------------------------------------------------------------------ */
static void net_client_draw_nameplates(const NetClient *cli, const Vector3 *carPositions, const int *carTeams, const int *demolished, Camera3D cam)
{
    if (!cli || cli->state != NET_CONNECTED) return;
    int sw = GetScreenWidth(), sh = GetScreenHeight();

    for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
        if (!cli->players[i].active || i == cli->localSlot) continue;
        if (demolished && demolished[i]) continue;

        Vector3 headPos = V3(carPositions[i].x, carPositions[i].y + 1.8f, carPositions[i].z);
        float dist = Vector3Distance(cam.position, headPos);
        if (dist > 180.0f) continue;

        Vector3 camToTarget = Vector3Subtract(headPos, cam.position);
        Vector3 camFwd = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
        if (Vector3DotProduct(camToTarget, camFwd) <= 0.1f) continue;

        Vector2 sp = GetWorldToScreen(headPos, cam);
        if (sp.x < -100 || sp.x > sw + 100 || sp.y < -100 || sp.y > sh + 100) continue;

        const char *name = cli->players[i].name;
        int fontSize = dist < 40.0f ? 16 : dist < 90.0f ? 14 : 12;
        int tw = MeasureText(name, fontSize);
        int padX = 10, padY = 5;
        Rectangle plate = { sp.x - tw / 2.0f - padX, sp.y - fontSize / 2.0f - padY, (float)tw + padX * 2, (float)fontSize + padY * 2 };

        int team = (carTeams != NULL) ? carTeams[i] : cli->players[i].team;
        Color teamBg = team == 0 ? (Color){ 20, 70, 160, 200 } : (Color){ 180, 70, 20, 200 };
        Color teamBorder = team == 0 ? (Color){ 60, 150, 255, 230 } : (Color){ 255, 140, 50, 230 };

        DrawRectangleRounded(plate, 0.4f, 4, teamBg);
        DrawRectangleRoundedLinesEx(plate, 0.4f, 4, 1.2f, teamBorder);
        DrawText(name, (int)plate.x + padX, (int)plate.y + padY, fontSize, RAYWHITE);
    }
}

static void net_client_draw_scoreboard(const NetClient *cli, int sw, int sh)
{
    if (!cli || cli->state != NET_CONNECTED) return;

    int panelW = 680, panelH = 340;
    int px = sw / 2 - panelW / 2, py = sh / 2 - panelH / 2;

    DrawRectangleRounded((Rectangle){ (float)px + 4, (float)py + 6, (float)panelW, (float)panelH }, 0.15f, 6, (Color){ 0, 0, 0, 160 });
    DrawRectangleRounded((Rectangle){ (float)px, (float)py, (float)panelW, (float)panelH }, 0.15f, 6, (Color){ 12, 16, 26, 240 });
    DrawRectangleRoundedLinesEx((Rectangle){ (float)px, (float)py, (float)panelW, (float)panelH }, 0.15f, 6, 2.0f, (Color){ 45, 75, 120, 220 });

    DrawText("ONLINE MATCH SCOREBOARD", px + 24, py + 16, 22, (Color){ 255, 185, 50, 255 });
    const char *srvInfo = TextFormat("%s:%d  |  Ping: %.0f ms", cli->serverIp, cli->serverPort, cli->pingMs);
    DrawText(srvInfo, px + panelW - MeasureText(srvInfo, 14) - 24, py + 22, 14, (Color){ 170, 195, 225, 220 });
    DrawRectangle(px + 24, py + 48, panelW - 48, 1, (Color){ 45, 65, 95, 180 });

    int colW = panelW / 2 - 32;

    DrawRectangleRounded((Rectangle){ (float)(px + 20), (float)(py + 60), (float)colW, 32 }, 0.2f, 4, (Color){ 20, 65, 150, 230 });
    DrawText(TextFormat("BLUE TEAM   (%d)", cli->scoreBlue), px + 32, py + 68, 16, (Color){ 200, 230, 255, 255 });

    DrawRectangleRounded((Rectangle){ (float)(px + panelW / 2 + 12), (float)(py + 60), (float)colW, 32 }, 0.2f, 4, (Color){ 165, 65, 15, 230 });
    DrawText(TextFormat("ORANGE TEAM   (%d)", cli->scoreOrange), px + panelW / 2 + 24, py + 68, 16, (Color){ 255, 225, 200, 255 });

    int blueRow = 0, orangeRow = 0;
    for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
        if (!cli->players[i].active) continue;
        int isBlue = (cli->players[i].team == 0);
        int rx = isBlue ? px + 20 : px + panelW / 2 + 12;
        int ry = py + 102 + (isBlue ? blueRow++ : orangeRow++) * 36;

        int isLocal = (i == cli->localSlot);
        Color rowBg = isLocal ? (Color){ 35, 60, 95, 240 } : (Color){ 18, 24, 36, 180 };
        Color rowBorder = isLocal ? (Color){ 255, 190, 50, 255 } : (Color){ 35, 45, 65, 160 };

        DrawRectangleRounded((Rectangle){ (float)rx, (float)ry, (float)colW, 30 }, 0.25f, 4, rowBg);
        DrawRectangleRoundedLinesEx((Rectangle){ (float)rx, (float)ry, (float)colW, 30 }, 0.25f, 4, 1.0f, rowBorder);

        const char *pName = cli->players[i].name;
        if (isLocal) pName = TextFormat("%s (YOU)", pName);
        DrawText(pName, rx + 12, ry + 7, 15, isLocal ? (Color){ 255, 225, 120, 255 } : RAYWHITE);

        const char *carStr = CAR_NAMES[cli->players[i].car_model % CAR_COUNT];
        DrawText(carStr, rx + colW - MeasureText(carStr, 13) - 12, ry + 9, 13, (Color){ 160, 180, 210, 200 });
    }

    DrawText("Release [TAB] to close", px + panelW / 2 - MeasureText("Release [TAB] to close", 14) / 2, py + panelH - 24, 14, (Color){ 150, 165, 185, 180 });
}

static void net_client_draw_hud(const NetClient *cli, int sw, int sh)
{
    (void)sh;
    if (!cli || cli->state != NET_CONNECTED) return;

    int bw = 145, bh = 28;
    int bx = sw - bw - 18, by = 14;

    DrawRectangleRounded((Rectangle){ (float)bx + 2, (float)by + 2, (float)bw, (float)bh }, 0.35f, 4, (Color){ 0, 0, 0, 100 });
    DrawRectangleRounded((Rectangle){ (float)bx, (float)by, (float)bw, (float)bh }, 0.35f, 4, (Color){ 12, 16, 26, 215 });
    DrawRectangleRoundedLinesEx((Rectangle){ (float)bx, (float)by, (float)bw, (float)bh }, 0.35f, 4, 1.2f, (Color){ 45, 65, 95, 180 });

    Color pingCol = cli->pingMs < 60.0f ? (Color){ 50, 220, 120, 255 } :
                    cli->pingMs < 120.0f ? (Color){ 240, 200, 50, 255 } : (Color){ 240, 70, 70, 255 };

    DrawCircle(bx + 14, by + bh / 2, 4.0f, pingCol);
    const char *pingStr = TextFormat("ONLINE %.0fms", cli->pingMs);
    DrawText(pingStr, bx + 26, by + 7, 13, RAYWHITE);

    if (cli->demoBannerTimer > 0.0f) {
        float alpha = fminf(1.0f, cli->demoBannerTimer);
        const char *demoMsg = TextFormat("%s DEMOLISHED %s!", cli->demoKiller, cli->demoVictim);
        int mw = MeasureText(demoMsg, 20);
        int mwW = mw + 40, mwH = 36;
        int mx = sw / 2 - mwW / 2, my = 75;

        DrawRectangleRounded((Rectangle){ (float)mx, (float)my, (float)mwW, (float)mwH }, 0.35f, 4, ColorAlpha((Color){ 220, 60, 20, 230 }, alpha));
        DrawRectangleRoundedLinesEx((Rectangle){ (float)mx, (float)my, (float)mwW, (float)mwH }, 0.35f, 4, 1.5f, ColorAlpha(YELLOW, alpha));
        DrawText(demoMsg, mx + mwW / 2 - mw / 2, my + 8, 20, ColorAlpha(RAYWHITE, alpha));
    }
}

/* ------------------------------------------------------------------------ */
/* Main                                                                       */
/* ------------------------------------------------------------------------ */
#ifdef _WIN32
__declspec(dllexport)
#endif
int sarpbc_main(int argc, char **argv)
{
    const char *carArg = NULL;
    char path[1024], arenaDir[512];
    Car cars[MAX_CARS];                 /* slot 0 = the player */
    CarRender crs[MAX_CARS];
    Bot bots[MAX_CARS];
    int team[MAX_CARS] = { 0 }, carModel[MAX_CARS], nCars = 1, botSkill = 1;
    Ball ball;
    ArenaRender ar;
    int haveArena = 0;
    Shader lit;
    Model field, ballModel;
    Texture2D fieldTex, ballTex;
    Camera3D cam = { 0 };
    Vector3 pads[PAD_COUNT];
    float padTimer[PAD_COUNT] = { 0 };
    int scoreBlue = 0, scoreOrange = 0, ballCam = 0, i, j, lastScorer = 0;
    float matchLen = MATCH_TIME, matchTime = MATCH_TIME, stateTimer = 3.0f, acc = 0.0f, fov = 60.0f, menuT = 0.0f;
    GameState state = ST_COUNTDOWN;
    Screen screen = SCR_MENU, settingsFrom = SCR_MENU;
    int menuSel = 0, setSel = 0, pauseSel = 0, quit = 0;
    Input in = { 0 };
    int pendingJump = 0, testMode = 0, shotMode = 0, frameNo = 0;
    /* render interpolation + camera state */
    Vector3 prevPos[MAX_CARS] = { { 0 } }, prevBallPos = { 0 }, camDir = { 0 };
    Quaternion prevRot[MAX_CARS], prevBallRot;
    Car rcars[MAX_CARS];
    Ball rball;
    float camY = 0.0f;
    int camSnap = 1;
    NetClient netClient;
    int isOnline = 0, onlineSel = 4, prefTeamSel = 2, autoConnect = 0, onlineFirstSnap = 0;
    char customIpInput[32] = "127.0.0.1";
    char playerNameInput[24] = "Striker";

    memset(crs, 0, sizeof(crs));
    memset(bots, 0, sizeof(bots));
    net_client_init(&netClient);
    for (i = 0; i < MAX_CARS; i++) {
        carModel[i] = -1;
        prevRot[i] = (Quaternion){ 0, 0, 0, 1 };
    }
    prevBallRot = (Quaternion){ 0, 0, 0, 1 };
    for (i = 1; i < argc; i++) {
        if      (strcmp(argv[i], "--test") == 0) testMode = 1;
        else if (strcmp(argv[i], "--shot") == 0) shotMode = 1;
        else if (strcmp(argv[i], "--menushot") == 0) shotMode = 2;
        else if (strcmp(argv[i], "--carshot") == 0) shotMode = 3;
        else if (strcmp(argv[i], "--ballshot") == 0) shotMode = 4;
        else if (strcmp(argv[i], "--connect") == 0 && i + 1 < argc) {
            autoConnect = 1;
            strncpy(customIpInput, argv[++i], sizeof(customIpInput) - 1);
        }
        else if (strcmp(argv[i], "--name") == 0 && i + 1 < argc) {
            strncpy(playerNameInput, argv[++i], sizeof(playerNameInput) - 1);
        }
        else if (strncmp(argv[i], "--", 2) != 0) carArg = argv[i];
    }

    if (!testMode) settings_load();
    if (carArg) {
        for (i = 0; i < CAR_COUNT; i++) if (!strcmp(carArg, CAR_NAMES[i])) g_set.car = i;
    }

    if (!testMode) {
        SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);   /* no MSAA: the scene renders off-screen, FXAA does AA */
        InitWindow(1600, 900, "SARPBC-C");
        SetExitKey(KEY_NULL);               /* Esc opens the pause menu instead of quitting */
        SetTargetFPS(144);
        if (g_set.fullscreen) ToggleBorderlessWindowed();
    }

    /* arena: next to the exe, then ../export_c/arena (falls back to a box arena) */
    snprintf(arenaDir, sizeof(arenaDir), "%sassets/arena/", GetApplicationDirectory());
    snprintf(path, sizeof(path), "%sarena_col.bin", arenaDir);
    if (!FileExists(path)) {
        snprintf(arenaDir, sizeof(arenaDir), "%s../export_c/arena/", GetApplicationDirectory());
        snprintf(path, sizeof(path), "%sarena_col.bin", arenaDir);
    }
    if (!FileExists(path)) {
        snprintf(arenaDir, sizeof(arenaDir), "assets/arena/");
        snprintf(path, sizeof(path), "%sarena_col.bin", arenaDir);
    }
    if (!FileExists(path)) {
        snprintf(arenaDir, sizeof(arenaDir), "../export_c/arena/");
        snprintf(path, sizeof(path), "%sarena_col.bin", arenaDir);
    }
    if (!FileExists(path)) {
        snprintf(arenaDir, sizeof(arenaDir), "export_c/arena/");
        snprintf(path, sizeof(path), "%sarena_col.bin", arenaDir);
    }
    if (!arena_mesh_load(path)) TraceLog(LOG_WARNING, "ARENA: %s not found, using the fallback box arena", path);

    if (testMode) {
        Shader none = { 0 };
        if (!load_car(CAR_NAMES[g_set.car], &cars[0], &crs[0], none, 0)) return 1;
        run_physics_test(&cars[0]);
        return 0;
    }

    rlSetClipPlanes(0.1, 2000.0);
    gfx_init();
    lit = G.lit;

    if (!load_car(CAR_NAMES[g_set.car], &cars[0], &crs[0], lit, 1)) {
        g_set.car = 0;
        if (!load_car(CAR_NAMES[0], &cars[0], &crs[0], lit, 1)) { CloseWindow(); return 1; }
    }
    haveArena = arena_render_load(&ar, arenaDir, lit);

    fieldTex = make_field_texture();
    field = LoadModelFromMesh(GenMeshPlane(2.0f * ARENA_W, 2.0f * (ARENA_L + GOAL_D), 4, 4));
    field.materials[0].shader = lit;
    field.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = fieldTex;

    g_particleTex = make_particle_texture();
    memset(g_particles, 0, sizeof(g_particles));
    g_particleHead = 0;

    ballTex = make_tech_ball_texture();
    ballModel = LoadModelFromMesh(GenMeshSphere(BALL_R, 32, 48));
    ballModel.materials[0].shader = lit;
    ballModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = ballTex;

    pads[0] = V3(-78, 0, -(ARENA_L - 24)); pads[1] = V3(78, 0, -(ARENA_L - 24));
    pads[2] = V3(-78, 0,  (ARENA_L - 24)); pads[3] = V3(78, 0,  (ARENA_L - 24));
    pads[4] = V3(-82, 0, 0);               pads[5] = V3(82, 0, 0);

    /* team sizes from the mode; bots get a mix of the other cars */
#define SETUP_CARS() do { int ts_ = g_set.mode == 0 ? 1 : g_set.mode; \
        nCars = g_set.mode == 0 ? 1 : 2 * ts_; botSkill = g_set.botSkill; \
        for (i = 0; i < nCars; i++) { \
            team[i] = i < ts_ ? 0 : 1; \
            if (i > 0) { int mdl_ = (g_set.car + 3 * i) % CAR_COUNT; \
                if (carModel[i] != mdl_ && load_car(CAR_NAMES[mdl_], &cars[i], &crs[i], lit, 1)) carModel[i] = mdl_; } \
        } } while (0)
#define KICKOFF() do { int ts_ = nCars == 1 ? 1 : nCars / 2; \
        for (i = 0; i < nCars; i++) { Vector3 p_; float y_; \
            kickoff_spot(i % ts_, team[i], ts_, &p_, &y_); car_reset(&cars[i], p_, y_); } \
        memset(bots, 0, sizeof(bots)); ball_reset(&ball); \
        state = ST_COUNTDOWN; stateTimer = 3.0f; acc = 0.0f; pendingJump = 0; camSnap = 1; demoBannerTimer = 0.0f; } while (0)
#define NEW_MATCH() do { SETUP_CARS(); scoreBlue = scoreOrange = 0; matchLen = MATCH_LENGTHS[g_set.matchIdx]; matchTime = matchLen; \
                         for (i = 0; i < PAD_COUNT; i++) { padTimer[i] = 0.0f; } \
                         memset(g_particles, 0, sizeof(g_particles)); g_particleHead = 0; KICKOFF(); } while (0)

    carModel[0] = g_set.car;

    float demoBannerTimer = 0.0f;
    char demoBannerText[64] = { 0 };
    Color demoBannerColor = ORANGE;
    float camShake = 0.0f;

    NEW_MATCH();
    for (i = 0; i < MAX_CARS; i++) rcars[i] = cars[i];
    rball = ball;
    cam.position = V3(0, 6, -KICKOFF_Z - 12);
    cam.target = cars[0].pos;
    cam.up = V3(0, 1, 0);
    cam.fovy = fov;
    cam.projection = CAMERA_PERSPECTIVE;
    if (shotMode == 1) screen = SCR_GAME;
    if (autoConnect) {
        net_client_connect(&netClient, customIpInput, netClient.serverPort, playerNameInput, g_set.car, g_set.skin, prefTeamSel);
        screen = SCR_ONLINE_JOIN;
    }

    while (!quit && !WindowShouldClose()) {
        float dt = fminf(GetFrameTime(), 0.1f);
        Screen screenAtStart = screen;
        Nav nav = read_nav();
        int act = -1, adj = 0;

        /* ================= game update ================= */
        if (isOnline) {
            net_client_poll(&netClient, dt);
            if (netClient.state == NET_DISCONNECTED) {
                isOnline = 0;
                screen = SCR_MENU;
            }
        }

        if (screen == SCR_GAME) {
            Input frame = read_input();
            int frozen;


            if (demoBannerTimer > 0.0f) demoBannerTimer -= dt;
            if (camShake > 0.0f) camShake = fmaxf(0.0f, camShake - 2.5f * dt);

            if (IsKeyPressed(KEY_ESCAPE) || IsKeyPressed(KEY_P) ||
                (IsGamepadAvailable(0) && IsGamepadButtonPressed(0, GAMEPAD_BUTTON_MIDDLE_RIGHT))) {
                screen = SCR_PAUSE; pauseSel = 0;
            }
            if (shotMode) {
                if (frameNo == 0) { state = ST_PLAY; cars[0].boost = BOOST_MAX; }
                frame.throttle = 1.0f;
                frame.boost = frameNo > 60 && frameNo < 200;
                frame.steer = frameNo > 150 && frameNo < 175 ? 1.0f : 0.0f;
                if (frameNo == 80) TakeScreenshot("shot_ball.png");
                if (frameNo == 172) TakeScreenshot("shot_kickoff.png");
                if (++frameNo == 330) { TakeScreenshot("shot.png"); break; }
            }
            frozen = state == ST_COUNTDOWN || state == ST_OVER || screen != SCR_GAME;

            if (IsKeyPressed(KEY_C) || (IsGamepadAvailable(0) && IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_FACE_UP)))
                ballCam = !ballCam;
            if (IsKeyPressed(KEY_R) || (IsGamepadAvailable(0) && IsGamepadButtonPressed(0, GAMEPAD_BUTTON_MIDDLE_LEFT))) {
                if (!isOnline) {
                    if (state == ST_OVER) NEW_MATCH(); else KICKOFF();
                }
            }

            /* edge-triggered input must survive frames with zero physics ticks */
            pendingJump |= frame.jumpPressed;
            in = frame;
            if (frozen) { memset(&in, 0, sizeof(in)); pendingJump = 0; }

            /* --- fixed-step simulation ----------------------------------- */
            if (screen == SCR_GAME) acc += dt;
            while (acc >= 1.0f / PHYS_HZ) {
                float h = 1.0f / PHYS_HZ;
                Input tick = in;
                int activeCars = isOnline ? SARP_MAX_CLIENTS : nCars;
                int mySlot = isOnline ? netClient.localSlot : 0;
                for (i = 0; i < activeCars; i++) { prevPos[i] = cars[i].pos; prevRot[i] = cars[i].rot; }
                prevBallPos = ball.pos; prevBallRot = ball.rot;
                tick.jumpPressed = pendingJump;
                pendingJump = 0;
                if (isOnline) {
                    NetInput netIn;
                    memset(&netIn, 0, sizeof(netIn));
                    netIn.throttle = tick.throttle;
                    netIn.steer = tick.steer;
                    netIn.pitch = tick.pitch;
                    netIn.yaw = tick.yaw;
                    netIn.roll = tick.roll;
                    netIn.jump = (uint8_t)tick.jump;
                    netIn.jumpPressed = (uint8_t)tick.jumpPressed;
                    netIn.boost = (uint8_t)tick.boost;
                    netIn.slide = (uint8_t)tick.slide;
                    net_client_send_tick(&netClient, &netIn);
                }

                if (!isOnline) {
                    /* Demolition respawn timers */
                    for (i = 0; i < nCars; i++) {
                        if (cars[i].demolished) {
                            cars[i].demoTimer -= h;
                            if (cars[i].demoTimer <= 0.0f) {
                                float spawnX = (p_randf() < 0.5f) ? -65.0f : 65.0f;
                                float spawnZ = (team[i] == 0) ? -(ARENA_L - 28.0f) : (ARENA_L - 28.0f);
                                float spawnYaw = (team[i] == 0) ? 0.0f : PI;
                                car_reset(&cars[i], V3(spawnX, 0, spawnZ), spawnYaw);
                                cars[i].boost = BOOST_START;
                                cars[i].demolished = 0;
                                cars[i].demoTimer = 0.0f;
                                if (i == 0) camSnap = 1;
                                particles_impact_burst(cars[i].pos, V3(0, 1, 0), 12.0f);
                            }
                        }
                    }

                    if (!cars[0].demolished) car_step(&cars[0], &tick, h);
                    if (nCars > 1) bot_world_update(&ball, h, pads, padTimer, PAD_COUNT);
                    for (i = 1; i < nCars; i++) {
                        if (cars[i].demolished) continue;
                        Input bi = bot_think(i, cars, team, nCars, &ball, &bots[i], botSkill, h);
                        if (frozen) memset(&bi, 0, sizeof(bi));
                        car_step(&cars[i], &bi, h);
                    }
                    ball_step(&ball, h);
                    for (i = 0; i < nCars; i++) {
                        if (cars[i].demolished) continue;
                        Vector3 prevBVel = ball.vel;
                        if (car_ball_collide(&cars[i], &ball)) {
                            float hitDelta = Vector3Distance(ball.vel, prevBVel);
                            if (hitDelta > 3.0f) {
                                Vector3 hitPoint = Vector3Lerp(cars[i].pos, ball.pos, 0.5f);
                                Vector3 hitNorm = Vector3Normalize(Vector3Subtract(ball.pos, cars[i].pos));
                                particles_impact_burst(hitPoint, hitNorm, hitDelta);
                            }
                        }
                    }
                    for (i = 0; i < nCars; i++) {
                        if (cars[i].demolished) continue;
                        for (j = i + 1; j < nCars; j++) {
                            if (cars[j].demolished) continue;
                            Vector3 velDiff = Vector3Subtract(cars[j].vel, cars[i].vel);
                            float relSpeed = Vector3Length(velDiff);

                            int demo = check_demolition(&cars[i], &cars[j], team[i], team[j]);
                            if (demo != 0) {
                                if (demo == 1) {
                                    demolish_car(j, i, cars, carModel, &demoBannerTimer, demoBannerText, sizeof(demoBannerText), &demoBannerColor, &camShake);
                                } else if (demo == 2) {
                                    demolish_car(i, j, cars, carModel, &demoBannerTimer, demoBannerText, sizeof(demoBannerText), &demoBannerColor, &camShake);
                                } else if (demo == 3) {
                                    demolish_car(i, j, cars, carModel, &demoBannerTimer, demoBannerText, sizeof(demoBannerText), &demoBannerColor, &camShake);
                                    demolish_car(j, i, cars, carModel, &demoBannerTimer, demoBannerText, sizeof(demoBannerText), &demoBannerColor, &camShake);
                                }
                                continue;
                            }

                            if (car_car_collide(&cars[i], &cars[j])) {
                                if (relSpeed > 6.0f) {
                                    Vector3 hitPoint = Vector3Lerp(cars[i].pos, cars[j].pos, 0.5f);
                                    Vector3 hitNorm = Vector3Normalize(velDiff);
                                    particles_impact_burst(hitPoint, hitNorm, relSpeed);
                                }
                            }
                        }
                    }

                    for (i = 0; i < PAD_COUNT; i++) {
                        padTimer[i] -= h;
                        for (j = 0; j < nCars && padTimer[i] <= 0.0f; j++) {
                            if (cars[j].demolished) continue;
                            float dx = cars[j].pos.x - pads[i].x, dz = cars[j].pos.z - pads[i].z;
                            if (dx*dx + dz*dz < PAD_RADIUS * PAD_RADIUS && cars[j].pos.y < 4.0f) {
                                cars[j].boost = BOOST_MAX;
                                padTimer[i] = PAD_RESPAWN;
                                particles_pad_pickup(pads[i]);
                            }
                        }
                    }

                    if (state == ST_PLAY) {
                        if (fabsf(ball.pos.z) > ARENA_L + BALL_R) {
                            lastScorer = ball.pos.z > 0 ? 1 : 2;
                            if (lastScorer == 1) scoreBlue++; else scoreOrange++;
                            state = ST_GOAL; stateTimer = 3.0f;
                            particles_goal_explosion(ball.pos, lastScorer);
                            camShake = 1.0f;

                            /* Shockwave blast impulse on nearby cars */
                            for (i = 0; i < nCars; i++) {
                                if (cars[i].demolished) continue;
                                Vector3 toCar = Vector3Subtract(cars[i].pos, ball.pos);
                                float d = Vector3Length(toCar);
                                if (d < 65.0f) {
                                    float p = (1.0f - d / 65.0f);
                                    Vector3 dir = d > 0.1f ? Vector3Scale(toCar, 1.0f / d) : V3(0, 1, 0);
                                    dir.y = fmaxf(dir.y, 0.45f);
                                    dir = Vector3Normalize(dir);
                                    cars[i].vel = Vector3Add(cars[i].vel, Vector3Scale(dir, p * 45.0f + 10.0f));
                                    cars[i].angVel = Vector3Add(cars[i].angVel, p_randv3(-8.0f, 8.0f));
                                }
                            }
                        }
                        if (matchLen > 0.0f) {
                            matchTime -= h;
                            if (matchTime <= 0.0f) { matchTime = 0.0f; state = ST_OVER; }
                        }
                    }
                } else {
                    /* ONLINE Authoritative Physics Simulation */
                    /* 1. Client prediction on local car */
                    if (!cars[mySlot].demolished) {
                        car_step(&cars[mySlot], &tick, h);
                    }

                    /* 2. Ball simulation & prediction */
                    ball_step(&ball, h);
                    if (!cars[mySlot].demolished) {
                        Vector3 prevBVel = ball.vel;
                        if (car_ball_collide(&cars[mySlot], &ball)) {
                            float hitDelta = Vector3Distance(ball.vel, prevBVel);
                            if (hitDelta > 3.0f) {
                                Vector3 hitPoint = Vector3Lerp(cars[mySlot].pos, ball.pos, 0.5f);
                                Vector3 hitNorm = Vector3Normalize(Vector3Subtract(ball.pos, cars[mySlot].pos));
                                particles_impact_burst(hitPoint, hitNorm, hitDelta);
                            }
                        }
                    }

                    /* 3. Reconcile local car with authoritative server target */
                    if (netClient.players[mySlot].active) {
                        Vector3 srvPos = V3(netClient.players[mySlot].targetPos.x, netClient.players[mySlot].targetPos.y, netClient.players[mySlot].targetPos.z);
                        Vector3 srvVel = V3(netClient.players[mySlot].targetVel.x, netClient.players[mySlot].targetVel.y, netClient.players[mySlot].targetVel.z);
                        Quaternion srvRot = (Quaternion){ netClient.players[mySlot].targetRot.x, netClient.players[mySlot].targetRot.y, netClient.players[mySlot].targetRot.z, netClient.players[mySlot].targetRot.w };
                        float rotLenSq = srvRot.x*srvRot.x + srvRot.y*srvRot.y + srvRot.z*srvRot.z + srvRot.w*srvRot.w;

                        if (!onlineFirstSnap) {
                            cars[mySlot].pos = srvPos;
                            cars[mySlot].vel = srvVel;
                            if (rotLenSq > 0.5f) cars[mySlot].rot = srvRot;
                            prevPos[mySlot] = srvPos;
                            prevRot[mySlot] = cars[mySlot].rot;
                            rcars[mySlot] = cars[mySlot];
                            onlineFirstSnap = 1;
                            camSnap = 1;
                        } else {
                            float latencySec = (netClient.pingMs * 0.5f) / 1000.0f;
                            if (latencySec > 0.12f) latencySec = 0.12f;
                            Vector3 srvProj = Vector3Add(srvPos, Vector3Scale(srvVel, latencySec));
                            float errDist = Vector3Distance(cars[mySlot].pos, srvProj);

                            if (errDist > 4.0f) {
                                cars[mySlot].pos = srvPos;
                                cars[mySlot].vel = srvVel;
                                if (rotLenSq > 0.5f) cars[mySlot].rot = srvRot;
                                camSnap = 1;
                            } else if (Vector3Length(cars[mySlot].vel) < 0.8f) {
                                if (errDist > 0.02f) cars[mySlot].pos = Vector3Lerp(cars[mySlot].pos, srvPos, 0.15f);
                                cars[mySlot].vel = Vector3Lerp(cars[mySlot].vel, srvVel, 0.15f);
                                if (rotLenSq > 0.5f) cars[mySlot].rot = QuaternionSlerp(cars[mySlot].rot, srvRot, 0.15f);
                            } else if (errDist > 0.8f) {
                                cars[mySlot].pos = Vector3Lerp(cars[mySlot].pos, srvProj, 0.04f);
                            }
                        }
                        cars[mySlot].boost = netClient.players[mySlot].boost;
                        cars[mySlot].demolished = netClient.players[mySlot].demolished;
                        cars[mySlot].demoTimer = netClient.players[mySlot].demoTimer;
                    }

                    /* 4. Remote cars smooth interpolation */
                    for (i = 0; i < SARP_MAX_CLIENTS; i++) {
                        if (i == mySlot) continue;
                        if (netClient.players[i].active) {
                            if (carModel[i] != netClient.players[i].car_model) {
                                load_car(CAR_NAMES[netClient.players[i].car_model % CAR_COUNT], &cars[i], &crs[i], lit, 1);
                                carModel[i] = netClient.players[i].car_model % CAR_COUNT;
                            }
                            team[i] = netClient.players[i].team;
                            Vector3 rPos = V3(netClient.players[i].targetPos.x, netClient.players[i].targetPos.y, netClient.players[i].targetPos.z);
                            Vector3 rVel = V3(netClient.players[i].targetVel.x, netClient.players[i].targetVel.y, netClient.players[i].targetVel.z);
                            Quaternion rRot = (Quaternion){ netClient.players[i].targetRot.x, netClient.players[i].targetRot.y, netClient.players[i].targetRot.z, netClient.players[i].targetRot.w };
                            float rotLenSq = rRot.x*rRot.x + rRot.y*rRot.y + rRot.z*rRot.z + rRot.w*rRot.w;
                            if (rotLenSq < 0.5f) rRot = (Quaternion){ 0, 0, 0, 1 };

                            if (cars[i].pos.y < -100.0f || Vector3Distance(cars[i].pos, rPos) > 8.0f) {
                                cars[i].pos = rPos;
                                cars[i].vel = rVel;
                                cars[i].rot = rRot;
                                prevPos[i] = rPos;
                                prevRot[i] = rRot;
                            } else {
                                cars[i].pos = Vector3Lerp(cars[i].pos, rPos, 0.40f);
                                cars[i].vel = rVel;
                                cars[i].rot = QuaternionSlerp(cars[i].rot, rRot, 0.40f);
                            }
                            cars[i].steerAngle = netClient.players[i].steerAngle;
                            cars[i].wheelSpin = netClient.players[i].wheelSpin;
                            cars[i].boost = netClient.players[i].boost;
                            cars[i].demolished = netClient.players[i].demolished;
                            cars[i].boosting = (Vector3Length(cars[i].vel) > 12.0f && cars[i].boost > 0.0f);
                        } else {
                            cars[i].pos = V3(0, -500, 0);
                            cars[i].rot = (Quaternion){ 0, 0, 0, 1 };
                        }
                    }

                    /* 5. Ball reconciliation */
                    if (netClient.serverTick > 0) {
                        Vector3 sbPos = V3(netClient.ballTargetPos.x, netClient.ballTargetPos.y, netClient.ballTargetPos.z);
                        Vector3 sbVel = V3(netClient.ballTargetVel.x, netClient.ballTargetVel.y, netClient.ballTargetVel.z);
                        Quaternion sbRot = (Quaternion){ netClient.ballTargetRot.x, netClient.ballTargetRot.y, netClient.ballTargetRot.z, netClient.ballTargetRot.w };
                        float rotLenSq = sbRot.x*sbRot.x + sbRot.y*sbRot.y + sbRot.z*sbRot.z + sbRot.w*sbRot.w;
                        if (rotLenSq < 0.5f) sbRot = (Quaternion){ 0, 0, 0, 1 };
                        float bDist = Vector3Distance(ball.pos, sbPos);
                        if (bDist > 5.0f || Vector3Length(ball.pos) < 0.1f) {
                            ball.pos = sbPos;
                            ball.vel = sbVel;
                            ball.rot = sbRot;
                        } else if (bDist > 0.05f) {
                            ball.pos = Vector3Lerp(ball.pos, sbPos, 0.40f);
                            ball.vel = Vector3Lerp(ball.vel, sbVel, 0.40f);
                            ball.rot = QuaternionSlerp(ball.rot, sbRot, 0.40f);
                        }
                    }

                    /* 6. Match state & scores */
                    scoreBlue = netClient.scoreBlue;
                    scoreOrange = netClient.scoreOrange;
                    matchTime = netClient.serverMatchTime;
                    if (netClient.serverGameState == 0) state = ST_COUNTDOWN;
                    else if (netClient.serverGameState == 1) state = ST_PLAY;
                    else if (netClient.serverGameState == 2) state = ST_GOAL;
                    else if (netClient.serverGameState == 3) state = ST_OVER;
                }

                for (i = 0; i < activeCars; i++) {
                    if (isOnline && !netClient.players[i].active) continue;
                    if (cars[i].demolished) continue;
                    if (cars[i].boosting && screen == SCR_GAME) {
                        particles_boost_emit(&cars[i], team[i] == 0);
                    }
                    Vector3 cr = car_right(&cars[i]);
                    float latSpeed = fabsf(Vector3DotProduct(cars[i].vel, cr));
                    int isSliding = (i == mySlot) ? (in.slide) : 0;
                    particles_drift_emit(&cars[i], latSpeed, isSliding);
                    particles_supersonic_emit(&cars[i]);
                }
                particles_update(h);
                acc -= h;
            }

            /* --- match state machine ------------------------------------- */
            if (screen == SCR_GAME && !isOnline) {
                stateTimer -= dt;
                if (state == ST_COUNTDOWN && stateTimer <= 0.0f) state = ST_PLAY;
                if (state == ST_GOAL && stateTimer <= 0.0f) {
                    int over = matchLen > 0.0f && matchTime <= 0.0f;
                    KICKOFF();
                    if (over) state = ST_OVER;
                }
            }

            /* --- render state: interpolate between the last two physics ticks so motion
             * is smooth at any frame rate (teleports such as kickoffs snap) --------- */
            {
                float alpha = clampf(acc * PHYS_HZ, 0.0f, 1.0f);
                int activeCars = isOnline ? SARP_MAX_CLIENTS : nCars;
                int mySlot = isOnline ? netClient.localSlot : 0;
                for (i = 0; i < activeCars; i++) {
                    if (isOnline && i != mySlot && !netClient.players[i].active) {
                        rcars[i].pos = V3(0, -500, 0);
                        continue;
                    }
                    rcars[i] = cars[i];
                    if (Vector3Distance(prevPos[i], cars[i].pos) < 8.0f) {
                        rcars[i].pos = Vector3Lerp(prevPos[i], cars[i].pos, alpha);
                        rcars[i].rot = QuaternionSlerp(prevRot[i], cars[i].rot, alpha);
                    }
                }
                rball = ball;
                if (Vector3Distance(prevBallPos, ball.pos) < 8.0f) {
                    rball.pos = Vector3Lerp(prevBallPos, ball.pos, alpha);
                    rball.rot = QuaternionSlerp(prevBallRot, ball.rot, alpha);
                }
            }

            /* --- chase camera: rigidly attached at a smoothed heading -------------- */
            if (screen == SCR_GAME) {
                int mySlot = isOnline ? netClient.localSlot : 0;
                const Car *pc = &rcars[mySlot];
                Vector3 f = car_fwd(pc), dirv, want, look;
                float kd = 1.0f - expf(-8.0f * dt);
                if (IsKeyDown(KEY_LEFT_BRACKET))  g_set.camDist = fmaxf(3.0f,  g_set.camDist - 4.0f * dt);
                if (IsKeyDown(KEY_RIGHT_BRACKET)) g_set.camDist = fminf(14.0f, g_set.camDist + 4.0f * dt);
                if (ballCam) { dirv = Vector3Subtract(rball.pos, pc->pos); }
                else         { dirv = pc->wheelsOnGround >= 3 ? f : pc->vel; if (Vector3Length(dirv) < 2.0f) dirv = f; }
                dirv.y = 0.0f;
                if (Vector3Length(dirv) < 0.01f) dirv = V3(0, 0, 1);
                dirv = Vector3Normalize(dirv);
                /* only the heading is smoothed; the distance to the car stays fixed */
                if (Vector3Length(camDir) < 0.5f || camSnap) { camDir = dirv; camY = pc->pos.y; camSnap = 0; }
                camDir = Vector3Normalize(Vector3Lerp(camDir, dirv, kd));
                camY = Lerp(camY, pc->pos.y, 1.0f - expf(-12.0f * dt));   /* soften jump/landing bob */
                want = V3(pc->pos.x - camDir.x * g_set.camDist, camY + g_set.camDist * g_set.camHeight, pc->pos.z - camDir.z * g_set.camDist);
                want.x = clampf(want.x, -ARENA_W + 1, ARENA_W - 1);
                want.y = clampf(want.y, 0.5f, ARENA_H - 1);
                want.z = clampf(want.z, -ARENA_L - GOAL_D + 1, ARENA_L + GOAL_D - 1);
                look = ballCam ? Vector3Lerp(V3(pc->pos.x, camY + 1.0f, pc->pos.z), rball.pos, 0.25f)
                               : V3(pc->pos.x + camDir.x * 2.0f, camY + 0.9f, pc->pos.z + camDir.z * 2.0f);
                cam.position = want;
                cam.target   = look;
                if (camShake > 0.01f) {
                    cam.position = Vector3Add(cam.position, p_randv3(-camShake * 0.45f, camShake * 0.45f));
                    cam.target   = Vector3Add(cam.target,   p_randv3(-camShake * 0.25f, camShake * 0.25f));
                }
                if (shotMode == 1 && frameNo > 30 && frameNo < 150) {   /* jitter metric: car-to-camera distance spread */
                    static float dmin = 1e9f, dmax = 0.0f, dsum = 0.0f, dprev = -1.0f, jmax = 0.0f;
                    float dcc = Vector3Distance(cam.position, pc->pos);
                    dmin = fminf(dmin, dcc); dmax = fmaxf(dmax, dcc);
                    if (dprev >= 0.0f) { dsum += fabsf(dcc - dprev); jmax = fmaxf(jmax, fabsf(dcc - dprev)); }
                    dprev = dcc;
                    if (frameNo == 149) printf("camera-car distance: %.3f..%.3f m, mean frame change %.4f m, max %.4f m\n", dmin, dmax, dsum / 118.0f, jmax);
                }
                /* raylib fovy is VERTICAL: 60 deg ~= 95 deg horizontal at 16:9 */
                fov = Lerp(fov, g_set.fov + (pc->boosting && g_set.boostFov ? 8.0f : 0.0f), 1.0f - expf(-4.0f * dt));
                cam.fovy = fov;
            }
        } else if (screen == SCR_MENU || screen == SCR_ONLINE_JOIN || (screen == SCR_SETTINGS && settingsFrom == SCR_MENU)) {
            /* menu backdrop: parked car, slow orbit */
            float a;
            menuT += dt;
            a = menuT * 0.18f;
            car_reset(&cars[0], V3(0, 0, -KICKOFF_Z), -PI / 2.0f + menuT * 0.35f);
            cam.target   = Vector3Add(cars[0].pos, V3(0, 0.7f, 0));
            cam.position = Vector3Add(cars[0].pos, V3(sinf(a) * 7.5f, 2.4f, cosf(a) * 7.5f));
            cam.fovy = fov = 50.0f;
            for (i = 0; i < nCars; i++) rcars[i] = cars[i];
            rball = ball;
            if (shotMode == 2 && ++frameNo == 60) { TakeScreenshot("menu.png"); quit = 1; }
            if (shotMode == 3) {   /* --carshot: 4 views, 90 deg apart */
                int view = frameNo / 20;
                car_reset(&cars[0], V3(0, 0, -KICKOFF_Z), 0.0f);
                a = view * PI / 2.0f + 0.6f;
                cam.target   = Vector3Add(cars[0].pos, V3(0, 0.3f, 0));
                cam.position = Vector3Add(cars[0].pos, V3(sinf(a) * 5.0f, 1.4f, cosf(a) * 5.0f));
                if (frameNo % 20 == 19) TakeScreenshot(TextFormat("car_%d.png", view));
                if (++frameNo >= 80) quit = 1;
            }
            if (shotMode == 4) {   /* --ballshot: 4 views of the tech ball up close */
                int view = frameNo / 20;
                a = view * PI / 2.0f + 0.4f;
                cam.target   = ball.pos;
                cam.position = Vector3Add(ball.pos, V3(sinf(a) * 4.0f, 1.8f, cosf(a) * 4.0f));
                if (frameNo % 20 == 19) TakeScreenshot(TextFormat("ball_%d.png", view));
                if (++frameNo >= 80) quit = 1;
            }
        }

        /* ================= draw ================= */
        gfx_resize();
        /* dynamic lights: goal glows, live boost pads, boosting cars */
        G.ptCount = 0;
        gfx_light(V3(0, 5.0f, -ARENA_L - 3.0f), 55.0f, V3(0.5f, 1.3f, 4.0f));
        gfx_light(V3(0, 5.0f,  ARENA_L + 3.0f), 55.0f, V3(4.0f, 1.6f, 0.4f));
        for (i = 0; i < PAD_COUNT; i++)
            if (padTimer[i] <= 0.0f) gfx_light(V3(pads[i].x, 1.6f, pads[i].z), 10.0f, V3(3.0f, 1.5f, 0.35f));
        for (i = 0; i < nCars; i++)
            if (rcars[i].boosting && !rcars[i].demolished && screen == SCR_GAME)
                gfx_light(Vector3Add(rcars[i].pos, car_to_world(&rcars[i], V3(rcars[i].boxMin.x - 0.9f, 0.3f, 0))),
                          9.0f, team[i] == 0 ? V3(0.8f, 2.2f, 5.0f) : V3(5.0f, 2.2f, 0.6f));

        if (g_set.shadows) {
            /* near cascade sits a bit ahead of the player, where the camera looks */
            Vector3 look = Vector3Subtract(cam.target, cam.position), focus;
            int k;
            int mySlot = isOnline ? netClient.localSlot : 0;
            int activeCars = isOnline ? SARP_MAX_CLIENTS : nCars;
            look.y = 0.0f;
            focus = Vector3Add(rcars[mySlot].pos, Vector3Scale(Vector3Normalize(look), SHADOW_NEAR_SIZE * 0.25f));
            for (k = 0; k < 2; k++) {
                Shader keep = ballModel.materials[0].shader;
                gfx_shadow_begin(k, focus);
                if (haveArena) arena_render_draw(&ar, 0, &G.depth);
                for (i = 0; i < activeCars; i++) {
                    if (isOnline && !netClient.players[i].active) continue;
                    if (!rcars[i].demolished)
                        car_render_draw(&crs[i], &rcars[i], team[i], i == mySlot ? g_set.skin : (isOnline ? netClient.players[i].skin : 0), &G.depth);
                }
                ballModel.materials[0].shader = G.depth;
                ballModel.transform = QuaternionToMatrix(rball.rot);
                DrawModel(ballModel, rball.pos, 1.0f, WHITE);
                ballModel.materials[0].shader = keep;
                gfx_shadow_end();
            }
        }

        gfx_scene_begin(cam, g_set.shadows);
            if (haveArena) {
                arena_render_draw(&ar, 0, NULL);
            } else {
                gfx_mat(0.2f, 24.0f, 0.15f, 0.0f);
                DrawModel(field, V3(0, 0, 0), 1.0f, WHITE);
                gfx_emissive_begin(1.0f);   /* unlit vertex colours: just sRGB -> linear for the HDR target */
                draw_arena_shell();
                EndShaderMode();
            }
            gfx_emissive_begin(1.6f);
            for (i = 0; i < PAD_COUNT; i++) {
                Color pc = padTimer[i] <= 0.0f ? (Color){ 255, 170, 40, 255 } : (Color){ 90, 70, 40, 255 };
                DrawCylinder(V3(pads[i].x, 0.06f, pads[i].z), PAD_RADIUS, PAD_RADIUS, 0.12f, 24, pc);
            }
            EndShaderMode();
            gfx_emissive_begin(4.0f);   /* deep orange: brighter/yellower tints clip to flat yellow in the tonemap */
            for (i = 0; i < PAD_COUNT; i++)
                if (padTimer[i] <= 0.0f)
                    DrawSphere(V3(pads[i].x, 1.6f + 0.3f * sinf((float)GetTime() * 3.0f), pads[i].z), 0.7f, (Color){ 255, 120, 25, 255 });
            EndShaderMode();
            if (!g_set.shadows) {   /* blob shadows only when real shadows are off */
                int activeCars = isOnline ? SARP_MAX_CLIENTS : nCars;
                DrawCylinder(V3(rball.pos.x, 0.07f, rball.pos.z), BALL_R * 0.9f, BALL_R * 0.9f, 0.01f, 24, (Color){ 0, 0, 0, 90 });
                for (i = 0; i < activeCars; i++) {
                    if (isOnline && !netClient.players[i].active) continue;
                    if (!rcars[i].demolished)
                        DrawCylinder(V3(rcars[i].pos.x, 0.07f, rcars[i].pos.z), 1.3f, 1.3f, 0.01f, 24, (Color){ 0, 0, 0, 90 });
                }
            }

            gfx_mat(0.60f, 64.0f, 0.25f, 0.05f);
            ballModel.transform = QuaternionToMatrix(rball.rot);
            DrawModel(ballModel, rball.pos, 1.0f, WHITE);
            {
                int activeCars = isOnline ? SARP_MAX_CLIENTS : nCars;
                int mySlot = isOnline ? netClient.localSlot : 0;
                for (i = 0; i < activeCars; i++) {
                    if (isOnline && !netClient.players[i].active) continue;
                    if (!rcars[i].demolished)
                        car_render_draw(&crs[i], &rcars[i], team[i], i == mySlot ? g_set.skin : (isOnline ? netClient.players[i].skin : 0), NULL);
                }
            }
            /* boost flame: additive, no depth writes (the core sits inside the outer cone);
             * team-colored outer plume (blue plasma vs orange fire) and bright white-hot core */
            rlDrawRenderBatchActive();
            rlDisableDepthMask();
            BeginBlendMode(BLEND_ADDITIVE);
            for (j = 0; j < 2; j++) {
                int activeCars = isOnline ? SARP_MAX_CLIENTS : nCars;
                gfx_emissive_begin(j == 0 ? 2.5f : 5.5f);
                for (i = 0; i < activeCars; i++) {
                    if (isOnline && !netClient.players[i].active) continue;
                    Car *c = &rcars[i];
                    if (c->boosting && !c->demolished && screen == SCR_GAME) {
                        float t = (float)GetTime() * 40.0f + (float)i * 1.7f;
                        Vector3 back = Vector3Add(c->pos, car_to_world(c, V3(c->boxMin.x - 0.2f, 0.12f, 0)));
                        Vector3 bk = Vector3Scale(car_fwd(c), -1.0f);
                        int isBlue = (team[i] == 0);
                        if (j == 0) {
                            Vector3 tail = Vector3Add(back, Vector3Scale(bk, 1.7f + 0.45f * sinf(t)));
                            Color outerCol = isBlue ? (Color){ 30, 140, 255, 210 } : (Color){ 255, 85, 15, 210 };
                            DrawCylinderEx(back, tail, 0.32f, 0.03f, 12, outerCol);
                        } else {
                            Vector3 core = Vector3Add(back, Vector3Scale(bk, 0.9f + 0.25f * sinf(t * 1.3f + 1.0f)));
                            Color coreCol = isBlue ? (Color){ 180, 235, 255, 240 } : (Color){ 255, 220, 130, 240 };
                            DrawCylinderEx(back, core, 0.18f, 0.02f, 10, coreCol);
                        }
                    }
                }
                EndShaderMode();   /* flushes, so each layer gets its own intensity */
            }
            EndBlendMode();

            /* --- Particles: alpha smoke then additive emissive sparks & flames --- */
            if (screen == SCR_GAME || screen == SCR_PAUSE || (screen == SCR_SETTINGS && settingsFrom == SCR_PAUSE)) {
                rlDisableBackfaceCulling();
                BeginBlendMode(BLEND_ALPHA);
                gfx_emissive_begin(1.0f);
                particles_draw_pass(cam, g_particleTex, 0);
                EndShaderMode();
                EndBlendMode();

                BeginBlendMode(BLEND_ADDITIVE);
                gfx_emissive_begin(3.0f);
                particles_draw_pass(cam, g_particleTex, 1);
                EndShaderMode();
                EndBlendMode();
                rlEnableBackfaceCulling();
            }

            rlEnableDepthMask();
            if (haveArena) {
                gfx_emissive_begin(2.5f);
                draw_goal_nets();
                EndShaderMode();
                arena_render_draw(&ar, 1, NULL);
            } else {
                gfx_emissive_begin(1.0f);
                draw_goals();
                EndShaderMode();
            }
        gfx_scene_end();
        {
            int mySlot = isOnline ? netClient.localSlot : 0;
            gfx_post(g_set.bloom, screen == SCR_GAME && rcars[mySlot].boosting ? 1.0f : 0.0f);
        }

        BeginDrawing();
        ClearBackground(BLACK);
        gfx_present();

        /* --- HUD (in game and behind the pause menu) --------------------- */
        if (screen == SCR_GAME || screen == SCR_PAUSE || (screen == SCR_SETTINGS && settingsFrom == SCR_PAUSE)) {
            int sw = GetScreenWidth(), sh = GetScreenHeight();
            int mySlot = isOnline ? netClient.localSlot : 0;
            float speed = Vector3Length(rcars[mySlot].vel);

            draw_hud_scoreboard(sw, scoreBlue, scoreOrange, matchTime, matchLen, state);
            draw_hud_boost_and_speed(sw, sh, rcars[mySlot].boost, speed, rcars[mySlot].boosting);
            draw_hud_tactical_badges(sw, sh, ballCam,
                                     CAR_NAMES[isOnline ? netClient.players[mySlot].car_model : g_set.car],
                                     isOnline ? (team[mySlot] == 0 ? "BLUE TEAM" : "ORANGE TEAM") : MODE_NAMES[g_set.mode],
                                     isOnline ? "DEDICATED SERVER" : SKILL_NAMES[botSkill],
                                     g_set.showFps, g_set.showHints && screen == SCR_GAME);

            if (!isOnline) {
                for (i = 1; i < nCars; i++) {
                    if (rcars[i].demolished) continue;
                    Vector3 above = Vector3Add(rcars[i].pos, V3(0, 2.2f, 0));
                    Vector3 toC = Vector3Subtract(above, cam.position);
                    Vector2 sp;
                    const char *tag = TextFormat("BOT %s", CAR_NAMES[carModel[i] < 0 ? 0 : carModel[i]]);
                    if (Vector3DotProduct(toC, Vector3Subtract(cam.target, cam.position)) <= 0.0f) continue;   /* behind the camera */
                    sp = GetWorldToScreen(above, cam);
                    DrawText(tag, (int)sp.x - MeasureText(tag, 16) / 2, (int)sp.y, 16,
                             team[i] == 0 ? (Color){ 120, 180, 255, 230 } : (Color){ 255, 170, 90, 230 });
                }
            } else {
                Vector3 cpos[SARP_MAX_CLIENTS];
                int cdemo[SARP_MAX_CLIENTS];
                for (i = 0; i < SARP_MAX_CLIENTS; i++) {
                    cpos[i] = rcars[i].pos;
                    cdemo[i] = rcars[i].demolished;
                }
                net_client_draw_nameplates(&netClient, cpos, team, cdemo, cam);
                net_client_draw_hud(&netClient, sw, sh);
                if (IsKeyDown(KEY_TAB)) {
                    net_client_draw_scoreboard(&netClient, sw, sh);
                }
            }

            if (screen == SCR_GAME) {
                if (demoBannerTimer > 0.0f) {
                    float alpha = fminf(demoBannerTimer * 2.5f, 1.0f);
                    int bw = MeasureText(demoBannerText, 38);
                    int cardW = bw + 80, cardH = 54;
                    int bx = sw/2 - cardW/2, by = sh/2 - 135;
                    DrawRectangleRounded((Rectangle){ (float)bx + 2, (float)by + 3, (float)cardW, (float)cardH }, 0.35f, 6, (Color){ 0, 0, 0, (unsigned char)(160 * alpha) });
                    DrawRectangleRounded((Rectangle){ (float)bx, (float)by, (float)cardW, (float)cardH }, 0.35f, 6, (Color){ 18, 12, 12, (unsigned char)(235 * alpha) });
                    DrawRectangleRoundedLinesEx((Rectangle){ (float)bx, (float)by, (float)cardW, (float)cardH }, 0.35f, 6, 2.0f, (Color){ 255, 70, 30, (unsigned char)(255 * alpha) });
                    DrawText("▲", bx + 16, by + cardH/2 - 10, 20, (Color){ 255, 80, 40, (unsigned char)(255 * alpha) });
                    DrawText("▲", bx + cardW - 30, by + cardH/2 - 10, 20, (Color){ 255, 80, 40, (unsigned char)(255 * alpha) });
                    DrawText(demoBannerText, sw/2 - bw/2 + 2, by + 10 + 2, 38, (Color){ 0, 0, 0, (unsigned char)(220 * alpha) });
                    DrawText(demoBannerText, sw/2 - bw/2, by + 10, 38, (Color){ demoBannerColor.r, demoBannerColor.g, demoBannerColor.b, (unsigned char)(255 * alpha) });
                }
                if (cars[mySlot].demolished) {
                    const char *respawnMsg = TextFormat("RESPAWNING IN %d...", (int)ceilf(fmaxf(0.01f, cars[mySlot].demoTimer)));
                    int rw = MeasureText(respawnMsg, 28);
                    int rCardW = rw + 50, rCardH = 44;
                    int rx = sw/2 - rCardW/2, ry = sh/2 - 65;
                    DrawRectangleRounded((Rectangle){ (float)rx, (float)ry, (float)rCardW, (float)rCardH }, 0.35f, 6, (Color){ 10, 14, 22, 220 });
                    DrawRectangleRoundedLinesEx((Rectangle){ (float)rx, (float)ry, (float)rCardW, (float)rCardH }, 0.35f, 6, 1.5f, (Color){ 220, 60, 40, 220 });
                    DrawText(respawnMsg, sw/2 - rw/2, ry + 8, 28, (Color){ 245, 245, 250, 245 });
                }

                if (state == ST_COUNTDOWN) {
                    const char *t = TextFormat("%d", (int)ceilf(stateTimer));
                    int tw = MeasureText(t, 84);
                    int cdR = 58;
                    DrawCircle(sw/2 + 2, sh/2 - 95 + 3, cdR, (Color){ 0, 0, 0, 140 });
                    DrawCircle(sw/2, sh/2 - 95, cdR, (Color){ 12, 16, 26, 235 });
                    DrawCircleLines(sw/2, sh/2 - 95, cdR, (Color){ 255, 175, 45, 240 });
                    float tFrac = fminf(1.0f, fmaxf(0.0f, stateTimer - floorf(stateTimer)));
                    DrawRing((Vector2){ (float)(sw/2), (float)(sh/2 - 95) }, cdR - 5, cdR, 0, 360.0f * tFrac, 36, (Color){ 255, 210, 80, 255 });
                    DrawText(t, sw/2 - tw/2, sh/2 - 95 - 42, 84, RAYWHITE);
                } else if (state == ST_GOAL) {
                    const char *t = lastScorer == 1 ? "BLUE SCORES!" : "ORANGE SCORES!";
                    int gw = MeasureText(t, 58);
                    int bW = gw + 90, bH = 72;
                    int bx = sw/2 - bW/2, by = sh/2 - 120;
                    Color teamGlow = lastScorer == 1 ? (Color){ 30, 120, 255, 255 } : (Color){ 255, 120, 30, 255 };
                    Color teamBorder = lastScorer == 1 ? (Color){ 90, 180, 255, 255 } : (Color){ 255, 170, 50, 255 };
                    DrawRectangleRounded((Rectangle){ (float)bx + 3, (float)by + 4, (float)bW, (float)bH }, 0.25f, 6, (Color){ 0, 0, 0, 150 });
                    DrawRectangleRounded((Rectangle){ (float)bx, (float)by, (float)bW, (float)bH }, 0.25f, 6, (Color){ 12, 16, 26, 240 });
                    DrawRectangleRoundedLinesEx((Rectangle){ (float)bx, (float)by, (float)bW, (float)bH }, 0.25f, 6, 2.5f, teamBorder);
                    DrawRectangle(bx + 20, by + bH - 5, bW - 40, 3, teamBorder);
                    DrawText(t, sw/2 - gw/2 + 2, by + 8 + 2, 58, (Color){ 0, 0, 0, 220 });
                    DrawText(t, sw/2 - gw/2, by + 8, 58, teamGlow);
                } else if (state == ST_OVER) {
                    const char *t = scoreBlue > scoreOrange ? "BLUE WINS!" : scoreOrange > scoreBlue ? "ORANGE WINS!" : "DRAW!";
                    int ow = MeasureText(t, 58);
                    int bW = ow + 110, bH = 100;
                    int bx = sw/2 - bW/2, by = sh/2 - 130;
                    DrawRectangleRounded((Rectangle){ (float)bx, (float)by, (float)bW, (float)bH }, 0.20f, 6, (Color){ 10, 14, 22, 245 });
                    DrawRectangleRoundedLinesEx((Rectangle){ (float)bx, (float)by, (float)bW, (float)bH }, 0.20f, 6, 2.0f, (Color){ 255, 180, 50, 240 });
                    DrawText(t, sw/2 - ow/2, by + 12, 58, RAYWHITE);
                    const char *sub = "PRESS [R] TO PLAY AGAIN";
                    int sww = MeasureText(sub, 20);
                    DrawText(sub, sw/2 - sww/2, by + 74, 20, (Color){ 255, 190, 80, 240 });
                }
            }
        }

        /* --- menus (logic runs here so hit-testing matches what's drawn) -- */
        if (screen != screenAtStart) memset(&nav, 0, sizeof(nav));   /* don't let the key that opened a menu also act in it */
        if (screen == SCR_MENU && shotMode != 3 && shotMode != 4) {
            MenuItem it[8];
            int sw = GetScreenWidth();
            memset(it, 0, sizeof(it));
            it[0].label = "LOCAL PLAY";
            it[1].label = "ONLINE MULTIPLAYER";
            it[2].label = "CAR";      snprintf(it[2].value, sizeof(it[2].value), "%s", CAR_NAMES[g_set.car]);
            it[3].label = "SKIN";     snprintf(it[3].value, sizeof(it[3].value), "%s", g_set.skin ? CAR_SKIN_NAMES[g_set.car] : "Team (Default)");
            it[4].label = "MODE";     snprintf(it[4].value, sizeof(it[4].value), "%s", MODE_NAMES[g_set.mode]);
            it[5].label = "BOTS";     snprintf(it[5].value, sizeof(it[5].value), "%s", SKILL_NAMES[g_set.botSkill]);
            it[6].label = "SETTINGS";
            it[7].label = "QUIT";
            menu_run("", it, 8, &menuSel, nav, &act, &adj);
            const char *t1 = "SUPERSONIC ACROBATIC";
            const char *t2 = "ROCKET-POWERED BATTLE-CARS";
            const char *t3 = "CHAMPIONSHIP EDITION";
            int w1 = MeasureText(t1, 48);
            int w2 = MeasureText(t2, 48);
            int w3 = MeasureText(t3, 18);
            DrawText(t1, sw/2 - w1/2 + 2, 50 + 2, 48, (Color){ 0, 0, 0, 200 });
            DrawText(t1, sw/2 - w1/2, 50, 48, (Color){ 255, 175, 45, 255 });
            DrawText(t2, sw/2 - w2/2 + 2, 102 + 2, 48, (Color){ 0, 0, 0, 200 });
            DrawText(t2, sw/2 - w2/2, 102, 48, (Color){ 255, 140, 25, 255 });
            DrawText(t3, sw/2 - w3/2, 158, 18, (Color){ 180, 215, 255, 230 });
        } else if (screen == SCR_ONLINE_JOIN) {
            MenuItem it[6];
            memset(it, 0, sizeof(it));
            it[0].label = "SERVER IP";      snprintf(it[0].value, sizeof(it[0].value), "%s", customIpInput);
            it[1].label = "SERVER PORT";    snprintf(it[1].value, sizeof(it[1].value), "%d", netClient.serverPort);
            it[2].label = "PLAYER NAME";    snprintf(it[2].value, sizeof(it[2].value), "%s", playerNameInput);
            it[3].label = "TEAM";           snprintf(it[3].value, sizeof(it[3].value), "%s", prefTeamSel == 0 ? "Blue" : prefTeamSel == 1 ? "Orange" : "Auto-Balance");
            it[4].label = netClient.state == NET_CONNECTING ? "CONNECTING..." : "CONNECT TO SERVER";
            it[5].label = "BACK TO MENU";
            menu_run("ONLINE MULTIPLAYER", it, 6, &onlineSel, nav, &act, &adj);

            int sw = GetScreenWidth(), sh = GetScreenHeight();
            int stw = MeasureText(netClient.statusMsg, 18);
            Color statusColor = netClient.statusOk == 2 ? GREEN :
                                netClient.statusOk == 1 ? YELLOW :
                                netClient.statusOk == 3 ? RED : GRAY;
            DrawText(netClient.statusMsg, sw / 2 - stw / 2, sh / 2 + 190, 18, statusColor);
            DrawText("Tip: Use A/D to cycle presets, or type to edit IP/Name",
                     sw / 2 - MeasureText("Tip: Use A/D to cycle presets, or type to edit IP/Name", 14) / 2,
                     sh / 2 + 218, 14, (Color){ 160, 180, 205, 200 });
        } else if (screen == SCR_PAUSE) {
            MenuItem it[5];
            memset(it, 0, sizeof(it));
            it[0].label = "RESUME";
            it[1].label = isOnline ? "LEAVE MATCH" : "RESTART MATCH";
            it[2].label = "SETTINGS";
            it[3].label = "MAIN MENU";
            it[4].label = "QUIT GAME";
            menu_run("PAUSED", it, 5, &pauseSel, nav, &act, &adj);
        } else if (screen == SCR_SETTINGS) {
            MenuItem it[13];
            memset(it, 0, sizeof(it));
            it[0].label = "CAMERA DISTANCE";  snprintf(it[0].value, 48, "%.1f m", g_set.camDist);
            it[1].label = "CAMERA HEIGHT";    snprintf(it[1].value, 48, "%.2f", g_set.camHeight);
            it[2].label = "FIELD OF VIEW";    snprintf(it[2].value, 48, "%.0f", g_set.fov);
            it[3].label = "BOOST FOV KICK";   snprintf(it[3].value, 48, "%s", g_set.boostFov ? "On" : "Off");
            it[4].label = "MATCH LENGTH";     snprintf(it[4].value, 48, "%s", MATCH_NAMES[g_set.matchIdx]);
            it[5].label = "INVERT AIR PITCH"; snprintf(it[5].value, 48, "%s", g_set.invertPitch ? "On" : "Off");
            it[6].label = "FULLSCREEN";       snprintf(it[6].value, 48, "%s", g_set.fullscreen ? "On" : "Off");
            it[7].label = "SHOW FPS";         snprintf(it[7].value, 48, "%s", g_set.showFps ? "On" : "Off");
            it[8].label = "SHOW CONTROLS";    snprintf(it[8].value, 48, "%s", g_set.showHints ? "On" : "Off");
            it[9].label = "SHADOWS";          snprintf(it[9].value, 48, "%s", g_set.shadows ? "On" : "Off");
            it[10].label = "BLOOM";           snprintf(it[10].value, 48, "%s", g_set.bloom ? "On" : "Off");
            it[11].label = "USE ORIGINAL CAMERA"; snprintf(it[11].value, 48, "5.4 m / 59");
            it[12].label = "BACK";
            menu_run("SETTINGS", it, 13, &setSel, nav, &act, &adj);
            if (setSel == 4 && settingsFrom == SCR_PAUSE)
                DrawText("match length applies to the next match", GetScreenWidth()/2 - MeasureText("match length applies to the next match", 18)/2,
                         GetScreenHeight() - 70, 18, (Color){ 255, 190, 120, 255 });
        }
        EndDrawing();

        /* --- apply menu actions ------------------------------------------ */
        if (screen == SCR_MENU) {
            if (act == 0) { isOnline = 0; NEW_MATCH(); cam.position = Vector3Add(cars[0].pos, V3(-g_set.camDist, 2.0f, 0)); screen = SCR_GAME; }
            if (act == 1) { screen = SCR_ONLINE_JOIN; onlineSel = 4; }
            if (menuSel == 2 && adj) {
                int prev = g_set.car;
                g_set.car = (g_set.car + adj + CAR_COUNT) % CAR_COUNT;
                if (!load_car(CAR_NAMES[g_set.car], &cars[0], &crs[0], lit, 1)) { g_set.car = prev; load_car(CAR_NAMES[prev], &cars[0], &crs[0], lit, 1); }
                carModel[0] = g_set.car;
                settings_save();
            }
            if (menuSel == 3 && adj) { g_set.skin = (g_set.skin + adj + 2) % 2; settings_save(); }
            if (menuSel == 4 && adj) { g_set.mode = (g_set.mode + adj + 4) % 4; settings_save(); }
            if (menuSel == 5 && adj) { g_set.botSkill = (g_set.botSkill + adj + 3) % 3; settings_save(); }
            if (act == 6) { settingsFrom = SCR_MENU; setSel = 0; screen = SCR_SETTINGS; }
            if (act == 7) quit = 1;
        } else if (screen == SCR_ONLINE_JOIN) {
            int key = GetCharPressed();
            while (key > 0) {
                if (onlineSel == 0) {
                    int len = (int)strlen(customIpInput);
                    if (len < 28 && ((key >= '0' && key <= '9') || key == '.' || key == ':' || (key >= 'a' && key <= 'z'))) {
                        customIpInput[len] = (char)key; customIpInput[len + 1] = 0;
                    }
                } else if (onlineSel == 2) {
                    int len = (int)strlen(playerNameInput);
                    if (len < 16 && (key >= 32 && key <= 126)) {
                        playerNameInput[len] = (char)key; playerNameInput[len + 1] = 0;
                    }
                }
                key = GetCharPressed();
            }
            if (IsKeyPressed(KEY_BACKSPACE)) {
                if (onlineSel == 0 && strlen(customIpInput) > 0) customIpInput[strlen(customIpInput) - 1] = 0;
                if (onlineSel == 2 && strlen(playerNameInput) > 0) playerNameInput[strlen(playerNameInput) - 1] = 0;
            }

            if (onlineSel == 0 && adj) {
                static const char *presets[] = { "127.0.0.1", "192.168.1.100", "10.0.0.1", "localhost" };
                static int preIdx = 0;
                preIdx = (preIdx + adj + 4) % 4;
                strncpy(customIpInput, presets[preIdx], sizeof(customIpInput) - 1);
            }
            if (onlineSel == 3 && adj) {
                prefTeamSel = (prefTeamSel + adj + 3) % 3;
            }

            if (act == 4 || (onlineSel == 4 && nav.ok)) {
                net_client_connect(&netClient, customIpInput, netClient.serverPort, playerNameInput, g_set.car, g_set.skin, prefTeamSel);
            }
            if (act == 5 || nav.back) {
                net_client_disconnect(&netClient);
                screen = SCR_MENU;
            }

            net_client_poll(&netClient, dt);
            if (netClient.state == NET_CONNECTED) {
                isOnline = 1;
                onlineFirstSnap = 0;
                int mySlot = netClient.localSlot;
                carModel[mySlot] = g_set.car;
                team[mySlot] = netClient.localTeam;
                load_car(CAR_NAMES[g_set.car], &cars[mySlot], &crs[mySlot], lit, 1);

                Vector3 kp; float ky;
                kickoff_spot(mySlot / 2, team[mySlot], 4, &kp, &ky);
                car_reset(&cars[mySlot], kp, ky);
                cars[mySlot].boost = BOOST_START;
                cars[mySlot].demolished = 0;
                cars[mySlot].demoTimer = 0.0f;

                for (i = 0; i < SARP_MAX_CLIENTS; i++) {
                    if (i != mySlot) {
                        cars[i].pos = V3(0, -500, 0);
                        cars[i].vel = V3(0, 0, 0);
                        cars[i].rot = (Quaternion){ 0, 0, 0, 1 };
                        carModel[i] = -1;
                    }
                    prevPos[i] = cars[i].pos;
                    prevRot[i] = cars[i].rot;
                    rcars[i] = cars[i];
                }
                ball_reset(&ball);
                prevBallPos = ball.pos;
                prevBallRot = ball.rot;
                rball = ball;
                state = ST_PLAY;
                stateTimer = 0.0f;
                acc = 0.0f;
                pendingJump = 0;
                camSnap = 1;
                screen = SCR_GAME;
            }
        } else if (screen == SCR_PAUSE) {
            if (act == 0 || nav.back) screen = SCR_GAME;
            if (act == 1) {
                if (isOnline) {
                    net_client_disconnect(&netClient);
                    isOnline = 0;
                    screen = SCR_MENU;
                } else {
                    NEW_MATCH();
                    screen = SCR_GAME;
                }
            }
            if (act == 2) { settingsFrom = SCR_PAUSE; setSel = 0; screen = SCR_SETTINGS; }
            if (act == 3) {
                if (isOnline) {
                    net_client_disconnect(&netClient);
                    isOnline = 0;
                }
                menuSel = 0;
                screen = SCR_MENU;
            }
            if (act == 4) quit = 1;
        } else if (screen == SCR_SETTINGS) {
            if (adj) {
                switch (setSel) {
                case 0: g_set.camDist   = clampf(g_set.camDist + 0.5f * adj, 3.0f, 14.0f); break;
                case 1: g_set.camHeight = clampf(g_set.camHeight + 0.02f * adj, 0.10f, 0.80f); break;
                case 2: g_set.fov       = clampf(g_set.fov + 1.0f * adj, 45.0f, 90.0f); break;
                case 3: g_set.boostFov    = !g_set.boostFov; break;
                case 4: g_set.matchIdx    = (g_set.matchIdx + adj + 4) % 4; break;
                case 5: g_set.invertPitch = !g_set.invertPitch; break;
                case 6: g_set.fullscreen  = !g_set.fullscreen; ToggleBorderlessWindowed(); break;
                case 7: g_set.showFps     = !g_set.showFps; break;
                case 8: g_set.showHints   = !g_set.showHints; break;
                case 9: g_set.shadows     = !g_set.shadows; break;
                case 10: g_set.bloom      = !g_set.bloom; break;
                default: break;
                }
            }
            if (act == 11) { g_set.camDist = 5.4f; g_set.camHeight = 0.11f; g_set.fov = 59.0f; }
            if (act == 12 || nav.back) { settings_save(); screen = settingsFrom; }
        }
    }
#undef KICKOFF
#undef NEW_MATCH
#undef SETUP_CARS

    if (netClient.state == NET_CONNECTED) {
        net_client_disconnect(&netClient);
    }
    net_client_shutdown();

    settings_save();
    for (i = 0; i < MAX_CARS; i++) car_render_unload(&crs[i]);
    UnloadTexture(fieldTex);
    UnloadTexture(ballTex);
    if (g_particleTex.id) UnloadTexture(g_particleTex);
    gfx_unload();
    CloseWindow();
    return 0;
}

#ifndef SARPBC_DLL
int main(int argc, char **argv)
{
    return sarpbc_main(argc, argv);
}
#endif
