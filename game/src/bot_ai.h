/* Bot AI shared by the game (offline matches) and the dedicated server
 * (quick-match bot fill). Include after physics_sim.h. */
#ifndef BOT_AI_H
#define BOT_AI_H
#include <stdlib.h>
#include <string.h>
/* ------------------------------------------------------------------------ */
/* Bots                                                                       */
/* ------------------------------------------------------------------------ */
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


/* Shared world knowledge, refreshed once per physics tick by bot_world_update().
 * One BotWorld per simulated match: the server points g_bw at the match being
 * stepped (the game just uses the default one). */
typedef struct BotWorld {
    Vector3        pred[PRED_N], predVel[PRED_N];   /* ball path, sample i = i*PRED_DT after predAge ago */
    float          predAge;
    int            predCalls;
    const Vector3 *pads;
    const float   *padTimer;
    int            padCount;
} BotWorld;
static BotWorld  g_botWorldDefault;
static BotWorld *g_bw = &g_botWorldDefault;

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
    g_bw->pads = pads; g_bw->padTimer = padTimer; g_bw->padCount = npads;
    g_bw->predAge += dt;
    k = (int)(g_bw->predAge / PRED_DT + 0.5f);
    if (g_bw->predCalls++ > 0 && k < PRED_N - 30 && (g_bw->predCalls % 6) != 0 &&
        Vector3Distance(g_bw->predVel[k], b->vel) < 1.5f)
        return;
    g_bw->predAge = 0.0f;
    for (i = 0; i < PRED_N; i++) {
        g_bw->pred[i] = s.pos; g_bw->predVel[i] = s.vel;
        ball_step(&s, PRED_DT * 0.5f);
        ball_step(&s, PRED_DT * 0.5f);
    }
}

static Vector3 pred_at(float t)
{
    float f = (t + g_bw->predAge) / PRED_DT;
    int i = (int)f;
    if (i < 0) return g_bw->pred[0];
    if (i >= PRED_N - 1) return g_bw->pred[PRED_N - 1];
    return Vector3Lerp(g_bw->pred[i], g_bw->pred[i + 1], f - (float)i);
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
    Intercept r = { PRED_N * PRED_DT, g_bw->pred[PRED_N - 1], 0, 0, 0 };
    int i;
    drive_cover(c, useBoost, cov);
    for (i = 0; i < PRED_N; i++) {
        float t = (float)i * PRED_DT - g_bw->predAge, d, tt;
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
    for (i = 0; i < g_bw->padCount; i++) {
        float dToPad, dPadToTgt, directDist, detour, score;
        if (g_bw->padTimer[i] > 0.5f) continue; /* must be active or about to respawn */
        dToPad = flat_dist(c->pos, g_bw->pads[i]);
        dPadToTgt = flat_dist(g_bw->pads[i], target);
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
        if (bt->kickJitter == 0.0f) bt->kickJitter = (float)(rand() % 201 - 100) / 100.0f + 0.001f;
        I.x += bt->kickJitter * 0.9f;
    }

    /* is the ball about to go into our net? */
    for (i = 0; i < (int)(3.0f / PRED_DT); i++)
        if (g_bw->pred[i].z * as < -(ARENA_L - 1.0f) && fabsf(g_bw->pred[i].x) < GOAL_HW + 2.5f) { danger = 1; break; }

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
            if (p >= 0) target = g_bw->pads[p];
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
                if (p >= 0) target = g_bw->pads[p];
            }
        }
    } else {
        /* Support / Second Man: shadow play closely (18-24 m), pounce on rebounds */
        target = V3(b->pos.x * 0.65f, 0, b->pos.z - as * 20.0f);
        if (skill >= 1 && c->boost < 2.0f) {
            int p = best_pad(c, target, 35.0f);
            if (p >= 0) target = g_bw->pads[p];
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

#endif /* BOT_AI_H */
