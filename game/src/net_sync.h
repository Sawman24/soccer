#ifndef NET_SYNC_H
#define NET_SYNC_H

/*
 * Conversions between the simulation types (physics_sim.h) and the wire
 * format (net_protocol.h). Shared by the dedicated server and the client so
 * both sides always agree on exactly which state is replicated.
 *
 * Include after physics_sim.h and net_protocol.h.
 */

static inline int8_t ns_q8(float v)
{
    v = clampf(v, -1.0f, 1.0f) * 127.0f;
    return (int8_t)(v < 0.0f ? v - 0.5f : v + 0.5f);
}

static inline NetInput ns_input_to_net(const Input *in)
{
    NetInput n;
    memset(&n, 0, sizeof(n));
    n.throttle = in->throttle; n.steer = in->steer;
    n.pitch = in->pitch; n.yaw = in->yaw; n.roll = in->roll;
    n.jump = (uint8_t)(in->jump != 0);
    n.jumpPressed = (uint8_t)(in->jumpPressed != 0);
    n.boost = (uint8_t)(in->boost != 0);
    n.slide = (uint8_t)(in->slide != 0);
    return n;
}

static inline Input ns_input_from_net(const NetInput *n)
{
    Input in;
    in.throttle = clampf(n->throttle, -1.0f, 1.0f);
    in.steer    = clampf(n->steer,    -1.0f, 1.0f);
    in.pitch    = clampf(n->pitch,    -1.0f, 1.0f);
    in.yaw      = clampf(n->yaw,      -1.0f, 1.0f);
    in.roll     = clampf(n->roll,     -1.0f, 1.0f);
    in.jump = n->jump != 0; in.jumpPressed = n->jumpPressed != 0;
    in.boost = n->boost != 0; in.slide = n->slide != 0;
    return in;
}

/* Server -> wire: full replicated state for one car plus the input it used. */
static inline void ns_car_to_net(const Car *c, const Input *applied, NetCarState *n)
{
    n->demolished       = (uint8_t)(c->demolished != 0);
    n->wheels_on_ground = (uint8_t)c->wheelsOnGround;
    n->supersonic       = (uint8_t)(Vector3Length(c->vel) >= 37.0f);
    n->boost      = c->boost;
    n->pos        = (NetVec3){ c->pos.x, c->pos.y, c->pos.z };
    n->vel        = (NetVec3){ c->vel.x, c->vel.y, c->vel.z };
    n->angVel     = (NetVec3){ c->angVel.x, c->angVel.y, c->angVel.z };
    n->rot        = (NetQuat){ c->rot.x, c->rot.y, c->rot.z, c->rot.w };
    n->steerAngle = c->steerAngle;
    n->wheelSpin  = c->wheelSpin;
    n->jumpTimer = c->jumpTimer;   n->flipTimer = c->flipTimer;
    n->airTime = c->airTime;       n->landTimer = c->landTimer;
    n->boostMinTimer = c->boostMinTimer; n->uprightTimer = c->uprightTimer;
    n->flipDirX = c->flipDir.x;    n->flipDirY = c->flipDir.y;
    n->flags = (uint8_t)((c->hasJumped ? NCF_HAS_JUMPED : 0) |
                         (c->hasFlipped ? NCF_HAS_FLIPPED : 0) |
                         (c->boosting ? NCF_BOOSTING : 0));
    n->demo_tenths = (uint8_t)clampf(c->demoTimer * 10.0f + 0.99f, 0.0f, 255.0f);
    if (applied) {
        n->in_throttle = ns_q8(applied->throttle); n->in_steer = ns_q8(applied->steer);
        n->in_pitch = ns_q8(applied->pitch); n->in_yaw = ns_q8(applied->yaw); n->in_roll = ns_q8(applied->roll);
        n->in_buttons = (uint8_t)((applied->jump ? NIB_JUMP : 0) | (applied->boost ? NIB_BOOST : 0) |
                                  (applied->slide ? NIB_SLIDE : 0));
    }
}

/* Wire -> sim: overwrite the replicated state, keep model dimensions intact. */
static inline void ns_car_from_net(const NetCarState *n, Car *c)
{
    Quaternion q = (Quaternion){ n->rot.x, n->rot.y, n->rot.z, n->rot.w };
    float ql = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    c->pos    = V3(n->pos.x, n->pos.y, n->pos.z);
    c->vel    = V3(n->vel.x, n->vel.y, n->vel.z);
    c->angVel = V3(n->angVel.x, n->angVel.y, n->angVel.z);
    c->rot    = ql > 0.5f ? q : QuaternionIdentity();
    c->boost  = n->boost;
    c->steerAngle = n->steerAngle;
    c->jumpTimer = n->jumpTimer;   c->flipTimer = n->flipTimer;
    c->airTime = n->airTime;       c->landTimer = n->landTimer;
    c->boostMinTimer = n->boostMinTimer; c->uprightTimer = n->uprightTimer;
    c->flipDir = (Vector2){ n->flipDirX, n->flipDirY };
    c->hasJumped  = (n->flags & NCF_HAS_JUMPED) != 0;
    c->hasFlipped = (n->flags & NCF_HAS_FLIPPED) != 0;
    c->boosting   = (n->flags & NCF_BOOSTING) != 0;
    c->demolished = n->demolished;
    c->demoTimer  = n->demo_tenths * 0.1f;
}

/* Best guess of what a remote player is pressing: their last applied input. */
static inline Input ns_remote_input(const NetCarState *n)
{
    Input in;
    in.throttle = n->in_throttle / 127.0f; in.steer = n->in_steer / 127.0f;
    in.pitch = n->in_pitch / 127.0f; in.yaw = n->in_yaw / 127.0f; in.roll = n->in_roll / 127.0f;
    in.jump  = (n->in_buttons & NIB_JUMP) != 0;
    in.boost = (n->in_buttons & NIB_BOOST) != 0;
    in.slide = (n->in_buttons & NIB_SLIDE) != 0;
    in.jumpPressed = 0;   /* never predict new jumps/flips for other players */
    return in;
}

static inline void ns_ball_to_net(const Ball *b, NetBallState *n)
{
    n->pos    = (NetVec3){ b->pos.x, b->pos.y, b->pos.z };
    n->vel    = (NetVec3){ b->vel.x, b->vel.y, b->vel.z };
    n->angVel = (NetVec3){ b->angVel.x, b->angVel.y, b->angVel.z };
    n->rot    = (NetQuat){ b->rot.x, b->rot.y, b->rot.z, b->rot.w };
}

static inline void ns_ball_from_net(const NetBallState *n, Ball *b)
{
    Quaternion q = (Quaternion){ n->rot.x, n->rot.y, n->rot.z, n->rot.w };
    float ql = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    b->pos    = V3(n->pos.x, n->pos.y, n->pos.z);
    b->vel    = V3(n->vel.x, n->vel.y, n->vel.z);
    b->angVel = V3(n->angVel.x, n->angVel.y, n->angVel.z);
    b->rot    = ql > 0.5f ? q : QuaternionIdentity();
}

#endif /* NET_SYNC_H */
