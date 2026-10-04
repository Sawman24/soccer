#ifndef NET_PROTOCOL_H
#define NET_PROTOCOL_H

#include <stdint.h>
#include <string.h>

#define SARP_NET_MAGIC      0x53415250   /* "SARP" */
#define SARP_NET_VERSION    1
#define SARP_DEFAULT_PORT   7777
#define SARP_MAX_CLIENTS    8
#define SARP_TICK_RATE      60

#pragma pack(push, 1)

typedef struct NetVec3 {
    float x, y, z;
} NetVec3;

typedef struct NetQuat {
    float x, y, z, w;
} NetQuat;

typedef enum PacketType {
    PKT_NONE = 0,
    PKT_PING,
    PKT_PONG,
    PKT_JOIN_REQ,
    PKT_JOIN_ACK,
    PKT_DISCONNECT,
    PKT_CLIENT_INPUT,
    PKT_SERVER_STATE,
    PKT_EVENT_GOAL,
    PKT_EVENT_DEMO
} PacketType;

typedef struct NetHeader {
    uint32_t magic;      /* SARP_NET_MAGIC */
    uint16_t version;    /* SARP_NET_VERSION */
    uint8_t  type;       /* PacketType */
    uint8_t  player_id;  /* Sender player ID (or 0xFF for server) */
    uint32_t seq;        /* Packet sequence number */
    uint32_t ack;        /* Highest received sequence number from peer */
} NetHeader;

typedef struct PktPing {
    NetHeader header;
    uint32_t  client_time_ms;
} PktPing;

typedef struct PktPong {
    NetHeader header;
    uint32_t  client_time_ms;
    uint32_t  server_time_ms;
} PktPong;

typedef struct PktJoinReq {
    NetHeader header;
    char      player_name[24];
    uint8_t   car_model;     /* 0..12 */
    uint8_t   skin;          /* 0 = default livery, 1 = custom */
    uint8_t   pref_team;     /* 0 = Blue, 1 = Orange, 2 = Auto */
} PktJoinReq;

typedef struct PktJoinAck {
    NetHeader header;
    uint8_t   accepted;      /* 1 = OK, 0 = rejected (server full) */
    uint8_t   assigned_id;   /* 0..7 */
    uint8_t   assigned_team; /* 0 = Blue, 1 = Orange */
    uint8_t   car_model;
    uint8_t   skin;
    uint16_t  tick_rate;     /* 60 */
    char      server_name[32];
} PktJoinAck;

typedef struct PktDisconnect {
    NetHeader header;
    uint8_t   reason;        /* 0 = user quit, 1 = kicked, 2 = timeout */
} PktDisconnect;

/* Packed input for 1 physics tick */
typedef struct NetInput {
    float   throttle;        /* -1.0 .. 1.0 */
    float   steer;           /* -1.0 .. 1.0 */
    float   pitch;           /* -1.0 .. 1.0 */
    float   yaw;             /* -1.0 .. 1.0 */
    float   roll;            /* -1.0 .. 1.0 */
    uint8_t jump;
    uint8_t jumpPressed;
    uint8_t boost;
    uint8_t slide;
} NetInput;

typedef struct PktClientInput {
    NetHeader header;
    uint32_t  client_tick;
    uint32_t  server_tick_ack;
    NetInput  input;
    /* Redundant previous inputs (last 2 frames) so 1 dropped packet causes 0 loss */
    NetInput  prev_inputs[2];
} PktClientInput;

typedef struct NetBallState {
    NetVec3 pos;
    NetVec3 vel;
    NetVec3 angVel;
    NetQuat rot;
} NetBallState;

typedef struct NetCarState {
    uint8_t  active;
    uint8_t  player_id;
    uint8_t  team;           /* 0 = Blue, 1 = Orange */
    uint8_t  car_model;      /* 0..12 */
    uint8_t  skin;
    uint8_t  demolished;
    uint8_t  wheels_on_ground;
    uint8_t  supersonic;
    char     player_name[20];
    float    boost;
    NetVec3  pos;
    NetVec3  vel;
    NetVec3  angVel;
    NetQuat  rot;
    float    steerAngle;
    float    wheelSpin;
} NetCarState;

typedef struct PktServerState {
    NetHeader    header;
    uint32_t     server_tick;
    uint8_t      game_state;     /* 0 = COUNTDOWN, 1 = PLAY, 2 = GOAL, 3 = OVER */
    float        match_time;     /* seconds remaining */
    uint8_t      score_blue;
    uint8_t      score_orange;
    uint8_t      countdown_sec;
    uint8_t      car_count;
    NetBallState ball;
    NetCarState  cars[SARP_MAX_CLIENTS];
} PktServerState;

typedef struct PktEventGoal {
    NetHeader header;
    uint8_t   scorer_id;
    uint8_t   team;
    float     ball_speed_kmh;
} PktEventGoal;

typedef struct PktEventDemo {
    NetHeader header;
    uint8_t   killer_id;
    uint8_t   victim_id;
} PktEventDemo;

#pragma pack(pop)

#endif /* NET_PROTOCOL_H */
