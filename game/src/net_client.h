#ifndef NET_CLIENT_H
#define NET_CLIENT_H

#include <stdint.h>
#include <string.h>
#include "net_protocol.h"

typedef enum NetClientState {
    NET_DISCONNECTED = 0,
    NET_CONNECTING,
    NET_CONNECTED
} NetClientState;

typedef struct NetClientPlayer {
    int      active;
    char     name[24];
    int      team;          /* 0 = Blue, 1 = Orange */
    int      car_model;
    int      skin;
    int      demolished;
    float    demoTimer;
    float    boost;
    int      wheelsOnGround;
    int      supersonic;
    int      boosting;
    float    steerAngle;
    float    wheelSpin;
    NetVec3  targetPos;
    NetVec3  targetVel;
    NetQuat  targetRot;
} NetClientPlayer;

typedef struct NetClient {
    uintptr_t        sock;
    NetClientState   state;
    char             serverIp[64];
    int              serverPort;
    char             playerName[24];
    int              preferredTeam;   /* 0 = Blue, 1 = Orange, 2 = Auto */
    int              localSlot;       /* 0..7 assigned by server */
    int              localTeam;       /* 0 = Blue, 1 = Orange */
    int              carModel;
    int              skin;
    uint32_t         clientSeq;
    uint32_t         serverSeqAck;
    uint32_t         clientTick;
    float            connectTimer;
    int              connectAttempts;
    float            pingMs;
    float            pingTimer;
    double           lastPacketTime;
    char             statusMsg[128];
    int              statusOk;        /* 0 = gray, 1 = yellow, 2 = green, 3 = red */

    NetInput         inputHistory[3];

    /* Authoritative ball state from server */
    NetVec3          ballTargetPos;
    NetVec3          ballTargetVel;
    NetQuat          ballTargetRot;

    /* Authoritative match state */
    uint8_t          serverGameState; /* 0 = COUNTDOWN, 1 = PLAY, 2 = GOAL, 3 = OVER */
    float            serverMatchTime;
    uint8_t          scoreBlue;
    uint8_t          scoreOrange;
    uint32_t         serverTick;

    /* Players list */
    NetClientPlayer  players[SARP_MAX_CLIENTS];
    int              playerCount;

    /* Events */
    int              hasGoalEvent;
    int              goalTeam;
    float            goalBallSpeed;
    float            goalBannerTimer;

    int              hasDemoEvent;
    char             demoKiller[24];
    char             demoVictim[24];
    float            demoBannerTimer;
} NetClient;

void net_client_init(NetClient *cli);
int  net_client_connect(NetClient *cli, const char *ip, int port, const char *name, int car_model, int skin, int pref_team);
void net_client_disconnect(NetClient *cli);
void net_client_send_tick(NetClient *cli, const NetInput *curInput);
void net_client_poll(NetClient *cli, float dt);
void net_client_shutdown(void);

#endif /* NET_CLIENT_H */
