#ifndef NET_CLIENT_H
#define NET_CLIENT_H

#include <stdint.h>
#include <string.h>
#include "net_protocol.h"

typedef enum NetClientState {
    NET_DISCONNECTED = 0,
    NET_CONNECTING,   /* waiting for the server to answer           */
    NET_QUEUED,       /* searching for a match (quick-match queue)  */
    NET_CONNECTED     /* in a match                                 */
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
    int      isBot;
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
    uint8_t          serverAddr[16];  /* resolved sockaddr_in (resolved once at connect) */
    char             playerName[24];
    int              preferredTeam;   /* 0 = Blue, 1 = Orange, 2 = Auto */
    int              playlist;        /* players per team: 1, 2, 3 */
    int              localSlot;       /* 0..7 assigned by server */
    int              localTeam;       /* 0 = Blue, 1 = Orange */
    int              carModel;
    int              skin;
    uint32_t         clientSeq;
    uint32_t         serverSeqAck;
    uint32_t         clientTick;      /* number of the last input sent (= last predicted tick) */
    float            connectTimer;
    int              connectAttempts;
    float            pingMs;          /* smoothed round-trip time */
    double           pingTimer;
    double           lastPacketTime;
    char             statusMsg[128];
    int              statusOk;        /* 0 = gray, 1 = yellow, 2 = green, 3 = red */

    NetInput         inputHistory[3];

    /* Authoritative ball state from server */
    NetVec3          ballTargetPos;
    NetVec3          ballTargetVel;
    NetQuat          ballTargetRot;

    /* Quick-match queue (NET_QUEUED) */
    int              queueInQueue, queueNeeded, queuePlayersOnline, queueMatches;
    float            queueSearchSec;  /* extrapolated locally between updates */
    float            queueBotFillSec; /* < 0 = bot fill disabled */
    int              disconnectReason;/* DISC_* of the last server disconnect, -1 = none */

    /* Authoritative match state */
    uint8_t          serverGameState; /* 0 = COUNTDOWN, 1 = PLAY, 2 = GOAL, 3 = OVER */
    float            serverStateTimer;
    float            serverMatchTime;
    uint8_t          scoreBlue;
    uint8_t          scoreOrange;
    uint8_t          serverPlaylist;
    uint8_t          overtime;
    uint32_t         serverTick;

    /* Latest raw snapshot, consumed by the game's rollback/reconcile step */
    uint32_t         snapCount;       /* bumps every time a newer snapshot arrives */
    uint32_t         snapAckTick;     /* last of OUR input ticks the server had simulated */
    NetBallState     snapBall;
    NetCarState      snapCars[SARP_MAX_CLIENTS];

    /* Connection quality */
    uint32_t         snapsReceived;
    uint32_t         snapsLost;
    uint32_t         snapsLate;       /* arrived out of order, discarded */
    float            lossPct;         /* recent snapshot loss % */

    /* Players list */
    NetClientPlayer  players[SARP_MAX_CLIENTS];
    int              playerCount;

    /* Events */
    int              hasGoalEvent;
    int              goalTeam;
    float            goalBallSpeed;
    float            goalBannerTimer;

    int              hasDemoEvent;
    int              demoKillerId, demoVictimId;
    char             demoKiller[24];
    char             demoVictim[24];
    float            demoBannerTimer;
} NetClient;

void net_client_init(NetClient *cli);
/* Connect and start searching the given playlist (players per team). */
int  net_client_connect(NetClient *cli, const char *ip, int port, const char *name, int car_model, int skin, int playlist);
void net_client_disconnect(NetClient *cli);
void net_client_send_tick(NetClient *cli, const NetInput *curInput);
void net_client_poll(NetClient *cli, float dt);
void net_client_shutdown(void);

#endif /* NET_CLIENT_H */
