#define _CRT_SECURE_NO_WARNINGS
#include "net_protocol.h"
#include "net_socket.h"
#include "net_client.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
  #include <windows.h>
  static inline double net_get_time_sec(void)
  {
      static LARGE_INTEGER freq;
      static int initialized = 0;
      if (!initialized) {
          QueryPerformanceFrequency(&freq);
          initialized = 1;
      }
      LARGE_INTEGER count;
      QueryPerformanceCounter(&count);
      return (double)count.QuadPart / (double)freq.QuadPart;
  }
#else
  #include <sys/time.h>
  static inline double net_get_time_sec(void)
  {
      struct timeval tv;
      gettimeofday(&tv, NULL);
      return (double)tv.tv_sec + (double)tv.tv_usec * 1e-6;
  }
#endif

void net_client_init(NetClient *cli)
{
    if (!cli) return;
    memset(cli, 0, sizeof(NetClient));
    cli->sock = (uintptr_t)NET_INVALID_SOCKET;
    cli->state = NET_DISCONNECTED;
    cli->serverPort = SARP_DEFAULT_PORT;
    snprintf(cli->serverIp, sizeof(cli->serverIp), "127.0.0.1");
    snprintf(cli->playerName, sizeof(cli->playerName), "Striker");
    cli->preferredTeam = 2; /* Auto */
    cli->localSlot = 0;
    cli->statusOk = 0; /* Normal/gray */
    snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Ready to connect.");
}

int net_client_connect(NetClient *cli, const char *ip, int port, const char *name, int car_model, int skin, int pref_team)
{
    if (!cli) return 0;
    net_init();

    if ((NetSocket)cli->sock != NET_INVALID_SOCKET) {
        net_socket_close((NetSocket)cli->sock);
        cli->sock = (uintptr_t)NET_INVALID_SOCKET;
    }

    NetSocket s = net_socket_create(0, 1); /* Ephemeral port, non-blocking */
    if (s == NET_INVALID_SOCKET) {
        snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Error: Failed to create UDP socket.");
        cli->statusOk = 3; /* Red */
        cli->state = NET_DISCONNECTED;
        return 0;
    }
    cli->sock = (uintptr_t)s;

    if (ip && ip[0]) snprintf(cli->serverIp, sizeof(cli->serverIp), "%s", ip);
    if (port > 0) cli->serverPort = port;
    if (name && name[0]) snprintf(cli->playerName, sizeof(cli->playerName), "%s", name);
    cli->carModel = car_model;
    cli->skin = skin;
    cli->preferredTeam = pref_team;

    cli->state = NET_CONNECTING;
    cli->connectTimer = 0.0f;
    cli->connectAttempts = 0;
    cli->clientSeq = 1;
    cli->serverSeqAck = 0;
    cli->clientTick = 0;
    cli->pingMs = 0.0f;
    cli->pingTimer = 0.0f;
    cli->lastPacketTime = net_get_time_sec();

    snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Connecting to %s:%d...", cli->serverIp, cli->serverPort);
    cli->statusOk = 1; /* Yellow */

    /* Send initial Join Request */
    PktJoinReq req;
    memset(&req, 0, sizeof(req));
    req.header.magic = SARP_NET_MAGIC;
    req.header.version = SARP_NET_VERSION;
    req.header.type = PKT_JOIN_REQ;
    req.header.player_id = 0xFF;
    req.header.seq = ++cli->clientSeq;
    req.header.ack = 0;
    snprintf(req.player_name, sizeof(req.player_name), "%s", cli->playerName);
    req.car_model = (uint8_t)cli->carModel;
    req.skin = (uint8_t)cli->skin;
    req.pref_team = (uint8_t)cli->preferredTeam;

    NetAddr srvAddr = net_addr_create(cli->serverIp, cli->serverPort);
    net_socket_send(s, &srvAddr, &req, sizeof(req));
    cli->connectAttempts = 1;
    return 1;
}

void net_client_disconnect(NetClient *cli)
{
    if (!cli) return;
    NetSocket s = (NetSocket)cli->sock;
    if (s != NET_INVALID_SOCKET && cli->state == NET_CONNECTED) {
        PktDisconnect pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.header.magic = SARP_NET_MAGIC;
        pkt.header.version = SARP_NET_VERSION;
        pkt.header.type = PKT_DISCONNECT;
        pkt.header.player_id = (uint8_t)cli->localSlot;
        pkt.header.seq = ++cli->clientSeq;
        pkt.reason = 0;
        NetAddr srvAddr = net_addr_create(cli->serverIp, cli->serverPort);
        net_socket_send(s, &srvAddr, &pkt, sizeof(pkt));
    }
    if (s != NET_INVALID_SOCKET) {
        net_socket_close(s);
        cli->sock = (uintptr_t)NET_INVALID_SOCKET;
    }
    cli->state = NET_DISCONNECTED;
    cli->statusOk = 0;
    snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Disconnected from server.");
}

void net_client_send_tick(NetClient *cli, const NetInput *curInput)
{
    if (!cli || (NetSocket)cli->sock == NET_INVALID_SOCKET || cli->state != NET_CONNECTED) return;
    NetSocket s = (NetSocket)cli->sock;

    /* Shift input history */
    cli->inputHistory[2] = cli->inputHistory[1];
    cli->inputHistory[1] = cli->inputHistory[0];
    if (curInput) cli->inputHistory[0] = *curInput;

    PktClientInput pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.header.magic = SARP_NET_MAGIC;
    pkt.header.version = SARP_NET_VERSION;
    pkt.header.type = PKT_CLIENT_INPUT;
    pkt.header.player_id = (uint8_t)cli->localSlot;
    pkt.header.seq = ++cli->clientSeq;
    pkt.header.ack = cli->serverSeqAck;

    pkt.client_tick = ++cli->clientTick;
    pkt.server_tick_ack = cli->serverSeqAck;
    pkt.input = cli->inputHistory[0];
    pkt.prev_inputs[0] = cli->inputHistory[1];
    pkt.prev_inputs[1] = cli->inputHistory[2];

    NetAddr srvAddr = net_addr_create(cli->serverIp, cli->serverPort);
    net_socket_send(s, &srvAddr, &pkt, sizeof(pkt));

    /* Periodic Ping (every 1 second) */
    double now = net_get_time_sec();
    if (now - cli->pingTimer > 1.0) {
        cli->pingTimer = (float)now;
        PktPing ping;
        memset(&ping, 0, sizeof(ping));
        ping.header.magic = SARP_NET_MAGIC;
        ping.header.version = SARP_NET_VERSION;
        ping.header.type = PKT_PING;
        ping.header.player_id = (uint8_t)cli->localSlot;
        ping.header.seq = ++cli->clientSeq;
        ping.header.ack = cli->serverSeqAck;
        ping.client_time_ms = (uint32_t)(now * 1000.0);
        net_socket_send(s, &srvAddr, &ping, sizeof(ping));
    }
}

void net_client_poll(NetClient *cli, float dt)
{
    if (!cli || (NetSocket)cli->sock == NET_INVALID_SOCKET) return;
    NetSocket s = (NetSocket)cli->sock;

    if (cli->goalBannerTimer > 0.0f) cli->goalBannerTimer -= dt;
    if (cli->demoBannerTimer > 0.0f) cli->demoBannerTimer -= dt;

    /* Handle Connecting Retries & Timeout */
    if (cli->state == NET_CONNECTING) {
        cli->connectTimer += dt;
        if (cli->connectTimer >= 0.5f) {
            cli->connectTimer = 0.0f;
            if (cli->connectAttempts >= 8) {
                cli->state = NET_DISCONNECTED;
                cli->statusOk = 3; /* Red */
                snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Connection timed out. Server unreachable.");
                return;
            }
            /* Resend Join Req */
            PktJoinReq req;
            memset(&req, 0, sizeof(req));
            req.header.magic = SARP_NET_MAGIC;
            req.header.version = SARP_NET_VERSION;
            req.header.type = PKT_JOIN_REQ;
            req.header.player_id = 0xFF;
            req.header.seq = ++cli->clientSeq;
            req.header.ack = 0;
            snprintf(req.player_name, sizeof(req.player_name), "%s", cli->playerName);
            req.car_model = (uint8_t)cli->carModel;
            req.skin = (uint8_t)cli->skin;
            req.pref_team = (uint8_t)cli->preferredTeam;
            NetAddr srvAddr = net_addr_create(cli->serverIp, cli->serverPort);
            net_socket_send(s, &srvAddr, &req, sizeof(req));
            cli->connectAttempts++;
            snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Connecting to %s:%d... (attempt %d/8)",
                     cli->serverIp, cli->serverPort, cli->connectAttempts);
        }
    }

    /* Process all pending incoming UDP packets */
    uint8_t buf[2048];
    NetAddr sender;
    while (1) {
        int n = net_socket_recv(s, &sender, buf, sizeof(buf));
        if (n <= 0) break;
        if ((size_t)n < sizeof(NetHeader)) continue;

        NetHeader *hdr = (NetHeader *)buf;
        if (hdr->magic != SARP_NET_MAGIC || hdr->version != SARP_NET_VERSION) continue;

        cli->lastPacketTime = net_get_time_sec();

        if (hdr->type == PKT_JOIN_ACK) {
            PktJoinAck *ack = (PktJoinAck *)buf;
            if (ack->accepted) {
                cli->state = NET_CONNECTED;
                cli->localSlot = ack->assigned_id;
                cli->localTeam = ack->assigned_team;
                cli->serverSeqAck = hdr->seq;
                cli->statusOk = 2; /* Green */
                snprintf(cli->statusMsg, sizeof(cli->statusMsg),
                         "Connected! Joined as %s Team (Slot %d)",
                         cli->localTeam == 0 ? "BLUE" : "ORANGE", cli->localSlot);
            } else {
                cli->state = NET_DISCONNECTED;
                cli->statusOk = 3; /* Red */
                snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Server full or connection rejected.");
            }
        } else if (hdr->type == PKT_PONG) {
            PktPong *pong = (PktPong *)buf;
            uint32_t now_ms = (uint32_t)(net_get_time_sec() * 1000.0);
            cli->pingMs = (float)(now_ms - pong->client_time_ms);
            cli->serverSeqAck = hdr->seq;
        } else if (hdr->type == PKT_SERVER_STATE) {
            PktServerState *st = (PktServerState *)buf;
            cli->serverSeqAck = st->server_tick;
            cli->serverTick = st->server_tick;
            cli->serverGameState = st->game_state;
            cli->serverMatchTime = st->match_time;
            cli->scoreBlue = st->score_blue;
            cli->scoreOrange = st->score_orange;

            /* Unpack authoritative ball */
            cli->ballTargetPos = st->ball.pos;
            cli->ballTargetVel = st->ball.vel;
            cli->ballTargetRot = st->ball.rot;

            /* Unpack authoritative cars */
            int activeCount = 0;
            for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                NetCarState *ncs = &st->cars[i];
                NetClientPlayer *p = &cli->players[i];
                p->active = ncs->active;
                if (!p->active) continue;
                activeCount++;

                p->team = ncs->team;
                p->car_model = ncs->car_model;
                p->skin = ncs->skin;
                snprintf(p->name, sizeof(p->name), "%s", ncs->player_name[0] ? ncs->player_name : "Player");
                p->wheelsOnGround = ncs->wheels_on_ground;
                p->supersonic = ncs->supersonic;
                p->boost = ncs->boost;
                p->steerAngle = ncs->steerAngle;
                p->wheelSpin = ncs->wheelSpin;

                p->targetPos = ncs->pos;
                p->targetVel = ncs->vel;
                p->targetRot = ncs->rot;

                if (ncs->demolished && !p->demolished) {
                    p->demolished = 1;
                    p->demoTimer = 3.0f;
                } else if (!ncs->demolished) {
                    p->demolished = 0;
                }
            }
            cli->playerCount = activeCount;
        } else if (hdr->type == PKT_EVENT_GOAL) {
            PktEventGoal *eg = (PktEventGoal *)buf;
            cli->hasGoalEvent = 1;
            cli->goalTeam = eg->team;
            cli->goalBallSpeed = eg->ball_speed_kmh;
            cli->goalBannerTimer = 3.5f;
        } else if (hdr->type == PKT_EVENT_DEMO) {
            PktEventDemo *ed = (PktEventDemo *)buf;
            cli->hasDemoEvent = 1;
            const char *kName = (ed->killer_id < SARP_MAX_CLIENTS) ? cli->players[ed->killer_id].name : "Player";
            const char *vName = (ed->victim_id < SARP_MAX_CLIENTS) ? cli->players[ed->victim_id].name : "Player";
            snprintf(cli->demoKiller, sizeof(cli->demoKiller), "%s", kName);
            snprintf(cli->demoVictim, sizeof(cli->demoVictim), "%s", vName);
            cli->demoBannerTimer = 3.0f;
        } else if (hdr->type == PKT_DISCONNECT) {
            cli->state = NET_DISCONNECTED;
            cli->statusOk = 3; /* Red */
            snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Server closed or match ended.");
        }
    }

    /* Watchdog: drop if no packet received for 5 seconds while connected */
    if (cli->state == NET_CONNECTED && net_get_time_sec() - cli->lastPacketTime > 5.0) {
        cli->state = NET_DISCONNECTED;
        cli->statusOk = 3; /* Red */
        snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Lost connection to server (timed out).");
    }
}

void net_client_shutdown(void)
{
    net_shutdown();
}
