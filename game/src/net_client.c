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

static NetAddr cli_server_addr(const NetClient *cli)
{
    NetAddr a;
    memcpy(&a.in, cli->serverAddr, sizeof(a.in));
    return a;
}

static void cli_send(NetClient *cli, const void *pkt, int len)
{
    NetAddr a = cli_server_addr(cli);
    net_socket_send((NetSocket)cli->sock, &a, pkt, len);
}

static void cli_header(NetClient *cli, NetHeader *h, uint8_t type, uint8_t player_id)
{
    h->magic = SARP_NET_MAGIC;
    h->version = SARP_NET_VERSION;
    h->type = type;
    h->player_id = player_id;
    h->seq = ++cli->clientSeq;
    h->ack = cli->serverSeqAck;
}

static void cli_send_join(NetClient *cli)
{
    PktJoinReq req;
    memset(&req, 0, sizeof(req));
    cli_header(cli, &req.header, PKT_JOIN_REQ, 0xFF);
    req.header.ack = 0;
    snprintf(req.player_name, sizeof(req.player_name), "%s", cli->playerName);
    req.car_model = (uint8_t)cli->carModel;
    req.skin = (uint8_t)cli->skin;
    req.pref_team = (uint8_t)cli->preferredTeam;
    req.playlist = (uint8_t)cli->playlist;
    cli_send(cli, &req, sizeof(req));
}

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
    cli->playlist = 2;
    cli->disconnectReason = -1;
    cli->localSlot = 0;
    cli->statusOk = 0; /* Normal/gray */
    snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Ready to connect.");
}

int net_client_connect(NetClient *cli, const char *ip, int port, const char *name, int car_model, int skin, int playlist)
{
    NetAddr resolved;
    if (!cli) return 0;
    net_init();

    if ((NetSocket)cli->sock != NET_INVALID_SOCKET) {
        net_socket_close((NetSocket)cli->sock);
        cli->sock = (uintptr_t)NET_INVALID_SOCKET;
    }

    if (ip && ip[0]) snprintf(cli->serverIp, sizeof(cli->serverIp), "%s", ip);
    if (port > 0) cli->serverPort = port;
    if (name && name[0]) snprintf(cli->playerName, sizeof(cli->playerName), "%s", name);
    cli->carModel = car_model;
    cli->skin = skin;
    cli->preferredTeam = 2;
    cli->playlist = (playlist >= 1 && playlist <= 3) ? playlist : 2;
    cli->disconnectReason = -1;
    cli->queueInQueue = 0; cli->queueNeeded = 2 * cli->playlist;
    cli->queueSearchSec = 0.0f; cli->queueBotFillSec = -1.0f;
    cli->overtime = 0;

    /* Resolve once (supports host names); previously inet_addr() ran on every packet */
    if (!net_addr_resolve(cli->serverIp, cli->serverPort, &resolved)) {
        snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Error: could not resolve '%s'.", cli->serverIp);
        cli->statusOk = 3;
        cli->state = NET_DISCONNECTED;
        return 0;
    }
    memcpy(cli->serverAddr, &resolved.in, sizeof(resolved.in));

    NetSocket s = net_socket_create(0, 1); /* Ephemeral port, non-blocking */
    if (s == NET_INVALID_SOCKET) {
        snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Error: Failed to create UDP socket.");
        cli->statusOk = 3; /* Red */
        cli->state = NET_DISCONNECTED;
        return 0;
    }
    cli->sock = (uintptr_t)s;

    cli->state = NET_CONNECTING;
    cli->connectTimer = 0.0f;
    cli->connectAttempts = 0;
    cli->clientSeq = 1;
    cli->serverSeqAck = 0;
    cli->clientTick = 0;
    cli->pingMs = 0.0f;
    cli->pingTimer = 0.0;
    cli->lastPacketTime = net_get_time_sec();
    cli->serverTick = 0;
    cli->snapCount = 0;
    cli->snapAckTick = 0;
    cli->snapsReceived = cli->snapsLost = cli->snapsLate = 0;
    cli->lossPct = 0.0f;
    memset(cli->inputHistory, 0, sizeof(cli->inputHistory));
    memset(cli->players, 0, sizeof(cli->players));
    memset(cli->snapCars, 0, sizeof(cli->snapCars));

    snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Connecting to %s:%d...", cli->serverIp, cli->serverPort);
    cli->statusOk = 1; /* Yellow */

    cli_send_join(cli);
    cli->connectAttempts = 1;
    return 1;
}

void net_client_disconnect(NetClient *cli)
{
    if (!cli) return;
    NetSocket s = (NetSocket)cli->sock;
    if (s != NET_INVALID_SOCKET && (cli->state == NET_CONNECTED || cli->state == NET_QUEUED)) {
        PktDisconnect pkt;
        memset(&pkt, 0, sizeof(pkt));
        cli_header(cli, &pkt.header, PKT_DISCONNECT, (uint8_t)cli->localSlot);
        pkt.reason = 0;
        cli_send(cli, &pkt, sizeof(pkt));
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

    /* Shift input history (last 3 inputs ride along so one lost packet costs nothing) */
    cli->inputHistory[2] = cli->inputHistory[1];
    cli->inputHistory[1] = cli->inputHistory[0];
    if (curInput) cli->inputHistory[0] = *curInput;

    PktClientInput pkt;
    memset(&pkt, 0, sizeof(pkt));
    cli_header(cli, &pkt.header, PKT_CLIENT_INPUT, (uint8_t)cli->localSlot);
    pkt.client_tick = ++cli->clientTick;
    pkt.server_tick_ack = cli->serverTick;
    pkt.input = cli->inputHistory[0];
    pkt.prev_inputs[0] = cli->inputHistory[1];
    pkt.prev_inputs[1] = cli->inputHistory[2];
    cli_send(cli, &pkt, sizeof(pkt));

    /* Periodic Ping (every 1 second) */
    double now = net_get_time_sec();
    if (now - cli->pingTimer > 1.0) {
        cli->pingTimer = now;
        PktPing ping;
        memset(&ping, 0, sizeof(ping));
        cli_header(cli, &ping.header, PKT_PING, (uint8_t)cli->localSlot);
        ping.client_time_ms = (uint32_t)(now * 1000.0);
        cli_send(cli, &ping, sizeof(ping));
    }
}

static void cli_handle_snapshot(NetClient *cli, const PktServerState *st)
{
    /* Drop duplicates / out-of-order packets, but accept a big jump back (server restart). */
    if (cli->snapCount > 0 && st->server_tick <= cli->serverTick && st->server_tick + 1200 > cli->serverTick) {
        cli->snapsLate++;
        return;
    }
    if (cli->snapCount > 0 && st->server_tick > cli->serverTick) {
        uint32_t gap = (st->server_tick - cli->serverTick) / 2;   /* snapshots go out every 2 ticks */
        uint32_t lost = gap > 1 ? gap - 1 : 0;
        if (lost > 60) lost = 60;
        cli->snapsLost += lost;
        cli->lossPct = cli->lossPct * 0.97f + (100.0f * (float)lost / (float)(lost + 1)) * 0.03f;
    }
    cli->snapsReceived++;
    cli->snapCount++;

    cli->serverSeqAck = st->header.seq;
    cli->serverTick = st->server_tick;
    cli->snapAckTick = st->ack_input_tick;
    cli->serverGameState = st->game_state;
    cli->serverStateTimer = st->state_timer;
    cli->serverMatchTime = st->match_time;
    cli->scoreBlue = st->score_blue;
    cli->scoreOrange = st->score_orange;
    cli->serverPlaylist = st->playlist;
    cli->overtime = st->overtime;
    cli->snapBall = st->ball;
    memcpy(cli->snapCars, st->cars, sizeof(cli->snapCars));

    cli->ballTargetPos = st->ball.pos;
    cli->ballTargetVel = st->ball.vel;
    cli->ballTargetRot = st->ball.rot;

    int activeCount = 0;
    for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
        const NetCarState *ncs = &st->cars[i];
        NetClientPlayer *p = &cli->players[i];
        p->active = ncs->active;
        if (!p->active) continue;
        activeCount++;

        p->team = ncs->team;
        p->car_model = ncs->car_model;
        p->skin = ncs->skin;
        snprintf(p->name, sizeof(p->name), "%.19s", ncs->player_name[0] ? ncs->player_name : "Player");
        p->wheelsOnGround = ncs->wheels_on_ground;
        p->supersonic = ncs->supersonic;
        p->isBot = (ncs->flags & NCF_BOT) != 0;
        p->boost = ncs->boost;
        p->boosting = (ncs->flags & NCF_BOOSTING) != 0;
        p->steerAngle = ncs->steerAngle;
        p->wheelSpin = ncs->wheelSpin;
        p->targetPos = ncs->pos;
        p->targetVel = ncs->vel;
        p->targetRot = ncs->rot;
        p->demolished = ncs->demolished;
        p->demoTimer = ncs->demo_tenths * 0.1f;
    }
    cli->playerCount = activeCount;
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
                snprintf(cli->statusMsg, sizeof(cli->statusMsg),
                         "Connection timed out. Server unreachable (or running an older version).");
                return;
            }
            cli_send_join(cli);
            cli->connectAttempts++;
            snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Connecting to %s:%d... (attempt %d/8)",
                     cli->serverIp, cli->serverPort, cli->connectAttempts);
        }
    }

    /* Searching: keep the session alive (and measure ping) */
    if (cli->state == NET_QUEUED) {
        double now = net_get_time_sec();
        cli->queueSearchSec += dt;
        if (cli->queueBotFillSec > 0.0f) cli->queueBotFillSec = cli->queueBotFillSec - dt > 0.0f ? cli->queueBotFillSec - dt : 0.0f;
        if (now - cli->pingTimer > 1.0) {
            PktPing ping;
            cli->pingTimer = now;
            memset(&ping, 0, sizeof(ping));
            cli_header(cli, &ping.header, PKT_PING, 0xFF);
            ping.client_time_ms = (uint32_t)(now * 1000.0);
            cli_send(cli, &ping, sizeof(ping));
        }
    }

    /* Process all pending incoming UDP packets */
    uint8_t buf[2048];
    NetAddr sender;
    NetAddr srv = cli_server_addr(cli);
    while (1) {
        int n = net_socket_recv(s, &sender, buf, sizeof(buf));
        if (n <= 0) break;
        if ((size_t)n < sizeof(NetHeader)) continue;
        if (!net_addr_equal(&sender, &srv)) continue;   /* ignore strays */

        NetHeader *hdr = (NetHeader *)buf;
        if (hdr->magic != SARP_NET_MAGIC) continue;
        if (hdr->version != SARP_NET_VERSION) {
            if (cli->state == NET_CONNECTING) {
                cli->state = NET_DISCONNECTED;
                cli->statusOk = 3;
                snprintf(cli->statusMsg, sizeof(cli->statusMsg),
                         "Version mismatch: server v%u, game v%d. Rebuild both.", hdr->version, SARP_NET_VERSION);
            }
            continue;
        }

        cli->lastPacketTime = net_get_time_sec();

        if (hdr->type == PKT_JOIN_ACK && (size_t)n >= sizeof(PktJoinAck)) {
            PktJoinAck *ack = (PktJoinAck *)buf;
            if (cli->state != NET_CONNECTING && cli->state != NET_QUEUED) continue;   /* duplicate ack */
            if (ack->accepted && ack->assigned_id < SARP_MAX_CLIENTS) {
                cli->state = NET_CONNECTED;
                cli->localSlot = ack->assigned_id;
                cli->localTeam = ack->assigned_team;
                cli->serverSeqAck = hdr->seq;
                cli->statusOk = 2; /* Green */
                snprintf(cli->statusMsg, sizeof(cli->statusMsg),
                         "Match found! Playing for %s", cli->localTeam == 0 ? "BLUE" : "ORANGE");
            } else if (ack->accepted) {
                continue;
            } else {
                cli->state = NET_DISCONNECTED;
                cli->statusOk = 3; /* Red */
                snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Connection rejected: %.31s",
                         ack->server_name[0] ? ack->server_name : "server full");
            }
        } else if (hdr->type == PKT_QUEUE_STATUS && (size_t)n >= sizeof(PktQueueStatus)) {
            const PktQueueStatus *q = (const PktQueueStatus *)buf;
            if (cli->state != NET_CONNECTING && cli->state != NET_QUEUED) continue;
            if (cli->state == NET_CONNECTING) {
                cli->state = NET_QUEUED;
                cli->statusOk = 1;
            }
            cli->queueInQueue = q->in_queue;
            cli->queueNeeded = q->needed;
            cli->queuePlayersOnline = q->players_online;
            cli->queueMatches = q->matches_active;
            cli->queueSearchSec = q->search_sec;
            cli->queueBotFillSec = q->bot_fill_sec;
            snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Searching... %d/%d players", q->in_queue, q->needed);
        } else if (hdr->type == PKT_PONG && (size_t)n >= sizeof(PktPong)) {
            PktPong *pong = (PktPong *)buf;
            uint32_t now_ms = (uint32_t)(net_get_time_sec() * 1000.0);
            float sample = (float)(uint32_t)(now_ms - pong->client_time_ms);
            if (sample >= 0.0f && sample < 5000.0f)
                cli->pingMs = cli->pingMs <= 0.0f ? sample : cli->pingMs * 0.75f + sample * 0.25f;
        } else if (hdr->type == PKT_SERVER_STATE && (size_t)n >= sizeof(PktServerState)) {
            const PktServerState *st = (const PktServerState *)buf;
            if ((cli->state == NET_QUEUED || cli->state == NET_CONNECTING) && st->your_slot < SARP_MAX_CLIENTS) {
                /* match started (our JOIN_ACK may have been lost): snapshots name our car */
                cli->state = NET_CONNECTED;
                cli->localSlot = st->your_slot;
                cli->localTeam = st->cars[st->your_slot].team;
                cli->statusOk = 2;
                snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Match found!");
            }
            if (cli->state == NET_CONNECTED) cli_handle_snapshot(cli, st);
        } else if (hdr->type == PKT_EVENT_GOAL && (size_t)n >= sizeof(PktEventGoal)) {
            PktEventGoal *eg = (PktEventGoal *)buf;
            cli->hasGoalEvent = 1;
            cli->goalTeam = eg->team;
            cli->goalBallSpeed = eg->ball_speed_kmh;
            cli->goalBannerTimer = 3.0f;
        } else if (hdr->type == PKT_EVENT_DEMO && (size_t)n >= sizeof(PktEventDemo)) {
            PktEventDemo *ed = (PktEventDemo *)buf;
            cli->hasDemoEvent = 1;
            cli->demoKillerId = ed->killer_id;
            cli->demoVictimId = ed->victim_id;
            const char *kName = (ed->killer_id < SARP_MAX_CLIENTS) ? cli->players[ed->killer_id].name : "Player";
            const char *vName = (ed->victim_id < SARP_MAX_CLIENTS) ? cli->players[ed->victim_id].name : "Player";
            snprintf(cli->demoKiller, sizeof(cli->demoKiller), "%s", kName);
            snprintf(cli->demoVictim, sizeof(cli->demoVictim), "%s", vName);
            cli->demoBannerTimer = 3.0f;
        } else if (hdr->type == PKT_DISCONNECT) {
            const PktDisconnect *d = (const PktDisconnect *)buf;
            cli->disconnectReason = (size_t)n >= sizeof(PktDisconnect) ? d->reason : DISC_KICKED;
            cli->state = NET_DISCONNECTED;
            cli->statusOk = cli->disconnectReason == DISC_MATCH_OVER ? 0 : 3;
            snprintf(cli->statusMsg, sizeof(cli->statusMsg), "%s",
                     cli->disconnectReason == DISC_MATCH_OVER ? "Match complete." : "Disconnected by the server.");
        }
    }

    /* Watchdog: drop if no packet received for 5 seconds while connected */
    if ((cli->state == NET_CONNECTED || cli->state == NET_QUEUED) && net_get_time_sec() - cli->lastPacketTime > 5.0) {
        cli->state = NET_DISCONNECTED;
        cli->statusOk = 3; /* Red */
        snprintf(cli->statusMsg, sizeof(cli->statusMsg), "Lost connection to server (timed out).");
    }
}

void net_client_shutdown(void)
{
    net_shutdown();
}
