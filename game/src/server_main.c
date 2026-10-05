#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdint.h>

#include "net_protocol.h"
#include "net_socket.h"
#define SARPBC_MODEL_IMPLEMENTATION
#include "physics_sim.h"
#include "net_sync.h"

#ifdef _WIN32
  #include <windows.h>
  #include <mmsystem.h>
  #include <conio.h>
  static inline double server_time_sec(void)
  {
      static LARGE_INTEGER freq;
      static int init = 0;
      if (!init) { QueryPerformanceFrequency(&freq); init = 1; }
      LARGE_INTEGER now;
      QueryPerformanceCounter(&now);
      return (double)now.QuadPart / (double)freq.QuadPart;
  }
#else
  #include <sys/time.h>
  #include <unistd.h>
  static inline double server_time_sec(void)
  {
      struct timeval tv;
      gettimeofday(&tv, NULL);
      return (double)tv.tv_sec + (double)tv.tv_usec * 1e-6;
  }
#endif

/* ------------------------------------------------------------------------ */
/* Input queue                                                                */
/*                                                                            */
/* Clients simulate one physics tick per input and number them. The server    */
/* consumes exactly one numbered input per tick (with a small jitter cushion) */
/* so its simulation matches the client's prediction tick-for-tick, and it    */
/* reports the last consumed number back so the client can rewind/replay.     */
/* ------------------------------------------------------------------------ */
#define INPUT_RING       128
#define INPUT_CUSHION    2      /* ticks buffered before consuming (~17 ms)   */
#define INPUT_MAX_DEPTH  16     /* hard cap: skip ahead beyond this (~133 ms) */
#define DRAIN_WINDOW     120    /* ticks between buffer-depth trims (1 s)     */

#define GS_COUNTDOWN 0
#define GS_PLAY      1
#define GS_GOAL      2
#define GS_OVER      3

#define COUNTDOWN_TIME  3.0f
#define GOAL_TIME       3.0f
#define OVER_TIME       8.0f

typedef struct ClientSession {
    int      active;
    NetAddr  addr;
    uint32_t last_seen_ms;
    uint8_t  player_id;
    uint8_t  team;
    uint8_t  car_model;
    uint8_t  skin;
    char     name[24];

    NetInput ring[INPUT_RING];
    uint32_t ring_tick[INPUT_RING];
    uint32_t newest_tick;     /* highest client tick received          */
    uint32_t next_tick;       /* next client tick to consume (0 = idle) */
    uint32_t last_processed;  /* acknowledged back to the client        */
    int      min_depth;       /* smallest buffer depth in this window   */
    int      window_ticks;
    uint32_t starved, dropped;
    Input    cur;             /* input applied on the latest tick       */
} ClientSession;

static void input_store(ClientSession *cs, uint32_t tick, const NetInput *ni)
{
    uint32_t slot;
    if (tick == 0) return;
    if (cs->next_tick && tick < cs->next_tick) return;            /* too late, already simulated */
    if (cs->newest_tick > INPUT_RING && tick + INPUT_RING / 2 < cs->newest_tick) return;
    slot = tick % INPUT_RING;
    cs->ring[slot] = *ni;
    cs->ring_tick[slot] = tick;
    if (tick > cs->newest_tick) cs->newest_tick = tick;
}

static int input_has(const ClientSession *cs, uint32_t tick)
{
    return cs->ring_tick[tick % INPUT_RING] == tick;
}

static void input_consume(ClientSession *cs)
{
    int depth;
    if (cs->newest_tick == 0) { memset(&cs->cur, 0, sizeof(cs->cur)); return; }
    if (cs->next_tick == 0)
        cs->next_tick = cs->newest_tick > INPUT_CUSHION ? cs->newest_tick - INPUT_CUSHION + 1 : 1;

    /* way behind (client burst after a hitch): jump forward, keep any jump edge */
    if (cs->newest_tick >= cs->next_tick + INPUT_MAX_DEPTH) {
        uint32_t to = cs->newest_tick - INPUT_CUSHION + 1, t;
        int jp = 0;
        for (t = cs->next_tick; t < to; t++) if (input_has(cs, t) && cs->ring[t % INPUT_RING].jumpPressed) jp = 1;
        cs->dropped += to - cs->next_tick;
        cs->next_tick = to;
        if (jp && input_has(cs, to)) cs->ring[to % INPUT_RING].jumpPressed = 1;
    }

    depth = (int)cs->newest_tick - (int)cs->next_tick + 1;
    if (depth < cs->min_depth) cs->min_depth = depth;
    if (++cs->window_ticks >= DRAIN_WINDOW) {
        /* buffer has been needlessly deep for a whole second: shave one tick of latency */
        if (cs->min_depth > INPUT_CUSHION + 1 && input_has(cs, cs->next_tick) && input_has(cs, cs->next_tick + 1)) {
            if (cs->ring[cs->next_tick % INPUT_RING].jumpPressed) cs->ring[(cs->next_tick + 1) % INPUT_RING].jumpPressed = 1;
            cs->next_tick++;
            cs->dropped++;
        }
        cs->min_depth = 1 << 30;
        cs->window_ticks = 0;
    }

    if (input_has(cs, cs->next_tick)) {
        cs->cur = ns_input_from_net(&cs->ring[cs->next_tick % INPUT_RING]);
        cs->last_processed = cs->next_tick++;
    } else if (cs->next_tick <= cs->newest_tick) {
        /* this tick's input was lost (all redundant copies): hold the previous one */
        cs->cur.jumpPressed = 0;
        cs->last_processed = cs->next_tick++;
        cs->dropped++;
    } else {
        /* starved: client is running behind. Hold input, don't advance, which
         * also grows the cushion so the next hiccup is absorbed. */
        cs->cur.jumpPressed = 0;
        cs->starved++;
    }
}

/* ------------------------------------------------------------------------ */
/* Match helpers                                                              */
/* ------------------------------------------------------------------------ */
static int team_spot(const ClientSession *clients, int slot)
{
    int i, n = 0;
    for (i = 0; i < slot; i++) if (clients[i].active && clients[i].team == clients[slot].team) n++;
    return n;
}

static void spawn_at_kickoff(const ClientSession *clients, Car *cars, int i)
{
    Vector3 kp; float ky;
    kickoff_spot(team_spot(clients, i), clients[i].team, 4, &kp, &ky);
    car_reset(&cars[i], kp, ky);
}

static void full_kickoff(const ClientSession *clients, Car *cars, Ball *ball, float *padTimer)
{
    int i;
    ball_reset(ball);
    for (i = 0; i < SARP_MAX_CLIENTS; i++) if (clients[i].active) spawn_at_kickoff(clients, cars, i);
    for (i = 0; i < PAD_COUNT; i++) padTimer[i] = 0.0f;
}

static int active_count(const ClientSession *clients)
{
    int i, n = 0;
    for (i = 0; i < SARP_MAX_CLIENTS; i++) n += clients[i].active != 0;
    return n;
}

static int load_server_car(int model, Car *car)
{
    if (load_car_physics(CAR_NAMES[model], car)) return 1;
    printf("[WARNING] Could not load car '%s' (.sarm not found): using fallback hitbox. "
           "Clients will disagree with the server! Run the server from the repo root.\n", CAR_NAMES[model]);
    return 0;
}

static void broadcast(NetSocket sock, const ClientSession *clients, const void *pkt, int len)
{
    int c;
    for (c = 0; c < SARP_MAX_CLIENTS; c++) if (clients[c].active) net_socket_send(sock, &clients[c].addr, pkt, len);
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    int port = SARP_DEFAULT_PORT;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) port = atoi(argv[++i]);
    }

    printf("\n============================================================\n");
    printf("        SARPBC DEDICATED SERVER (120 Hz AUTH SIM, proto v%d)\n", SARP_NET_VERSION);
    printf("============================================================\n");
    printf("[SERVER] Initializing network subsystem...\n");
    if (!net_init()) {
        fprintf(stderr, "[ERROR] Failed to initialize network!\n");
        return 1;
    }
#ifdef _WIN32
    /* Default Windows timer granularity is ~15.6 ms, which made Sleep(1) bunch
     * ticks and snapshots into bursts. Ask for 1 ms scheduling. */
    timeBeginPeriod(1);
#endif

    NetSocket sock = net_socket_create(port, 1);
    if (sock == NET_INVALID_SOCKET) {
        fprintf(stderr, "[ERROR] Could not bind UDP port %d!\n", port);
        net_shutdown();
        return 1;
    }
    printf("[SERVER] Listening on UDP port %d (Non-blocking)\n", port);

    /* Load Arena Collision */
    const char *col_paths[] = {
        "export_c/arena/arena_col.bin",
        "../export_c/arena/arena_col.bin",
        "../../export_c/arena/arena_col.bin"
    };
    int loaded = 0;
    for (size_t i = 0; i < sizeof(col_paths) / sizeof(col_paths[0]); i++) {
        if (arena_mesh_load(col_paths[i])) { loaded = 1; break; }
    }
    if (!loaded) {
        printf("[WARNING] Collision mesh arena_col.bin not found! Using fallback bounds.\n");
    }

    static Car cars[SARP_MAX_CLIENTS];
    static ClientSession clients[SARP_MAX_CLIENTS];
    memset(clients, 0, sizeof(clients));
    for (int i = 0; i < SARP_MAX_CLIENTS; i++) load_car_physics(CAR_NAMES[0], &cars[i]);

    Ball ball;
    ball_reset(&ball);

    Vector3 pads[PAD_COUNT];
    get_boost_pads(pads);
    float padTimer[PAD_COUNT] = { 0 };

    uint32_t server_tick = 0, pong_seq = 0;
    uint8_t game_state = GS_COUNTDOWN;
    float state_timer = COUNTDOWN_TIME;
    float match_time = MATCH_TIME;
    uint8_t score_blue = 0, score_orange = 0;

    printf("[SERVER] Match ready! Waiting for players to join...\n");
    printf("[SERVER] Available console commands: status, reset, quit\n\n");
    fflush(stdout);

    double last_time = server_time_sec();
    double phys_acc = 0.0;
    const float h = 1.0f / PHYS_HZ; /* 120 Hz tick */
    int running = 1;

    char recv_buf[2048];
    NetAddr sender;

    while (running) {
        double now = server_time_sec();
        uint32_t now_ms = (uint32_t)(now * 1000.0);

        /* --- 1. Receive & Process Incoming UDP Packets --- */
        int bytes = 0;
        while ((bytes = net_socket_recv(sock, &sender, recv_buf, sizeof(recv_buf))) > 0) {
            if (bytes < (int)sizeof(NetHeader)) continue;
            NetHeader *hdr = (NetHeader *)recv_buf;
            if (hdr->magic != SARP_NET_MAGIC) continue;
            if (hdr->version != SARP_NET_VERSION) {
                if (hdr->type == PKT_JOIN_REQ) {   /* tell old/new clients why they can't join */
                    PktJoinAck nak;
                    memset(&nak, 0, sizeof(nak));
                    nak.header.magic = SARP_NET_MAGIC;
                    nak.header.version = SARP_NET_VERSION;
                    nak.header.type = PKT_JOIN_ACK;
                    nak.header.player_id = 0xFF;
                    nak.accepted = 0;
                    snprintf(nak.server_name, sizeof(nak.server_name), "VERSION MISMATCH");
                    net_socket_send(sock, &sender, &nak, sizeof(nak));
                }
                continue;
            }

            if (hdr->type == PKT_PING && bytes >= (int)sizeof(PktPing)) {
                PktPing *ping = (PktPing *)recv_buf;
                PktPong pong;
                memset(&pong, 0, sizeof(pong));
                pong.header.magic = SARP_NET_MAGIC;
                pong.header.version = SARP_NET_VERSION;
                pong.header.type = PKT_PONG;
                pong.header.player_id = 0xFF;
                pong.header.seq = ++pong_seq;      /* (used to bump server_tick and break snapshot cadence) */
                pong.header.ack = hdr->seq;
                pong.client_time_ms = ping->client_time_ms;
                pong.server_time_ms = now_ms;
                net_socket_send(sock, &sender, &pong, sizeof(pong));
                for (int i = 0; i < SARP_MAX_CLIENTS; i++)
                    if (clients[i].active && net_addr_equal(&clients[i].addr, &sender)) clients[i].last_seen_ms = now_ms;
            } else if (hdr->type == PKT_JOIN_REQ && bytes >= (int)sizeof(PktJoinReq)) {
                PktJoinReq *req = (PktJoinReq *)recv_buf;
                int slot = -1, existing = 0;
                for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                    if (clients[i].active && net_addr_equal(&clients[i].addr, &sender)) {
                        slot = i; existing = 1; break;
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
                    if (!existing) {
                        /* Brand-new session. (A re-sent join from a client whose ack got lost
                         * must NOT reset its car mid-game, hence the 'existing' check.) */
                        int wasEmpty = active_count(clients) == 0;
                        memset(cs, 0, sizeof(*cs));
                        cs->active = 1;
                        cs->addr = sender;
                        cs->player_id = (uint8_t)slot;
                        cs->car_model = req->car_model < CAR_COUNT ? req->car_model : 0;
                        cs->skin = req->skin;
                        if (req->pref_team <= 1) {
                            cs->team = req->pref_team;
                        } else {   /* auto-balance */
                            int nb = 0, no = 0;
                            for (int i = 0; i < SARP_MAX_CLIENTS; i++)
                                if (i != slot && clients[i].active) { if (clients[i].team == 0) nb++; else no++; }
                            cs->team = nb <= no ? 0 : 1;
                        }
                        snprintf(cs->name, sizeof(cs->name), "%.23s", req->player_name[0] ? req->player_name : "Player");
                        cs->min_depth = 1 << 30;

                        load_server_car(cs->car_model, &cars[slot]);
                        if (wasEmpty) {
                            /* first player in: fresh kickoff */
                            full_kickoff(clients, cars, &ball, padTimer);
                            game_state = GS_COUNTDOWN; state_timer = COUNTDOWN_TIME;
                        } else {
                            spawn_at_kickoff(clients, cars, slot);
                        }

                        char addr_str[64];
                        net_addr_to_string(&sender, addr_str, sizeof(addr_str));
                        printf("[JOIN] Slot %d (%s) connected from %s (Team %s, Car: %s)\n",
                               slot, cs->name, addr_str, cs->team == 0 ? "BLUE" : "ORANGE", CAR_NAMES[cs->car_model]);
                    }
                    cs->last_seen_ms = now_ms;
                    ack.accepted = 1;
                    ack.assigned_id = (uint8_t)slot;
                    ack.assigned_team = cs->team;
                    ack.car_model = cs->car_model;
                    ack.skin = cs->skin;
                    ack.tick_rate = (uint16_t)PHYS_HZ;
                    snprintf(ack.server_name, sizeof(ack.server_name), "SARPBC Official #1");
                } else {
                    ack.accepted = 0;
                    snprintf(ack.server_name, sizeof(ack.server_name), "SERVER FULL");
                }
                net_socket_send(sock, &sender, &ack, sizeof(ack));
            } else if (hdr->type == PKT_CLIENT_INPUT && bytes >= (int)sizeof(PktClientInput)) {
                PktClientInput *cin = (PktClientInput *)recv_buf;
                int pid = hdr->player_id;
                if (pid >= 0 && pid < SARP_MAX_CLIENTS && clients[pid].active &&
                    net_addr_equal(&clients[pid].addr, &sender)) {
                    ClientSession *cs = &clients[pid];
                    uint32_t t = cin->client_tick;
                    cs->last_seen_ms = now_ms;
                    /* redundant copies first, so the newest one wins on overlap */
                    if (t > 2) input_store(cs, t - 2, &cin->prev_inputs[1]);
                    if (t > 1) input_store(cs, t - 1, &cin->prev_inputs[0]);
                    input_store(cs, t, &cin->input);
                }
            } else if (hdr->type == PKT_DISCONNECT) {
                int pid = hdr->player_id;
                if (pid >= 0 && pid < SARP_MAX_CLIENTS && clients[pid].active &&
                    net_addr_equal(&clients[pid].addr, &sender)) {
                    printf("[LEAVE] Slot %d (%s) disconnected\n", pid, clients[pid].name);
                    clients[pid].active = 0;
                }
            }
        }

        /* --- 2. Check Client Timeouts (5 seconds) --- */
        for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
            if (clients[i].active && now_ms - clients[i].last_seen_ms > 5000) {
                printf("[TIMEOUT] Slot %d (%s) timed out\n", i, clients[i].name);
                clients[i].active = 0;
            }
        }

        /* --- 3. Step Authoritative 120 Hz Physics Ticks --- */
        double frame_dt = now - last_time;
        if (frame_dt > 0.1) frame_dt = 0.1;
        last_time = now;
        phys_acc += frame_dt;

        while (phys_acc >= h) {
            server_tick++;

            /* Pull exactly one queued input per client */
            for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                if (!clients[i].active) continue;
                input_consume(&clients[i]);
                if (game_state == GS_COUNTDOWN) memset(&clients[i].cur, 0, sizeof(Input));
            }

            /* Step Active Cars */
            for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                if (!clients[i].active) continue;
                if (cars[i].demolished) {
                    cars[i].demoTimer -= h;
                    if (cars[i].demoTimer <= 0.0f) {
                        float spawnX = (rand() % 2 == 0) ? -65.0f : 65.0f;
                        float spawnZ = (clients[i].team == 0) ? -(ARENA_L - 28.0f) : (ARENA_L - 28.0f);
                        float spawnYaw = (clients[i].team == 0) ? 0.0f : PI;
                        car_reset(&cars[i], V3(spawnX, 0, spawnZ), spawnYaw);
                    }
                } else {
                    car_step(&cars[i], &clients[i].cur, h);
                }
            }

            /* Step Ball + Car-Ball Collisions (same order as the client) */
            ball_step(&ball, h);
            for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                if (clients[i].active && !cars[i].demolished) car_ball_collide(&cars[i], &ball);
            }

            /* Car-Car Collisions & Demolitions */
            for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                if (!clients[i].active || cars[i].demolished) continue;
                for (int j = i + 1; j < SARP_MAX_CLIENTS; j++) {
                    if (!clients[j].active || cars[j].demolished) continue;
                    int demo = check_demolition(&cars[i], &cars[j], clients[i].team, clients[j].team);
                    if (demo) {
                        int killer = demo == 2 ? j : i, victim = demo == 2 ? i : j;
                        if (demo == 3) {
                            cars[i].demolished = 1; cars[i].demoTimer = 3.0f;
                            cars[j].demolished = 1; cars[j].demoTimer = 3.0f;
                            printf("[DEMO] Mutual demolition between %s and %s!\n", clients[i].name, clients[j].name);
                        } else {
                            cars[victim].demolished = 1; cars[victim].demoTimer = 3.0f;
                            printf("[DEMO] %s demolished %s!\n", clients[killer].name, clients[victim].name);
                        }
                        PktEventDemo ed;
                        memset(&ed, 0, sizeof(ed));
                        ed.header.magic = SARP_NET_MAGIC; ed.header.version = SARP_NET_VERSION;
                        ed.header.type = PKT_EVENT_DEMO; ed.header.player_id = 0xFF; ed.header.seq = server_tick;
                        ed.killer_id = (uint8_t)killer; ed.victim_id = (uint8_t)victim;
                        broadcast(sock, clients, &ed, sizeof(ed));
                        if (demo == 3) {
                            ed.killer_id = (uint8_t)victim; ed.victim_id = (uint8_t)killer;
                            broadcast(sock, clients, &ed, sizeof(ed));
                        }
                        continue;
                    }
                    car_car_collide(&cars[i], &cars[j]);
                }
            }

            /* Boost Pads */
            for (int p = 0; p < PAD_COUNT; p++) {
                if (padTimer[p] > 0.0f) {
                    padTimer[p] -= h;
                } else {
                    for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                        if (!clients[i].active || cars[i].demolished) continue;
                        float dx = cars[i].pos.x - pads[p].x, dz = cars[i].pos.z - pads[p].z;
                        if (dx*dx + dz*dz < PAD_RADIUS * PAD_RADIUS && cars[i].pos.y < 4.0f) {
                            cars[i].boost = BOOST_MAX;
                            padTimer[p] = PAD_RESPAWN;
                            break;
                        }
                    }
                }
            }

            /* Match state machine */
            if (game_state == GS_PLAY) {
                if (fabsf(ball.pos.z) > ARENA_L + BALL_R) {
                    int scorer = ball.pos.z > 0 ? 0 : 1; /* 0 = Blue scored on Orange net, 1 = Orange scored */
                    if (scorer == 0) score_blue++; else score_orange++;
                    float ballSpeedKmh = Vector3Length(ball.vel) * 3.6f;
                    printf("[GOAL!] %s scored! (Blue %d - %d Orange)\n",
                           scorer == 0 ? "BLUE TEAM" : "ORANGE TEAM", score_blue, score_orange);

                    PktEventGoal eg;
                    memset(&eg, 0, sizeof(eg));
                    eg.header.magic = SARP_NET_MAGIC; eg.header.version = SARP_NET_VERSION;
                    eg.header.type = PKT_EVENT_GOAL; eg.header.player_id = 0xFF; eg.header.seq = server_tick;
                    eg.scorer_id = 0xFF; eg.team = (uint8_t)scorer; eg.ball_speed_kmh = ballSpeedKmh;
                    broadcast(sock, clients, &eg, sizeof(eg));

                    game_state = GS_GOAL; state_timer = GOAL_TIME;
                }
                if (game_state == GS_PLAY && match_time > 0.0f) {
                    match_time -= h;
                    if (match_time <= 0.0f) {
                        match_time = 0.0f;
                        game_state = GS_OVER; state_timer = OVER_TIME;
                        printf("[SERVER] Match over: Blue %d - %d Orange\n", score_blue, score_orange);
                    }
                }
            } else {
                state_timer -= h;
                if (state_timer <= 0.0f) {
                    if (game_state == GS_COUNTDOWN) {
                        game_state = GS_PLAY; state_timer = 0.0f;
                    } else if (game_state == GS_GOAL) {
                        full_kickoff(clients, cars, &ball, padTimer);
                        if (match_time <= 0.0f) { game_state = GS_OVER; state_timer = OVER_TIME; }
                        else { game_state = GS_COUNTDOWN; state_timer = COUNTDOWN_TIME; }
                    } else if (game_state == GS_OVER) {
                        score_blue = score_orange = 0;
                        match_time = MATCH_TIME;
                        full_kickoff(clients, cars, &ball, padTimer);
                        game_state = GS_COUNTDOWN; state_timer = COUNTDOWN_TIME;
                        printf("[SERVER] New match starting.\n");
                    }
                }
            }

            /* --- 4. Broadcast Authoritative Snapshot (at 60 Hz = every 2 ticks) --- */
            if (server_tick % 2 == 0) {
                PktServerState state_pkt;
                memset(&state_pkt, 0, sizeof(state_pkt));
                state_pkt.header.magic = SARP_NET_MAGIC;
                state_pkt.header.version = SARP_NET_VERSION;
                state_pkt.header.type = PKT_SERVER_STATE;
                state_pkt.header.player_id = 0xFF;
                state_pkt.header.seq = server_tick;

                state_pkt.server_tick = server_tick;
                state_pkt.game_state = game_state;
                state_pkt.state_timer = state_timer;
                state_pkt.countdown_sec = (uint8_t)ceilf(fmaxf(0.0f, state_timer));
                state_pkt.match_time = match_time;
                state_pkt.score_blue = score_blue;
                state_pkt.score_orange = score_orange;
                ns_ball_to_net(&ball, &state_pkt.ball);

                uint8_t n_active = 0;
                for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                    NetCarState *ncs = &state_pkt.cars[i];
                    ncs->active = clients[i].active ? 1 : 0;
                    if (!ncs->active) continue;
                    n_active++;
                    ncs->player_id = (uint8_t)i;
                    ncs->team = clients[i].team;
                    ncs->car_model = clients[i].car_model;
                    ncs->skin = clients[i].skin;
                    snprintf(ncs->player_name, sizeof(ncs->player_name), "%s", clients[i].name);
                    ns_car_to_net(&cars[i], &clients[i].cur, ncs);
                }
                state_pkt.car_count = n_active;

                for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                    if (clients[i].active) {
                        state_pkt.header.ack = clients[i].last_processed;
                        state_pkt.ack_input_tick = clients[i].last_processed;
                        net_socket_send(sock, &clients[i].addr, &state_pkt, sizeof(state_pkt));
                    }
                }
            }

            phys_acc -= h;
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
                    printf("\n--- SERVER STATUS (Tick %u | Score: Blue %d - %d Orange | State %d) ---\n",
                           server_tick, score_blue, score_orange, game_state);
                    int count = 0;
                    for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                        if (clients[i].active) {
                            char astr[64];
                            net_addr_to_string(&clients[i].addr, astr, sizeof(astr));
                            printf("  Slot %d: [%s] '%s' | Car: %s | %s | inbuf %d | starved %u | dropped %u\n",
                                   i, clients[i].team == 0 ? "BLUE" : "ORANGE", clients[i].name,
                                   CAR_NAMES[clients[i].car_model], astr,
                                   (int)clients[i].newest_tick - (int)clients[i].next_tick + 1,
                                   clients[i].starved, clients[i].dropped);
                            count++;
                        }
                    }
                    if (count == 0) printf("  (No players connected)\n");
                    printf("-----------------------------------------------------------\n\n");
                } else if (strcmp(cmd, "reset") == 0) {
                    score_blue = score_orange = 0;
                    match_time = MATCH_TIME;
                    full_kickoff(clients, cars, &ball, padTimer);
                    game_state = GS_COUNTDOWN; state_timer = COUNTDOWN_TIME;
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
#ifdef _WIN32
    timeEndPeriod(1);
#endif
    if (g_tris) free(g_tris);
    if (g_cellStart) free(g_cellStart);
    if (g_cellItems) free(g_cellItems);
    if (g_stamp) free(g_stamp);
    printf("[SERVER] Goodbye!\n");
    return 0;
}
