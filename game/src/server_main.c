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

#ifdef _WIN32
  #include <windows.h>
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

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    int port = SARP_DEFAULT_PORT;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) port = atoi(argv[++i]);
    }

    printf("\n============================================================\n");
    printf("        SARPBC DEDICATED SERVER (120 Hz AUTH SIM)\n");
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

    Car cars[SARP_MAX_CLIENTS];
    ClientSession clients[SARP_MAX_CLIENTS];
    memset(clients, 0, sizeof(clients));

    for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
        load_car_physics(CAR_NAMES[0], &cars[i]);
    }

    Ball ball;
    ball_reset(&ball);

    Vector3 pads[PAD_COUNT];
    get_boost_pads(pads);
    float padTimer[PAD_COUNT] = { 0 };

    uint32_t server_tick = 0;
    uint8_t game_state = 1; /* 1 = PLAY */
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
                    snprintf(cs->name, sizeof(cs->name), "%.23s", req->player_name[0] ? req->player_name : "Player");

                    /* Load authentic model dimensions for selected car */
                    load_car_physics(CAR_NAMES[cs->car_model], &cars[slot]);

                    /* Kickoff position */
                    Vector3 kp; float ky;
                    kickoff_spot(slot / 2, cs->team, 4, &kp, &ky);
                    car_reset(&cars[slot], kp, ky);

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
                    cs->latest_input.jumpPressed |= cin->input.jumpPressed; /* Latch jump */
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

        /* --- 3. Step Authoritative 120 Hz Physics Ticks --- */
        double frame_dt = now - last_time;
        if (frame_dt > 0.1) frame_dt = 0.1;
        last_time = now;
        phys_acc += frame_dt;

        while (phys_acc >= h) {
            server_tick++;

            /* Step Active Cars */
            for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                if (clients[i].active) {
                    if (cars[i].demolished) {
                        cars[i].demoTimer -= h;
                        if (cars[i].demoTimer <= 0.0f) {
                            cars[i].demolished = 0;
                            cars[i].demoTimer = 0.0f;
                            float spawnX = (rand() % 2 == 0) ? -65.0f : 65.0f;
                            float spawnZ = (clients[i].team == 0) ? -(ARENA_L - 28.0f) : (ARENA_L - 28.0f);
                            float spawnYaw = (clients[i].team == 0) ? 0.0f : PI;
                            car_reset(&cars[i], V3(spawnX, 0, spawnZ), spawnYaw);
                            cars[i].boost = BOOST_START;
                        }
                    } else {
                        Input cur = clients[i].latest_input;
                        car_step(&cars[i], &cur, h);
                        clients[i].latest_input.jumpPressed = 0;
                    }
                }
            }

            /* Step Ball */
            ball_step(&ball, h);

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
                    car_car_collide(&cars[i], &cars[j]);
                    int demo = check_demolition(&cars[i], &cars[j], clients[i].team, clients[j].team);
                    if (demo == 1) {
                        cars[j].demolished = 1; cars[j].demoTimer = 3.0f;
                        printf("[DEMO] %s demolished %s!\n", clients[i].name, clients[j].name);
                        fflush(stdout);
                        PktEventDemo ed = { .header = { SARP_NET_MAGIC, SARP_NET_VERSION, PKT_EVENT_DEMO, 0xFF, server_tick, 0 },
                                            .killer_id = (uint8_t)i, .victim_id = (uint8_t)j };
                        for (int c = 0; c < SARP_MAX_CLIENTS; c++) if (clients[c].active) net_socket_send(sock, &clients[c].addr, &ed, sizeof(ed));
                    } else if (demo == 2) {
                        cars[i].demolished = 1; cars[i].demoTimer = 3.0f;
                        printf("[DEMO] %s demolished %s!\n", clients[j].name, clients[i].name);
                        fflush(stdout);
                        PktEventDemo ed = { .header = { SARP_NET_MAGIC, SARP_NET_VERSION, PKT_EVENT_DEMO, 0xFF, server_tick, 0 },
                                            .killer_id = (uint8_t)j, .victim_id = (uint8_t)i };
                        for (int c = 0; c < SARP_MAX_CLIENTS; c++) if (clients[c].active) net_socket_send(sock, &clients[c].addr, &ed, sizeof(ed));
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
                    padTimer[p] -= h;
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
                float ballSpeedKmh = Vector3Length(ball.vel) * 3.6f;
                printf("[GOAL!] %s scored! (Blue %d - %d Orange)\n",
                       scorer == 0 ? "BLUE TEAM" : "ORANGE TEAM", score_blue, score_orange);
                fflush(stdout);

                PktEventGoal eg = { .header = { SARP_NET_MAGIC, SARP_NET_VERSION, PKT_EVENT_GOAL, 0xFF, server_tick, 0 },
                                    .scorer_id = 0xFF, .team = (uint8_t)scorer, .ball_speed_kmh = ballSpeedKmh };
                for (int c = 0; c < SARP_MAX_CLIENTS; c++) if (clients[c].active) net_socket_send(sock, &clients[c].addr, &eg, sizeof(eg));

                ball_reset(&ball);
                for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                    if (clients[i].active) {
                        Vector3 kp; float ky;
                        kickoff_spot(i / 2, clients[i].team, 4, &kp, &ky);
                        car_reset(&cars[i], kp, ky);
                    }
                }
            }

            /* Match Time */
            if (match_time > 0.0f) {
                match_time -= h;
                if (match_time <= 0.0f) { match_time = 0.0f; game_state = 3; }
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
                state_pkt.match_time = match_time;
                state_pkt.score_blue = score_blue;
                state_pkt.score_orange = score_orange;

                state_pkt.ball.pos = (NetVec3){ ball.pos.x, ball.pos.y, ball.pos.z };
                state_pkt.ball.vel = (NetVec3){ ball.vel.x, ball.vel.y, ball.vel.z };
                state_pkt.ball.angVel = (NetVec3){ ball.angVel.x, ball.angVel.y, ball.angVel.z };
                state_pkt.ball.rot = (NetQuat){ ball.rot.x, ball.rot.y, ball.rot.z, ball.rot.w };

                uint8_t active_count = 0;
                for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                    NetCarState *ncs = &state_pkt.cars[i];
                    ncs->active = clients[i].active ? 1 : 0;
                    if (!ncs->active) continue;
                    active_count++;
                    ncs->player_id = (uint8_t)i;
                    ncs->team = clients[i].team;
                    ncs->car_model = clients[i].car_model;
                    ncs->skin = clients[i].skin;
                    snprintf(ncs->player_name, sizeof(ncs->player_name), "%s", clients[i].name);
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
                state_pkt.car_count = active_count;

                for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
                    if (clients[i].active) {
                        state_pkt.header.ack = clients[i].last_input_tick;
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
                    fflush(stdout);
                } else if (strcmp(cmd, "reset") == 0) {
                    ball_reset(&ball);
                    score_blue = score_orange = 0;
                    match_time = MATCH_TIME;
                    printf("[SERVER] Match reset.\n");
                    fflush(stdout);
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
