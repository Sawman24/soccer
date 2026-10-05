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
#include "bot_ai.h"

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
  #include <sys/select.h>
  #include <unistd.h>
  static inline double server_time_sec(void)
  {
      struct timeval tv;
      gettimeofday(&tv, NULL);
      return (double)tv.tv_sec + (double)tv.tv_usec * 1e-6;
  }
#endif

/* ------------------------------------------------------------------------ */
/* Quick-match server                                                         */
/*                                                                            */
/*  Players connect, pick a playlist (1v1 / 2v2 / 3v3) and wait in that       */
/*  playlist's queue. As soon as enough players are searching a match is      */
/*  created. Searching players are also dropped into running matches that     */
/*  have a bot in them (a bot that filled in or replaced a leaver). If nobody */
/*  else shows up for --botfill seconds (default 180), the match starts with  */
/*  bots filling the empty seats. Players that leave a match are replaced by  */
/*  a bot. Each match is simulated at 120 Hz exactly like the client.         */
/* ------------------------------------------------------------------------ */
#define MAX_SESSIONS     64
#define MAX_MATCHES      16
#define MAX_SLOTS        6        /* 3v3 */

/* Input queue (see input_consume) */
#define INPUT_RING       128
#define INPUT_CUSHION    2      /* ticks buffered before consuming (~17 ms)   */
#define INPUT_MAX_DEPTH  16     /* hard cap: skip ahead beyond this (~133 ms) */
#define DRAIN_WINDOW     120    /* ticks between buffer-depth trims (1 s)     */

#define GS_COUNTDOWN 0
#define GS_PLAY      1
#define GS_GOAL      2
#define GS_OVER      3

#define START_COUNTDOWN 5.0f    /* first kickoff of a match */
#define COUNTDOWN_TIME  3.0f
#define GOAL_TIME       3.0f
#define OVER_TIME       12.0f   /* post-match scoreboard before everyone is sent back */
#define TIMEOUT_MS      5000
#define BACKFILL_MIN_TIME 45.0f /* don't drop searching players into nearly finished matches */

enum { SS_FREE = 0, SS_QUEUED, SS_MATCH };
enum { SLOT_EMPTY = 0, SLOT_HUMAN, SLOT_BOT };

typedef struct ClientSession {
    int      state;           /* SS_* */
    NetAddr  addr;
    uint32_t last_seen_ms;
    char     name[24];
    uint8_t  car_model, skin;
    int      playlist;        /* players per team */
    double   queue_start, last_status;
    int      match, slot;

    /* numbered input queue */
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

typedef struct Match {
    int      active;
    int      id;
    int      playlist;        /* players per team */
    int      nslots;          /* 2 * playlist; slots [0, playlist) are blue */
    uint32_t tick;
    int      kind[MAX_SLOTS];
    int      session[MAX_SLOTS];
    char     name[MAX_SLOTS][24];
    int      team[MAX_SLOTS];
    uint8_t  car_model[MAX_SLOTS], skin[MAX_SLOTS];
    Car      cars[MAX_SLOTS];
    Bot      bots[MAX_SLOTS];
    Input    botIn[MAX_SLOTS];
    Ball     ball;
    float    padTimer[PAD_COUNT];
    uint8_t  state;
    float    state_timer, match_time;
    int      overtime;
    uint8_t  score_blue, score_orange;
    BotWorld bw;
} Match;

static ClientSession g_sess[MAX_SESSIONS];
static Match         g_match[MAX_MATCHES];
static NetSocket     g_sock;
static Vector3       g_pads[PAD_COUNT];
static float         g_botFill = 180.0f;
static int           g_botSkill = 1;
static float         g_matchLen = MATCH_TIME;
static int           g_matchSerial = 0;

static const char *BOT_NAMES[] = {
    "Armstrong", "Bandit", "Beast", "Boomer", "Buzz", "Casper", "Caveman", "C-Block", "Centice",
    "Chipper", "Cougar", "Dude", "Foamer", "Fury", "Gerwin", "Goose", "Heater", "Hollywood",
    "Hound", "Iceman", "Imp", "Jester", "Junker", "Khan", "Marley", "Maverick", "Merlin",
    "Middy", "Mountain", "Myrtle", "Outlaw", "Poncho", "Rainmaker", "Raja", "Rex", "Roundhouse",
    "Sabretooth", "Saltie", "Samara", "Scout", "Shepard", "Slider", "Squall", "Sticks", "Stinger",
    "Storm", "Sultan", "Sundown", "Swabbie", "Tex", "Tusk", "Viper", "Wolfman", "Yuri"
};
#define BOT_NAME_COUNT ((int)(sizeof(BOT_NAMES) / sizeof(BOT_NAMES[0])))

static const char *PLAYLIST_NAMES[] = { "?", "Duel 1v1", "Doubles 2v2", "Standard 3v3" };

/* ------------------------------------------------------------------------ */
/* Input queue                                                                */
/*                                                                            */
/* Clients simulate one physics tick per input and number them. The server    */
/* consumes exactly one numbered input per tick (with a small jitter cushion) */
/* so its simulation matches the client's prediction tick-for-tick, and it    */
/* reports the last consumed number back so the client can rewind/replay.     */
/* ------------------------------------------------------------------------ */
static void input_reset(ClientSession *cs)
{
    memset(cs->ring_tick, 0, sizeof(cs->ring_tick));
    cs->newest_tick = cs->next_tick = cs->last_processed = 0;
    cs->min_depth = 1 << 30;
    cs->window_ticks = 0;
    cs->starved = cs->dropped = 0;
    memset(&cs->cur, 0, sizeof(cs->cur));
}

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
/* Packets                                                                    */
/* ------------------------------------------------------------------------ */
static void hdr_init(NetHeader *h, uint8_t type, uint32_t seq)
{
    h->magic = SARP_NET_MAGIC;
    h->version = SARP_NET_VERSION;
    h->type = type;
    h->player_id = 0xFF;
    h->seq = seq;
    h->ack = 0;
}

static void send_join_ack(const ClientSession *cs, int accepted, const char *msg)
{
    PktJoinAck ack;
    memset(&ack, 0, sizeof(ack));
    hdr_init(&ack.header, PKT_JOIN_ACK, 0);
    ack.accepted = (uint8_t)accepted;
    if (accepted && cs->state == SS_MATCH) {
        const Match *m = &g_match[cs->match];
        ack.assigned_id = (uint8_t)cs->slot;
        ack.assigned_team = (uint8_t)m->team[cs->slot];
        ack.car_model = cs->car_model;
        ack.skin = cs->skin;
        ack.tick_rate = (uint16_t)PHYS_HZ;
    }
    snprintf(ack.server_name, sizeof(ack.server_name), "%s", msg);
    net_socket_send(g_sock, &cs->addr, &ack, sizeof(ack));
}

static void send_disconnect(const NetAddr *to, uint8_t reason)
{
    PktDisconnect d;
    memset(&d, 0, sizeof(d));
    hdr_init(&d.header, PKT_DISCONNECT, 0);
    d.reason = reason;
    net_socket_send(g_sock, to, &d, sizeof(d));
}

static int queue_count(int playlist)
{
    int i, n = 0;
    for (i = 0; i < MAX_SESSIONS; i++) n += g_sess[i].state == SS_QUEUED && g_sess[i].playlist == playlist;
    return n;
}

static int players_online(void)
{
    int i, n = 0;
    for (i = 0; i < MAX_SESSIONS; i++) n += g_sess[i].state != SS_FREE;
    return n;
}

static int matches_active(void)
{
    int i, n = 0;
    for (i = 0; i < MAX_MATCHES; i++) n += g_match[i].active;
    return n;
}

static double oldest_wait(int playlist, double now)
{
    int i;
    double w = 0.0;
    for (i = 0; i < MAX_SESSIONS; i++)
        if (g_sess[i].state == SS_QUEUED && g_sess[i].playlist == playlist && now - g_sess[i].queue_start > w)
            w = now - g_sess[i].queue_start;
    return w;
}

static void send_queue_status(ClientSession *cs, double now)
{
    PktQueueStatus q;
    memset(&q, 0, sizeof(q));
    hdr_init(&q.header, PKT_QUEUE_STATUS, 0);
    q.playlist = (uint8_t)cs->playlist;
    q.in_queue = (uint8_t)queue_count(cs->playlist);
    q.needed = (uint8_t)(2 * cs->playlist);
    q.matches_active = (uint8_t)matches_active();
    q.players_online = (uint16_t)players_online();
    q.search_sec = (float)(now - cs->queue_start);
    q.bot_fill_sec = g_botFill < 0.0f ? -1.0f : fmaxf(0.0f, g_botFill - (float)oldest_wait(cs->playlist, now));
    net_socket_send(g_sock, &cs->addr, &q, sizeof(q));
    cs->last_status = now;
}

static void broadcast_match(const Match *m, const void *pkt, int len)
{
    int s;
    for (s = 0; s < m->nslots; s++)
        if (m->kind[s] == SLOT_HUMAN) net_socket_send(g_sock, &g_sess[m->session[s]].addr, pkt, len);
}

/* ------------------------------------------------------------------------ */
/* Matches                                                                    */
/* ------------------------------------------------------------------------ */
static int load_server_car(int model, Car *car)
{
    if (load_car_physics(CAR_NAMES[model], car)) return 1;
    printf("[WARNING] Could not load car '%s' (.sarm not found): using fallback hitbox. "
           "Clients will disagree with the server! Run the server from the repo root.\n", CAR_NAMES[model]);
    return 0;
}

static void spawn_at_kickoff(Match *m, int s)
{
    Vector3 kp; float ky;
    kickoff_spot(s % m->playlist, m->team[s], m->playlist, &kp, &ky);
    car_reset(&m->cars[s], kp, ky);
}

static void spawn_at_respawn(Match *m, int s)
{
    float spawnX = (rand() % 2 == 0) ? -65.0f : 65.0f;
    float spawnZ = (m->team[s] == 0) ? -(ARENA_L - 28.0f) : (ARENA_L - 28.0f);
    car_reset(&m->cars[s], V3(spawnX, 0, spawnZ), m->team[s] == 0 ? 0.0f : PI);
}

static void match_kickoff(Match *m, float countdown)
{
    int s;
    ball_reset(&m->ball);
    for (s = 0; s < m->nslots; s++) if (m->kind[s] != SLOT_EMPTY) spawn_at_kickoff(m, s);
    for (s = 0; s < PAD_COUNT; s++) m->padTimer[s] = 0.0f;
    memset(m->bots, 0, sizeof(m->bots));
    memset(m->botIn, 0, sizeof(m->botIn));
    memset(&m->bw, 0, sizeof(m->bw));
    m->state = GS_COUNTDOWN;
    m->state_timer = countdown;
}

static int match_humans(const Match *m)
{
    int s, n = 0;
    for (s = 0; s < m->nslots; s++) n += m->kind[s] == SLOT_HUMAN;
    return n;
}

static int match_bots(const Match *m)
{
    int s, n = 0;
    for (s = 0; s < m->nslots; s++) n += m->kind[s] == SLOT_BOT;
    return n;
}

static void match_set_bot(Match *m, int s)
{
    int tries, k, ok;
    char nm[24];
    /* a name nobody in this match uses */
    for (tries = 0; tries < 64; tries++) {
        snprintf(nm, sizeof(nm), "%s", BOT_NAMES[rand() % BOT_NAME_COUNT]);
        ok = 1;
        for (k = 0; k < m->nslots; k++) if (k != s && m->kind[k] != SLOT_EMPTY && !strcmp(m->name[k], nm)) ok = 0;
        if (ok) break;
    }
    m->kind[s] = SLOT_BOT;
    m->session[s] = -1;
    snprintf(m->name[s], sizeof(m->name[s]), "%s", nm);
    m->car_model[s] = (uint8_t)(rand() % CAR_COUNT);
    m->skin[s] = 0;
    memset(&m->bots[s], 0, sizeof(m->bots[s]));
    memset(&m->botIn[s], 0, sizeof(m->botIn[s]));
}

/* Put a queued session into slot s (empty or bot) of match m. */
static void match_set_human(Match *m, int s, int si)
{
    ClientSession *cs = &g_sess[si];
    m->kind[s] = SLOT_HUMAN;
    m->session[s] = si;
    snprintf(m->name[s], sizeof(m->name[s]), "%s", cs->name);
    m->car_model[s] = cs->car_model;
    m->skin[s] = cs->skin;
    cs->state = SS_MATCH;
    cs->match = (int)(m - g_match);
    cs->slot = s;
    input_reset(cs);
}

static Match *match_alloc(int playlist)
{
    int i;
    for (i = 0; i < MAX_MATCHES; i++) {
        if (g_match[i].active) continue;
        Match *m = &g_match[i];
        memset(m, 0, sizeof(*m));
        m->active = 1;
        m->id = ++g_matchSerial;
        m->playlist = playlist;
        m->nslots = 2 * playlist;
        m->match_time = g_matchLen;
        for (int s = 0; s < MAX_SLOTS; s++) { m->session[s] = -1; m->team[s] = s < playlist ? 0 : 1; }
        return m;
    }
    return NULL;
}

/* Create a match from the given queued sessions (oldest first), filling the rest with bots. */
static void match_create(int playlist, const int *sessIdx, int n)
{
    Match *m = match_alloc(playlist);
    int k, s, nb = 0, no = 0;
    if (!m) { printf("[QUEUE] No free match slots (max %d)!\n", MAX_MATCHES); return; }
    /* alternate humans between the teams so bot-filled games stay fair */
    for (k = 0; k < n; k++) {
        int team = (nb <= no) ? 0 : 1;
        s = team == 0 ? nb++ : playlist + no++;
        match_set_human(m, s, sessIdx[k]);
    }
    for (s = 0; s < m->nslots; s++) if (m->kind[s] == SLOT_EMPTY) match_set_bot(m, s);
    for (s = 0; s < m->nslots; s++) load_server_car(m->car_model[s], &m->cars[s]);
    match_kickoff(m, START_COUNTDOWN);
    printf("[MATCH %d] %s started: %d player(s), %d bot(s)\n", m->id, PLAYLIST_NAMES[playlist], n, m->nslots - n);
    for (s = 0; s < m->nslots; s++)
        printf("           %-6s %-20s %s%s\n", m->team[s] == 0 ? "BLUE" : "ORANGE", m->name[s],
               CAR_NAMES[m->car_model[s]], m->kind[s] == SLOT_BOT ? "  [BOT]" : "");
    for (s = 0; s < m->nslots; s++)
        if (m->kind[s] == SLOT_HUMAN) send_join_ack(&g_sess[m->session[s]], 1, "MATCH FOUND");
}

static void match_free(Match *m)
{
    printf("[MATCH %d] closed\n", m->id);
    m->active = 0;
}

/* Human leaves (quit or timeout): a bot takes over the car, or the match closes if nobody is left. */
static void match_remove_human(Match *m, int s, const char *why)
{
    ClientSession *cs = &g_sess[m->session[s]];
    printf("[MATCH %d] %s left (%s)\n", m->id, m->name[s], why);
    cs->state = SS_FREE;
    m->kind[s] = SLOT_EMPTY;
    if (match_humans(m) == 0) { match_free(m); return; }
    if (m->state != GS_OVER) {
        uint8_t model = m->car_model[s];
        match_set_bot(m, s);
        m->car_model[s] = model;   /* the bot takes over the car exactly where it is */
        printf("[MATCH %d] bot %s replaces them\n", m->id, m->name[s]);
    } else {
        m->kind[s] = SLOT_BOT;     /* keep the scoreboard intact for the post-game screen */
        m->session[s] = -1;
    }
}

static void match_end(Match *m)
{
    int s;
    for (s = 0; s < m->nslots; s++) {
        if (m->kind[s] != SLOT_HUMAN) continue;
        send_disconnect(&g_sess[m->session[s]].addr, DISC_MATCH_OVER);
        g_sess[m->session[s]].state = SS_FREE;
    }
    match_free(m);
}

/* Searching player joins a running match in place of a bot. */
static int try_backfill(int si)
{
    ClientSession *cs = &g_sess[si];
    int i, s, best = -1, bestSlot = -1, bestBots = 0;
    for (i = 0; i < MAX_MATCHES; i++) {
        Match *m = &g_match[i];
        int nbots;
        if (!m->active || m->playlist != cs->playlist || m->state == GS_OVER) continue;
        if (!m->overtime && m->match_time < BACKFILL_MIN_TIME) continue;
        nbots = match_bots(m);
        if (nbots == 0 || nbots <= bestBots) continue;   /* prefer the match with most bots */
        /* replace a bot on the team with fewer humans */
        {
            int hb = 0, ho = 0, cand = -1;
            for (s = 0; s < m->nslots; s++) if (m->kind[s] == SLOT_HUMAN) { if (m->team[s] == 0) hb++; else ho++; }
            for (s = 0; s < m->nslots && cand < 0; s++)
                if (m->kind[s] == SLOT_BOT && m->team[s] == (hb <= ho ? 0 : 1)) cand = s;
            for (s = 0; s < m->nslots && cand < 0; s++) if (m->kind[s] == SLOT_BOT) cand = s;
            if (cand >= 0) { best = i; bestSlot = cand; bestBots = nbots; }
        }
    }
    if (best < 0) return 0;
    {
        Match *m = &g_match[best];
        char botName[24];
        snprintf(botName, sizeof(botName), "%s", m->name[bestSlot]);
        match_set_human(m, bestSlot, si);
        load_server_car(m->car_model[bestSlot], &m->cars[bestSlot]);
        if (m->state == GS_COUNTDOWN) spawn_at_kickoff(m, bestSlot); else spawn_at_respawn(m, bestSlot);
        printf("[MATCH %d] %s joined in progress (replacing bot %s)\n", m->id, cs->name, botName);
        send_join_ack(cs, 1, "MATCH FOUND");
    }
    return 1;
}

static void matchmake(double now)
{
    int p;
    for (p = 1; p <= 3; p++) {
        int idx[MAX_SESSIONS], n = 0, i, j, need = 2 * p;
        for (i = 0; i < MAX_SESSIONS; i++)
            if (g_sess[i].state == SS_QUEUED && g_sess[i].playlist == p) idx[n++] = i;
        /* oldest first */
        for (i = 1; i < n; i++)
            for (j = i; j > 0 && g_sess[idx[j]].queue_start < g_sess[idx[j - 1]].queue_start; j--) {
                int t = idx[j]; idx[j] = idx[j - 1]; idx[j - 1] = t;
            }
        /* 1. full lobbies of real players */
        while (n >= need) {
            match_create(p, idx, need);
            memmove(idx, idx + need, (size_t)(n - need) * sizeof(int));
            n -= need;
        }
        /* 2. take a bot's seat in a running match */
        for (i = 0; i < n; ) {
            if (try_backfill(idx[i])) { memmove(idx + i, idx + i + 1, (size_t)(n - i - 1) * sizeof(int)); n--; }
            else i++;
        }
        /* 3. waited long enough: start with bots */
        if (n > 0 && g_botFill >= 0.0f && now - g_sess[idx[0]].queue_start >= g_botFill)
            match_create(p, idx, n);
    }
}

/* One 120 Hz tick of a match (same order as the client's prediction). */
static void match_step(Match *m, float h)
{
    int i, j;
    int frozen;
    m->tick++;
    frozen = m->state == GS_COUNTDOWN || m->state == GS_OVER;

    /* inputs: one queued input per human, AI for bots */
    g_bw = &m->bw;
    bot_world_update(&m->ball, h, g_pads, m->padTimer, PAD_COUNT);
    for (i = 0; i < m->nslots; i++) {
        if (m->kind[i] == SLOT_HUMAN) {
            ClientSession *cs = &g_sess[m->session[i]];
            input_consume(cs);
            if (frozen) memset(&cs->cur, 0, sizeof(Input));
        } else if (m->kind[i] == SLOT_BOT) {
            if (m->cars[i].demolished || frozen) memset(&m->botIn[i], 0, sizeof(Input));
            else m->botIn[i] = bot_think(i, m->cars, m->team, m->nslots, &m->ball, &m->bots[i], g_botSkill, h);
        }
    }

    for (i = 0; i < m->nslots; i++) {
        if (m->kind[i] == SLOT_EMPTY) continue;
        if (m->cars[i].demolished) {
            m->cars[i].demoTimer -= h;
            if (m->cars[i].demoTimer <= 0.0f) spawn_at_respawn(m, i);
        } else {
            const Input *in = m->kind[i] == SLOT_HUMAN ? &g_sess[m->session[i]].cur : &m->botIn[i];
            car_step(&m->cars[i], in, h);
        }
    }

    ball_step(&m->ball, h);
    for (i = 0; i < m->nslots; i++)
        if (m->kind[i] != SLOT_EMPTY && !m->cars[i].demolished) car_ball_collide(&m->cars[i], &m->ball);

    for (i = 0; i < m->nslots; i++) {
        if (m->kind[i] == SLOT_EMPTY || m->cars[i].demolished) continue;
        for (j = i + 1; j < m->nslots; j++) {
            if (m->kind[j] == SLOT_EMPTY || m->cars[j].demolished) continue;
            int demo = check_demolition(&m->cars[i], &m->cars[j], m->team[i], m->team[j]);
            if (demo) {
                int killer = demo == 2 ? j : i, victim = demo == 2 ? i : j;
                PktEventDemo ed;
                if (demo == 3) {
                    m->cars[i].demolished = 1; m->cars[i].demoTimer = 3.0f;
                    m->cars[j].demolished = 1; m->cars[j].demoTimer = 3.0f;
                } else {
                    m->cars[victim].demolished = 1; m->cars[victim].demoTimer = 3.0f;
                }
                memset(&ed, 0, sizeof(ed));
                hdr_init(&ed.header, PKT_EVENT_DEMO, m->tick);
                ed.killer_id = (uint8_t)killer; ed.victim_id = (uint8_t)victim;
                broadcast_match(m, &ed, sizeof(ed));
                if (demo == 3) {
                    ed.killer_id = (uint8_t)victim; ed.victim_id = (uint8_t)killer;
                    broadcast_match(m, &ed, sizeof(ed));
                }
                continue;
            }
            car_car_collide(&m->cars[i], &m->cars[j]);
        }
    }

    for (i = 0; i < PAD_COUNT; i++) {
        if (m->padTimer[i] > 0.0f) { m->padTimer[i] -= h; continue; }
        for (j = 0; j < m->nslots; j++) {
            if (m->kind[j] == SLOT_EMPTY || m->cars[j].demolished) continue;
            float dx = m->cars[j].pos.x - g_pads[i].x, dz = m->cars[j].pos.z - g_pads[i].z;
            if (dx*dx + dz*dz < PAD_RADIUS * PAD_RADIUS && m->cars[j].pos.y < 4.0f) {
                m->cars[j].boost = BOOST_MAX;
                m->padTimer[i] = PAD_RESPAWN;
                break;
            }
        }
    }

    /* match state machine */
    if (m->state == GS_PLAY) {
        if (fabsf(m->ball.pos.z) > ARENA_L + BALL_R) {
            int scorer = m->ball.pos.z > 0 ? 0 : 1;   /* 0 = Blue scored on the orange net */
            PktEventGoal eg;
            if (scorer == 0) m->score_blue++; else m->score_orange++;
            memset(&eg, 0, sizeof(eg));
            hdr_init(&eg.header, PKT_EVENT_GOAL, m->tick);
            eg.scorer_id = 0xFF; eg.team = (uint8_t)scorer; eg.ball_speed_kmh = Vector3Length(m->ball.vel) * 3.6f;
            broadcast_match(m, &eg, sizeof(eg));
            printf("[MATCH %d] GOAL %s (Blue %d - %d Orange)%s\n", m->id, scorer == 0 ? "BLUE" : "ORANGE",
                   m->score_blue, m->score_orange, m->overtime ? " in overtime" : "");
            m->state = GS_GOAL; m->state_timer = GOAL_TIME;
        } else if (m->overtime) {
            m->match_time -= h;                            /* counts up (negative) during overtime */
        } else if (m->match_time > 0.0f) {
            m->match_time -= h;
            if (m->match_time <= 0.0f) {
                m->match_time = 0.0f;
                if (m->score_blue == m->score_orange) {
                    m->overtime = 1;
                    printf("[MATCH %d] OVERTIME\n", m->id);
                } else {
                    m->state = GS_OVER; m->state_timer = OVER_TIME;
                    printf("[MATCH %d] final: Blue %d - %d Orange\n", m->id, m->score_blue, m->score_orange);
                }
            }
        }
    } else {
        m->state_timer -= h;
        if (m->state_timer <= 0.0f) {
            if (m->state == GS_COUNTDOWN) {
                m->state = GS_PLAY; m->state_timer = 0.0f;
            } else if (m->state == GS_GOAL) {
                if (m->overtime || m->match_time <= 0.0f) {
                    m->state = GS_OVER; m->state_timer = OVER_TIME;
                    printf("[MATCH %d] final: Blue %d - %d Orange\n", m->id, m->score_blue, m->score_orange);
                } else {
                    match_kickoff(m, COUNTDOWN_TIME);
                    m->bw.predCalls = 0;
                }
            } else if (m->state == GS_OVER) {
                match_end(m);
                return;
            }
        }
    }

    /* snapshot at 60 Hz */
    if (m->tick % 2 == 0) {
        PktServerState st;
        memset(&st, 0, sizeof(st));
        hdr_init(&st.header, PKT_SERVER_STATE, m->tick);
        st.server_tick = m->tick;
        st.game_state = m->state;
        st.state_timer = m->state_timer;
        st.countdown_sec = (uint8_t)ceilf(fmaxf(0.0f, m->state_timer));
        st.match_time = m->match_time;
        st.score_blue = m->score_blue;
        st.score_orange = m->score_orange;
        st.playlist = (uint8_t)m->playlist;
        st.overtime = (uint8_t)m->overtime;
        ns_ball_to_net(&m->ball, &st.ball);
        for (i = 0; i < m->nslots; i++) {
            NetCarState *ncs = &st.cars[i];
            if (m->kind[i] == SLOT_EMPTY) continue;
            ncs->active = 1;
            ncs->player_id = (uint8_t)i;
            ncs->team = (uint8_t)m->team[i];
            ncs->car_model = m->car_model[i];
            ncs->skin = m->skin[i];
            snprintf(ncs->player_name, sizeof(ncs->player_name), "%s", m->name[i]);
            ns_car_to_net(&m->cars[i], m->kind[i] == SLOT_HUMAN ? &g_sess[m->session[i]].cur : &m->botIn[i], ncs);
            if (m->kind[i] == SLOT_BOT) ncs->flags |= NCF_BOT;
            st.car_count++;
        }
        for (i = 0; i < m->nslots; i++) {
            if (m->kind[i] != SLOT_HUMAN) continue;
            ClientSession *cs = &g_sess[m->session[i]];
            st.header.ack = cs->last_processed;
            st.ack_input_tick = cs->last_processed;
            st.your_slot = (uint8_t)i;
            net_socket_send(g_sock, &cs->addr, &st, sizeof(st));
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Sessions                                                                   */
/* ------------------------------------------------------------------------ */
static int session_find(const NetAddr *a)
{
    int i;
    for (i = 0; i < MAX_SESSIONS; i++) if (g_sess[i].state != SS_FREE && net_addr_equal(&g_sess[i].addr, a)) return i;
    return -1;
}

static void session_drop(int si, const char *why)
{
    ClientSession *cs = &g_sess[si];
    if (cs->state == SS_QUEUED) {
        printf("[QUEUE] %s stopped searching %s (%s)\n", cs->name, PLAYLIST_NAMES[cs->playlist], why);
        cs->state = SS_FREE;
    } else if (cs->state == SS_MATCH) {
        Match *m = &g_match[cs->match];
        if (m->active && m->kind[cs->slot] == SLOT_HUMAN && m->session[cs->slot] == si) match_remove_human(m, cs->slot, why);
        else cs->state = SS_FREE;
    }
}

static void print_status(void)
{
    int i, s, p;
    double now = server_time_sec();
    printf("\n--- SERVER STATUS: %d online, %d match(es) ---\n", players_online(), matches_active());
    for (p = 1; p <= 3; p++) {
        int n = queue_count(p);
        if (n) printf("  Queue %-13s %d searching (longest %.0f s)\n", PLAYLIST_NAMES[p], n, oldest_wait(p, now));
    }
    for (i = 0; i < MAX_MATCHES; i++) {
        const Match *m = &g_match[i];
        if (!m->active) continue;
        printf("  Match %d  %s  Blue %d - %d Orange  %s%d:%02d  state %d\n", m->id, PLAYLIST_NAMES[m->playlist],
               m->score_blue, m->score_orange, m->overtime ? "OT +" : "",
               (int)fabsf(m->match_time) / 60, (int)fabsf(m->match_time) % 60, m->state);
        for (s = 0; s < m->nslots; s++) {
            if (m->kind[s] == SLOT_HUMAN) {
                const ClientSession *cs = &g_sess[m->session[s]];
                char astr[64];
                net_addr_to_string(&cs->addr, astr, sizeof(astr));
                printf("     %-6s %-20s %-10s %s | inbuf %d | starved %u | dropped %u\n",
                       m->team[s] == 0 ? "BLUE" : "ORANGE", m->name[s], CAR_NAMES[m->car_model[s]], astr,
                       (int)cs->newest_tick - (int)cs->next_tick + 1, cs->starved, cs->dropped);
            } else if (m->kind[s] == SLOT_BOT) {
                printf("     %-6s %-20s %-10s [BOT]\n", m->team[s] == 0 ? "BLUE" : "ORANGE", m->name[s], CAR_NAMES[m->car_model[s]]);
            }
        }
    }
    if (!players_online()) printf("  (nobody connected)\n");
    printf("-----------------------------------------------------------\n\n");
}

static void usage(void)
{
    printf("usage: sarpbc_server [-p port] [--botfill seconds|-1] [--bot-skill 0-2] [--match-time seconds]\n");
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    int port = SARP_DEFAULT_PORT;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) port = atoi(argv[++i]);
        else if (strcmp(argv[i], "--botfill") == 0 && i + 1 < argc) g_botFill = (float)atof(argv[++i]);
        else if (strcmp(argv[i], "--bot-skill") == 0 && i + 1 < argc) g_botSkill = (int)clampf((float)atoi(argv[++i]), 0, 2);
        else if (strcmp(argv[i], "--match-time") == 0 && i + 1 < argc) g_matchLen = fmaxf(10.0f, (float)atof(argv[++i]));
        else { usage(); if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) return 0; }
    }
    srand((unsigned)time(NULL));

    printf("\n============================================================\n");
    printf("     SARPBC QUICK-MATCH SERVER (120 Hz AUTH SIM, proto v%d)\n", SARP_NET_VERSION);
    printf("============================================================\n");
    if (!net_init()) {
        fprintf(stderr, "[ERROR] Failed to initialize network!\n");
        return 1;
    }
#ifdef _WIN32
    /* Default Windows timer granularity is ~15.6 ms, which made Sleep(1) bunch
     * ticks and snapshots into bursts. Ask for 1 ms scheduling. */
    timeBeginPeriod(1);
#endif

    g_sock = net_socket_create(port, 1);
    if (g_sock == NET_INVALID_SOCKET) {
        fprintf(stderr, "[ERROR] Could not bind UDP port %d!\n", port);
        net_shutdown();
        return 1;
    }
    printf("[SERVER] Listening on UDP port %d\n", port);

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
    if (!loaded) printf("[WARNING] Collision mesh arena_col.bin not found! Using fallback bounds.\n");
    {   /* make sure the car files are where the clients expect them */
        Car probe;
        load_server_car(0, &probe);
    }
    get_boost_pads(g_pads);

    printf("[SERVER] Playlists: Duel 1v1, Doubles 2v2, Standard 3v3 | match %.0f s | bots: %s\n",
           g_matchLen, g_botSkill == 0 ? "Rookie" : g_botSkill == 1 ? "Pro" : "All-Star");
    if (g_botFill >= 0) printf("[SERVER] Bots fill empty seats after %.0f s of searching\n", g_botFill);
    else printf("[SERVER] Bot fill disabled (matches only start when full)\n");
    printf("[SERVER] Console commands: status, quit\n\n");

    double last_time = server_time_sec(), last_mm = 0.0;
    double phys_acc = 0.0;
    const float h = 1.0f / PHYS_HZ;
    uint32_t pong_seq = 0;
    int running = 1;
    char recv_buf[2048];
    NetAddr sender;

    while (running) {
        double now = server_time_sec();
        uint32_t now_ms = (uint32_t)(now * 1000.0);

        /* --- 1. packets --- */
        int bytes;
        while ((bytes = net_socket_recv(g_sock, &sender, recv_buf, sizeof(recv_buf))) > 0) {
            if (bytes < (int)sizeof(NetHeader)) continue;
            NetHeader *hdr = (NetHeader *)recv_buf;
            if (hdr->magic != SARP_NET_MAGIC) continue;
            if (hdr->version != SARP_NET_VERSION) {
                if (hdr->type == PKT_JOIN_REQ) {   /* tell old/new clients why they can't join */
                    PktJoinAck nak;
                    memset(&nak, 0, sizeof(nak));
                    hdr_init(&nak.header, PKT_JOIN_ACK, 0);
                    nak.accepted = 0;
                    snprintf(nak.server_name, sizeof(nak.server_name), "VERSION MISMATCH");
                    net_socket_send(g_sock, &sender, &nak, sizeof(nak));
                }
                continue;
            }
            int si = session_find(&sender);
            if (si >= 0) g_sess[si].last_seen_ms = now_ms;

            if (hdr->type == PKT_PING && bytes >= (int)sizeof(PktPing)) {
                PktPing *ping = (PktPing *)recv_buf;
                PktPong pong;
                memset(&pong, 0, sizeof(pong));
                hdr_init(&pong.header, PKT_PONG, ++pong_seq);
                pong.header.ack = hdr->seq;
                pong.client_time_ms = ping->client_time_ms;
                pong.server_time_ms = now_ms;
                net_socket_send(g_sock, &sender, &pong, sizeof(pong));
            } else if (hdr->type == PKT_JOIN_REQ && bytes >= (int)sizeof(PktJoinReq)) {
                PktJoinReq *req = (PktJoinReq *)recv_buf;
                if (si >= 0) {
                    /* resent join (our reply got lost): re-acknowledge, never reset */
                    if (g_sess[si].state == SS_QUEUED) send_queue_status(&g_sess[si], now);
                    else send_join_ack(&g_sess[si], 1, "MATCH FOUND");
                    continue;
                }
                for (si = 0; si < MAX_SESSIONS && g_sess[si].state != SS_FREE; si++) {}
                if (si == MAX_SESSIONS) {
                    ClientSession tmp;
                    memset(&tmp, 0, sizeof(tmp));
                    tmp.addr = sender;
                    send_join_ack(&tmp, 0, "SERVER FULL");
                    continue;
                }
                ClientSession *cs = &g_sess[si];
                memset(cs, 0, sizeof(*cs));
                cs->state = SS_QUEUED;
                cs->addr = sender;
                cs->last_seen_ms = now_ms;
                cs->car_model = req->car_model < CAR_COUNT ? req->car_model : 0;
                cs->skin = req->skin;
                cs->playlist = (req->playlist >= 1 && req->playlist <= 3) ? req->playlist : 2;
                cs->queue_start = now;
                cs->match = cs->slot = -1;
                snprintf(cs->name, sizeof(cs->name), "%.19s", req->player_name[0] ? req->player_name : "Player");
                {
                    char astr[64];
                    net_addr_to_string(&sender, astr, sizeof(astr));
                    printf("[QUEUE] %s (%s) searching %s  [%d in queue]\n", cs->name, astr,
                           PLAYLIST_NAMES[cs->playlist], queue_count(cs->playlist));
                }
                send_queue_status(cs, now);
                last_mm = 0.0;   /* matchmake right away */
            } else if (hdr->type == PKT_CLIENT_INPUT && bytes >= (int)sizeof(PktClientInput)) {
                PktClientInput *cin = (PktClientInput *)recv_buf;
                if (si < 0 || g_sess[si].state != SS_MATCH || hdr->player_id != g_sess[si].slot) continue;
                ClientSession *cs = &g_sess[si];
                uint32_t t = cin->client_tick;
                /* redundant copies first, so the newest one wins on overlap */
                if (t > 2) input_store(cs, t - 2, &cin->prev_inputs[1]);
                if (t > 1) input_store(cs, t - 1, &cin->prev_inputs[0]);
                input_store(cs, t, &cin->input);
            } else if (hdr->type == PKT_DISCONNECT) {
                if (si >= 0) session_drop(si, "quit");
            }
        }

        /* --- 2. timeouts + queue updates --- */
        for (int i = 0; i < MAX_SESSIONS; i++) {
            ClientSession *cs = &g_sess[i];
            if (cs->state == SS_FREE) continue;
            if (now_ms - cs->last_seen_ms > TIMEOUT_MS) { session_drop(i, "timed out"); continue; }
            if (cs->state == SS_QUEUED && now - cs->last_status >= 0.5) send_queue_status(cs, now);
        }
        if (now - last_mm >= 0.25) { matchmake(now); last_mm = now; }

        /* --- 3. simulate every match at 120 Hz --- */
        double frame_dt = now - last_time;
        if (frame_dt > 0.1) frame_dt = 0.1;
        last_time = now;
        phys_acc += frame_dt;
        while (phys_acc >= h) {
            for (int i = 0; i < MAX_MATCHES; i++) if (g_match[i].active) match_step(&g_match[i], h);
            phys_acc -= h;
        }

        /* --- 4. console --- */
#ifdef _WIN32
        if (_kbhit()) {
            char cmd[64] = { 0 };
            if (fgets(cmd, sizeof(cmd), stdin)) {
                cmd[strcspn(cmd, "\r\n")] = 0;
                if (strcmp(cmd, "quit") == 0 || strcmp(cmd, "exit") == 0) running = 0;
                else if (strcmp(cmd, "status") == 0) print_status();
            }
        }
        Sleep(1);
#else
        {
            fd_set fds; struct timeval tv = { 0, 0 };
            FD_ZERO(&fds); FD_SET(0, &fds);
            if (select(1, &fds, NULL, NULL, &tv) > 0) {
                char cmd[64] = { 0 };
                if (fgets(cmd, sizeof(cmd), stdin)) {
                    cmd[strcspn(cmd, "\r\n")] = 0;
                    if (strcmp(cmd, "quit") == 0 || strcmp(cmd, "exit") == 0) running = 0;
                    else if (strcmp(cmd, "status") == 0) print_status();
                }
            }
        }
        usleep(1000);
#endif
    }

    printf("[SERVER] Shutting down...\n");
    for (int i = 0; i < MAX_SESSIONS; i++) if (g_sess[i].state != SS_FREE) send_disconnect(&g_sess[i].addr, DISC_KICKED);
    net_socket_close(g_sock);
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
