/*
 * SARPBC-C  --  a from-scratch C99 + raylib re-creation of the Soccar mode of
 * "Supersonic Acrobatic Rocket-Powered Battle-Cars" (Psyonix, 2008).
 *
 * Milestone 1: one car, one ball, one arena. Driving, jump / double jump /
 * dodge, boost, air control, ball physics, goals, score and match timer.
 *
 * The engine code is original. The car model/textures are loaded at runtime
 * from YOUR OWN extracted game files (export_c/cars/<name>/); nothing from
 * the game is compiled into the executable.
 *
 * World frame: right-handed, +Y up, metres. The field runs along Z
 * (blue goal at -Z, orange goal at +Z), its width along X.
 * Car local frame (same as the .sarm export): +X forward, +Y up, +Z right.
 *
 * Unreal units: 1 uu = 1 cm, so config values are divided by 100.
 * SARPBC works at roughly 1.75x Rocket League scale (the Octane is 207 uu
 * long here vs 118 uu in RL), so wherever a value is NOT in the PS3 configs
 * we use the RL value * 1.75. Those places are marked "RLx1.75".
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#define SARPBC_MODEL_IMPLEMENTATION
#include "sarpbc_model.h"
#undef SARPBC_MODEL_IMPLEMENTATION

#include "net_client.h"
#include "physics_sim.h"
#include "net_sync.h"

static const Vector3 LIGHT_DIR = { -0.35f, -0.85f, 0.4f };

typedef struct CarRender {
    Mesh     mesh[16];
    int      material[16];
    int      count;
    Material matBody, matGlass, matTire;
    Texture2D texBlue, texOrange, texCustom;
    /* wheels split out of the body at load time so they can spin/steer/bounce;
     * vertices are relative to whub (mesh space, unscaled) */
    Mesh     wmesh[4][3];
    int      wvalid[4][3], wanim[4], wfront[4];
    Vector3  whub[4];
} CarRender;

typedef enum { ST_COUNTDOWN, ST_PLAY, ST_GOAL, ST_OVER } GameState;

/* ------------------------------------------------------------------------ */
/* Bots (AI lives in bot_ai.h, shared with the dedicated server)             */
/* ------------------------------------------------------------------------ */
#define MAX_CARS 8
#include "bot_ai.h"

static const char *SKILL_NAMES[] = { "Rookie", "Pro", "All-Star" };
static const char *MODE_NAMES[]  = { "Free play", "1 v 1", "2 v 2", "3 v 3" };
static const char *PLAYLIST_LABELS[] = { "QUICK MATCH", "DUEL 1v1", "DOUBLES 2v2", "STANDARD 3v3" };


/* ------------------------------------------------------------------------ */
/* Settings (settings.ini next to the exe)                                    */
/* ------------------------------------------------------------------------ */
static const char *CAR_SKIN_NAMES[] = {
    "Synthwave",     /* octane */
    "Hellfire",      /* backfire */
    "Gold Rush",     /* scarab */
    "Stealth Jet",   /* aftershock */
    "Desert Camo",   /* renegade */
    "Cyberpunk",     /* zippy */
    "Urban Hazard"   /* marauder */
};
static const float MATCH_LENGTHS[] = { 180.0f, 300.0f, 600.0f, 0.0f };   /* 0 = unlimited */
static const char *MATCH_NAMES[]   = { "3 min", "5 min", "10 min", "Unlimited" };

typedef struct Settings {
    float camDist, camHeight, fov;      /* metres, height/distance ratio, vertical degrees */
    int   boostFov, matchIdx, invertPitch, fullscreen, showFps, showHints, car;
    int   mode, botSkill;               /* MODE_NAMES / SKILL_NAMES index */
    int   shadows, bloom;               /* graphics toggles */
    int   skin;                         /* 0 = Team, 1 = Custom */
    char  name[24];                     /* online player name */
    char  server[64];                   /* quick-match server address (host or host:port) */
    int   playlist;                     /* last quick-match playlist: players per team */
} Settings;
static Settings g_set = { 4.6f, 0.44f, 75.0f, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, "Striker", "127.0.0.1", 2 };

static void settings_path(char *out, size_t n) {
    if (FileExists("settings.ini")) snprintf(out, n, "settings.ini");
    else snprintf(out, n, "%ssettings.ini", GetApplicationDirectory());
}

static void settings_load(void)
{
    char path[600], line[160], key[64], sv[64];
    int i;
    FILE *f;
    settings_path(path, sizeof(path));
    if (!(f = fopen(path, "r"))) return;
    while (fgets(line, sizeof(line), f)) {
        float v;
        if (!strncmp(line, "player_name=", 12)) {   /* may contain spaces */
            snprintf(g_set.name, sizeof(g_set.name), "%.19s", line + 12);
            g_set.name[strcspn(g_set.name, "\r\n")] = 0;
            continue;
        }
        if (sscanf(line, " %63[^= ] = %63s", key, sv) != 2) continue;
        v = (float)atof(sv);
        if      (!strcmp(key, "camera_distance")) g_set.camDist     = clampf(v, 3.0f, 14.0f);
        else if (!strcmp(key, "camera_height"))   g_set.camHeight   = clampf(v, 0.1f, 0.8f);
        else if (!strcmp(key, "fov"))             g_set.fov         = clampf(v, 45.0f, 90.0f);
        else if (!strcmp(key, "boost_fov"))       g_set.boostFov    = v != 0;
        else if (!strcmp(key, "match_length"))    g_set.matchIdx    = (int)clampf(v, 0, 3);
        else if (!strcmp(key, "invert_pitch"))    g_set.invertPitch = v != 0;
        else if (!strcmp(key, "fullscreen"))      g_set.fullscreen  = v != 0;
        else if (!strcmp(key, "show_fps"))        g_set.showFps     = v != 0;
        else if (!strcmp(key, "show_hints"))      g_set.showHints   = v != 0;
        else if (!strcmp(key, "mode"))            g_set.mode        = (int)clampf(v, 0, 3);
        else if (!strcmp(key, "bot_skill"))       g_set.botSkill    = (int)clampf(v, 0, 2);
        else if (!strcmp(key, "shadows"))         g_set.shadows     = v != 0;
        else if (!strcmp(key, "bloom"))           g_set.bloom       = v != 0;
        else if (!strcmp(key, "skin"))            g_set.skin        = (int)clampf(v, 0, 1);
        else if (!strcmp(key, "playlist"))        g_set.playlist    = (int)clampf(v, 1, 3);
        else if (!strcmp(key, "server"))          snprintf(g_set.server, sizeof(g_set.server), "%s", sv);
        else if (!strcmp(key, "car"))
            for (i = 0; i < CAR_COUNT; i++) if (!strcmp(sv, CAR_NAMES[i])) g_set.car = i;
    }
    fclose(f);
}

static void settings_save(void)
{
    char path[600];
    FILE *f;
    settings_path(path, sizeof(path));
    if (!(f = fopen(path, "w"))) return;
    fprintf(f, "camera_distance=%.2f\ncamera_height=%.2f\nfov=%.0f\nboost_fov=%d\nmatch_length=%d\n"
               "invert_pitch=%d\nfullscreen=%d\nshow_fps=%d\nshow_hints=%d\ncar=%s\nmode=%d\nbot_skill=%d\n"
               "shadows=%d\nbloom=%d\nskin=%d\nplaylist=%d\nserver=%s\nplayer_name=%s\n",
            g_set.camDist, g_set.camHeight, g_set.fov, g_set.boostFov, g_set.matchIdx,
            g_set.invertPitch, g_set.fullscreen, g_set.showFps, g_set.showHints, CAR_NAMES[g_set.car],
            g_set.mode, g_set.botSkill, g_set.shadows, g_set.bloom, g_set.skin,
            g_set.playlist, g_set.server[0] ? g_set.server : "127.0.0.1", g_set.name);
    fclose(f);
}

/* ------------------------------------------------------------------------ */
/* Input                                                                      */
/* ------------------------------------------------------------------------ */
static Input read_input(void)
{
    static int trigSeen[2] = { 0, 0 };
    Input in;
    float kx, ky, roll;
    memset(&in, 0, sizeof(in));

    kx = (float)(IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) - (float)(IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT));
    ky = (float)(IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN))  - (float)(IsKeyDown(KEY_W) || IsKeyDown(KEY_UP));
    roll = (float)IsKeyDown(KEY_E) - (float)IsKeyDown(KEY_Q);
    in.throttle    = -ky;
    in.steer       = kx;
    in.pitch       = ky;
    in.jump        = IsKeyDown(KEY_SPACE) || IsMouseButtonDown(MOUSE_BUTTON_LEFT);
    in.jumpPressed = IsKeyPressed(KEY_SPACE) || IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
    in.boost       = IsKeyDown(KEY_LEFT_SHIFT) || IsMouseButtonDown(MOUSE_BUTTON_RIGHT);
    in.slide       = IsKeyDown(KEY_LEFT_CONTROL);

    if (IsGamepadAvailable(0)) {
        float gx = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X);
        float gy = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y);
        float rt = GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_TRIGGER);
        float lt = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_TRIGGER);
        /* triggers can report 0 until first touched; only trust them once seen at rest (-1) */
        if (rt < -0.5f) trigSeen[0] = 1;
        if (lt < -0.5f) trigSeen[1] = 1;
        rt = trigSeen[0] ? (rt + 1.0f) * 0.5f : 0.0f;
        lt = trigSeen[1] ? (lt + 1.0f) * 0.5f : 0.0f;
        if (fabsf(gx) < 0.12f) gx = 0;
        if (fabsf(gy) < 0.12f) gy = 0;
        if (gx != 0) in.steer = gx;
        if (gy != 0) in.pitch = gy;
        if (rt > 0.05f || lt > 0.05f) in.throttle = rt - lt;
        in.jump        |= IsGamepadButtonDown(0, GAMEPAD_BUTTON_RIGHT_FACE_DOWN);
        in.jumpPressed |= IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_FACE_DOWN);
        in.boost       |= IsGamepadButtonDown(0, GAMEPAD_BUTTON_RIGHT_FACE_RIGHT);
        in.slide       |= IsGamepadButtonDown(0, GAMEPAD_BUTTON_RIGHT_FACE_LEFT);
        if (IsGamepadButtonDown(0, GAMEPAD_BUTTON_LEFT_TRIGGER_1))  roll -= 1.0f;
        if (IsGamepadButtonDown(0, GAMEPAD_BUTTON_RIGHT_TRIGGER_1)) roll += 1.0f;
    }

    if (g_set.invertPitch) in.pitch = -in.pitch;

    /* powerslide button doubles as air roll (steer -> roll) */
    if (in.slide) { in.roll = in.steer; in.yaw = 0.0f; }
    else          { in.yaw = in.steer; }
    if (roll != 0.0f) in.roll = clampf(roll, -1, 1);
    return in;
}

/* ------------------------------------------------------------------------ */
/* Rendering                                                                  */
/* ------------------------------------------------------------------------ */
/* ---- lighting + post-processing ------------------------------------------
 * scene -> RGBA16F HDR target (linear light: sun + sky ambient + specular +
 * fresnel sky reflection + point lights + 4096^2 PCF shadow map + fog), then
 * bloom mip chain -> ACES tonemap / grade / vignette -> FXAA -> backbuffer.  */
#define MAX_PT_LIGHTS 16
#define SHADOW_RES    4096
#define BLOOM_MIPS    6
#define SHADOW_NEAR_RES  2048
#define SHADOW_NEAR_SIZE 56.0f      /* metres covered by the sharp cascade around the player */
static const Vector3 SUN_DIR = { -0.38f, -0.78f, 0.50f };   /* crisp daytime stadium sun */

/* shared by the lit + sky shaders: authentic UE3 daytime sky, linear HDR */
#define SKY_GLSL \
    "uniform vec3 sunDir;\n" \
    "vec3 skyColor(vec3 d){\n" \
    "  float y = d.y;\n" \
    "  vec3 zen = vec3(0.08, 0.26, 0.68), mid = vec3(0.32, 0.54, 0.84), hor = vec3(0.72, 0.82, 0.92), c;\n" \
    "  if (y >= 0.0) c = mix(mix(hor, mid, smoothstep(0.0, 0.22, y)), zen, smoothstep(0.16, 0.85, y));\n" \
    "  else c = mix(hor * 0.65, vec3(0.14, 0.16, 0.20), clamp(-y * 5.0, 0.0, 1.0));\n" \
    "  float s = max(dot(d, -sunDir), 0.0);\n" \
    "  c += vec3(1.70, 1.45, 1.10) * (pow(s, 1800.0) * 60.0 + pow(s, 32.0) * 1.5 + pow(s, 6.0) * 0.35);\n" \
    "  float az = atan(d.z, d.x);\n" \
    "  if (y > 0.02) {\n" \
    "    vec2 uv = d.xz / (y + 0.10) * 0.50;\n" \
    "    float n1 = sin(uv.x * 2.0 + cos(uv.y * 1.5)) * 0.5 + 0.5;\n" \
    "    float n2 = sin(uv.x * 4.6 - uv.y * 3.8 + n1 * 3.2) * 0.5 + 0.5;\n" \
    "    float n3 = sin(uv.x * 9.5 + uv.y * 8.2) * 0.5 + 0.5;\n" \
    "    float cloud = smoothstep(0.48, 0.78, n1 * 0.55 + n2 * 0.32 + n3 * 0.13) * smoothstep(0.02, 0.15, y);\n" \
    "    vec3 cloudCol = mix(vec3(0.65, 0.70, 0.82), vec3(1.45, 1.40, 1.30), pow(max(dot(normalize(vec3(d.x, 0.35, d.z)), -sunDir), 0.0), 3.0) * 0.8 + 0.25);\n" \
    "    c = mix(c, cloudCol, cloud * 0.90);\n" \
    "  }\n" \
    "  float towerAngle = abs(fract((az + 0.785398) / 1.570796) - 0.5) * 1.570796;\n" \
    "  float inTowerY = smoothstep(0.12, 0.20, y) * smoothstep(0.42, 0.30, y);\n" \
    "  float towerLight = pow(max(1.0 - towerAngle * 10.0, 0.0), 3.5) * inTowerY;\n" \
    "  c += vec3(2.2, 2.3, 2.6) * (towerLight * 16.0);\n" \
    "  float mast = smoothstep(0.015, 0.005, towerAngle) * smoothstep(0.05, 0.12, y) * smoothstep(0.38, 0.30, y);\n" \
    "  c = mix(c, vec3(0.12, 0.14, 0.18), mast * 0.85);\n" \
    "  float beam = pow(max(1.0 - towerAngle * 4.5, 0.0), 4.0) * smoothstep(0.02, 0.22, y) * smoothstep(0.42, 0.26, y);\n" \
    "  c += vec3(0.6, 0.7, 0.9) * beam * 0.8;\n" \
    "  float mElev = 0.075 + 0.038 * sin(az * 3.0 + 1.2) + 0.020 * sin(az * 7.0 - 0.5) + 0.012 * cos(az * 13.0);\n" \
    "  if (y < mElev && y > 0.0) {\n" \
    "    float mFog = clamp(y / mElev, 0.0, 1.0);\n" \
    "    vec3 mCol = mix(vec3(0.20, 0.26, 0.36), hor * 0.82, mFog * 0.65);\n" \
    "    c = mix(c, mCol, smoothstep(mElev + 0.008, mElev - 0.002, y));\n" \
    "  }\n" \
    "  float sRim = smoothstep(0.085, 0.075, y) * smoothstep(0.045, 0.060, y);\n" \
    "  c = mix(c, vec3(0.16, 0.20, 0.25), sRim * 0.85);\n" \
    "  float rimLights = sin(az * 45.0) > 0.5 ? 1.0 : 0.0;\n" \
    "  c += vec3(0.9, 0.7, 0.4) * sRim * rimLights * 1.2;\n" \
    "  return c; }\n"

static const char *LIT_VS =
    "#version 330\n"
    "in vec3 vertexPosition; in vec2 vertexTexCoord; in vec3 vertexNormal;\n"
    "uniform mat4 mvp; uniform mat4 matModel; uniform mat4 matNormal;\n"
    "out vec2 fragUV; out vec3 fragN; out vec3 fragPos;\n"
    "void main(){ fragUV = vertexTexCoord;\n"
    "  fragPos = (matModel * vec4(vertexPosition, 1.0)).xyz;\n"
    "  fragN = normalize((matNormal * vec4(vertexNormal, 0.0)).xyz);\n"
    "  gl_Position = mvp * vec4(vertexPosition, 1.0); }\n";

static const char *LIT_FS =
    "#version 330\n"
    "in vec2 fragUV; in vec3 fragN; in vec3 fragPos; out vec4 finalColor;\n"
    "uniform sampler2D texture0; uniform vec4 colDiffuse;\n"
    "uniform sampler2DShadow shadowMap, shadowNear; uniform mat4 lightVP, lightVPN; uniform int shadowOn;\n"
    "uniform float shadowBias, shadowBiasN, shadowTexel, shadowTexelN;\n"
    "uniform vec3 sunCol; uniform vec3 viewPos; uniform vec4 matParams;\n"   /* spec, shininess, reflection, emissive */
    "uniform vec4 ptPos[16]; uniform vec4 ptCol[16]; uniform int ptCount;\n"
    "uniform vec3 fogCol; uniform float fogDensity;\n"
    SKY_GLSL
    /* 3x3 taps of hardware 2x2 bilinear compares = smooth 4x4-ish PCF */
    "float pcf(sampler2DShadow m, vec3 c, float t){ float s = 0.0;\n"
    "  for (int y = -1; y <= 1; y++) for (int x = -1; x <= 1; x++) s += texture(m, vec3(c.xy + vec2(x, y) * t, c.z));\n"
    "  return s / 9.0; }\n"
    "float shadowTerm(vec3 p, vec3 n, float ndl){\n"
    "  if (shadowOn == 0) return 1.0;\n"
    "  float slope = 1.0 - ndl, far = 1.0, near = 1.0, e;\n"
    "  vec3 c = (lightVPN * vec4(p + n * (0.015 + 0.05 * slope), 1.0)).xyz * 0.5 + 0.5;\n"
    "  bool inNear;\n"
    "  e = max(abs(c.x - 0.5), abs(c.y - 0.5)) * 2.0;\n"
    "  inNear = e < 0.95 && c.z < 1.0;\n"
    "  if (inNear) { near = pcf(shadowNear, vec3(c.xy, c.z - shadowBiasN), shadowTexelN); if (e < 0.8) return near; }\n"
    "  c = (lightVP * vec4(p + n * (0.05 + 0.15 * slope), 1.0)).xyz * 0.5 + 0.5;\n"
    "  if (c.x > 0.0 && c.y > 0.0 && c.x < 1.0 && c.y < 1.0 && c.z < 1.0) far = pcf(shadowMap, vec3(c.xy, c.z - shadowBias), shadowTexel);\n"
    "  return inNear ? mix(near, far, smoothstep(0.8, 0.95, e)) : far; }\n"
    "void main(){ vec4 t = texture(texture0, fragUV);\n"
    "  if (t.a < 0.33) discard;\n"
    "  vec4 base = t * colDiffuse;\n"
    "  vec3 alb = pow(base.rgb, vec3(2.2));\n"
    "  vec3 n = normalize(gl_FrontFacing ? fragN : -fragN);\n"
    "  vec3 v = normalize(viewPos - fragPos), L = -sunDir;\n"
    "  float spec = matParams.x, shin = matParams.y, nv = max(dot(n, v), 0.0);\n"
    "  float ndl = max(dot(n, L), 0.0);\n"
    "  float sh = ndl > 0.0 ? shadowTerm(fragPos, n, ndl) : 0.0;\n"
    "  float norm = (shin + 8.0) / 25.13;\n"
    "  vec3 amb = mix(vec3(0.32, 0.30, 0.28), vec3(0.48, 0.54, 0.65), n.y * 0.5 + 0.5);\n"
    "  vec3 col = alb * amb;\n"
    "  col += sunCol * sh * ndl * (alb + spec * norm * pow(max(dot(n, normalize(L + v)), 0.0), shin));\n"
    "  for (int i = 0; i < ptCount; i++) {\n"
    "    vec3 d = ptPos[i].xyz - fragPos; float dist = length(d);\n"
    "    float a = clamp(1.0 - dist / ptPos[i].w, 0.0, 1.0); a *= a;\n"
    "    if (a <= 0.0) continue;\n"
    "    vec3 l = d / max(dist, 0.001); float nl = max(dot(n, l), 0.0);\n"
    "    col += ptCol[i].rgb * a * nl * (alb + spec * norm * pow(max(dot(n, normalize(l + v)), 0.0), shin));\n"
    "  }\n"
    "  float F = matParams.z * mix(0.04, 0.45, pow(1.0 - nv, 4.0));\n"
    "  col += skyColor(reflect(-v, n)) * F * (0.4 + 0.6 * max(sh, 0.35));\n"
    "  col += alb * matParams.w;\n"
    "  float alpha = base.a < 1.0 ? clamp(base.a * 0.7 + F * 0.25, 0.0, 1.0) : 1.0;\n"
    "  float fog = 1.0 - exp(-length(viewPos - fragPos) * fogDensity);\n"
    "  finalColor = vec4(mix(col, fogCol, fog), alpha); }\n";

static const char *DEPTH_VS =
    "#version 330\n"
    "in vec3 vertexPosition; in vec2 vertexTexCoord; uniform mat4 mvp; out vec2 fragUV;\n"
    "void main(){ fragUV = vertexTexCoord; gl_Position = mvp * vec4(vertexPosition, 1.0); }\n";
static const char *DEPTH_FS =
    "#version 330\n"
    "in vec2 fragUV; uniform sampler2D texture0; out vec4 finalColor;\n"
    "void main(){ if (texture(texture0, fragUV).a < 0.33) discard; finalColor = vec4(1.0); }\n";

static const char *SKY_VS =
    "#version 330\n"
    "in vec3 vertexPosition; uniform mat4 mvp; out vec3 dir;\n"
    "void main(){ dir = vertexPosition; gl_Position = mvp * vec4(vertexPosition, 1.0); }\n";
static const char *SKY_FS =
    "#version 330\n"
    "in vec3 dir; out vec4 finalColor;\n"
    SKY_GLSL
    "void main(){ finalColor = vec4(skyColor(normalize(dir)), 1.0); }\n";

/* rlgl batch geometry (pads, flame, nets): vertex colour in sRGB, scaled into HDR so it blooms */
static const char *EMIS_FS =
    "#version 330\n"
    "in vec2 fragTexCoord; in vec4 fragColor; out vec4 finalColor;\n"
    "uniform sampler2D texture0; uniform float intensity;\n"
    "void main(){ vec4 c = fragColor * texture(texture0, fragTexCoord);\n"
    "  finalColor = vec4(pow(c.rgb, vec3(2.2)) * intensity, c.a); }\n";

/* bloom: 4-tap bilinear downsample (first pass = soft-threshold prefilter) */
static const char *DOWN_FS =
    "#version 330\n"
    "in vec2 fragTexCoord; out vec4 finalColor;\n"
    "uniform sampler2D texture0; uniform vec2 texel; uniform int prefilter; uniform float threshold;\n"
    "void main(){ vec2 uv = fragTexCoord;\n"
    "  vec3 c = texture(texture0, uv).rgb * 0.5\n"
    "    + (texture(texture0, uv + texel * vec2(-1.0, -1.0)).rgb + texture(texture0, uv + texel * vec2(1.0, -1.0)).rgb\n"
    "     + texture(texture0, uv + texel * vec2(-1.0,  1.0)).rgb + texture(texture0, uv + texel * vec2(1.0,  1.0)).rgb) * 0.125;\n"
    "  if (prefilter == 1) { c = min(c, vec3(40.0));\n"
    "    float br = max(c.r, max(c.g, c.b)), knee = threshold * 0.5;\n"
    "    float soft = clamp(br - threshold + knee, 0.0, 2.0 * knee); soft = soft * soft / (4.0 * knee + 1e-4);\n"
    "    c *= max(soft, br - threshold) / max(br, 1e-4); }\n"
    "  finalColor = vec4(c, 1.0); }\n";

/* bloom: 9-tap tent upsample, blended additively onto the next larger mip */
static const char *UP_FS =
    "#version 330\n"
    "in vec2 fragTexCoord; out vec4 finalColor;\n"
    "uniform sampler2D texture0; uniform vec2 texel;\n"
    "void main(){ vec2 uv = fragTexCoord, o = texel;\n"
    "  vec3 c = texture(texture0, uv).rgb * 4.0;\n"
    "  c += (texture(texture0, uv + vec2(-o.x, 0.0)).rgb + texture(texture0, uv + vec2(o.x, 0.0)).rgb\n"
    "      + texture(texture0, uv + vec2(0.0, -o.y)).rgb + texture(texture0, uv + vec2(0.0, o.y)).rgb) * 2.0;\n"
    "  c += texture(texture0, uv - o).rgb + texture(texture0, uv + o).rgb\n"
    "     + texture(texture0, uv + vec2(o.x, -o.y)).rgb + texture(texture0, uv + vec2(-o.x, o.y)).rgb;\n"
    "  finalColor = vec4(c / 16.0, 1.0); }\n";

static const char *COMP_FS =
    "#version 330\n"
    "in vec2 fragTexCoord; out vec4 finalColor;\n"
    "uniform sampler2D texture0; uniform sampler2D bloomTex;\n"
    "uniform float exposure, bloomStr, vignette, aberration;\n"
    "void main(){ vec2 uv = fragTexCoord, dc = uv - 0.5;\n"
    "  vec3 h;\n"
    "  if (aberration > 0.0) {\n"
    "    vec2 dir = dc * aberration;\n"
    "    h  = texture(texture0, uv).rgb * 0.45;\n"
    "    h += texture(texture0, uv - dir * 0.6).rgb * 0.33;\n"
    "    h += texture(texture0, uv - dir * 1.2).rgb * 0.22;\n"
    "  } else {\n"
    "    h = texture(texture0, uv).rgb;\n"
    "  }\n"
    "  if (bloomStr > 0.0) h += texture(bloomTex, uv).rgb * bloomStr;\n"
    "  vec3 x = h * exposure;\n"
    "  vec3 c = (x * (1.0 + x * 0.11)) / (1.0 + x);\n"       /* UE3 extended Reinhard tonemap */
    "  float l = dot(c, vec3(0.2126, 0.7152, 0.0722));\n"
    "  c = max(mix(vec3(l), c, 1.08), 0.0);\n"                /* clean SARPBC arcade saturation */
    "  if (vignette > 0.0) c *= 1.0 - vignette * pow(length(dc * vec2(1.0, 0.85)) * 1.3, 2.0);\n"
    "  c = pow(clamp(c, 0.0, 1.0), vec3(1.0 / 2.2));\n"       /* DisplayGamma=2.2 */
    "  finalColor = vec4(c, dot(c, vec3(0.299, 0.587, 0.114))); }\n";             /* luma in alpha for FXAA */

static const char *FXAA_FS =
    "#version 330\n"
    "in vec2 fragTexCoord; out vec4 finalColor;\n"
    "uniform sampler2D texture0; uniform vec2 texel;\n"
    "void main(){ vec2 uv = fragTexCoord;\n"
    "  float nw = texture(texture0, uv + vec2(-1.0, -1.0) * texel).a, ne = texture(texture0, uv + vec2(1.0, -1.0) * texel).a;\n"
    "  float sw = texture(texture0, uv + vec2(-1.0,  1.0) * texel).a, se = texture(texture0, uv + vec2(1.0,  1.0) * texel).a;\n"
    "  vec4 m = texture(texture0, uv);\n"
    "  float lmin = min(m.a, min(min(nw, ne), min(sw, se))), lmax = max(m.a, max(max(nw, ne), max(sw, se)));\n"
    "  if (lmax - lmin < max(0.0312, lmax * 0.125)) { finalColor = vec4(m.rgb, 1.0); return; }\n"
    "  vec2 dir = vec2(-((nw + ne) - (sw + se)), (nw + sw) - (ne + se));\n"
    "  float red = max((nw + ne + sw + se) * 0.03125, 1.0 / 128.0);\n"
    "  dir = clamp(dir / (min(abs(dir.x), abs(dir.y)) + red), vec2(-8.0), vec2(8.0)) * texel;\n"
    "  vec3 a = 0.5 * (texture(texture0, uv - dir / 6.0).rgb + texture(texture0, uv + dir / 6.0).rgb);\n"
    "  vec3 b = a * 0.5 + 0.25 * (texture(texture0, uv - dir * 0.5).rgb + texture(texture0, uv + dir * 0.5).rgb);\n"
    "  float lb = dot(b, vec3(0.299, 0.587, 0.114));\n"
    "  finalColor = vec4((lb < lmin || lb > lmax) ? a : b, 1.0); }\n";

#ifdef _WIN32
__declspec(dllimport) void __stdcall glDrawBuffer(unsigned int mode);   /* GL 1.1, exported by opengl32 */
__declspec(dllimport) void __stdcall glReadBuffer(unsigned int mode);
__declspec(dllimport) void __stdcall glTexParameteri(unsigned int target, unsigned int pname, int param);
#endif

typedef struct Gfx {
    Shader lit, depth, sky, emis, down, up, comp, fxaa;
    int lMat, lView, lLightVP, lLightVPN, lShadowOn, lPtPos, lPtCol, lPtCount;
    int eInt, dTexel, dPre, dThr, uTexel, cBloom, cExp, cBloomStr, cVig, cAb, fTexel;
    RenderTexture2D shadow, shadowN, hdr, ldr, bloom[BLOOM_MIPS];
    int w, h;
    float depthRange, zNear, zFar;
    Matrix lightView, lightProj, lightProjN;
    Mesh skyMesh;
    Material skyMat;
    Vector4 ptPos[MAX_PT_LIGHTS], ptCol[MAX_PT_LIGHTS];
    int ptCount;
} Gfx;
static Gfx G;

static RenderTexture2D gfx_make_rt(int w, int h, int format, int depth)
{
    RenderTexture2D rt = { 0 };
    rt.id = rlLoadFramebuffer();
    rlEnableFramebuffer(rt.id);
    rt.texture.id = rlLoadTexture(NULL, w, h, format, 1);
    rt.texture.width = w; rt.texture.height = h; rt.texture.format = format; rt.texture.mipmaps = 1;
    rlFramebufferAttach(rt.id, rt.texture.id, RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_TEXTURE2D, 0);
    if (depth) {
        rt.depth.id = rlLoadTextureDepth(w, h, true);
        rt.depth.width = w; rt.depth.height = h;
        rlFramebufferAttach(rt.id, rt.depth.id, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_RENDERBUFFER, 0);
    }
    if (!rlFramebufferComplete(rt.id)) TraceLog(LOG_WARNING, "GFX: render target %dx%d incomplete", w, h);
    rlDisableFramebuffer();
    SetTextureFilter(rt.texture, TEXTURE_FILTER_BILINEAR);
    SetTextureWrap(rt.texture, TEXTURE_WRAP_CLAMP);
    return rt;
}

static void gfx_free_targets(void)
{
    int i;
    if (G.hdr.id) UnloadRenderTexture(G.hdr);
    if (G.ldr.id) UnloadRenderTexture(G.ldr);
    for (i = 0; i < BLOOM_MIPS; i++) if (G.bloom[i].id) UnloadRenderTexture(G.bloom[i]);
    G.hdr.id = G.ldr.id = 0;
    for (i = 0; i < BLOOM_MIPS; i++) G.bloom[i].id = 0;
}

/* (re)create the screen-sized targets when the window size changes */
static void gfx_resize(void)
{
    int w = GetRenderWidth(), h = GetRenderHeight(), i;
    if (w <= 0 || h <= 0 || (w == G.w && h == G.h)) return;
    gfx_free_targets();
    G.w = w; G.h = h;
    G.hdr = gfx_make_rt(w, h, PIXELFORMAT_UNCOMPRESSED_R16G16B16A16, 1);
    G.ldr = gfx_make_rt(w, h, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8, 0);
    for (i = 0; i < BLOOM_MIPS; i++) {
        int bw = w >> (i + 1), bh = h >> (i + 1);
        G.bloom[i] = gfx_make_rt(bw < 1 ? 1 : bw, bh < 1 ? 1 : bh, PIXELFORMAT_UNCOMPRESSED_R16G16B16A16, 0);
    }
}

static Shader gfx_shader(const char *vs, const char *fs)
{
    Shader s = LoadShaderFromMemory(vs, fs);
    if (!IsShaderValid(s)) TraceLog(LOG_WARNING, "GFX: shader failed to compile");
    return s;
}

/* depth-only FBO; the texture is set up for sampler2DShadow (compare mode + linear = 2x2 PCF in hardware) */
static RenderTexture2D gfx_make_shadow(int res)
{
    RenderTexture2D rt = { 0 };
    rt.id = rlLoadFramebuffer();
    rt.texture.id = rlLoadTextureDepth(res, res, false);
    rt.texture.width = rt.texture.height = res;
    rt.texture.format = PIXELFORMAT_UNCOMPRESSED_R32; rt.texture.mipmaps = 1;
    rt.depth = rt.texture;
    rlFramebufferAttach(rt.id, rt.texture.id, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
#ifdef _WIN32
    rlEnableFramebuffer(rt.id);
    glDrawBuffer(0); glReadBuffer(0);    /* GL_NONE: no colour attachment */
    rlDisableFramebuffer();
    rlEnableTexture(rt.texture.id);
    glTexParameteri(0x0DE1 /*GL_TEXTURE_2D*/, 0x884C /*GL_TEXTURE_COMPARE_MODE*/, 0x884E /*GL_COMPARE_REF_TO_TEXTURE*/);
    glTexParameteri(0x0DE1, 0x884D /*GL_TEXTURE_COMPARE_FUNC*/, 0x0203 /*GL_LEQUAL*/);
    glTexParameteri(0x0DE1, 0x2801 /*MIN_FILTER*/, 0x2601 /*GL_LINEAR*/);
    glTexParameteri(0x0DE1, 0x2800 /*MAG_FILTER*/, 0x2601);
    glTexParameteri(0x0DE1, 0x2802 /*WRAP_S*/, 0x812F /*CLAMP_TO_EDGE*/);
    glTexParameteri(0x0DE1, 0x2803 /*WRAP_T*/, 0x812F);
    rlDisableTexture();
#endif
    if (!rlFramebufferComplete(rt.id)) TraceLog(LOG_WARNING, "GFX: shadow framebuffer %d incomplete", res);
    return rt;
}

static void gfx_init(void)
{
    Vector3 sun = Vector3Normalize(SUN_DIR);
    Vector3 sunCol = { 1.55f, 1.45f, 1.30f };   /* authentic warm UE3 directional sun */
    Vector3 fogCol = { 0.68f, 0.75f, 0.85f };   /* atmospheric blue horizon haze */
    Vector3 lo = { 1e9f, 1e9f, 1e9f }, hi = { -1e9f, -1e9f, -1e9f }, ctr = { 0, 10, 0 };
    float fogD = 0.0007f, bias, texel = 1.0f / SHADOW_RES, thr = 0.85f;
    int slot = 10, k;
    memset(&G, 0, sizeof(G));

    G.lit = gfx_shader(LIT_VS, LIT_FS);
    G.lit.locs[SHADER_LOC_MATRIX_MODEL]  = GetShaderLocation(G.lit, "matModel");
    G.lit.locs[SHADER_LOC_MATRIX_NORMAL] = GetShaderLocation(G.lit, "matNormal");
    G.lMat      = GetShaderLocation(G.lit, "matParams");
    G.lView     = GetShaderLocation(G.lit, "viewPos");
    G.lLightVP  = GetShaderLocation(G.lit, "lightVP");
    G.lLightVPN = GetShaderLocation(G.lit, "lightVPN");
    G.lShadowOn = GetShaderLocation(G.lit, "shadowOn");
    G.lPtPos    = GetShaderLocation(G.lit, "ptPos");
    G.lPtCol    = GetShaderLocation(G.lit, "ptCol");
    G.lPtCount  = GetShaderLocation(G.lit, "ptCount");
    SetShaderValue(G.lit, GetShaderLocation(G.lit, "sunDir"), &sun, SHADER_UNIFORM_VEC3);
    SetShaderValue(G.lit, GetShaderLocation(G.lit, "sunCol"), &sunCol, SHADER_UNIFORM_VEC3);
    SetShaderValue(G.lit, GetShaderLocation(G.lit, "fogCol"), &fogCol, SHADER_UNIFORM_VEC3);
    SetShaderValue(G.lit, GetShaderLocation(G.lit, "fogDensity"), &fogD, SHADER_UNIFORM_FLOAT);
    SetShaderValue(G.lit, GetShaderLocation(G.lit, "shadowMap"), &slot, SHADER_UNIFORM_INT);
    SetShaderValue(G.lit, GetShaderLocation(G.lit, "shadowTexel"), &texel, SHADER_UNIFORM_FLOAT);

    G.depth = gfx_shader(DEPTH_VS, DEPTH_FS);
    G.sky   = gfx_shader(SKY_VS, SKY_FS);
    SetShaderValue(G.sky, GetShaderLocation(G.sky, "sunDir"), &sun, SHADER_UNIFORM_VEC3);
    G.emis  = gfx_shader(NULL, EMIS_FS);   G.eInt = GetShaderLocation(G.emis, "intensity");
    G.down  = gfx_shader(NULL, DOWN_FS);
    G.dTexel = GetShaderLocation(G.down, "texel"); G.dPre = GetShaderLocation(G.down, "prefilter");
    G.dThr   = GetShaderLocation(G.down, "threshold");
    SetShaderValue(G.down, G.dThr, &thr, SHADER_UNIFORM_FLOAT);
    G.up    = gfx_shader(NULL, UP_FS);     G.uTexel = GetShaderLocation(G.up, "texel");
    G.comp  = gfx_shader(NULL, COMP_FS);
    G.cBloom = GetShaderLocation(G.comp, "bloomTex"); G.cExp = GetShaderLocation(G.comp, "exposure");
    G.cBloomStr = GetShaderLocation(G.comp, "bloomStr"); G.cVig = GetShaderLocation(G.comp, "vignette");
    G.cAb = GetShaderLocation(G.comp, "aberration");
    G.fxaa  = gfx_shader(NULL, FXAA_FS);   G.fTexel = GetShaderLocation(G.fxaa, "texel");

    G.skyMesh = GenMeshSphere(900.0f, 24, 48);
    G.skyMat = LoadMaterialDefault();
    G.skyMat.shader = G.sky;

    /* shadow maps: depth-only FBOs sampled with hardware depth compare + bilinear (PCF) */
    G.shadow  = gfx_make_shadow(SHADOW_RES);
    G.shadowN = gfx_make_shadow(SHADOW_NEAR_RES);

    /* far cascade: fitted once to the whole arena; the near one follows the player (gfx_shadow_begin) */
    G.lightView = MatrixLookAt(Vector3Subtract(ctr, Vector3Scale(sun, 400.0f)), ctr, V3(0, 1, 0));
    for (k = 0; k < 8; k++) {
        Vector3 p = V3(k & 1 ? ARENA_W + 4 : -ARENA_W - 4, k & 2 ? ARENA_H + 2 : -2.0f,
                       k & 4 ? ARENA_L + GOAL_D + 4 : -ARENA_L - GOAL_D - 4);
        p = Vector3Transform(p, G.lightView);
        lo = Vector3Min(lo, p); hi = Vector3Max(hi, p);
    }
    G.zNear = -hi.z - 5.0f; G.zFar = -lo.z + 5.0f;
    G.lightProj = MatrixOrtho(lo.x, hi.x, lo.y, hi.y, G.zNear, G.zFar);
    G.lightProjN = G.lightProj;
    G.depthRange = (hi.z - lo.z) + 10.0f;
    {
        Matrix vp = MatrixMultiply(G.lightView, G.lightProj);
        float biasN = 0.025f / G.depthRange, texelN = 1.0f / SHADOW_NEAR_RES;
        int slotN = 11;
        SetShaderValueMatrix(G.lit, G.lLightVP, vp);
        SetShaderValueMatrix(G.lit, G.lLightVPN, vp);
        SetShaderValue(G.lit, GetShaderLocation(G.lit, "shadowNear"), &slotN, SHADER_UNIFORM_INT);
        SetShaderValue(G.lit, GetShaderLocation(G.lit, "shadowBiasN"), &biasN, SHADER_UNIFORM_FLOAT);
        SetShaderValue(G.lit, GetShaderLocation(G.lit, "shadowTexelN"), &texelN, SHADER_UNIFORM_FLOAT);
    }
    bias = 0.06f / G.depthRange;   /* ~6 cm in [0,1] depth units */
    SetShaderValue(G.lit, GetShaderLocation(G.lit, "shadowBias"), &bias, SHADER_UNIFORM_FLOAT);
    TraceLog(LOG_INFO, "GFX: shadows: far %.0f x %.0f m (%.1f cm/texel), near %.0f m (%.1f cm/texel)", hi.x - lo.x, hi.y - lo.y,
             100.0f * (hi.x - lo.x) / SHADOW_RES, SHADOW_NEAR_SIZE, 100.0f * SHADOW_NEAR_SIZE / SHADOW_NEAR_RES);
    gfx_resize();
}

static void gfx_unload(void)
{
    gfx_free_targets();
    rlUnloadFramebuffer(G.shadow.id);
    rlUnloadTexture(G.shadow.texture.id);
    rlUnloadFramebuffer(G.shadowN.id);
    rlUnloadTexture(G.shadowN.texture.id);
    UnloadMesh(G.skyMesh);
    MemFree(G.skyMat.maps);
    UnloadShader(G.lit); UnloadShader(G.depth); UnloadShader(G.sky); UnloadShader(G.emis);
    UnloadShader(G.down); UnloadShader(G.up); UnloadShader(G.comp); UnloadShader(G.fxaa);
}

/* per-draw material response: specular strength, shininess, sky reflection, emissive */
static void gfx_mat(float spec, float shin, float refl, float emis)
{
    float v[4] = { spec, shin, refl, emis };
    SetShaderValue(G.lit, G.lMat, v, SHADER_UNIFORM_VEC4);
}

static void gfx_light(Vector3 p, float radius, Vector3 col)
{
    if (G.ptCount >= MAX_PT_LIGHTS) return;
    G.ptPos[G.ptCount] = (Vector4){ p.x, p.y, p.z, radius };
    G.ptCol[G.ptCount] = (Vector4){ col.x, col.y, col.z, 0.0f };
    G.ptCount++;
}

static void gfx_emissive_begin(float intensity)
{
    SetShaderValue(G.emis, G.eInt, &intensity, SHADER_UNIFORM_FLOAT);
    BeginShaderMode(G.emis);
}

/* shadow pass for one cascade (0 = whole arena, 1 = sharp map around `focus`);
 * caller draws casters with G.depth as the override shader */
static void gfx_shadow_begin(int cascade, Vector3 focus)
{
    rlActiveTextureSlot(10); rlDisableTexture();          /* don't sample what we write */
    rlActiveTextureSlot(11); rlDisableTexture(); rlActiveTextureSlot(0);
    if (cascade == 1) {
        /* centre in light space, snapped to whole texels so edges don't crawl as the player moves */
        Vector3 c = Vector3Transform(focus, G.lightView);
        float h = SHADOW_NEAR_SIZE * 0.5f, tx = SHADOW_NEAR_SIZE / SHADOW_NEAR_RES;
        Matrix vp;
        c.x = floorf(c.x / tx) * tx; c.y = floorf(c.y / tx) * tx;
        G.lightProjN = MatrixOrtho(c.x - h, c.x + h, c.y - h, c.y + h, G.zNear, G.zFar);   /* same depth range = same bias units */
        vp = MatrixMultiply(G.lightView, G.lightProjN);
        SetShaderValueMatrix(G.lit, G.lLightVPN, vp);
    }
    BeginTextureMode(cascade == 1 ? G.shadowN : G.shadow);
    rlClearScreenBuffers();
    rlSetMatrixProjection(cascade == 1 ? G.lightProjN : G.lightProj);
    rlSetMatrixModelview(G.lightView);
    rlEnableDepthTest();
}

static void gfx_shadow_end(void)
{
    rlDisableDepthTest();
    EndTextureMode();
}

static void gfx_scene_begin(Camera3D cam, int shadows)
{
    int on = shadows;
    SetShaderValue(G.lit, G.lShadowOn, &on, SHADER_UNIFORM_INT);
    SetShaderValue(G.lit, G.lView, &cam.position, SHADER_UNIFORM_VEC3);
    SetShaderValue(G.lit, G.lPtCount, &G.ptCount, SHADER_UNIFORM_INT);
    if (G.ptCount > 0) {
        SetShaderValueV(G.lit, G.lPtPos, G.ptPos, SHADER_UNIFORM_VEC4, G.ptCount);
        SetShaderValueV(G.lit, G.lPtCol, G.ptCol, SHADER_UNIFORM_VEC4, G.ptCount);
    }
    rlActiveTextureSlot(10); rlEnableTexture(G.shadow.texture.id);
    rlActiveTextureSlot(11); rlEnableTexture(G.shadowN.texture.id); rlActiveTextureSlot(0);
    BeginTextureMode(G.hdr);
    ClearBackground(BLACK);
    BeginMode3D(cam);
    rlDisableDepthMask(); rlDisableBackfaceCulling();
    DrawMesh(G.skyMesh, G.skyMat, MatrixTranslate(cam.position.x, cam.position.y, cam.position.z));
    rlEnableDepthMask(); rlEnableBackfaceCulling();
}

static void gfx_scene_end(void)
{
    EndMode3D();
    EndTextureMode();
}

/* full-screen pass; extraLoc >= 0 binds a second sampler (must happen after the shader switch,
 * which flushes the batch and resets the extra texture slots) */
static void gfx_blit2(RenderTexture2D *dst, Shader sh, Texture2D src, int extraLoc, Texture2D extra)
{
    if (dst) BeginTextureMode(*dst);
    rlDisableColorBlend();                       /* straight copy: alpha carries data (FXAA luma) */
    BeginShaderMode(sh);
    if (extraLoc >= 0) SetShaderValueTexture(sh, extraLoc, extra);
    DrawTexturePro(src, (Rectangle){ 0, 0, (float)src.width, -(float)src.height },
                   (Rectangle){ 0, 0, (float)(dst ? dst->texture.width : GetScreenWidth()),
                                      (float)(dst ? dst->texture.height : GetScreenHeight()) },
                   (Vector2){ 0, 0 }, 0.0f, WHITE);
    EndShaderMode();                             /* flushes the batch before blending comes back */
    rlEnableColorBlend();
    if (dst) EndTextureMode();
}

static void gfx_blit(RenderTexture2D *dst, Shader sh, Texture2D src)
{
    Texture2D none = { 0 };
    gfx_blit2(dst, sh, src, -1, none);
}

/* HDR -> bloom -> tonemapped LDR target; boost (0..1) adds radial motion streak */
static void gfx_post(int bloomOn, float boost)
{
    float exposure = 1.0f, bloomStr = bloomOn ? 0.25f : 0.0f, vig = 0.04f, ab = 0.022f * boost;
    int i;
    if (bloomOn) {
        for (i = 0; i < BLOOM_MIPS; i++) {
            Texture2D src = i == 0 ? G.hdr.texture : G.bloom[i - 1].texture;
            Vector2 tx = { 1.0f / src.width, 1.0f / src.height };
            int pre = i == 0;
            SetShaderValue(G.down, G.dTexel, &tx, SHADER_UNIFORM_VEC2);
            SetShaderValue(G.down, G.dPre, &pre, SHADER_UNIFORM_INT);
            gfx_blit(&G.bloom[i], G.down, src);
        }
        for (i = BLOOM_MIPS - 1; i > 0; i--) {
            Vector2 tx = { 1.0f / G.bloom[i].texture.width, 1.0f / G.bloom[i].texture.height };
            SetShaderValue(G.up, G.uTexel, &tx, SHADER_UNIFORM_VEC2);
            BeginTextureMode(G.bloom[i - 1]);
            BeginBlendMode(BLEND_ADDITIVE);
            BeginShaderMode(G.up);
            DrawTexturePro(G.bloom[i].texture, (Rectangle){ 0, 0, (float)G.bloom[i].texture.width, -(float)G.bloom[i].texture.height },
                           (Rectangle){ 0, 0, (float)G.bloom[i - 1].texture.width, (float)G.bloom[i - 1].texture.height },
                           (Vector2){ 0, 0 }, 0.0f, WHITE);
            EndShaderMode();
            EndBlendMode();
            EndTextureMode();
        }
    }
    SetShaderValue(G.comp, G.cExp, &exposure, SHADER_UNIFORM_FLOAT);
    SetShaderValue(G.comp, G.cBloomStr, &bloomStr, SHADER_UNIFORM_FLOAT);
    SetShaderValue(G.comp, G.cVig, &vig, SHADER_UNIFORM_FLOAT);
    SetShaderValue(G.comp, G.cAb, &ab, SHADER_UNIFORM_FLOAT);
    gfx_blit2(&G.ldr, G.comp, G.hdr.texture, G.cBloom, G.bloom[0].texture);
}

/* LDR target -> backbuffer through FXAA (call inside BeginDrawing) */
static void gfx_present(void)
{
    Vector2 tx = { 1.0f / G.w, 1.0f / G.h };
    SetShaderValue(G.fxaa, G.fTexel, &tx, SHADER_UNIFORM_VEC2);
    gfx_blit(NULL, G.fxaa, G.ldr.texture);
}

static Texture2D load_tga_texture(const char *path, int opaque)
{
    Texture2D t = { 0 };
    int w, h;
    unsigned char *px = sarm_load_tga(path, &w, &h);
    if (px) {
        Image img = { px, w, h, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
        if (opaque) { int k; for (k = 0; k < w * h; k++) px[k * 4 + 3] = 255; }
        t = LoadTextureFromImage(img);
        GenTextureMipmaps(&t);
        SetTextureFilter(t, TEXTURE_FILTER_TRILINEAR);
        free(px);
    } else {
        TraceLog(LOG_WARNING, "could not load %s", path);
    }
    return t;
}

/* de-indexed raylib mesh from a list of SARM indices, positions relative to origin */
static Mesh mesh_from_indices(const SarmModel *m, const uint32_t *idx, int count, Vector3 origin)
{
    Mesh mesh = { 0 };
    int i;
    mesh.vertexCount   = count;
    mesh.triangleCount = count / 3;
    mesh.vertices  = (float *)MemAlloc(sizeof(float) * 3 * count);
    mesh.normals   = (float *)MemAlloc(sizeof(float) * 3 * count);
    mesh.texcoords = (float *)MemAlloc(sizeof(float) * 2 * count);
    for (i = 0; i < count; i++) {
        const SarmVertex *v = &m->vertices[idx[i]];
        mesh.vertices[i*3+0] = v->pos[0] - origin.x;
        mesh.vertices[i*3+1] = v->pos[1] - origin.y;
        mesh.vertices[i*3+2] = v->pos[2] - origin.z;
        memcpy(&mesh.normals[i*3], v->normal, sizeof(float) * 3);
        memcpy(&mesh.texcoords[i*2], v->uv, sizeof(float) * 2);
    }
    UploadMesh(&mesh, false);
    return mesh;
}

static int nearest_wheel(const SarmModel *m, const float *p)
{
    int w, best = 0;
    float bd = 1e9f;
    for (w = 0; w < 4; w++) {
        float dx = p[0] - m->header.wheel_pos[w][0], dy = p[1] - m->header.wheel_pos[w][1], dz = p[2] - m->header.wheel_pos[w][2];
        float d = dx*dx + dy*dy + dz*dz;
        if (d < bd) { bd = d; best = w; }
    }
    return best;
}

static int car_render_load(CarRender *cr, const SarmModel *m, const char *dir, Shader lit)
{
    uint32_t s, i;
    int w, k;
    char path[1024];
    Texture2D tTire;
    float bmin[4][3], bmax[4][3], R[4], halfW[4];
    uint32_t *wl[4][3];
    int wn[4][3];
    memset(cr, 0, sizeof(*cr));

    /* 1. wheel extents from the tyre geometry (each tyre vertex -> nearest hub) */
    for (w = 0; w < 4; w++) for (k = 0; k < 3; k++) { bmin[w][k] = 1e9f; bmax[w][k] = -1e9f; }
    for (s = 0; s < m->header.submesh_count; s++) {
        const SarmSubmesh *sm = &m->submeshes[s];
        if (sm->material != SARM_MAT_TIRE) continue;
        for (i = 0; i < sm->index_count; i++) {
            const float *p = m->vertices[m->indices[sm->first_index + i]].pos;
            w = nearest_wheel(m, p);
            for (k = 0; k < 3; k++) { bmin[w][k] = fminf(bmin[w][k], p[k]); bmax[w][k] = fmaxf(bmax[w][k], p[k]); }
        }
    }
    for (w = 0; w < 4; w++) {
        if (bmax[w][0] < bmin[w][0]) { cr->wanim[w] = 0; R[w] = 0; continue; }
        cr->wanim[w]  = 1;
        cr->whub[w]   = V3(0.5f * (bmin[w][0] + bmax[w][0]), 0.5f * (bmin[w][1] + bmax[w][1]), 0.5f * (bmin[w][2] + bmax[w][2]));
        R[w]          = 0.5f * fmaxf(bmax[w][0] - bmin[w][0], bmax[w][1] - bmin[w][1]);
        halfW[w]      = 0.5f * (bmax[w][2] - bmin[w][2]);
        cr->wfront[w] = cr->whub[w].x > 0.0f;
    }

    /* 2. split triangles: tyres + anything fully inside a wheel cylinder (rims, hubs) */
    for (w = 0; w < 4; w++) for (k = 0; k < 3; k++) {
        wl[w][k] = (uint32_t *)MemAlloc(sizeof(uint32_t) * m->header.index_count);
        wn[w][k] = 0;
    }
    for (s = 0; s < m->header.submesh_count && cr->count < 16; s++) {
        const SarmSubmesh *sm = &m->submeshes[s];
        uint32_t *body = (uint32_t *)MemAlloc(sizeof(uint32_t) * (sm->index_count + 3));
        int nb = 0, mat = (int)sm->material;
        if (mat < 0 || mat > 2) mat = SARM_MAT_BODY;
        for (i = 0; i + 2 < sm->index_count; i += 3) {
            const uint32_t *tri = &m->indices[sm->first_index + i];
            int dest = -1, v;
            if (mat == SARM_MAT_TIRE) {
                float c[3];
                for (k = 0; k < 3; k++) c[k] = (m->vertices[tri[0]].pos[k] + m->vertices[tri[1]].pos[k] + m->vertices[tri[2]].pos[k]) / 3.0f;
                dest = nearest_wheel(m, c);
            } else {
                for (w = 0; w < 4 && dest < 0; w++) {
                    int inside = cr->wanim[w];
                    for (v = 0; v < 3 && inside; v++) {
                        const float *p = m->vertices[tri[v]].pos;
                        float dx = p[0] - cr->whub[w].x, dy = p[1] - cr->whub[w].y, dz = p[2] - cr->whub[w].z;
                        inside = dx*dx + dy*dy < R[w] * R[w] * 0.96f && fabsf(dz) < halfW[w] * 1.15f;
                    }
                    if (inside) dest = w;
                }
            }
            if (dest >= 0 && cr->wanim[dest]) {
                for (v = 0; v < 3; v++) wl[dest][mat][wn[dest][mat]++] = tri[v];
            } else {
                for (v = 0; v < 3; v++) body[nb++] = tri[v];
            }
        }
        if (nb > 0) {
            cr->mesh[cr->count] = mesh_from_indices(m, body, nb, V3(0, 0, 0));
            cr->material[cr->count] = mat;
            cr->count++;
        }
        MemFree(body);
    }
    for (w = 0; w < 4; w++) for (k = 0; k < 3; k++) {
        if (wn[w][k] > 0) { cr->wmesh[w][k] = mesh_from_indices(m, wl[w][k], wn[w][k], cr->whub[w]); cr->wvalid[w][k] = 1; }
        MemFree(wl[w][k]);
    }

    snprintf(path, sizeof(path), "%sbody_blue.tga", dir);    cr->texBlue   = load_tga_texture(path, 0);
    snprintf(path, sizeof(path), "%sbody_orange.tga", dir);  cr->texOrange = load_tga_texture(path, 0);
    snprintf(path, sizeof(path), "%sbody_custom.tga", dir);  cr->texCustom = load_tga_texture(path, 0);
    snprintf(path, sizeof(path), "%stire_diffuse.tga", dir); tTire = load_tga_texture(path, 1);   /* alpha = rim mask, not opacity */

    cr->matBody  = LoadMaterialDefault(); cr->matBody.shader  = lit;
    cr->matGlass = LoadMaterialDefault(); cr->matGlass.shader = lit;
    cr->matTire  = LoadMaterialDefault(); cr->matTire.shader  = lit;
    if (cr->texBlue.id) cr->matBody.maps[MATERIAL_MAP_DIFFUSE].texture = cr->texBlue;
    if (tTire.id) cr->matTire.maps[MATERIAL_MAP_DIFFUSE].texture = tTire;
    cr->matGlass.maps[MATERIAL_MAP_DIFFUSE].color = (Color){ 15, 23, 41, 153 };
    return cr->count > 0;
}

/* team 0 = blue livery, 1 = orange; ovr = depth shader for the shadow pass (NULL = lit) */
static void car_mat(const CarRender *cr, int kind, const Shader *ovr, Material *out)
{
    *out = kind == SARM_MAT_TIRE ? cr->matTire : kind == SARM_MAT_GLASS ? cr->matGlass : cr->matBody;
    if (ovr) { out->shader = *ovr; return; }
    if (kind == SARM_MAT_TIRE)       gfx_mat(0.12f, 12.0f, 0.04f, 0.0f);
    else if (kind == SARM_MAT_GLASS) gfx_mat(0.90f, 160.0f, 0.70f, 0.0f);
    else                             gfx_mat(0.60f, 60.0f, 0.28f, 0.0f);   /* glossy car paint */
}

static void car_render_draw(CarRender *cr, const Car *c, int team, int skin, const Shader *ovr)
{
    Matrix xf = MatrixMultiply(MatrixMultiply(MatrixScale(CAR_SCALE, CAR_SCALE, CAR_SCALE), QuaternionToMatrix(c->rot)),
                               MatrixTranslate(c->pos.x, c->pos.y, c->pos.z));
    int pass, i, w;
    Material m;
    Texture2D body = cr->texBlue;
    if (skin == 1 && cr->texCustom.id)        body = cr->texCustom;
    else if (team == 1 && cr->texOrange.id)   body = cr->texOrange;
    if (body.id) cr->matBody.maps[MATERIAL_MAP_DIFFUSE].texture = body;
    rlDisableBackfaceCulling();                  /* thin fender/flap planes are single-sided in the source meshes */
    for (pass = 0; pass < 2; pass++) {          /* opaque first, glass second */
        for (i = 0; i < cr->count; i++) {
            int glass = cr->material[i] == SARM_MAT_GLASS;
            if (glass != pass) continue;
            car_mat(cr, cr->material[i], ovr, &m);
            DrawMesh(cr->mesh[i], m, xf);
        }
        /* wheels: roll, steer (fronts), then follow the suspension */
        for (w = 0; w < 4; w++) {
            Matrix wx;
            float steer = cr->wfront[w] ? c->steerAngle * DEG2RAD : 0.0f;
            if (!cr->wanim[w]) continue;
            wx = MatrixMultiply(MatrixRotateZ(c->wheelSpin), MatrixRotateY(steer));
            wx = MatrixMultiply(wx, MatrixTranslate(cr->whub[w].x, cr->whub[w].y + c->wheelOfs[w] / CAR_SCALE, cr->whub[w].z));
            wx = MatrixMultiply(wx, xf);
            for (i = 0; i < 3; i++) {
                if (!cr->wvalid[w][i] || (i == SARM_MAT_GLASS) != pass) continue;
                car_mat(cr, i, ovr, &m);
                DrawMesh(cr->wmesh[w][i], m, wx);
            }
        }
    }
    rlEnableBackfaceCulling();
}

/* free GPU meshes/textures; materials only own their maps array (the shader is shared) */
static void car_render_unload(CarRender *cr)
{
    Material *ms[3];
    int i, k;
    ms[0] = &cr->matBody; ms[1] = &cr->matGlass; ms[2] = &cr->matTire;
    for (i = 0; i < cr->count; i++) UnloadMesh(cr->mesh[i]);
    for (i = 0; i < 4; i++) for (k = 0; k < 3; k++) if (cr->wvalid[i][k]) UnloadMesh(cr->wmesh[i][k]);
    if (cr->texBlue.id)   UnloadTexture(cr->texBlue);
    if (cr->texOrange.id) UnloadTexture(cr->texOrange);
    if (cr->texCustom.id) UnloadTexture(cr->texCustom);
    for (i = 0; i < 3; i++) {
        if (!ms[i]->maps) continue;
        if (i != 0 && ms[i]->maps[MATERIAL_MAP_DIFFUSE].texture.id != rlGetTextureIdDefault())
            UnloadTexture(ms[i]->maps[MATERIAL_MAP_DIFFUSE].texture);
        MemFree(ms[i]->maps);
    }
    memset(cr, 0, sizeof(*cr));
}

/* ---- arena visual mesh (export_c/arena/arena.sarm + arena_materials.txt) ---- */
typedef struct ArenaRender {
    Mesh     mesh[16];
    Material mat[16];
    int      translucent[16];
    float    par[16][4];
    int      count;
} ArenaRender;

static int arena_render_load(ArenaRender *ar, const char *dir, Shader lit)
{
    char path[1024], line[256], names[16][64], texs[16][64];
    int cols[16][4], nmat = 0;
    uint32_t s, i;
    SarmModel m;
    FILE *f;
    memset(ar, 0, sizeof(*ar));
    snprintf(path, sizeof(path), "%sarena_materials.txt", dir);
    f = fopen(path, "r");
    if (!f) return 0;
    while (nmat < 16 && fgets(line, sizeof(line), f))
        if (sscanf(line, "%63s %63s %d %d %d %d", names[nmat], texs[nmat],
                   &cols[nmat][0], &cols[nmat][1], &cols[nmat][2], &cols[nmat][3]) == 6) nmat++;
    fclose(f);
    snprintf(path, sizeof(path), "%sarena.sarm", dir);
    if (sarm_load(path, &m) != SARM_OK) return 0;

    for (s = 0; s < m.header.submesh_count && ar->count < 16; s++) {
        const SarmSubmesh *sm = &m.submeshes[s];
        Mesh mesh = { 0 };
        Material mat;
        int mi = (int)sm->material;
        if (sm->index_count == 0 || mi >= nmat) continue;
        mesh.vertexCount   = (int)sm->index_count;
        mesh.triangleCount = (int)sm->index_count / 3;
        mesh.vertices  = (float *)MemAlloc(sizeof(float) * 3 * sm->index_count);
        mesh.normals   = (float *)MemAlloc(sizeof(float) * 3 * sm->index_count);
        mesh.texcoords = (float *)MemAlloc(sizeof(float) * 2 * sm->index_count);
        for (i = 0; i < sm->index_count; i++) {
            const SarmVertex *v = &m.vertices[m.indices[sm->first_index + i]];
            memcpy(&mesh.vertices[i*3], v->pos, sizeof(float) * 3);
            memcpy(&mesh.normals[i*3], v->normal, sizeof(float) * 3);
            memcpy(&mesh.texcoords[i*2], v->uv, sizeof(float) * 2);
        }
        UploadMesh(&mesh, false);
        mat = LoadMaterialDefault();
        mat.shader = lit;
        mat.maps[MATERIAL_MAP_DIFFUSE].color = (Color){ (unsigned char)cols[mi][0], (unsigned char)cols[mi][1],
                                                        (unsigned char)cols[mi][2], (unsigned char)cols[mi][3] };
        if (strcmp(texs[mi], "-") != 0) {
            Texture2D t;
            snprintf(path, sizeof(path), "%s%s", dir, texs[mi]);
            t = load_tga_texture(path, 0);
            if (t.id) {
                SetTextureWrap(t, TEXTURE_WRAP_REPEAT);
                rlTextureParameters(t.id, RL_TEXTURE_FILTER_ANISOTROPIC, 8);
                mat.maps[MATERIAL_MAP_DIFFUSE].texture = t;
            }
        }
        ar->mesh[ar->count] = mesh;
        ar->mat[ar->count] = mat;
        ar->translucent[ar->count] = cols[mi][3] < 255;
        {   /* spec, shininess, sky reflection, emissive -- picked per surface type */
            const char *nm = names[mi];
            float p[4] = { 0.15f, 16.0f, 0.10f, 0.0f };
            if      (strstr(nm, "Pavement"))   { p[0] = 0.22f; p[1] = 24.0f;  p[2] = 0.18f; }
            else if (strstr(nm, "StreetLine")) { p[0] = 0.25f; p[1] = 24.0f;  p[2] = 0.18f; }
            else if (strstr(nm, "SideWalk"))   { p[0] = 0.12f; p[1] = 14.0f;  p[2] = 0.10f; }
            else if (strstr(nm, "GrayTiles"))  { p[0] = 0.30f; p[1] = 40.0f;  p[2] = 0.25f; }
            else if (strstr(nm, "Brick"))      { p[0] = 0.05f; p[1] = 8.0f;   p[2] = 0.02f; }
            else if (strstr(nm, "Metal"))      { p[0] = 0.60f; p[1] = 48.0f;  p[2] = 0.45f; }
            if (ar->translucent[ar->count])    { p[0] = 0.60f; p[1] = 200.0f; p[2] = 0.12f; }   /* dome: subtle, mostly see-through */
            memcpy(ar->par[ar->count], p, sizeof(p));
        }
        ar->count++;
    }
    TraceLog(LOG_INFO, "ARENA: %u render tris in %d materials", m.header.index_count / 3, ar->count);
    sarm_free(&m);
    return ar->count > 0;
}

/* ovr = depth shader for the shadow pass (glass doesn't cast) */
static void arena_render_draw(const ArenaRender *ar, int translucentPass, const Shader *ovr)
{
    int i;
    if (translucentPass) { rlDisableDepthMask(); rlDisableBackfaceCulling(); }
    for (i = 0; i < ar->count; i++) {
        Material m = ar->mat[i];
        if (ar->translucent[i] != translucentPass) continue;
        if (ovr) m.shader = *ovr;
        else     gfx_mat(ar->par[i][0], ar->par[i][1], ar->par[i][2], ar->par[i][3]);
        DrawMesh(ar->mesh[i], m, MatrixIdentity());
    }
    if (translucentPass) { rlEnableDepthMask(); rlEnableBackfaceCulling(); }
}

/* team-coloured "net" across each goal mouth so you can tell the ends apart */
static void draw_goal_nets(void)
{
    int s;
    rlDisableBackfaceCulling();
    rlDisableDepthMask();
    rlBegin(RL_QUADS);
    for (s = -1; s <= 1; s += 2) {
        float z = (ARENA_L + 1.0f) * s;
        Color c = s < 0 ? (Color){ 40, 110, 255, 60 } : (Color){ 255, 130, 30, 60 };
        rlColor4ub(c.r, c.g, c.b, c.a);
        rlVertex3f(-GOAL_HW, 0.05f, z); rlVertex3f(GOAL_HW, 0.05f, z);
        rlVertex3f(GOAL_HW, GOAL_H, z); rlVertex3f(-GOAL_HW, GOAL_H, z);
    }
    rlEnd();
    rlDrawRenderBatchActive();   /* state changes below don't flush the batch themselves */
    rlEnableDepthMask();
    rlEnableBackfaceCulling();
}

static Color shade(Color c, Vector3 n)
{
    float l = 0.5f + 0.5f * fmaxf(0.0f, Vector3DotProduct(n, Vector3Negate(Vector3Normalize(LIGHT_DIR))));
    return (Color){ (unsigned char)(c.r * l), (unsigned char)(c.g * l), (unsigned char)(c.b * l), c.a };
}

static void quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Color col)
{
    rlColor4ub(col.r, col.g, col.b, col.a);
    rlVertex3f(a.x, a.y, a.z); rlVertex3f(b.x, b.y, b.z);
    rlVertex3f(c.x, c.y, c.z); rlVertex3f(d.x, d.y, d.z);
}

static void draw_arena_shell(void)
{
    const float W = ARENA_W, L = ARENA_L, H = ARENA_H, C = RAMP_C, HW = GOAL_HW, GH = GOAL_H;
    const float S2 = 0.70710678f;
    Color ramp = { 70, 78, 92, 255 }, wall = { 52, 58, 72, 255 };
    int s;
    rlDisableBackfaceCulling();
    rlBegin(RL_QUADS);
    for (s = -1; s <= 1; s += 2) {
        float fs = (float)s;
        /* side ramp + wall (x = s*W) */
        quad(V3(fs*(W-C), 0, -(L-C)), V3(fs*(W-C), 0, L-C), V3(fs*W, C, L), V3(fs*W, C, -L),
             shade(ramp, V3(-fs*S2, S2, 0)));
        quad(V3(fs*W, C, -L), V3(fs*W, C, L), V3(fs*W, H, L), V3(fs*W, H, -L), shade(wall, V3(-fs, 0, 0)));
        /* end ramps, split around the goal mouth (z = s*L) */
        quad(V3(-(W-C), 0, fs*(L-C)), V3(-HW, 0, fs*(L-C)), V3(-HW, C, fs*L), V3(-W, C, fs*L),
             shade(ramp, V3(0, S2, -fs*S2)));
        quad(V3(HW, 0, fs*(L-C)), V3(W-C, 0, fs*(L-C)), V3(W, C, fs*L), V3(HW, C, fs*L),
             shade(ramp, V3(0, S2, -fs*S2)));
        quad(V3(-HW, 0, fs*(L-C)), V3(-HW, C, fs*L), V3(-HW, 0, fs*L), V3(-HW, 0, fs*L), shade(ramp, V3(1, 0, 0)));
        quad(V3( HW, 0, fs*(L-C)), V3( HW, C, fs*L), V3( HW, 0, fs*L), V3( HW, 0, fs*L), shade(ramp, V3(-1, 0, 0)));
        /* end wall pieces */
        quad(V3(-W, C, fs*L), V3(-HW, C, fs*L), V3(-HW, H, fs*L), V3(-W, H, fs*L), shade(wall, V3(0, 0, -fs)));
        quad(V3(HW, C, fs*L), V3(W, C, fs*L), V3(W, H, fs*L), V3(HW, H, fs*L), shade(wall, V3(0, 0, -fs)));
        quad(V3(-HW, GH, fs*L), V3(HW, GH, fs*L), V3(HW, H, fs*L), V3(-HW, H, fs*L), shade(wall, V3(0, 0, -fs)));
        quad(V3(-HW, C, fs*L), V3(-HW, GH, fs*L), V3(-HW, GH, fs*L), V3(-HW, C, fs*L), wall);
    }
    rlEnd();

    /* wall grid lines for depth perception */
    rlBegin(RL_LINES);
    rlColor4ub(90, 100, 120, 255);
    {
        float z, y;
        for (s = -1; s <= 1; s += 2) {
            for (z = -L; z <= L + 0.1f; z += 16.0f) { rlVertex3f(s*W, C, z); rlVertex3f(s*W, H, z); }
            for (y = C; y <= H + 0.1f; y += 10.0f) { rlVertex3f(s*W, y, -L); rlVertex3f(s*W, y, L); }
        }
    }
    rlEnd();
    rlDrawRenderBatchActive();   /* culling state changes don't flush the batch themselves */
    rlEnableBackfaceCulling();
}

static void draw_goals(void)
{
    const float L = ARENA_L, HW = GOAL_HW, GH = GOAL_H, D = GOAL_D;
    int s;
    for (s = -1; s <= 1; s += 2) {
        float fs = (float)s;
        Color team = s < 0 ? (Color){ 40, 110, 255, 255 } : (Color){ 255, 130, 30, 255 };
        Color glow = team; glow.a = 70;
        DrawCylinderEx(V3(-HW, 0, fs*L), V3(-HW, GH, fs*L), POST_R, POST_R, 10, RAYWHITE);
        DrawCylinderEx(V3( HW, 0, fs*L), V3( HW, GH, fs*L), POST_R, POST_R, 10, RAYWHITE);
        DrawCylinderEx(V3(-HW, GH, fs*L), V3(HW, GH, fs*L), POST_R, POST_R, 10, RAYWHITE);
        rlDisableBackfaceCulling();
        rlDisableDepthMask();
        rlBegin(RL_QUADS);
        quad(V3(-HW, 0, fs*(L+D)), V3(HW, 0, fs*(L+D)), V3(HW, GH, fs*(L+D)), V3(-HW, GH, fs*(L+D)), glow);
        quad(V3(-HW, 0, fs*L), V3(-HW, 0, fs*(L+D)), V3(-HW, GH, fs*(L+D)), V3(-HW, GH, fs*L), glow);
        quad(V3( HW, 0, fs*L), V3( HW, 0, fs*(L+D)), V3( HW, GH, fs*(L+D)), V3( HW, GH, fs*L), glow);
        quad(V3(-HW, GH, fs*L), V3(HW, GH, fs*L), V3(HW, GH, fs*(L+D)), V3(-HW, GH, fs*(L+D)), glow);
        rlEnd();
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
        rlEnableBackfaceCulling();
    }
}

static Texture2D make_field_texture(void)
{
    const float W = ARENA_W, LT = ARENA_L + GOAL_D;
    const int iw = 1024, ih = (int)(1024.0f * LT / W);
    Image img = GenImageColor(iw, ih, (Color){ 46, 110, 52, 255 });
    float ppm = iw / (2.0f * W);                       /* pixels per metre */
    int i, lw = (int)fmaxf(2.0f, 0.5f * ppm);
    Color line = (Color){ 235, 235, 235, 255 };
#define PX(x) ((int)(((x) + W) * ppm))
#define PZ(z) ((int)(((z) + LT) * ppm))
    for (i = 0; i < 18; i++)                           /* mowing stripes */
        if (i & 1) ImageDrawRectangle(&img, 0, PZ(-ARENA_L + i * (2*ARENA_L/18)), iw,
                                      (int)(2*ARENA_L/18 * ppm), (Color){ 52, 122, 58, 255 });
    ImageDrawRectangle(&img, 0, PZ(-ARENA_L - GOAL_D), iw, (int)(GOAL_D * ppm), (Color){ 60, 60, 66, 255 });
    ImageDrawRectangle(&img, 0, PZ(ARENA_L), iw, (int)(GOAL_D * ppm) + 2, (Color){ 60, 60, 66, 255 });
    ImageDrawRectangle(&img, 0, PZ(0) - lw/2, iw, lw, line);                         /* halfway   */
    for (i = 0; i < lw; i++) ImageDrawCircleLines(&img, PX(0), PZ(0), (int)(18 * ppm) - i, line);
    ImageDrawRectangle(&img, PX(-GOAL_HW*2), PZ(-ARENA_L), lw, (int)(24*ppm), line); /* boxes */
    ImageDrawRectangle(&img, PX( GOAL_HW*2), PZ(-ARENA_L), lw, (int)(24*ppm), line);
    ImageDrawRectangle(&img, PX(-GOAL_HW*2), PZ(-ARENA_L + 24), (int)(GOAL_HW*4*ppm) + lw, lw, line);
    ImageDrawRectangle(&img, PX(-GOAL_HW*2), PZ(ARENA_L - 24), lw, (int)(24*ppm), line);
    ImageDrawRectangle(&img, PX( GOAL_HW*2), PZ(ARENA_L - 24), lw, (int)(24*ppm), line);
    ImageDrawRectangle(&img, PX(-GOAL_HW*2), PZ(ARENA_L - 24), (int)(GOAL_HW*4*ppm) + lw, lw, line);
    ImageDrawRectangle(&img, PX(-GOAL_HW), PZ(-ARENA_L) - lw/2, (int)(GOAL_HW*2*ppm), lw, (Color){ 80, 150, 255, 255 });
    ImageDrawRectangle(&img, PX(-GOAL_HW), PZ( ARENA_L) - lw/2, (int)(GOAL_HW*2*ppm), lw, (Color){ 255, 150, 60, 255 });
#undef PX
#undef PZ
    {
        Texture2D t = LoadTextureFromImage(img);
        GenTextureMipmaps(&t);
        SetTextureFilter(t, TEXTURE_FILTER_TRILINEAR);
        UnloadImage(img);
        return t;
    }
}

/* ------------------------------------------------------------------------ */
/* 3D Visual Particle System                                                */
/* ------------------------------------------------------------------------ */
#define MAX_PARTICLES 2048

typedef struct {
    Vector3 pos;
    Vector3 vel;
    Color   colorStart;
    Color   colorEnd;
    float   sizeStart;
    float   sizeEnd;
    float   life;
    float   maxLife;
    float   drag;
    float   gravity;
    int     additive;    /* 1 = additive glow/bloom, 0 = alpha smoke */
    int     active;
} Particle;

static Particle  g_particles[MAX_PARTICLES];
static int       g_particleHead = 0;
static Texture2D g_particleTex;

static float p_randf(void)
{
    return (float)rand() / (float)RAND_MAX;
}

static float p_range(float min, float max)
{
    return min + p_randf() * (max - min);
}

static Vector3 p_randv3(float min, float max)
{
    return V3(p_range(min, max), p_range(min, max), p_range(min, max));
}

static Vector3 p_rand_dir(void)
{
    float z = p_range(-1.0f, 1.0f);
    float a = p_range(0.0f, 2.0f * PI);
    float r = sqrtf(fmaxf(0.0f, 1.0f - z * z));
    return V3(r * cosf(a), z, r * sinf(a));
}

static Texture2D make_particle_texture(void)
{
    int size = 64;
    Image img = GenImageColor(size, size, BLANK);
    Color *pixels = (Color *)img.data;
    float center = (size - 1) * 0.5f;
    float radius = center;
    int y, x;
    for (y = 0; y < size; y++) {
        for (x = 0; x < size; x++) {
            float dx = (x - center) / radius;
            float dy = (y - center) / radius;
            float r2 = dx * dx + dy * dy;
            if (r2 < 1.0f) {
                float a = expf(-r2 * 3.5f) * (1.0f - r2);
                unsigned char val = (unsigned char)clampf(a * 255.0f, 0.0f, 255.0f);
                pixels[y * size + x] = (Color){ 255, 255, 255, val };
            } else {
                pixels[y * size + x] = (Color){ 0, 0, 0, 0 };
            }
        }
    }
    {
        Texture2D tex = LoadTextureFromImage(img);
        GenTextureMipmaps(&tex);
        SetTextureFilter(tex, TEXTURE_FILTER_BILINEAR);
        UnloadImage(img);
        return tex;
    }
}

static Particle *particle_spawn(Vector3 pos, Vector3 vel, Color c0, Color c1,
                                float s0, float s1, float life, float drag,
                                float gravity, int additive)
{
    int i;
    for (i = 0; i < MAX_PARTICLES; i++) {
        int idx = (g_particleHead + i) % MAX_PARTICLES;
        if (!g_particles[idx].active) {
            g_particleHead = (idx + 1) % MAX_PARTICLES;
            Particle *p = &g_particles[idx];
            p->pos = pos;
            p->vel = vel;
            p->colorStart = c0;
            p->colorEnd = c1;
            p->sizeStart = s0;
            p->sizeEnd = s1;
            p->life = 0.0f;
            p->maxLife = life;
            p->drag = drag;
            p->gravity = gravity;
            p->additive = additive;
            p->active = 1;
            return p;
        }
    }
    /* Pool full: reuse oldest */
    Particle *p = &g_particles[g_particleHead];
    g_particleHead = (g_particleHead + 1) % MAX_PARTICLES;
    p->pos = pos;
    p->vel = vel;
    p->colorStart = c0;
    p->colorEnd = c1;
    p->sizeStart = s0;
    p->sizeEnd = s1;
    p->life = 0.0f;
    p->maxLife = life;
    p->drag = drag;
    p->gravity = gravity;
    p->additive = additive;
    p->active = 1;
    return p;
}

static void particles_update(float dt)
{
    int i;
    for (i = 0; i < MAX_PARTICLES; i++) {
        Particle *p = &g_particles[i];
        if (!p->active) continue;

        p->life += dt;
        if (p->life >= p->maxLife) {
            p->active = 0;
            continue;
        }

        p->vel.y -= p->gravity * dt;
        float d = 1.0f - p->drag * dt;
        if (d < 0.0f) d = 0.0f;
        p->vel = Vector3Scale(p->vel, d);
        p->pos = Vector3Add(p->pos, Vector3Scale(p->vel, dt));

        /* Pitch collision bounce for sparks & debris */
        if (p->pos.y < 0.05f) {
            p->pos.y = 0.05f;
            p->vel.y = -p->vel.y * 0.35f;
            p->vel.x *= 0.70f;
            p->vel.z *= 0.70f;
        }
    }
}

static void particles_draw_pass(Camera3D cam, Texture2D tex, int passAdditive)
{
    int i;
    Vector3 fwd, camUp, camRight;
    float hs, r, g, b, a, t, sz;
    Color col;

    if (!tex.id) return;

    fwd = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
    camUp = cam.up;
    camRight = Vector3Normalize(Vector3CrossProduct(fwd, camUp));
    camUp = Vector3CrossProduct(camRight, fwd);

    rlSetTexture(tex.id);
    rlBegin(RL_QUADS);

    for (i = 0; i < MAX_PARTICLES; i++) {
        Particle *p = &g_particles[i];
        if (!p->active || p->additive != passAdditive) continue;

        t = p->life / p->maxLife;
        sz = Lerp(p->sizeStart, p->sizeEnd, t);
        hs = sz * 0.5f;

        r = Lerp((float)p->colorStart.r, (float)p->colorEnd.r, t);
        g = Lerp((float)p->colorStart.g, (float)p->colorEnd.g, t);
        b = Lerp((float)p->colorStart.b, (float)p->colorEnd.b, t);
        a = Lerp((float)p->colorStart.a, (float)p->colorEnd.a, t);
        if (a < 1.0f) continue;

        col = (Color){ (unsigned char)clampf(r, 0.0f, 255.0f),
                       (unsigned char)clampf(g, 0.0f, 255.0f),
                       (unsigned char)clampf(b, 0.0f, 255.0f),
                       (unsigned char)clampf(a, 0.0f, 255.0f) };

        Vector3 rx = Vector3Scale(camRight, hs);
        Vector3 uy = Vector3Scale(camUp, hs);

        Vector3 p0 = Vector3Subtract(Vector3Subtract(p->pos, rx), uy);
        Vector3 p1 = Vector3Subtract(Vector3Add(p->pos, rx), uy);
        Vector3 p2 = Vector3Add(Vector3Add(p->pos, rx), uy);
        Vector3 p3 = Vector3Add(Vector3Subtract(p->pos, rx), uy);

        rlColor4ub(col.r, col.g, col.b, col.a);
        rlTexCoord2f(0.0f, 0.0f); rlVertex3f(p0.x, p0.y, p0.z);
        rlTexCoord2f(1.0f, 0.0f); rlVertex3f(p1.x, p1.y, p1.z);
        rlTexCoord2f(1.0f, 1.0f); rlVertex3f(p2.x, p2.y, p2.z);
        rlTexCoord2f(0.0f, 1.0f); rlVertex3f(p3.x, p3.y, p3.z);
    }

    rlEnd();
    rlSetTexture(0);
}

/* Emitters */
static void particles_boost_emit(const Car *c, int isBlue)
{
    Vector3 back = Vector3Add(c->pos, car_to_world(c, V3(c->boxMin.x - 0.25f, 0.14f, 0.0f)));
    Vector3 bk = Vector3Scale(car_fwd(c), -1.0f);
    int k;

    /* Plume particles: expanding, slowing plasma balls */
    for (k = 0; k < 2; k++) {
        Vector3 p = Vector3Add(back, p_randv3(-0.05f, 0.05f));
        Vector3 vel = Vector3Add(Vector3Scale(c->vel, 0.35f),
                                Vector3Scale(bk, p_range(16.0f, 26.0f)));
        vel = Vector3Add(vel, p_randv3(-1.8f, 1.8f));

        Color c0 = isBlue ? (Color){ 130, 220, 255, 230 } : (Color){ 255, 210, 80, 240 };
        Color c1 = isBlue ? (Color){ 20, 70, 220, 0 }     : (Color){ 230, 45, 10, 0 };
        float s0 = p_range(0.20f, 0.32f);
        float s1 = p_range(0.50f, 0.85f);
        float life = p_range(0.16f, 0.28f);

        particle_spawn(p, vel, c0, c1, s0, s1, life, 3.8f, -0.8f, 1);
    }

    /* Boost sparks: high speed jittery incandescent embers */
    if (p_randf() < 0.65f) {
        Vector3 p = Vector3Add(back, p_randv3(-0.04f, 0.04f));
        Vector3 vel = Vector3Add(Vector3Scale(c->vel, 0.50f),
                                Vector3Scale(bk, p_range(22.0f, 38.0f)));
        vel = Vector3Add(vel, p_randv3(-4.5f, 4.5f));

        Color c0 = (Color){ 255, 255, 230, 255 };
        Color c1 = isBlue ? (Color){ 80, 180, 255, 0 } : (Color){ 255, 120, 25, 0 };
        float s0 = p_range(0.08f, 0.14f);
        float s1 = 0.02f;
        float life = p_range(0.22f, 0.42f);

        particle_spawn(p, vel, c0, c1, s0, s1, life, 1.8f, 12.0f, 1);
    }
}

static void particles_drift_emit(const Car *c, float latSpeed, int sliding)
{
    if (c->wheelsOnGround < 2) return;
    float speed = Vector3Length(c->vel);
    if (latSpeed < 2.5f && (!sliding || speed < 4.5f)) return;

    Vector3 wl = Vector3Add(c->pos, car_to_world(c, c->wheelLocal[2]));
    Vector3 wr = Vector3Add(c->pos, car_to_world(c, c->wheelLocal[3]));
    wl.y = fmaxf(0.06f, wl.y);
    wr.y = fmaxf(0.06f, wr.y);

    Vector3 wheels[2] = { wl, wr };
    int w;
    for (w = 0; w < 2; w++) {
        if (p_randf() < 0.75f) {
            Vector3 p = Vector3Add(wheels[w], p_randv3(-0.1f, 0.1f));
            Vector3 vel = Vector3Add(Vector3Scale(c->vel, 0.12f),
                                    V3(p_range(-0.5f, 0.5f), p_range(0.6f, 1.8f), p_range(-0.5f, 0.5f)));
            Color c0 = (Color){ 200, 205, 215, (unsigned char)clampf(latSpeed * 22.0f + 60.0f, 60.0f, 160.0f) };
            Color c1 = (Color){ 160, 165, 175, 0 };
            float s0 = p_range(0.25f, 0.40f);
            float s1 = p_range(0.85f, 1.35f);
            float life = p_range(0.35f, 0.65f);

            particle_spawn(p, vel, c0, c1, s0, s1, life, 2.2f, -1.2f, 0);
        }

        if (latSpeed > 5.5f && p_randf() < 0.5f) {
            Vector3 sp = wheels[w];
            Vector3 svel = Vector3Add(Vector3Scale(c->vel, 0.4f),
                                     V3(p_range(-3.5f, 3.5f), p_range(1.5f, 5.0f), p_range(-3.5f, 3.5f)));
            Color sc0 = (Color){ 255, 220, 80, 255 };
            Color sc1 = (Color){ 255, 60, 10, 0 };
            particle_spawn(sp, svel, sc0, sc1, 0.08f, 0.02f, p_range(0.15f, 0.30f), 1.5f, 14.0f, 1);
        }
    }
}

static void particles_supersonic_emit(const Car *c)
{
    float speed = Vector3Length(c->vel);
    if (speed < 38.0f) return;

    Vector3 wl = Vector3Add(c->pos, car_to_world(c, c->wheelLocal[2]));
    Vector3 wr = Vector3Add(c->pos, car_to_world(c, c->wheelLocal[3]));
    Vector3 wheels[2] = { wl, wr };
    int w;
    for (w = 0; w < 2; w++) {
        if (p_randf() < 0.8f) {
            Vector3 p = Vector3Add(wheels[w], p_randv3(-0.06f, 0.06f));
            Vector3 vel = Vector3Scale(c->vel, 0.08f);
            Color c0 = (Color){ 200, 245, 255, 210 };
            Color c1 = (Color){ 70, 150, 255, 0 };
            float s0 = p_range(0.35f, 0.50f);
            float s1 = 0.08f;
            particle_spawn(p, vel, c0, c1, s0, s1, 0.22f, 0.5f, 0.0f, 1);
        }
    }
}

static void particles_impact_burst(Vector3 pos, Vector3 normal, float relSpeed)
{
    int numSparks = (int)clampf(relSpeed * 3.0f, 16.0f, 50.0f);
    int k;

    /* Central shockwave flash */
    for (k = 0; k < 2; k++) {
        Vector3 p = Vector3Add(pos, p_randv3(-0.1f, 0.1f));
        Vector3 vel = Vector3Scale(normal, p_range(1.0f, 3.0f));
        Color c0 = (Color){ 255, 255, 255, 255 };
        Color c1 = (Color){ 255, 180, 80, 0 };
        particle_spawn(p, vel, c0, c1, 0.4f, 1.8f, 0.12f, 2.0f, 0.0f, 1);
    }

    /* High speed sparks */
    for (k = 0; k < numSparks; k++) {
        Vector3 dir = Vector3Normalize(Vector3Add(Vector3Scale(normal, p_range(0.6f, 1.4f)), p_rand_dir()));
        float spd = p_range(12.0f, 32.0f) * fminf(relSpeed / 15.0f, 1.6f);
        Vector3 vel = Vector3Scale(dir, spd);
        Color c0 = (Color){ 255, 245, 180, 255 };
        Color c1 = (Color){ 255, 90, 15, 0 };
        float s0 = p_range(0.10f, 0.18f);
        float s1 = 0.02f;
        float life = p_range(0.25f, 0.55f);
        particle_spawn(pos, vel, c0, c1, s0, s1, life, 2.2f, 13.0f, 1);
    }

    /* Dust puff */
    for (k = 0; k < 6; k++) {
        Vector3 vel = Vector3Add(Vector3Scale(normal, p_range(1.0f, 4.0f)), p_randv3(-2.0f, 2.0f));
        Color c0 = (Color){ 190, 195, 205, 120 };
        Color c1 = (Color){ 140, 145, 155, 0 };
        particle_spawn(pos, vel, c0, c1, 0.35f, 1.10f, 0.35f, 3.0f, -0.5f, 0);
    }
}

static void particles_pad_pickup(Vector3 padPos)
{
    int k;
    Vector3 center = V3(padPos.x, 0.25f, padPos.z);

    /* Shock ring bursting upward and outward */
    for (k = 0; k < 28; k++) {
        float angle = (float)k / 28.0f * 2.0f * PI + p_range(-0.1f, 0.1f);
        float spd = p_range(6.0f, 14.0f);
        Vector3 pos = Vector3Add(center, V3(cosf(angle) * 0.8f, 0.1f, sinf(angle) * 0.8f));
        Vector3 vel = V3(cosf(angle) * spd, p_range(5.0f, 12.0f), sinf(angle) * spd);

        Color c0 = (Color){ 255, 215, 60, 255 };
        Color c1 = (Color){ 255, 100, 15, 0 };
        float s0 = p_range(0.20f, 0.35f);
        float s1 = p_range(0.40f, 0.70f);
        float life = p_range(0.40f, 0.65f);
        particle_spawn(pos, vel, c0, c1, s0, s1, life, 2.5f, 6.0f, 1);
    }

    /* Rising central golden spark fountain */
    for (k = 0; k < 16; k++) {
        Vector3 p = Vector3Add(center, p_randv3(-0.4f, 0.4f));
        Vector3 vel = V3(p_range(-1.8f, 1.8f), p_range(12.0f, 22.0f), p_range(-1.8f, 1.8f));
        Color c0 = (Color){ 255, 255, 210, 255 };
        Color c1 = (Color){ 255, 140, 20, 0 };
        particle_spawn(p, vel, c0, c1, 0.16f, 0.03f, p_range(0.45f, 0.75f), 2.2f, 12.0f, 1);
    }
}

static void particles_goal_explosion(Vector3 goalPos, int scorerTeam)
{
    int k;
    int isBlue = (scorerTeam == 1);

    /* 1. Core expanding shockwave flash */
    for (k = 0; k < 6; k++) {
        Vector3 p = Vector3Add(goalPos, p_randv3(-0.4f, 0.4f));
        Color c0 = (Color){ 255, 255, 255, 255 };
        Color c1 = isBlue ? (Color){ 100, 200, 255, 0 } : (Color){ 255, 160, 35, 0 };
        particle_spawn(p, V3(0, 0, 0), c0, c1, 2.5f, 18.0f, 0.28f, 0.8f, 0.0f, 1);
    }

    /* 2. Expanding horizontal shockwave ring */
    for (k = 0; k < 28; k++) {
        float a = (float)k / 28.0f * 2.0f * PI;
        Vector3 dir = V3(cosf(a), p_range(-0.1f, 0.1f), sinf(a));
        float spd = p_range(28.0f, 48.0f);
        Vector3 p = Vector3Add(goalPos, Vector3Scale(dir, 1.2f));
        Vector3 vel = Vector3Scale(dir, spd);
        Color c0 = (Color){ 255, 255, 255, 240 };
        Color c1 = isBlue ? (Color){ 40, 140, 255, 0 } : (Color){ 255, 100, 15, 0 };
        particle_spawn(p, vel, c0, c1, 1.4f, 7.5f, 0.45f, 2.0f, 0.0f, 1);
    }

    /* 3. Massive expanding fireball / plasma clouds */
    for (k = 0; k < 90; k++) {
        Vector3 dir = p_rand_dir();
        if (goalPos.z > 0.0f) dir.z = -fabsf(dir.z) * 1.5f;
        else                  dir.z =  fabsf(dir.z) * 1.5f;
        dir = Vector3Normalize(dir);

        float spd = p_range(16.0f, 52.0f);
        Vector3 vel = Vector3Scale(dir, spd);
        Color c0 = isBlue ? (Color){ 190, 245, 255, 255 } : (Color){ 255, 235, 130, 255 };
        Color c1 = isBlue ? (Color){ 15, 50, 230, 0 }      : (Color){ 220, 25, 5, 0 };
        float s0 = p_range(2.0f, 3.8f);
        float s1 = p_range(9.0f, 16.0f);
        float life = p_range(0.70f, 1.45f);
        particle_spawn(goalPos, vel, c0, c1, s0, s1, life, 2.4f, -3.0f, 1);
    }

    /* 4. Blazing high-speed fireworks / incandescent sparks */
    for (k = 0; k < 160; k++) {
        Vector3 dir = p_rand_dir();
        if (goalPos.z > 0.0f) dir.z = -fabsf(dir.z) * 1.4f;
        else                  dir.z =  fabsf(dir.z) * 1.4f;
        dir = Vector3Normalize(dir);

        float spd = p_range(35.0f, 95.0f);
        Vector3 vel = Vector3Scale(dir, spd);
        Color c0 = (Color){ 255, 255, 245, 255 };
        Color c1 = isBlue ? (Color){ 80, 200, 255, 0 } : (Color){ 255, 150, 30, 0 };
        float s0 = p_range(0.24f, 0.45f);
        float s1 = 0.04f;
        float life = p_range(1.0f, 2.2f);
        particle_spawn(goalPos, vel, c0, c1, s0, s1, life, 1.4f, 14.0f, 1);
    }

    /* 5. Billowing heavy smoke clouds */
    for (k = 0; k < 45; k++) {
        Vector3 dir = p_rand_dir();
        dir.y = fabsf(dir.y) * 1.2f + 0.4f;
        if (goalPos.z > 0.0f) dir.z = -fabsf(dir.z);
        else                  dir.z =  fabsf(dir.z);
        Vector3 vel = Vector3Scale(Vector3Normalize(dir), p_range(8.0f, 26.0f));
        Color c0 = (Color){ 100, 100, 110, 160 };
        Color c1 = (Color){ 40, 40, 45, 0 };
        float s0 = p_range(2.0f, 3.5f);
        float s1 = p_range(7.0f, 12.0f);
        float life = p_range(1.4f, 2.4f);
        particle_spawn(goalPos, vel, c0, c1, s0, s1, life, 1.8f, -2.0f, 0);
    }
}

static void particles_demolition_explosion(Vector3 pos)
{
    int k;

    /* 1. Core incandescent flash */
    for (k = 0; k < 4; k++) {
        Vector3 p = Vector3Add(pos, p_randv3(-0.2f, 0.2f));
        Color c0 = (Color){ 255, 255, 255, 255 };
        Color c1 = (Color){ 255, 180, 50, 0 };
        particle_spawn(p, V3(0, 0, 0), c0, c1, 1.8f, 9.0f, 0.20f, 1.2f, 0.0f, 1);
    }

    /* 2. Expanding shockwave ring */
    for (k = 0; k < 20; k++) {
        float a = (float)k / 20.0f * 2.0f * PI;
        Vector3 dir = V3(cosf(a), p_range(-0.1f, 0.1f), sinf(a));
        float spd = p_range(18.0f, 32.0f);
        Vector3 p = Vector3Add(pos, Vector3Scale(dir, 0.8f));
        Vector3 vel = Vector3Scale(dir, spd);
        Color c0 = (Color){ 255, 240, 160, 240 };
        Color c1 = (Color){ 255, 80, 10, 0 };
        particle_spawn(p, vel, c0, c1, 0.8f, 3.8f, 0.35f, 2.5f, 0.0f, 1);
    }

    /* 3. Fiery blast clouds */
    for (k = 0; k < 50; k++) {
        Vector3 dir = p_rand_dir();
        float spd = p_range(10.0f, 34.0f);
        Vector3 vel = Vector3Scale(dir, spd);
        Color c0 = (Color){ 255, 230, 90, 250 };
        Color c1 = (Color){ 210, 30, 5, 0 };
        float s0 = p_range(1.2f, 2.2f);
        float s1 = p_range(3.8f, 6.5f);
        float life = p_range(0.50f, 1.05f);
        particle_spawn(pos, vel, c0, c1, s0, s1, life, 3.0f, -2.0f, 1);
    }

    /* 4. High-velocity shrapnel sparks & glowing debris */
    for (k = 0; k < 90; k++) {
        Vector3 dir = p_rand_dir();
        float spd = p_range(25.0f, 65.0f);
        Vector3 vel = Vector3Scale(dir, spd);
        Color c0 = (Color){ 255, 255, 220, 255 };
        Color c1 = (Color){ 255, 110, 20, 0 };
        float s0 = p_range(0.18f, 0.32f);
        float s1 = 0.03f;
        float life = p_range(0.65f, 1.40f);
        particle_spawn(pos, vel, c0, c1, s0, s1, life, 1.8f, 16.0f, 1);
    }

    /* 5. Billowing black smoke plume */
    for (k = 0; k < 25; k++) {
        Vector3 dir = p_rand_dir();
        dir.y = fabsf(dir.y) + 0.5f;
        Vector3 vel = Vector3Scale(Vector3Normalize(dir), p_range(5.0f, 18.0f));
        Color c0 = (Color){ 80, 80, 85, 160 };
        Color c1 = (Color){ 30, 30, 35, 0 };
        float s0 = p_range(1.2f, 2.4f);
        float s1 = p_range(4.0f, 7.0f);
        float life = p_range(1.0f, 1.8f);
        particle_spawn(pos, vel, c0, c1, s0, s1, life, 2.2f, -1.8f, 0);
    }
}

/* Check if car A demolishes car B upon contact */
static void demolish_car(int victimIdx, int attackerIdx, Car *cars, const int *carModel,
                         float *demoBannerTimer, char *demoBannerText, int textLen, Color *bannerColor, float *camShake)
{
    Car *vic = &cars[victimIdx];
    vic->demolished = 1;
    vic->demoTimer = 3.0f;
    vic->vel = V3(0, 0, 0);
    vic->angVel = V3(0, 0, 0);
    vic->boosting = 0;

    particles_demolition_explosion(vic->pos);
    *camShake = 0.65f;

    if (victimIdx == 0) {
        snprintf(demoBannerText, textLen, "DEMOLISHED BY %s!",
                 attackerIdx >= 0 ? CAR_NAMES[carModel[attackerIdx] < 0 ? 0 : carModel[attackerIdx]] : "OPPONENT");
        *bannerColor = (Color){ 255, 60, 40, 255 };
        *demoBannerTimer = 3.0f;
    } else if (attackerIdx == 0) {
        snprintf(demoBannerText, textLen, "DEMOLITION! [%s]",
                 CAR_NAMES[carModel[victimIdx] < 0 ? 0 : carModel[victimIdx]]);
        *bannerColor = (Color){ 255, 185, 45, 255 };
        *demoBannerTimer = 2.5f;
    }
}

/* ------------------------------------------------------------------------ */
/* Headless physics self-test (sarpbc.exe --test)                             */
/* ------------------------------------------------------------------------ */
static void sim(Car *c, Input in, float seconds, int jumpAtStart)
{
    int n = (int)(seconds * PHYS_HZ), k;
    for (k = 0; k < n; k++) {
        Input t = in;
        t.jumpPressed = (k == 0 && jumpAtStart);
        car_step(c, &t, 1.0f / PHYS_HZ);
    }
}

static void run_physics_test(Car *c)
{
    Input in;
    Ball b;
    float maxY, y0, t;
    int k, touched = 0;

    car_reset(c, V3(0, 0, 0), 0.0f);
    memset(&in, 0, sizeof(in));
    sim(c, in, 2.0f, 0);
    printf("rest:   y=%.3f  wheels=%d  (box bottom %.3f, wheel r %.3f)\n",
           c->pos.y, c->wheelsOnGround, c->boxMin.y, c->wheelRadius);
    y0 = c->pos.y;

    in.jump = 1; maxY = y0;
    for (k = 0; k < (int)(2.0f * PHYS_HZ); k++) {
        Input s = in; s.jumpPressed = k == 0;
        car_step(c, &s, 1.0f / PHYS_HZ);
        if (c->pos.y > maxY) maxY = c->pos.y;
    }
    printf("jump (held):    apex %.2f m\n", maxY - y0);
    memset(&in, 0, sizeof(in)); sim(c, in, 2.0f, 0);

    in.jump = 1; maxY = y0;
    for (k = 0; k < (int)(3.0f * PHYS_HZ); k++) {
        Input s = in; s.jumpPressed = (k == 0 || k == (int)(0.25f * PHYS_HZ));
        if (k == 1) s.jump = 1;
        car_step(c, &s, 1.0f / PHYS_HZ);
        if (c->pos.y > maxY) maxY = c->pos.y;
    }
    printf("double jump:    apex %.2f m\n", maxY - y0);
    memset(&in, 0, sizeof(in)); sim(c, in, 2.0f, 0);

    car_reset(c, V3(0, 0, -100), -PI / 2.0f);
    memset(&in, 0, sizeof(in)); sim(c, in, 1.0f, 0);
    in.throttle = 1;
    sim(c, in, 1.0f, 0); printf("throttle 1s:    %.1f m/s\n", Vector3Length(c->vel));
    sim(c, in, 4.0f, 0); printf("throttle 5s:    %.1f m/s (no boost)\n", Vector3Length(c->vel));
    in.boost = 1;
    sim(c, in, 3.0f, 0); printf("boost 3s:       %.1f m/s, boost left %.2f\n", Vector3Length(c->vel), c->boost);

    car_reset(c, V3(0, 0, 0), 0.0f);
    memset(&in, 0, sizeof(in)); sim(c, in, 1.0f, 0);
    in.throttle = 1; sim(c, in, 1.5f, 0);
    {
        float sp = Vector3Length(c->vel), w;
        in.steer = 1; sim(c, in, 1.0f, 0);
        w = fabsf(c->angVel.y);
        printf("full lock @%.1f m/s: yaw %.2f rad/s, radius %.1f m (wheelbase %.2f m)\n", sp, w, w > 0 ? Vector3Length(c->vel) / w : 0, c->wheelBase);
        in.slide = 1; sim(c, in, 1.0f, 0);
        printf("  + powerslide 1s: yaw %.2f rad/s, speed %.1f m/s, sideways %.1f m/s\n",
               fabsf(c->angVel.y), Vector3Length(c->vel), fabsf(Vector3DotProduct(c->vel, car_right(c))));
    }

    /* forward dodge from a standstill */
    car_reset(c, V3(0, 0, 0), 0.0f);
    memset(&in, 0, sizeof(in)); sim(c, in, 1.0f, 0);
    sim(c, in, 0.15f, 1);
    in.pitch = -1;
    {
        Input s = in; s.jumpPressed = 1;
        car_step(c, &s, 1.0f / PHYS_HZ);
    }
    printf("fwd dodge:      horizontal %.1f m/s\n", sqrtf(c->vel.x*c->vel.x + c->vel.z*c->vel.z));
    memset(&in, 0, sizeof(in)); sim(c, in, 2.0f, 0);
    printf("  after landing: up.y %.2f wheels %d\n", car_up(c).y, c->wheelsOnGround);

    car_reset(c, V3(0, 0, 0), 0.0f);
    c->pos.y = 3.0f; c->rot = QuaternionFromAxisAngle(V3(1, 0, 0), 1.2f);
    memset(&in, 0, sizeof(in)); sim(c, in, 2.0f, 0);
    printf("tilted 69deg drop: up.y %.2f wheels %d\n", car_up(c).y, c->wheelsOnGround);

    car_reset(c, V3(0, 0, 0), 0.0f);
    c->pos.y = 2.0f; c->rot = QuaternionFromAxisAngle(V3(1, 0, 0), PI);
    memset(&in, 0, sizeof(in)); sim(c, in, 4.0f, 0);
    printf("roof drop:      up.y %.2f wheels %d (should stay on the roof)\n", car_up(c).y, c->wheelsOnGround);
    {
        Input s = in; s.jumpPressed = 1; s.jump = 1;
        car_step(c, &s, 1.0f / PHYS_HZ);
        sim(c, in, 2.0f, 0);
        printf("  + jump:       up.y %.2f wheels %d (should be back on its wheels)\n", car_up(c).y, c->wheelsOnGround);
    }
    car_reset(c, V3(0, 0, 0), 0.0f);
    c->pos.y = 2.0f; c->rot = QuaternionFromAxisAngle(V3(1, 0, 0), PI / 2.0f);
    memset(&in, 0, sizeof(in)); sim(c, in, 3.0f, 0);
    printf("side drop:      up.y %.2f wheels %d", car_up(c).y, c->wheelsOnGround);
    {
        Input s = in; s.jumpPressed = 1; s.jump = 1;
        car_step(c, &s, 1.0f / PHYS_HZ);
        sim(c, in, 2.0f, 0);
        printf("  -> jump: up.y %.2f wheels %d\n", car_up(c).y, c->wheelsOnGround);
    }

    car_reset(c, V3(0, 0, 0), 0.0f);
    c->pos.y = 8.0f; c->rot = QuaternionFromAxisAngle(V3(0, 0, 1), PI / 2.0f);   /* nose straight up */
    memset(&in, 0, sizeof(in)); in.boost = 1; c->boost = BOOST_MAX;
    sim(c, in, 1.0f, 0);
    printf("boost nose-up 1s: vy %.1f m/s (positive = can fly)\n", c->vel.y);

    /* drive into a resting ball at full throttle */
    car_reset(c, V3(0, 0, -30), -PI / 2.0f);
    ball_reset(&b);
    memset(&in, 0, sizeof(in)); in.throttle = 1;
    for (t = 0; t < 4.0f && !touched; t += 1.0f / PHYS_HZ) {
        car_step(c, &in, 1.0f / PHYS_HZ);
        ball_step(&b, 1.0f / PHYS_HZ);
        touched = car_ball_collide(c, &b);
    }
    for (k = 0; k < 4; k++) { ball_step(&b, 1.0f / PHYS_HZ); car_ball_collide(c, &b); }
    printf("ball hit:       touched=%d car %.1f m/s -> ball %.1f m/s\n", touched, Vector3Length(c->vel), Vector3Length(b.vel));

    /* drive at the side wall: should climb the ramp onto the wall */
    car_reset(c, V3(60, 0, 0), 0.0f);                 /* facing +X */
    memset(&in, 0, sizeof(in)); sim(c, in, 0.5f, 0);
    in.throttle = 1;
    maxY = 0;
    for (k = 0; k < (int)(6.0f * PHYS_HZ); k++) {
        car_step(c, &in, 1.0f / PHYS_HZ);
        if (c->pos.y > maxY) maxY = c->pos.y;
    }
    printf("wall drive:     max height %.1f m, now (%.1f, %.1f, %.1f) wheels %d up(%.2f %.2f %.2f)\n",
           maxY, c->pos.x, c->pos.y, c->pos.z, c->wheelsOnGround, car_up(c).x, car_up(c).y, car_up(c).z);

    /* ball rolled into the orange goal */
    ball_reset(&b); b.pos = V3(0, BALL_R, 120); b.vel = V3(0, 0, 30);
    for (k = 0; k < (int)(3.0f * PHYS_HZ); k++) ball_step(&b, 1.0f / PHYS_HZ);
    printf("ball into goal: z %.1f (goal line %.1f) y %.1f\n", b.pos.z, ARENA_L, b.pos.y);

    /* ball fired at the side glass */
    ball_reset(&b); b.pos = V3(50, 20, 0); b.vel = V3(40, 0, 0);
    for (k = 0; k < (int)(2.0f * PHYS_HZ); k++) ball_step(&b, 1.0f / PHYS_HZ);
    printf("ball off wall:  x %.1f vx %.1f (should be heading back, x < 102)\n", b.pos.x, b.vel.x);

    /* ball dropped from high: must stay inside the dome */
    ball_reset(&b); b.pos = V3(0, 50, 0); b.vel = V3(0, 30, 0);
    for (k = 0; k < (int)(3.0f * PHYS_HZ); k++) ball_step(&b, 1.0f / PHYS_HZ);
    printf("ball at ceiling: y %.1f vy %.1f\n", b.pos.y, b.vel.y);

    /* bots ------------------------------------------------------------------ */
    {
        static const float pxz[6][2] = { { -78, -(ARENA_L - 24) }, { 78, -(ARENA_L - 24) }, { -78, ARENA_L - 24 },
                                         { 78, ARENA_L - 24 }, { -82, 0 }, { 82, 0 } };
        Vector3 tp[6];
        float tpt[6];
        int skill, trial;
        for (k = 0; k < 6; k++) tp[k] = V3(pxz[k][0], 0, pxz[k][1]);

        /* shooting drill: one blue bot, empty net, ball dropped at 8 spots; time to score */
        for (skill = 0; skill < 3; skill++) {
            int scored = 0, own = 0;
            float total = 0.0f;
            for (trial = 0; trial < 8; trial++) {
                static const float spots[8][3] = { { 0, 1.62f, 40 }, { 40, 1.62f, 20 }, { -60, 6, 60 }, { 70, 1.62f, 100 },
                                                   { -30, 10, -20 }, { 0, 1.62f, -60 }, { 90, 3, -10 }, { -20, 1.62f, 125 } };
                Car cs[1];
                Bot bs[1];
                int tm[1] = { 0 };
                float t;
                cs[0] = *c; memset(bs, 0, sizeof(bs)); memset(tpt, 0, sizeof(tpt));
                car_reset(&cs[0], V3(0, 0, -KICKOFF_Z), -PI / 2.0f);
                ball_reset(&b); b.pos = V3(spots[trial][0], spots[trial][1], spots[trial][2]);
                g_bw->predCalls = 0;
                for (t = 0; t < 30.0f; t += 1.0f / PHYS_HZ) {
                    Input bi;
                    bot_world_update(&b, 1.0f / PHYS_HZ, tp, tpt, 6);
                    bi = bot_think(0, cs, tm, 1, &b, &bs[0], skill, 1.0f / PHYS_HZ);
                    car_step(&cs[0], &bi, 1.0f / PHYS_HZ);
                    ball_step(&b, 1.0f / PHYS_HZ);
                    car_ball_collide(&cs[0], &b);
                    for (k = 0; k < 6; k++) {
                        tpt[k] -= 1.0f / PHYS_HZ;
                        if (tpt[k] <= 0 && flat_dist(cs[0].pos, tp[k]) < PAD_RADIUS) { cs[0].boost = BOOST_MAX; tpt[k] = PAD_RESPAWN; }
                    }
                    if (fabsf(b.pos.z) > ARENA_L + BALL_R) break;
                }
                if (b.pos.z > ARENA_L) { scored++; total += t; } else if (b.pos.z < -ARENA_L) own++;
                if (getenv("SARPBC_BOTTRACE"))
                    printf("   %s trial %d: %s t=%.1f ball(%.0f,%.0f,%.0f v%.0f) car(%.0f,%.0f,%.0f v%.0f) boost %.1f\n", SKILL_NAMES[skill], trial,
                           b.pos.z > ARENA_L ? "GOAL" : "miss", t, b.pos.x, b.pos.y, b.pos.z, Vector3Length(b.vel),
                           cs[0].pos.x, cs[0].pos.y, cs[0].pos.z, Vector3Length(cs[0].vel), cs[0].boost);
            }
            printf("bot drill %-8s: scored %d/8 (own goals %d), avg %.1f s\n", SKILL_NAMES[skill], scored, own, scored ? total / scored : 0.0f);
        }

        /* matches: 2 v 2 at each skill, then All-Star vs Rookie and Pro vs Rookie */
        for (skill = 0; skill < 5; skill++) {
            Car cs[4];
            Bot bs[4];
            int tm[4] = { 0, 0, 1, 1 }, goals[2] = { 0, 0 }, touches[4] = { 0 }, jumps = 0, m;
            int sk[2];
            sk[0] = skill < 3 ? skill : (skill == 3 ? 2 : 1);
            sk[1] = skill < 3 ? skill : 0;
            for (m = 0; m < 4; m++) cs[m] = *c;
#define BOT_KICKOFF() do { for (m = 0; m < 4; m++) { Vector3 p_; float y_; kickoff_spot(m % 2, tm[m], 2, &p_, &y_); car_reset(&cs[m], p_, y_); } \
                           memset(bs, 0, sizeof(bs)); memset(tpt, 0, sizeof(tpt)); ball_reset(&b); g_bw->predCalls = 0; } while (0)
            BOT_KICKOFF();
            for (k = 0; k < (int)(180.0f * PHYS_HZ); k++) {
                int q;
                bot_world_update(&b, 1.0f / PHYS_HZ, tp, tpt, 6);
                for (m = 0; m < 4; m++) {
                    Input bi = bot_think(m, cs, tm, 4, &b, &bs[m], sk[tm[m]], 1.0f / PHYS_HZ);
                    jumps += bi.jumpPressed;
                    car_step(&cs[m], &bi, 1.0f / PHYS_HZ);
                }
                ball_step(&b, 1.0f / PHYS_HZ);
                for (m = 0; m < 4; m++) touches[m] += car_ball_collide(&cs[m], &b);
                for (m = 0; m < 4; m++) { int n2; for (n2 = m + 1; n2 < 4; n2++) car_car_collide(&cs[m], &cs[n2]); }
                for (q = 0; q < 6; q++) {
                    tpt[q] -= 1.0f / PHYS_HZ;
                    for (m = 0; m < 4 && tpt[q] <= 0; m++)
                        if (flat_dist(cs[m].pos, tp[q]) < PAD_RADIUS && cs[m].pos.y < 4) { cs[m].boost = BOOST_MAX; tpt[q] = PAD_RESPAWN; }
                }
                if (fabsf(b.pos.z) > ARENA_L + BALL_R) { goals[b.pos.z > 0 ? 0 : 1]++; BOT_KICKOFF(); }
                if (getenv("SARPBC_BOTTRACE") && skill == atoi(getenv("SARPBC_BOTTRACE")) && k % (int)(2.0f * PHYS_HZ) == 0)
                    printf("  t=%3.0f ball(%6.1f %5.1f %6.1f) | b0(%.0f,%.0f,%.0f v%.0f r%d) b1(%.0f,%.0f,%.0f v%.0f r%d) o0(%.0f,%.0f,%.0f v%.0f r%d)\n",
                           k / PHYS_HZ, b.pos.x, b.pos.y, b.pos.z,
                           cs[0].pos.x, cs[0].pos.y, cs[0].pos.z, Vector3Length(cs[0].vel), bs[0].role,
                           cs[1].pos.x, cs[1].pos.y, cs[1].pos.z, Vector3Length(cs[1].vel), bs[1].role,
                           cs[2].pos.x, cs[2].pos.y, cs[2].pos.z, Vector3Length(cs[2].vel), bs[2].role);
            }
#undef BOT_KICKOFF
            printf("bots 2v2 %-8s vs %-8s 3 min: blue %d - %d orange, touch ticks %d/%d/%d/%d, jumps %d\n",
                   SKILL_NAMES[sk[0]], SKILL_NAMES[sk[1]], goals[0], goals[1], touches[0], touches[1], touches[2], touches[3], jumps);
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Car loading (startup + menu car select)                                    */
/* ------------------------------------------------------------------------ */
/* looks next to the exe, then in ../export_c; fills dir (with trailing slash) */
static int load_car(const char *name, Car *car, CarRender *cr, Shader lit, int render)
{
    char dir[512], path[1024];
    SarmModel sm;
    int err;
    snprintf(dir, sizeof(dir), "%sassets/cars/%s/", GetApplicationDirectory(), name);
    snprintf(path, sizeof(path), "%s%s.sarm", dir, name);
    if (!FileExists(path)) {
        snprintf(dir, sizeof(dir), "%s../export_c/cars/%s/", GetApplicationDirectory(), name);
        snprintf(path, sizeof(path), "%s%s.sarm", dir, name);
    }
    if (!FileExists(path)) {
        snprintf(dir, sizeof(dir), "assets/cars/%s/", name);
        snprintf(path, sizeof(path), "%s%s.sarm", dir, name);
    }
    if (!FileExists(path)) {
        snprintf(dir, sizeof(dir), "../export_c/cars/%s/", name);
        snprintf(path, sizeof(path), "%s%s.sarm", dir, name);
    }
    if (!FileExists(path)) {
        snprintf(dir, sizeof(dir), "export_c/cars/%s/", name);
        snprintf(path, sizeof(path), "%s%s.sarm", dir, name);
    }
    err = sarm_load(path, &sm);
    if (err != SARM_OK) {
        TraceLog(LOG_ERROR, "sarm_load(%s): %s", path, sarm_error_string(err));
        return 0;
    }
    car_init(car, &sm);
    if (render) { car_render_unload(cr); car_render_load(cr, &sm, dir, lit); }
    sarm_free(&sm);
    return 1;
}

/* ------------------------------------------------------------------------ */
/* Menus                                                                      */
/* ------------------------------------------------------------------------ */
typedef enum {
    SCR_MENU,        /* main menu                                   */
    SCR_PLAY,        /* online / offline choice                     */
    SCR_ONLINE,      /* quick match: playlist + server              */
    SCR_SEARCHING,   /* in the quick-match queue                    */
    SCR_OFFLINE,     /* exhibition vs bots / free play              */
    SCR_GARAGE,      /* car + paint                                 */
    SCR_SETTINGS,
    SCR_GAME,
    SCR_PAUSE
} Screen;

typedef struct Nav { int up, down, left, right, ok, back; } Nav;

static int g_navTextMode = 0;   /* a text box has focus: letters type instead of navigating */

static Nav read_nav(void)
{
    static int stickHeld = 0;
    int letters = !g_navTextMode;
    Nav n;
    memset(&n, 0, sizeof(n));
    n.up    = IsKeyPressed(KEY_UP)    || (letters && IsKeyPressed(KEY_W));
    n.down  = IsKeyPressed(KEY_DOWN)  || (letters && IsKeyPressed(KEY_S)) || (!letters && IsKeyPressed(KEY_TAB));
    n.left  = IsKeyPressed(KEY_LEFT)  || (letters && IsKeyPressed(KEY_A));
    n.right = IsKeyPressed(KEY_RIGHT) || (letters && IsKeyPressed(KEY_D));
    n.ok    = IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER) || (letters && IsKeyPressed(KEY_SPACE));
    n.back  = IsKeyPressed(KEY_ESCAPE) || (letters && IsKeyPressed(KEY_BACKSPACE));
    if (IsGamepadAvailable(0)) {
        float gx = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X);
        float gy = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y);
        int dir = gy < -0.6f ? 1 : gy > 0.6f ? 2 : gx < -0.6f ? 3 : gx > 0.6f ? 4 : 0;
        if (dir && dir != stickHeld) { n.up |= dir == 1; n.down |= dir == 2; n.left |= dir == 3; n.right |= dir == 4; }
        stickHeld = dir;
        n.up    |= IsGamepadButtonPressed(0, GAMEPAD_BUTTON_LEFT_FACE_UP);
        n.down  |= IsGamepadButtonPressed(0, GAMEPAD_BUTTON_LEFT_FACE_DOWN);
        n.left  |= IsGamepadButtonPressed(0, GAMEPAD_BUTTON_LEFT_FACE_LEFT);
        n.right |= IsGamepadButtonPressed(0, GAMEPAD_BUTTON_LEFT_FACE_RIGHT);
        n.ok    |= IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_FACE_DOWN);
        n.back  |= IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_FACE_RIGHT);
    }
    return n;
}

typedef struct MenuItem { const char *label; char value[48]; } MenuItem;

/* Procedural authentic SARPBC high-tech soccer ball texture (1024x512 equirectangular) */
static Texture2D make_tech_ball_texture(void)
{
    const int w = 1024, h = 1024;
    unsigned char *px = (unsigned char *)MemAlloc(w * h * 4);
    if (!px) return (Texture2D){ 0 };

    /* 12 pentagon centers (icosahedron vertices) */
    Vector3 pent[12];
    float phi = (1.0f + sqrtf(5.0f)) * 0.5f;
    int idx = 0;
    for (int s1 = -1; s1 <= 1; s1 += 2) {
        for (int s2 = -1; s2 <= 1; s2 += 2) {
            pent[idx++] = Vector3Normalize(V3(0.0f, (float)s1, (float)s2 * phi));
            pent[idx++] = Vector3Normalize(V3((float)s1, (float)s2 * phi, 0.0f));
            pent[idx++] = Vector3Normalize(V3((float)s1 * phi, 0.0f, (float)s2));
        }
    }

    /* 20 hexagon centers (icosahedron face centers) */
    Vector3 hex[20];
    idx = 0;
    for (int i = 0; i < 12; i++) {
        for (int j = i + 1; j < 12; j++) {
            float d = Vector3DotProduct(pent[i], pent[j]);
            if (d > 0.4f && d < 0.6f) {
                for (int k = j + 1; k < 12; k++) {
                    float d2 = Vector3DotProduct(pent[i], pent[k]);
                    float d3 = Vector3DotProduct(pent[j], pent[k]);
                    if (d2 > 0.4f && d2 < 0.6f && d3 > 0.4f && d3 < 0.6f) {
                        Vector3 c = Vector3Normalize(V3(pent[i].x + pent[j].x + pent[k].x,
                                                       pent[i].y + pent[j].y + pent[k].y,
                                                       pent[i].z + pent[j].z + pent[k].z));
                        int dup = 0;
                        for (int m = 0; m < idx; m++) {
                            if (Vector3DotProduct(hex[m], c) > 0.99f) { dup = 1; break; }
                        }
                        if (!dup && idx < 20) hex[idx++] = c;
                    }
                }
            }
        }
    }

    const float seam_w  = 0.016f;
    const float glow_w  = 0.046f;
    const float bevel_w = 0.082f;

    for (int y = 0; y < h; y++) {
        float v = ((float)y + 0.5f) / (float)h;
        float phiA = v * 2.0f * PI;
        float sinP = sinf(phiA), cosP = cosf(phiA);
        for (int x = 0; x < w; x++) {
            float u = ((float)x + 0.5f) / (float)w;
            float theta = u * PI;
            float sinT = sinf(theta), cosT = cosf(theta);
            Vector3 p = V3(sinT * cosP, sinT * sinP, cosT);

            float bdot1 = -2.0f, bdot2 = -2.0f;
            int is_pent1 = 0;
            int p_idx1 = 0;

            for (int i = 0; i < 12; i++) {
                float d = Vector3DotProduct(p, pent[i]);
                if (d > bdot1) {
                    bdot2 = bdot1;
                    bdot1 = d; is_pent1 = 1; p_idx1 = i;
                } else if (d > bdot2) {
                    bdot2 = d;
                }
            }
            for (int j = 0; j < 20; j++) {
                float d = Vector3DotProduct(p, hex[j]);
                if (d > bdot1) {
                    bdot2 = bdot1;
                    bdot1 = d; is_pent1 = 0;
                } else if (d > bdot2) {
                    bdot2 = d;
                }
            }

            float ang1 = acosf(fminf(1.0f, fmaxf(-1.0f, bdot1)));
            float ang2 = acosf(fminf(1.0f, fmaxf(-1.0f, bdot2)));
            float edge_dist = (ang2 - ang1) * 0.5f;

            float r = 0, g = 0, b = 0;

            if (edge_dist < seam_w) {
                /* Deep dark recessed panel seam */
                float groove = edge_dist / seam_w;
                r = 18.0f + 14.0f * groove;
                g = 20.0f + 15.0f * groove;
                b = 26.0f + 18.0f * groove;
            } else if (edge_dist < glow_w) {
                /* Glowing neon energy circuitry tracks */
                float t = (edge_dist - seam_w) / (glow_w - seam_w);
                float glow = sinf(t * PI);
                if (is_pent1 && (p_idx1 & 1)) {
                    /* Electric Amber/Orange energy seam */
                    r = 255.0f * glow + 180.0f * (1.0f - glow);
                    g = 140.0f * glow + 70.0f * (1.0f - glow);
                    b = 30.0f * glow;
                } else {
                    /* Electric Cyan / Turquoise energy seam */
                    r = 40.0f + 200.0f * powf(glow, 2.5f);
                    g = 175.0f + 80.0f * glow;
                    b = 255.0f * glow + 210.0f * (1.0f - glow);
                }
            } else {
                /* Panel interior plate */
                float bevel = edge_dist < bevel_w ? (edge_dist - glow_w) / (bevel_w - glow_w) : 1.0f;
                bevel = 0.58f + 0.42f * bevel;

                int pat = ((int)(x * 3) ^ (int)(y * 3)) & 3;
                float micro = 1.0f - 0.05f * (float)pat;

                if (is_pent1) {
                    /* Carbon-fiber / stealth composite pentagon with glowing center emblem */
                    if (ang1 < 0.155f && ang1 > 0.105f) {
                        float ring_glow = 1.0f - fabsf(ang1 - 0.130f) / 0.025f;
                        if (p_idx1 & 1) {
                            r = 255.0f; g = 140.0f + 80.0f * ring_glow; b = 30.0f;
                        } else {
                            r = 40.0f + 180.0f * ring_glow; g = 190.0f + 65.0f * ring_glow; b = 255.0f;
                        }
                    } else if (ang1 <= 0.105f) {
                        r = 46.0f; g = 52.0f; b = 64.0f;
                    } else {
                        r = 40.0f * bevel * micro;
                        g = 44.0f * bevel * micro;
                        b = 54.0f * bevel * micro;
                    }
                } else {
                    /* Brushed titanium / silver hexagon plate */
                    float base_col = 205.0f * bevel * micro + 22.0f * cosf(ang1 * 8.0f);
                    r = fminf(255.0f, base_col);
                    g = fminf(255.0f, base_col * 1.02f);
                    b = fminf(255.0f, base_col * 1.06f);
                }
            }

            int pidx = (y * w + x) * 4;
            px[pidx + 0] = (unsigned char)fminf(255.0f, fmaxf(0.0f, r));
            px[pidx + 1] = (unsigned char)fminf(255.0f, fmaxf(0.0f, g));
            px[pidx + 2] = (unsigned char)fminf(255.0f, fmaxf(0.0f, b));
            px[pidx + 3] = 255;
        }
    }

    Image img = { px, w, h, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
    ExportImage(img, "tech_ball.png");
    Texture2D tex = LoadTextureFromImage(img);
    GenTextureMipmaps(&tex);
    SetTextureFilter(tex, TEXTURE_FILTER_TRILINEAR);
    MemFree(px);
    TraceLog(LOG_INFO, "BALL: generated high-def authentic tech ball texture (1024x512)");
    return tex;
}

/* Modern Stadium Broadcast Scoreboard */
static void draw_hud_scoreboard(int sw, int scoreBlue, int scoreOrange, float matchTime, float matchLen, GameState state)
{
    int ot = matchTime < 0.0f;   /* online overtime: the server counts up as a negative time */
    int tsec = ot ? (int)floorf(-matchTime) : (int)ceilf(fmaxf(0.0f, matchTime));
    int mins = tsec / 60, secs = tsec % 60;
    const char *clock = ot ? TextFormat("+%d:%02d", mins, secs) : matchLen > 0.0f ? TextFormat("%d:%02d", mins, secs) : "FREE";
    if (state == ST_OVER && scoreBlue == scoreOrange) clock = "+0:00";

    int totalW = 380, totalH = 54;
    int x0 = sw / 2 - totalW / 2, y0 = 12;

    /* Soft drop shadow */
    DrawRectangleRounded((Rectangle){ (float)x0 + 2, (float)y0 + 3, (float)totalW, (float)totalH }, 0.28f, 8, (Color){ 0, 0, 0, 140 });

    /* Main container: dark frosted glass */
    DrawRectangleRounded((Rectangle){ (float)x0, (float)y0, (float)totalW, (float)totalH }, 0.28f, 8, (Color){ 12, 16, 26, 235 });
    DrawRectangleRoundedLinesEx((Rectangle){ (float)x0, (float)y0, (float)totalW, (float)totalH }, 0.28f, 8, 2.0f, (Color){ 45, 65, 95, 200 });

    /* Blue Team section (left) */
    Rectangle blueBadge = { (float)x0 + 6, (float)y0 + 6, 110, (float)totalH - 12 };
    DrawRectangleRounded(blueBadge, 0.25f, 6, (Color){ 24, 72, 165, 240 });
    DrawRectangleRoundedLinesEx(blueBadge, 0.25f, 6, 1.5f, (Color){ 65, 145, 255, 255 });
    DrawText("BLUE", (int)blueBadge.x + 12, (int)blueBadge.y + 13, 16, (Color){ 180, 215, 255, 255 });
    const char *bScoreStr = TextFormat("%d", scoreBlue);
    DrawText(bScoreStr, (int)(blueBadge.x + blueBadge.width - 16 - MeasureText(bScoreStr, 32)), (int)blueBadge.y + 5, 32, (Color){ 240, 248, 255, 255 });

    /* Orange Team section (right) */
    Rectangle orangeBadge = { (float)(x0 + totalW - 116), (float)y0 + 6, 110, (float)totalH - 12 };
    DrawRectangleRounded(orangeBadge, 0.25f, 6, (Color){ 195, 75, 18, 240 });
    DrawRectangleRoundedLinesEx(orangeBadge, 0.25f, 6, 1.5f, (Color){ 255, 135, 40, 255 });
    const char *oScoreStr = TextFormat("%d", scoreOrange);
    DrawText(oScoreStr, (int)orangeBadge.x + 16, (int)orangeBadge.y + 5, 32, (Color){ 255, 248, 240, 255 });
    DrawText("ORANGE", (int)(orangeBadge.x + orangeBadge.width - 12 - MeasureText("ORANGE", 16)), (int)orangeBadge.y + 13, 16, (Color){ 255, 215, 180, 255 });

    /* Center clock pod */
    Rectangle clockPod = { (float)x0 + 124, (float)y0 + 8, (float)totalW - 248, (float)totalH - 16 };
    DrawRectangleRounded(clockPod, 0.25f, 6, (Color){ 6, 8, 14, 255 });
    DrawRectangleRoundedLinesEx(clockPod, 0.25f, 6, 1.0f, (Color){ 32, 44, 64, 200 });

    Color clockCol = RAYWHITE;
    if (ot) {
        Rectangle otr = { (float)(sw / 2 - 60), (float)(y0 + totalH + 6), 120, 24 };
        DrawRectangleRounded(otr, 0.5f, 6, (Color){ 200, 40, 30, 235 });
        DrawText("OVERTIME", (int)(otr.x + 60 - MeasureText("OVERTIME", 15) / 2), (int)otr.y + 5, 15, RAYWHITE);
    }
    if (!ot && matchTime < 30.0f && matchLen > 0.0f) {
        float pulse = 0.5f + 0.5f * sinf((float)GetTime() * 8.0f);
        clockCol = ColorAlpha((Color){ 255, 180, 60, 255 }, 0.7f + 0.3f * pulse);
    }
    int cw = MeasureText(clock, 26);
    DrawText(clock, (int)(clockPod.x + clockPod.width/2 - cw/2), (int)clockPod.y + 6, 26, clockCol);
}

/* Curved Radial Boost Gauge & Speedometer */
static void draw_hud_boost_and_speed(int sw, int sh, float boost, float speed, int boosting)
{
    float bPct = fminf(1.0f, fmaxf(0.0f, boost / BOOST_MAX));
    int bInt = (int)(bPct * 100.0f + 0.5f);
    int isSupersonic = speed >= 37.0f;

    /* Boost radial gauge at bottom right */
    Vector2 center = { (float)(sw - 95), (float)(sh - 95) };
    float rOut = 62.0f, rIn = 48.0f;

    /* Outer drop shadow */
    DrawCircle((int)center.x + 2, (int)center.y + 3, rOut + 2, (Color){ 0, 0, 0, 120 });

    /* Background ring track */
    DrawRing(center, rIn, rOut, 40.0f, 320.0f, 48, (Color){ 18, 24, 34, 230 });
    DrawRingLines(center, rIn, rOut, 40.0f, 320.0f, 48, (Color){ 40, 55, 80, 180 });

    /* Active energy fill arc */
    float fillAngle = 40.0f + bPct * 280.0f;
    Color arcCol;
    if (isSupersonic) arcCol = (Color){ 40, 230, 255, 255 };
    else if (boosting) arcCol = (Color){ 255, 235, 60, 255 };
    else if (bPct > 0.33f) arcCol = (Color){ 255, 145, 25, 255 };
    else arcCol = (Color){ 255, 65, 25, 255 };

    if (bPct > 0.005f) {
        DrawRing(center, rIn + 1.0f, rOut - 1.0f, 40.0f, fillAngle, 48, arcCol);
    }

    /* Inner pod */
    DrawCircle((int)center.x, (int)center.y, rIn - 1.0f, (Color){ 10, 14, 22, 245 });
    DrawCircleLines((int)center.x, (int)center.y, rIn - 1.0f, (Color){ 35, 48, 70, 200 });

    /* Boost number readout */
    const char *bNumStr = TextFormat("%d", bInt);
    int bnw = MeasureText(bNumStr, 34);
    Color numCol = isSupersonic ? (Color){ 120, 240, 255, 255 } : boosting ? (Color){ 255, 255, 180, 255 } : RAYWHITE;
    DrawText(bNumStr, (int)center.x - bnw / 2, (int)center.y - 20, 34, numCol);
    DrawText("BOOST", (int)center.x - MeasureText("BOOST", 11) / 2, (int)center.y + 14, 11, (Color){ 170, 185, 205, 220 });

    /* Speedometer badge above gauge */
    float kmh = speed * 3.6f;
    int spX = sw - 165, spY = sh - 185, spW = 140, spH = 36;
    DrawRectangleRounded((Rectangle){ (float)spX + 2, (float)spY + 2, (float)spW, (float)spH }, 0.35f, 6, (Color){ 0, 0, 0, 110 });
    DrawRectangleRounded((Rectangle){ (float)spX, (float)spY, (float)spW, (float)spH }, 0.35f, 6, (Color){ 12, 16, 26, 225 });
    DrawRectangleRoundedLinesEx((Rectangle){ (float)spX, (float)spY, (float)spW, (float)spH }, 0.35f, 6, 1.5f, (Color){ 45, 65, 95, 190 });

    const char *kmhNum = TextFormat("%.0f", kmh);
    int spnw = MeasureText(kmhNum, 22);
    Color spCol = isSupersonic ? (Color){ 40, 230, 255, 255 } : kmh > 100.0f ? (Color){ 255, 165, 40, 255 } : RAYWHITE;
    DrawText(kmhNum, spX + 16, spY + 7, 22, spCol);
    DrawText("KM/H", spX + 20 + spnw, spY + 12, 13, (Color){ 160, 185, 210, 220 });

    /* Speed mini-bar indicator */
    float spRatio = fminf(1.0f, speed / 40.0f);
    DrawRectangle(spX + 16, spY + spH - 6, spW - 32, 3, (Color){ 25, 32, 45, 255 });
    DrawRectangle(spX + 16, spY + spH - 6, (int)((spW - 32) * spRatio), 3, spCol);

    /* Supersonic Overdrive Alert Banner */
    if (isSupersonic) {
        float flash = 0.6f + 0.4f * sinf((float)GetTime() * 12.0f);
        int sbW = 150, sbH = 26;
        int sbX = sw - 170, sbY = sh - 222;
        DrawRectangleRounded((Rectangle){ (float)sbX, (float)sbY, (float)sbW, (float)sbH }, 0.35f, 6, ColorAlpha((Color){ 0, 140, 240, 255 }, flash * 0.9f));
        DrawRectangleRoundedLinesEx((Rectangle){ (float)sbX, (float)sbY, (float)sbW, (float)sbH }, 0.35f, 6, 1.5f, ColorAlpha((Color){ 100, 230, 255, 255 }, flash));
        const char *ssText = "SUPERSONIC";
        int sstw = MeasureText(ssText, 14);
        DrawText(ssText, sbX + sbW/2 - sstw/2, sbY + 6, 14, (Color){ 255, 255, 255, 255 });
    }
}

/* Tactical HUD Badges (Ball Cam, Player Card, Controls) */
static void draw_hud_tactical_badges(int sw, int sh, int ballCam, const char *carName, const char *modeName, const char *skillName, int showFps, int showHints)
{
    (void)sw;
    /* Player Car Info Card (Top Left) */
    int cardW = 210, cardH = 48;
    DrawRectangleRounded((Rectangle){ 18, 14, (float)cardW, (float)cardH }, 0.28f, 6, (Color){ 12, 16, 26, 215 });
    DrawRectangleRoundedLinesEx((Rectangle){ 18, 14, (float)cardW, (float)cardH }, 0.28f, 6, 1.5f, (Color){ 45, 65, 95, 180 });
    DrawRectangleRounded((Rectangle){ 23, 19, 4, (float)cardH - 10 }, 0.4f, 4, (Color){ 55, 145, 255, 255 });
    DrawText(carName, 34, 19, 18, RAYWHITE);
    DrawText(TextFormat("%s  -  %s", modeName, skillName), 34, 40, 13, (Color){ 175, 190, 215, 220 });

    if (showFps) {
        int fps = GetFPS();
        const char *fpsStr = TextFormat("%d FPS", fps);
        int fpsW = MeasureText(fpsStr, 14) + 16;
        DrawRectangleRounded((Rectangle){ 18, 68, (float)fpsW, 22 }, 0.35f, 4, (Color){ 10, 14, 20, 200 });
        DrawText(fpsStr, 26, 72, 14, (Color){ 70, 235, 130, 240 });
    }

    /* Ball Cam Pill Badge (Bottom Left) */
    int bcW = 145, bcH = 32;
    int bcX = 18, bcY = sh - (showHints ? 66 : 46);
    DrawRectangleRounded((Rectangle){ (float)bcX + 2, (float)bcY + 2, (float)bcW, (float)bcH }, 0.35f, 6, (Color){ 0, 0, 0, 110 });
    if (ballCam) {
        DrawRectangleRounded((Rectangle){ (float)bcX, (float)bcY, (float)bcW, (float)bcH }, 0.35f, 6, (Color){ 16, 42, 85, 230 });
        DrawRectangleRoundedLinesEx((Rectangle){ (float)bcX, (float)bcY, (float)bcW, (float)bcH }, 0.35f, 6, 1.5f, (Color){ 45, 175, 255, 255 });
        DrawCircle(bcX + 16, bcY + bcH/2, 5, (Color){ 45, 210, 255, 255 });
        DrawText("BALL CAM", bcX + 28, bcY + 8, 15, RAYWHITE);
        DrawText("ON", bcX + 112, bcY + 9, 13, (Color){ 45, 210, 255, 255 });
    } else {
        DrawRectangleRounded((Rectangle){ (float)bcX, (float)bcY, (float)bcW, (float)bcH }, 0.35f, 6, (Color){ 18, 22, 30, 200 });
        DrawRectangleRoundedLinesEx((Rectangle){ (float)bcX, (float)bcY, (float)bcW, (float)bcH }, 0.35f, 6, 1.2f, (Color){ 50, 60, 75, 180 });
        DrawCircleLines(bcX + 16, bcY + bcH/2, 4.5f, (Color){ 160, 170, 185, 220 });
        DrawText("CAR CAM", bcX + 28, bcY + 8, 15, (Color){ 180, 190, 205, 220 });
    }

    /* Keybind hint bar */
    if (showHints) {
        DrawText("W/S Drive   A/D Steer   SPACE Jump/Dodge   SHIFT Boost   CTRL Slide/Roll   C Ball Cam   ESC Menu",
                 18, sh - 26, 15, (Color){ 175, 185, 200, 190 });
    }
}

/* Draws a modernized frosted-glass vertical menu card */
static void menu_run(const char *title, MenuItem *it, int n, int *sel, Nav nav, int *act, int *adj)
{
    static Vector2 lastMouse = { -1, -1 };
    const int rowH = n > 11 ? 40 : 48;
    const int cardW = 540;
    const int cardH = n * rowH + (title[0] ? 100 : 50);
    const int fs = n > 11 ? 22 : 25;
    int sw = GetScreenWidth(), sh = GetScreenHeight();
    int cardX = sw / 2 - cardW / 2;
    int cardY = sh / 2 - cardH / 2 + (title[0] ? 15 : 45);
    int startY = cardY + (title[0] ? 80 : 25);
    Vector2 m = GetMousePosition();
    int moved = m.x != lastMouse.x || m.y != lastMouse.y;
    lastMouse = m;

    *act = -1; *adj = 0;
    if (nav.up)    *sel = (*sel + n - 1) % n;
    if (nav.down)  *sel = (*sel + 1) % n;
    if (nav.left)  *adj = -1;
    if (nav.right) *adj = 1;
    if (nav.ok) { if (it[*sel].value[0]) *adj = 1; else *act = *sel; }

    /* Screen dim overlay */
    DrawRectangle(0, 0, sw, sh, (Color){ 0, 0, 0, 130 });

    /* Card background shadow */
    DrawRectangleRounded((Rectangle){ (float)cardX + 4, (float)cardY + 6, (float)cardW, (float)cardH }, 0.08f, 6, (Color){ 0, 0, 0, 160 });

    /* Frosted glass card */
    DrawRectangleRounded((Rectangle){ (float)cardX, (float)cardY, (float)cardW, (float)cardH }, 0.08f, 6, (Color){ 12, 16, 26, 235 });
    DrawRectangleRoundedLinesEx((Rectangle){ (float)cardX, (float)cardY, (float)cardW, (float)cardH }, 0.08f, 6, 2.0f, (Color){ 45, 75, 120, 220 });

    /* Title at top of card */
    if (title[0]) {
        int tw = MeasureText(title, 36);
        DrawText(title, cardX + cardW / 2 - tw / 2, cardY + 24, 36, (Color){ 255, 185, 50, 255 });
        DrawRectangle(cardX + 50, cardY + 66, cardW - 100, 2, (Color){ 255, 160, 40, 160 });
    }

    /* Menu items */
    for (int i = 0; i < n; i++) {
        Rectangle r = { (float)(cardX + 24), (float)(startY + i * rowH), (float)(cardW - 48), (float)(rowH - 6) };
        int hover = CheckCollisionPointRec(m, r);
        if (hover && moved) *sel = i;
        if (hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            *sel = i;
            if (it[i].value[0]) *adj = 1; else *act = i;
        }
        if (hover && IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && it[i].value[0]) {
            *sel = i; *adj = -1;
        }
        int on = (i == *sel);

        if (on) {
            DrawRectangleRounded(r, 0.28f, 6, (Color){ 255, 140, 25, 240 });
            DrawRectangleRoundedLinesEx(r, 0.28f, 6, 1.5f, (Color){ 255, 210, 100, 255 });
            DrawText(">", (int)r.x + 14, (int)r.y + (rowH - 6)/2 - fs/2 + 2, fs, BLACK);
            DrawText(it[i].label, (int)r.x + 36, (int)r.y + (rowH - 6)/2 - fs/2 + 2, fs, BLACK);
        } else {
            DrawRectangleRounded(r, 0.28f, 6, hover ? (Color){ 32, 42, 60, 220 } : (Color){ 18, 24, 36, 180 });
            DrawRectangleRoundedLinesEx(r, 0.28f, 6, 1.0f, (Color){ 38, 52, 75, 160 });
            DrawText(it[i].label, (int)r.x + 24, (int)r.y + (rowH - 6)/2 - fs/2 + 2, fs, (Color){ 220, 228, 240, 255 });
        }

        if (it[i].value[0]) {
            const char *valStr = TextFormat("<  %s  >", it[i].value);
            int vw = MeasureText(valStr, fs - 2);
            DrawText(valStr, (int)(r.x + r.width - 18 - vw), (int)r.y + (rowH - 6)/2 - (fs - 2)/2 + 2, fs - 2,
                     on ? (Color){ 20, 20, 20, 255 } : (Color){ 255, 180, 80, 255 });
        }
    }

    /* Footer navigation hints */
    DrawText("ENTER / A  Select    < / >  Adjust    ESC / B  Back",
             sw / 2 - MeasureText("ENTER / A  Select    < / >  Adjust    ESC / B  Back", 16) / 2,
             sh - 36, 16, (Color){ 180, 195, 215, 220 });
}

/* ------------------------------------------------------------------------ */
/* Front-end UI: immediate-mode widgets. Mouse hover/click plus keyboard and  */
/* gamepad focus that moves spatially (up/down/left/right picks the nearest   */
/* widget in that direction, using last frame's layout).                      */
/* ------------------------------------------------------------------------ */
#define UI_MAX 48
enum { UIK_BUTTON = 0, UIK_SPINNER, UIK_TEXT };

typedef struct UiState {
    int       screen, focus, count, prevCount;
    Rectangle rect[UI_MAX], prevRect[UI_MAX];
    int       kind[UI_MAX], prevKind[UI_MAX];
    int       activated, adjustId, adjust;
    Vector2   lastMouse;
    int       mouseMoved;
    float     t;
} UiState;
static UiState g_ui = { .screen = -1 };

#define UI_ACCENT      (Color){ 255, 150, 30, 255 }
#define UI_ACCENT_HI   (Color){ 255, 214, 110, 255 }
#define UI_PANEL       (Color){ 12, 16, 26, 232 }
#define UI_PANEL_LINE  (Color){ 45, 70, 110, 210 }
#define UI_TEXT        (Color){ 228, 234, 245, 255 }
#define UI_TEXT_DIM    (Color){ 150, 166, 190, 230 }
#define UI_BLUE        (Color){ 60, 150, 255, 255 }
#define UI_ORANGE      (Color){ 255, 140, 40, 255 }

static void ui_begin(int screenId, Nav nav, int defaultFocus)
{
    Vector2 m = GetMousePosition();
    int dir, j, best = -1;
    float bestScore = 1e9f;
    if (screenId != g_ui.screen) {
        g_ui.screen = screenId;
        g_ui.focus = defaultFocus;
        g_ui.prevCount = 0;
    } else {
        g_ui.prevCount = g_ui.count;
        memcpy(g_ui.prevRect, g_ui.rect, sizeof(g_ui.rect));
        memcpy(g_ui.prevKind, g_ui.kind, sizeof(g_ui.kind));
    }
    g_ui.count = 0;
    g_ui.activated = g_ui.adjustId = -1;
    g_ui.adjust = 0;
    g_ui.mouseMoved = m.x != g_ui.lastMouse.x || m.y != g_ui.lastMouse.y;
    g_ui.lastMouse = m;
    g_ui.t += GetFrameTime();
    if (g_ui.prevCount == 0) return;
    if (g_ui.focus < 0 || g_ui.focus >= g_ui.prevCount) g_ui.focus = 0;
    if (g_ui.prevKind[g_ui.focus] != UIK_TEXT) while (GetCharPressed() > 0) {}   /* drop stray typing */

    dir = nav.up ? 0 : nav.down ? 1 : nav.left ? 2 : nav.right ? 3 : -1;
    if (dir >= 2 && g_ui.prevKind[g_ui.focus] == UIK_SPINNER) {
        g_ui.adjustId = g_ui.focus; g_ui.adjust = dir == 2 ? -1 : 1;
        dir = -1;
    }
    if (dir >= 0) {
        Rectangle a = g_ui.prevRect[g_ui.focus];
        float ax = a.x + a.width * 0.5f, ay = a.y + a.height * 0.5f;
        for (j = 0; j < g_ui.prevCount; j++) {
            Rectangle b = g_ui.prevRect[j];
            float dx, dy, prim, sec, score;
            if (j == g_ui.focus) continue;
            dx = b.x + b.width * 0.5f - ax; dy = b.y + b.height * 0.5f - ay;
            prim = dir == 0 ? -dy : dir == 1 ? dy : dir == 2 ? -dx : dx;
            sec  = dir < 2 ? fabsf(dx) : fabsf(dy);
            if (prim <= 1.0f) continue;
            score = prim + sec * 2.0f;
            if (score < bestScore) { bestScore = score; best = j; }
        }
        if (best >= 0) g_ui.focus = best;
        else if (dir < 2) g_ui.focus = dir == 0 ? g_ui.prevCount - 1 : 0;   /* wrap */
    }
    if (nav.ok) {
        if (g_ui.prevKind[g_ui.focus] == UIK_SPINNER) { g_ui.adjustId = g_ui.focus; g_ui.adjust = 1; }
        else if (g_ui.prevKind[g_ui.focus] == UIK_BUTTON) g_ui.activated = g_ui.focus;
    }
}

static int ui_add(Rectangle r, int kind)
{
    Vector2 m = GetMousePosition();
    int id = g_ui.count < UI_MAX ? g_ui.count++ : UI_MAX - 1;
    int hover = CheckCollisionPointRec(m, r);
    g_ui.rect[id] = r;
    g_ui.kind[id] = kind;
    if (hover && g_ui.mouseMoved) g_ui.focus = id;
    if (hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        g_ui.focus = id;
        if (kind == UIK_SPINNER) { g_ui.adjustId = id; g_ui.adjust = m.x < r.x + r.width * 0.5f ? -1 : 1; }
        else if (kind == UIK_BUTTON) g_ui.activated = id;
    }
    if (hover && IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && kind == UIK_SPINNER) { g_ui.adjustId = id; g_ui.adjust = -1; }
    return id;
}

static void ui_end(void)
{
    g_navTextMode = g_ui.focus >= 0 && g_ui.focus < g_ui.count && g_ui.kind[g_ui.focus] == UIK_TEXT;
}

static void ui_panel(Rectangle r, float round, Color fill, Color line)
{
    DrawRectangleRounded((Rectangle){ r.x + 3, r.y + 5, r.width, r.height }, round, 8, (Color){ 0, 0, 0, 120 });
    DrawRectangleRounded(r, round, 8, fill);
    if (line.a) DrawRectangleRoundedLinesEx(r, round, 8, 1.5f, line);
}

static void ui_text(const char *t, int x, int y, int fs, Color c)
{
    DrawText(t, x + 2, y + 2, fs, (Color){ 0, 0, 0, (unsigned char)(c.a * 0.7f) });
    DrawText(t, x, y, fs, c);
}

static void ui_text_c(const char *t, float cx, int y, int fs, Color c)
{
    ui_text(t, (int)(cx - MeasureText(t, fs) * 0.5f), y, fs, c);
}

/* Big menu button. Returns 1 when clicked / confirmed. */
static int ui_button(Rectangle r, const char *label, const char *sub, int primary)
{
    int id = ui_add(r, UIK_BUTTON), on = id == g_ui.focus;
    int fs = r.height >= 60 ? 28 : r.height >= 46 ? 22 : 18;
    int ty = (int)(r.y + r.height * 0.5f - (sub ? fs * 0.5f + 9 : fs * 0.5f));
    float round = r.height > 80 ? 0.12f : 0.25f;
    if (on) {
        ui_panel(r, round, (Color){ 255, 140, 25, 245 }, UI_ACCENT_HI);
        DrawRectangleRounded((Rectangle){ r.x + 4, r.y + 4, r.width - 8, r.height * 0.42f }, round, 8, (Color){ 255, 255, 255, 34 });
        DrawText(label, (int)r.x + 26, ty, fs, (Color){ 15, 12, 8, 255 });
        if (sub) DrawText(sub, (int)r.x + 26, ty + fs + 4, 15, (Color){ 50, 30, 10, 235 });
        DrawText(">", (int)(r.x + r.width - 30), (int)(r.y + r.height * 0.5f - 12), 24, (Color){ 15, 12, 8, 255 });
    } else {
        ui_panel(r, round, (Color){ 16, 22, 34, 225 }, primary ? (Color){ 255, 150, 40, 200 } : (Color){ 40, 56, 82, 190 });
        DrawRectangleRounded((Rectangle){ r.x + 8, r.y + 10, 4, r.height - 20 }, 0.6f, 4, primary ? UI_ACCENT : (Color){ 70, 120, 200, 200 });
        ui_text(label, (int)r.x + 26, ty, fs, UI_TEXT);
        if (sub) DrawText(sub, (int)r.x + 26, ty + fs + 4, 15, UI_TEXT_DIM);
    }
    return g_ui.activated == id;
}

/* "< value >" selector. Returns -1 / +1 when changed. */
static int ui_spinner(Rectangle r, const char *label, const char *value)
{
    int id = ui_add(r, UIK_SPINNER), on = id == g_ui.focus;
    int fs = r.height >= 50 ? 22 : 19;
    const char *v = TextFormat("<   %s   >", value);
    int vw = MeasureText(v, fs - 1);
    if (on) {
        ui_panel(r, 0.25f, (Color){ 255, 140, 25, 240 }, UI_ACCENT_HI);
        DrawText(label, (int)r.x + 22, (int)(r.y + r.height * 0.5f - fs * 0.5f), fs, (Color){ 15, 12, 8, 255 });
        DrawText(v, (int)(r.x + r.width - 20 - vw), (int)(r.y + r.height * 0.5f - (fs - 1) * 0.5f), fs - 1, (Color){ 15, 12, 8, 255 });
    } else {
        ui_panel(r, 0.25f, (Color){ 16, 22, 34, 220 }, (Color){ 40, 56, 82, 180 });
        ui_text(label, (int)r.x + 22, (int)(r.y + r.height * 0.5f - fs * 0.5f), fs, UI_TEXT);
        DrawText(v, (int)(r.x + r.width - 20 - vw), (int)(r.y + r.height * 0.5f - (fs - 1) * 0.5f), fs - 1, UI_ACCENT);
    }
    return g_ui.adjustId == id ? g_ui.adjust : 0;
}

/* Editable text box (mode 0 = name, 1 = host[:port]). */
static void ui_textfield(Rectangle r, const char *label, char *buf, int maxlen, int mode)
{
    int id = ui_add(r, UIK_TEXT), on = id == g_ui.focus, len = (int)strlen(buf);
    if (on) {
        int key;
        while ((key = GetCharPressed()) > 0) {
            int ok = mode == 0 ? (key >= 32 && key <= 126)
                               : ((key >= '0' && key <= '9') || (key >= 'a' && key <= 'z') || (key >= 'A' && key <= 'Z') ||
                                  key == '.' || key == ':' || key == '-' || key == '_');
            if (ok && len < maxlen - 1) { buf[len++] = (char)key; buf[len] = 0; }
        }
        if ((IsKeyPressed(KEY_BACKSPACE) || IsKeyPressedRepeat(KEY_BACKSPACE)) && len > 0) buf[--len] = 0;
    }
    ui_panel(r, 0.22f, on ? (Color){ 24, 32, 48, 245 } : (Color){ 16, 22, 34, 220 }, on ? UI_ACCENT : (Color){ 40, 56, 82, 180 });
    DrawText(label, (int)r.x + 18, (int)r.y + 9, 14, on ? UI_ACCENT : UI_TEXT_DIM);
    ui_text(buf[0] ? buf : (on ? "" : "-"), (int)r.x + 18, (int)r.y + 28, 22, UI_TEXT);
    if (on && fmodf(g_ui.t, 1.0f) < 0.55f)
        DrawRectangle((int)r.x + 20 + MeasureText(buf, 22), (int)r.y + 28, 3, 22, UI_ACCENT);
    if (on) DrawText("type to edit", (int)(r.x + r.width - 18 - MeasureText("type to edit", 13)), (int)r.y + 10, 13, UI_TEXT_DIM);
}

/* Little stick figures for playlist cards. */
static void ui_people(float cx, float y, int perTeam, float s)
{
    int k;
    float gap = 22.0f * s, half = perTeam * gap;
    for (k = 0; k < perTeam; k++) {
        float bx = cx - 26.0f * s - half + gap * (k + 0.5f), ox = cx + 26.0f * s + gap * (k + 0.5f);
        DrawCircle((int)bx, (int)y, 7 * s, UI_BLUE);
        DrawRectangleRounded((Rectangle){ bx - 9 * s, y + 9 * s, 18 * s, 20 * s }, 0.5f, 4, UI_BLUE);
        DrawCircle((int)ox, (int)y, 7 * s, UI_ORANGE);
        DrawRectangleRounded((Rectangle){ ox - 9 * s, y + 9 * s, 18 * s, 20 * s }, 0.5f, 4, UI_ORANGE);
    }
    DrawText("VS", (int)(cx - MeasureText("VS", (int)(16 * s)) * 0.5f), (int)(y + 4 * s), (int)(16 * s), UI_TEXT_DIM);
}

/* Large selectable card (playlists, play modes). Returns 1 when clicked / confirmed. */
static int ui_card(Rectangle r, const char *title, const char *tag, const char *desc, int selected, int people)
{
    int id = ui_add(r, UIK_BUTTON), on = id == g_ui.focus;
    Rectangle d = r;
    if (on) d.y -= 6;
    ui_panel(d, 0.08f, selected ? (Color){ 26, 34, 52, 245 } : UI_PANEL,
             on ? UI_ACCENT_HI : selected ? UI_ACCENT : UI_PANEL_LINE);
    if (selected || on) {
        DrawRectangleRounded((Rectangle){ d.x + 2, d.y + 2, d.width - 4, 8 }, 1.0f, 4, on ? UI_ACCENT_HI : UI_ACCENT);
        DrawRectangleRoundedLinesEx(d, 0.08f, 8, on ? 3.0f : 2.0f, on ? UI_ACCENT_HI : UI_ACCENT);
    }
    if (tag) {
        int tw = MeasureText(tag, 14) + 18;
        DrawRectangleRounded((Rectangle){ d.x + d.width - tw - 14, d.y + 18, (float)tw, 22 }, 0.5f, 4,
                             selected ? UI_ACCENT : (Color){ 40, 56, 82, 230 });
        DrawText(tag, (int)(d.x + d.width - tw - 5), (int)d.y + 22, 14, selected ? (Color){ 20, 14, 6, 255 } : UI_TEXT);
    }
    if (people > 0) ui_people(d.x + d.width * 0.5f, d.y + 78, people, 1.25f);
    ui_text(title, (int)d.x + 22, (int)(d.y + (people > 0 ? 136 : 62)), 32, selected || on ? UI_ACCENT_HI : UI_TEXT);
    if (desc) DrawText(desc, (int)d.x + 22, (int)(d.y + (people > 0 ? 178 : 106)), 16, UI_TEXT_DIM);
    return g_ui.activated == id;
}

/* Screen title band + bottom hint bar shared by every menu page. */
static void ui_chrome(const char *title, const char *crumb, const char *hints)
{
    int sw = GetScreenWidth(), sh = GetScreenHeight();
    DrawRectangleGradientV(0, 0, sw, 150, (Color){ 4, 6, 12, 235 }, (Color){ 4, 6, 12, 0 });
    DrawRectangleGradientV(0, sh - 90, sw, 90, (Color){ 4, 6, 12, 0 }, (Color){ 4, 6, 12, 235 });
    if (title) {
        int tw = MeasureText(title, 46);
        ui_text(title, 64, 38, 46, UI_TEXT);
        DrawRectangle(64, 90, tw, 4, UI_ACCENT);
        if (crumb) DrawText(crumb, 64 + tw + 22, 56, 18, UI_TEXT_DIM);
    }
    if (hints) DrawText(hints, 64, sh - 38, 16, UI_TEXT_DIM);
}

/* Player card (bottom right on menu pages). */
static void ui_profile(const char *name, const char *car, const char *paint)
{
    int sw = GetScreenWidth(), sh = GetScreenHeight();
    Rectangle r = { (float)sw - 330, (float)sh - 108, 290, 72 };
    ui_panel(r, 0.2f, UI_PANEL, UI_PANEL_LINE);
    DrawCircle((int)r.x + 36, (int)r.y + 36, 22, (Color){ 30, 60, 110, 255 });
    DrawCircleLines((int)r.x + 36, (int)r.y + 36, 22, UI_ACCENT);
    DrawText(TextFormat("%c", name[0] ? name[0] : '?'), (int)r.x + 36 - MeasureText(TextFormat("%c", name[0] ? name[0] : '?'), 24) / 2,
             (int)r.y + 24, 24, UI_TEXT);
    ui_text(name[0] ? name : "Player", (int)r.x + 72, (int)r.y + 14, 22, UI_TEXT);
    DrawText(TextFormat("%s  -  %s", car, paint), (int)r.x + 72, (int)r.y + 42, 15, UI_TEXT_DIM);
}

/* Rotating "searching" spinner. */
static void ui_spinner_anim(Vector2 c, float r, float t)
{
    int k;
    DrawRing(c, r - 6, r, 0, 360, 48, (Color){ 30, 40, 60, 220 });
    for (k = 0; k < 3; k++) {
        float a0 = t * 240.0f + k * 120.0f;
        DrawRing(c, r - 6, r, a0, a0 + 50.0f, 16, k == 0 ? UI_ACCENT : k == 1 ? UI_BLUE : UI_ORANGE);
    }
}

/* "host", "host:port" -> host + port (default port when missing). */
static void split_host_port(const char *in, char *host, int hostLen, int *port)
{
    const char *c = strrchr(in, ':');
    *port = SARP_DEFAULT_PORT;
    if (c && c != in && strchr(in, ':') == c && c[1]) {
        int n = (int)(c - in);
        if (n >= hostLen) n = hostLen - 1;
        memcpy(host, in, (size_t)n); host[n] = 0;
        *port = atoi(c + 1);
        if (*port <= 0 || *port > 65535) *port = SARP_DEFAULT_PORT;
    } else {
        snprintf(host, (size_t)hostLen, "%s", in[0] ? in : "127.0.0.1");
    }
}

/* ------------------------------------------------------------------------ */
/* Online Multiplayer HUD & Scoreboard                                       */
/* ------------------------------------------------------------------------ */
static void net_client_draw_nameplates(const NetClient *cli, const Vector3 *carPositions, const int *carTeams, const int *demolished, Camera3D cam)
{
    if (!cli || cli->state != NET_CONNECTED) return;
    int sw = GetScreenWidth(), sh = GetScreenHeight();

    for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
        if (!cli->players[i].active || i == cli->localSlot) continue;
        if (demolished && demolished[i]) continue;

        Vector3 headPos = V3(carPositions[i].x, carPositions[i].y + 1.8f, carPositions[i].z);
        float dist = Vector3Distance(cam.position, headPos);
        if (dist > 180.0f) continue;

        Vector3 camToTarget = Vector3Subtract(headPos, cam.position);
        Vector3 camFwd = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
        if (Vector3DotProduct(camToTarget, camFwd) <= 0.1f) continue;

        Vector2 sp = GetWorldToScreen(headPos, cam);
        if (sp.x < -100 || sp.x > sw + 100 || sp.y < -100 || sp.y > sh + 100) continue;

        const char *name = cli->players[i].isBot ? TextFormat("%s  BOT", cli->players[i].name) : cli->players[i].name;
        int fontSize = dist < 40.0f ? 16 : dist < 90.0f ? 14 : 12;
        int tw = MeasureText(name, fontSize);
        int padX = 10, padY = 5;
        Rectangle plate = { sp.x - tw / 2.0f - padX, sp.y - fontSize / 2.0f - padY, (float)tw + padX * 2, (float)fontSize + padY * 2 };

        int team = (carTeams != NULL) ? carTeams[i] : cli->players[i].team;
        Color teamBg = team == 0 ? (Color){ 20, 70, 160, 200 } : (Color){ 180, 70, 20, 200 };
        Color teamBorder = team == 0 ? (Color){ 60, 150, 255, 230 } : (Color){ 255, 140, 50, 230 };

        DrawRectangleRounded(plate, 0.4f, 4, teamBg);
        DrawRectangleRoundedLinesEx(plate, 0.4f, 4, 1.2f, teamBorder);
        DrawText(name, (int)plate.x + padX, (int)plate.y + padY, fontSize, RAYWHITE);
    }
}

static void net_client_draw_scoreboard(const NetClient *cli, int sw, int sh)
{
    if (!cli || cli->state != NET_CONNECTED) return;

    int panelW = 680, panelH = 340;
    int px = sw / 2 - panelW / 2, py = sh / 2 - panelH / 2;

    DrawRectangleRounded((Rectangle){ (float)px + 4, (float)py + 6, (float)panelW, (float)panelH }, 0.15f, 6, (Color){ 0, 0, 0, 160 });
    DrawRectangleRounded((Rectangle){ (float)px, (float)py, (float)panelW, (float)panelH }, 0.15f, 6, (Color){ 12, 16, 26, 240 });
    DrawRectangleRoundedLinesEx((Rectangle){ (float)px, (float)py, (float)panelW, (float)panelH }, 0.15f, 6, 2.0f, (Color){ 45, 75, 120, 220 });

    DrawText(cli->serverPlaylist == 1 ? "DUEL 1v1" : cli->serverPlaylist == 3 ? "STANDARD 3v3" : "DOUBLES 2v2", px + 24, py + 16, 22, (Color){ 255, 185, 50, 255 });
    const char *srvInfo = TextFormat("%s:%d  |  Ping: %.0f ms", cli->serverIp, cli->serverPort, cli->pingMs);
    DrawText(srvInfo, px + panelW - MeasureText(srvInfo, 14) - 24, py + 22, 14, (Color){ 170, 195, 225, 220 });
    DrawRectangle(px + 24, py + 48, panelW - 48, 1, (Color){ 45, 65, 95, 180 });

    int colW = panelW / 2 - 32;

    DrawRectangleRounded((Rectangle){ (float)(px + 20), (float)(py + 60), (float)colW, 32 }, 0.2f, 4, (Color){ 20, 65, 150, 230 });
    DrawText(TextFormat("BLUE TEAM   (%d)", cli->scoreBlue), px + 32, py + 68, 16, (Color){ 200, 230, 255, 255 });

    DrawRectangleRounded((Rectangle){ (float)(px + panelW / 2 + 12), (float)(py + 60), (float)colW, 32 }, 0.2f, 4, (Color){ 165, 65, 15, 230 });
    DrawText(TextFormat("ORANGE TEAM   (%d)", cli->scoreOrange), px + panelW / 2 + 24, py + 68, 16, (Color){ 255, 225, 200, 255 });

    int blueRow = 0, orangeRow = 0;
    for (int i = 0; i < SARP_MAX_CLIENTS; i++) {
        if (!cli->players[i].active) continue;
        int isBlue = (cli->players[i].team == 0);
        int rx = isBlue ? px + 20 : px + panelW / 2 + 12;
        int ry = py + 102 + (isBlue ? blueRow++ : orangeRow++) * 36;

        int isLocal = (i == cli->localSlot);
        Color rowBg = isLocal ? (Color){ 35, 60, 95, 240 } : (Color){ 18, 24, 36, 180 };
        Color rowBorder = isLocal ? (Color){ 255, 190, 50, 255 } : (Color){ 35, 45, 65, 160 };

        DrawRectangleRounded((Rectangle){ (float)rx, (float)ry, (float)colW, 30 }, 0.25f, 4, rowBg);
        DrawRectangleRoundedLinesEx((Rectangle){ (float)rx, (float)ry, (float)colW, 30 }, 0.25f, 4, 1.0f, rowBorder);

        const char *pName = cli->players[i].name;
        if (isLocal) pName = TextFormat("%s (YOU)", pName);
        else if (cli->players[i].isBot) pName = TextFormat("%s [BOT]", pName);
        DrawText(pName, rx + 12, ry + 7, 15, isLocal ? (Color){ 255, 225, 120, 255 } : RAYWHITE);

        const char *carStr = CAR_NAMES[cli->players[i].car_model % CAR_COUNT];
        DrawText(carStr, rx + colW - MeasureText(carStr, 13) - 12, ry + 9, 13, (Color){ 160, 180, 210, 200 });
    }

    DrawText("Release [TAB] to close", px + panelW / 2 - MeasureText("Release [TAB] to close", 14) / 2, py + panelH - 24, 14, (Color){ 150, 165, 185, 180 });
}

static void net_client_draw_hud(const NetClient *cli, int sw, int sh)
{
    (void)sh;
    if (!cli || cli->state != NET_CONNECTED) return;

    int bw = cli->lossPct >= 0.5f ? 215 : 145, bh = 28;
    int bx = sw - bw - 18, by = 14;

    DrawRectangleRounded((Rectangle){ (float)bx + 2, (float)by + 2, (float)bw, (float)bh }, 0.35f, 4, (Color){ 0, 0, 0, 100 });
    DrawRectangleRounded((Rectangle){ (float)bx, (float)by, (float)bw, (float)bh }, 0.35f, 4, (Color){ 12, 16, 26, 215 });
    DrawRectangleRoundedLinesEx((Rectangle){ (float)bx, (float)by, (float)bw, (float)bh }, 0.35f, 4, 1.2f, (Color){ 45, 65, 95, 180 });

    Color pingCol = (cli->pingMs < 60.0f && cli->lossPct < 2.0f) ? (Color){ 50, 220, 120, 255 } :
                    (cli->pingMs < 120.0f && cli->lossPct < 8.0f) ? (Color){ 240, 200, 50, 255 } : (Color){ 240, 70, 70, 255 };

    DrawCircle(bx + 14, by + bh / 2, 4.0f, pingCol);
    const char *pingStr = cli->lossPct >= 0.5f ? TextFormat("ONLINE %.0fms  %.1f%% loss", cli->pingMs, cli->lossPct)
                                               : TextFormat("ONLINE %.0fms", cli->pingMs);
    DrawText(pingStr, bx + 26, by + 7, 13, RAYWHITE);

    if (cli->demoBannerTimer > 0.0f) {
        float alpha = fminf(1.0f, cli->demoBannerTimer);
        const char *demoMsg = TextFormat("%s DEMOLISHED %s!", cli->demoKiller, cli->demoVictim);
        int mw = MeasureText(demoMsg, 20);
        int mwW = mw + 40, mwH = 36;
        int mx = sw / 2 - mwW / 2, my = 75;

        DrawRectangleRounded((Rectangle){ (float)mx, (float)my, (float)mwW, (float)mwH }, 0.35f, 4, ColorAlpha((Color){ 220, 60, 20, 230 }, alpha));
        DrawRectangleRoundedLinesEx((Rectangle){ (float)mx, (float)my, (float)mwW, (float)mwH }, 0.35f, 4, 1.5f, ColorAlpha(YELLOW, alpha));
        DrawText(demoMsg, mx + mwW / 2 - mw / 2, my + 8, 20, ColorAlpha(RAYWHITE, alpha));
    }
}

/* ------------------------------------------------------------------------ */
/* Online prediction / rollback                                               */
/*                                                                            */
/* Every client tick we simulate the whole world (our car with our real       */
/* input, remote cars with their last known input, the ball) and record our   */
/* input + resulting car state. Each server snapshot says which of our input  */
/* ticks it has simulated (ack). We restart from the authoritative state at   */
/* that tick and re-simulate up to the present with the recorded inputs, so   */
/* the world is always "server truth, fast-forwarded to now". Whatever small  */
/* jump that causes is moved into a visual offset that decays over ~0.1 s,    */
/* so corrections are invisible instead of rubber-banding or freezing.        */
/* ------------------------------------------------------------------------ */
#define PRED_HIST        256     /* ~2.1 s of history                          */
#define MAX_REPLAY       96      /* replay at most 0.8 s (covers ~700 ms RTT)  */
#define SNAP_CAR_DIST    6.0f    /* bigger errors teleport (kickoff, respawn)  */
#define SNAP_BALL_DIST   10.0f
#define VIS_DECAY_CAR    14.0f   /* 1/s: visual error smoothing rates          */
#define VIS_DECAY_REMOTE 10.0f
#define VIS_DECAY_BALL   12.0f

typedef struct PredFrame { uint32_t tick; Input in; Car car; } PredFrame;

typedef struct OnlineSim {
    PredFrame  hist[PRED_HIST];
    Input      remoteIn[SARP_MAX_CLIENTS];
    Vector3    visOfs[SARP_MAX_CLIENTS];
    Quaternion visRot[SARP_MAX_CLIENTS];
    Vector3    ballVisOfs;
    Quaternion ballVisRot;
    uint32_t   lastSnap;
    int        haveSnap;
    int        lastReplay;       /* ticks re-simulated on the last snapshot   */
    float      lastCorrection;   /* metres our car moved on the last snapshot */
} OnlineSim;
static OnlineSim g_on;

static void online_reset(void)
{
    int i;
    memset(&g_on, 0, sizeof(g_on));
    for (i = 0; i < SARP_MAX_CLIENTS; i++) g_on.visRot[i] = QuaternionIdentity();
    g_on.ballVisRot = QuaternionIdentity();
}

static int online_car_live(const NetClient *nc, const Car *cars, int i)
{
    if (i != nc->localSlot && !nc->players[i].active) return 0;
    return !cars[i].demolished;
}

/* One fixed tick of the client-side world, in the same order as the server. */
static void online_step_world(Car *cars, Ball *ball, const NetClient *nc, const Input *myIn, float h, int fx)
{
    int i, me = nc->localSlot;
    for (i = 0; i < SARP_MAX_CLIENTS; i++)
        if (online_car_live(nc, cars, i)) car_step(&cars[i], i == me ? myIn : &g_on.remoteIn[i], h);
    ball_step(ball, h);
    for (i = 0; i < SARP_MAX_CLIENTS; i++) {
        Vector3 pv;
        if (!online_car_live(nc, cars, i)) continue;
        pv = ball->vel;
        if (car_ball_collide(&cars[i], ball) && fx) {
            float hitDelta = Vector3Distance(ball->vel, pv);
            if (hitDelta > 3.0f)
                particles_impact_burst(Vector3Lerp(cars[i].pos, ball->pos, 0.5f),
                                       Vector3Normalize(Vector3Subtract(ball->pos, cars[i].pos)), hitDelta);
        }
    }
    /* car-car bumps (demolitions are decided by the server and arrive in snapshots) */
    for (i = 0; i < SARP_MAX_CLIENTS; i++) {
        int j;
        if (!online_car_live(nc, cars, i)) continue;
        for (j = i + 1; j < SARP_MAX_CLIENTS; j++) {
            if (!online_car_live(nc, cars, j)) continue;
            if (check_demolition(&cars[i], &cars[j], nc->players[i].team, nc->players[j].team)) continue;
            car_car_collide(&cars[i], &cars[j]);
        }
    }
}

static void online_record(const NetClient *nc, const Input *in, const Car *myCar)
{
    PredFrame *pf = &g_on.hist[nc->clientTick % PRED_HIST];
    pf->tick = nc->clientTick;
    pf->in = *in;
    pf->car = *myCar;
}

/* Move a correction from the simulation into the decaying visual offset. */
static void online_absorb(Vector3 *pos, Quaternion *rot, Vector3 newPos, Quaternion newRot,
                          Vector3 *visOfs, Quaternion *visRot, Vector3 *prevPos, Quaternion *prevRot)
{
    Vector3 d = Vector3Subtract(newPos, *pos);
    Quaternion D = QuaternionNormalize(QuaternionMultiply(newRot, QuaternionInvert(*rot)));
    *visOfs  = Vector3Subtract(*visOfs, d);
    *prevPos = Vector3Add(*prevPos, d);
    *visRot  = QuaternionNormalize(QuaternionMultiply(*visRot, QuaternionInvert(D)));
    *prevRot = QuaternionNormalize(QuaternionMultiply(D, *prevRot));
    *pos = newPos; *rot = newRot;
}

static void online_apply_snapshot(const NetClient *nc, Car *cars, Ball *ball,
                                  Vector3 *prevPos, Quaternion *prevRot,
                                  Vector3 *prevBallPos, Quaternion *prevBallRot, int *camSnap)
{
    static Car sim[SARP_MAX_CLIENTS];
    const float h = 1.0f / PHYS_HZ;
    int i, k, me = nc->localSlot, n = 0;
    uint32_t A = nc->snapAckTick, C = nc->clientTick;
    int canReplay = A > 0 && A <= C && (C - A) <= MAX_REPLAY && g_on.hist[A % PRED_HIST].tick == A;
    Ball b;

    /* 1. authoritative start state (our car keeps its predicted internals from tick A) */
    for (i = 0; i < SARP_MAX_CLIENTS; i++) {
        sim[i] = cars[i];
        if (!nc->players[i].active) continue;
        if (i == me && canReplay) sim[i] = g_on.hist[A % PRED_HIST].car;
        ns_car_from_net(&nc->snapCars[i], &sim[i]);
        if (i != me) g_on.remoteIn[i] = ns_remote_input(&nc->snapCars[i]);
    }
    ns_ball_from_net(&nc->snapBall, &b);

    /* 2. fast-forward to the present with our recorded inputs */
    if (canReplay) {
        n = (int)(C - A);
        for (k = 1; k <= n; k++) {
            PredFrame *pf = &g_on.hist[(A + k) % PRED_HIST];
            online_step_world(sim, &b, nc, &pf->in, h, 0);
            pf->car = sim[me];
        }
    }
    g_on.lastReplay = n;

    /* 3. adopt it, hiding the correction behind a decaying visual offset */
    for (i = 0; i < SARP_MAX_CLIENTS; i++) {
        float dl;
        if (!nc->players[i].active) {
            if (i != me) { cars[i].pos = V3(0, -500, 0); cars[i].vel = V3(0, 0, 0); }
            continue;
        }
        dl = Vector3Distance(sim[i].pos, cars[i].pos);
        if (i == me) g_on.lastCorrection = dl;
        if (!g_on.haveSnap || cars[i].pos.y < -100.0f || dl > SNAP_CAR_DIST || sim[i].demolished != cars[i].demolished) {
            g_on.visOfs[i] = V3(0, 0, 0);
            g_on.visRot[i] = QuaternionIdentity();
            prevPos[i] = sim[i].pos;
            prevRot[i] = sim[i].rot;
            if (i == me) *camSnap = 1;
            cars[i] = sim[i];
        } else {
            Vector3 p = cars[i].pos; Quaternion q = cars[i].rot;
            online_absorb(&p, &q, sim[i].pos, sim[i].rot, &g_on.visOfs[i], &g_on.visRot[i], &prevPos[i], &prevRot[i]);
            cars[i] = sim[i];
        }
    }

    if (!g_on.haveSnap || Vector3Distance(b.pos, ball->pos) > SNAP_BALL_DIST) {
        g_on.ballVisOfs = V3(0, 0, 0);
        g_on.ballVisRot = QuaternionIdentity();
        *prevBallPos = b.pos;
        *prevBallRot = b.rot;
        *ball = b;
    } else {
        Vector3 p = ball->pos; Quaternion q = ball->rot;
        online_absorb(&p, &q, b.pos, b.rot, &g_on.ballVisOfs, &g_on.ballVisRot, prevBallPos, prevBallRot);
        *ball = b;
    }
    g_on.haveSnap = 1;
}

static void online_decay_visuals(float dt, int me)
{
    int i;
    for (i = 0; i < SARP_MAX_CLIENTS; i++) {
        float k = 1.0f - expf(-(i == me ? VIS_DECAY_CAR : VIS_DECAY_REMOTE) * dt);
        g_on.visOfs[i] = Vector3Scale(g_on.visOfs[i], 1.0f - k);
        g_on.visRot[i] = QuaternionSlerp(g_on.visRot[i], QuaternionIdentity(), k);
    }
    {
        float k = 1.0f - expf(-VIS_DECAY_BALL * dt);
        g_on.ballVisOfs = Vector3Scale(g_on.ballVisOfs, 1.0f - k);
        g_on.ballVisRot = QuaternionSlerp(g_on.ballVisRot, QuaternionIdentity(), k);
    }
}

/* ------------------------------------------------------------------------ */
/* Main                                                                       */
/* ------------------------------------------------------------------------ */
#ifdef _WIN32
__declspec(dllexport)
#endif
int sarpbc_main(int argc, char **argv)
{
    const char *carArg = NULL;
    char path[1024], arenaDir[512];
    Car cars[MAX_CARS];                 /* slot 0 = the player */
    CarRender crs[MAX_CARS];
    Bot bots[MAX_CARS];
    int team[MAX_CARS] = { 0 }, carModel[MAX_CARS], nCars = 1, botSkill = 1;
    Ball ball;
    ArenaRender ar;
    int haveArena = 0;
    Shader lit;
    Model field, ballModel;
    Texture2D fieldTex, ballTex;
    Camera3D cam = { 0 };
    Vector3 pads[PAD_COUNT];
    float padTimer[PAD_COUNT] = { 0 };
    int scoreBlue = 0, scoreOrange = 0, ballCam = 0, i, j, lastScorer = 0;
    float matchLen = MATCH_TIME, matchTime = MATCH_TIME, stateTimer = 3.0f, acc = 0.0f, fov = 60.0f, menuT = 0.0f;
    GameState state = ST_COUNTDOWN;
    Screen screen = SCR_MENU, settingsFrom = SCR_MENU;
    int setSel = 0, quit = 0;
    Input in = { 0 };
    int pendingJump = 0, testMode = 0, shotMode = 0, frameNo = 0;
    /* render interpolation + camera state */
    Vector3 prevPos[MAX_CARS] = { { 0 } }, prevBallPos = { 0 }, camDir = { 0 };
    Quaternion prevRot[MAX_CARS], prevBallRot;
    Car rcars[MAX_CARS];
    Ball rball;
    float camY = 0.0f;
    int camSnap = 1;
    NetClient netClient;
    int isOnline = 0, autoConnect = 0;
    const char *connectArg = NULL, *nameArg = NULL;
    char onlineMsg[128] = "";          /* shown on the quick-match page (errors, match results) */

    memset(crs, 0, sizeof(crs));
    memset(bots, 0, sizeof(bots));
    net_client_init(&netClient);
    for (i = 0; i < MAX_CARS; i++) {
        carModel[i] = -1;
        prevRot[i] = (Quaternion){ 0, 0, 0, 1 };
    }
    prevBallRot = (Quaternion){ 0, 0, 0, 1 };
    for (i = 1; i < argc; i++) {
        if      (strcmp(argv[i], "--test") == 0) testMode = 1;
        else if (strcmp(argv[i], "--shot") == 0) shotMode = 1;
        else if (strcmp(argv[i], "--menushot") == 0) shotMode = 2;
        else if (strcmp(argv[i], "--carshot") == 0) shotMode = 3;
        else if (strcmp(argv[i], "--ballshot") == 0) shotMode = 4;
        else if (strcmp(argv[i], "--connect") == 0 && i + 1 < argc) {
            autoConnect = 1;
            connectArg = argv[++i];
        }
        else if (strcmp(argv[i], "--name") == 0 && i + 1 < argc) {
            nameArg = argv[++i];
        }
        else if (strncmp(argv[i], "--", 2) != 0) carArg = argv[i];
    }

    if (!testMode) settings_load();
    if (connectArg) snprintf(g_set.server, sizeof(g_set.server), "%s", connectArg);
    if (nameArg) snprintf(g_set.name, sizeof(g_set.name), "%.19s", nameArg);
    if (!g_set.name[0]) snprintf(g_set.name, sizeof(g_set.name), "Striker");
    if (!g_set.server[0]) snprintf(g_set.server, sizeof(g_set.server), "127.0.0.1");
    if (g_set.playlist < 1 || g_set.playlist > 3) g_set.playlist = 2;
    if (carArg) {
        for (i = 0; i < CAR_COUNT; i++) if (!strcmp(carArg, CAR_NAMES[i])) g_set.car = i;
    }

    if (!testMode) {
        SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);   /* no MSAA: the scene renders off-screen, FXAA does AA */
        InitWindow(1600, 900, "SARPBC-C");
        SetExitKey(KEY_NULL);               /* Esc opens the pause menu instead of quitting */
        SetTargetFPS(144);
        if (g_set.fullscreen) ToggleBorderlessWindowed();
    }

    /* arena: next to the exe, then ../export_c/arena (falls back to a box arena) */
    snprintf(arenaDir, sizeof(arenaDir), "%sassets/arena/", GetApplicationDirectory());
    snprintf(path, sizeof(path), "%sarena_col.bin", arenaDir);
    if (!FileExists(path)) {
        snprintf(arenaDir, sizeof(arenaDir), "%s../export_c/arena/", GetApplicationDirectory());
        snprintf(path, sizeof(path), "%sarena_col.bin", arenaDir);
    }
    if (!FileExists(path)) {
        snprintf(arenaDir, sizeof(arenaDir), "assets/arena/");
        snprintf(path, sizeof(path), "%sarena_col.bin", arenaDir);
    }
    if (!FileExists(path)) {
        snprintf(arenaDir, sizeof(arenaDir), "../export_c/arena/");
        snprintf(path, sizeof(path), "%sarena_col.bin", arenaDir);
    }
    if (!FileExists(path)) {
        snprintf(arenaDir, sizeof(arenaDir), "export_c/arena/");
        snprintf(path, sizeof(path), "%sarena_col.bin", arenaDir);
    }
    if (!arena_mesh_load(path)) TraceLog(LOG_WARNING, "ARENA: %s not found, using the fallback box arena", path);

    if (testMode) {
        Shader none = { 0 };
        if (!load_car(CAR_NAMES[g_set.car], &cars[0], &crs[0], none, 0)) return 1;
        run_physics_test(&cars[0]);
        return 0;
    }

    rlSetClipPlanes(0.1, 2000.0);
    gfx_init();
    lit = G.lit;

    if (!load_car(CAR_NAMES[g_set.car], &cars[0], &crs[0], lit, 1)) {
        g_set.car = 0;
        if (!load_car(CAR_NAMES[0], &cars[0], &crs[0], lit, 1)) { CloseWindow(); return 1; }
    }
    haveArena = arena_render_load(&ar, arenaDir, lit);

    fieldTex = make_field_texture();
    field = LoadModelFromMesh(GenMeshPlane(2.0f * ARENA_W, 2.0f * (ARENA_L + GOAL_D), 4, 4));
    field.materials[0].shader = lit;
    field.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = fieldTex;

    g_particleTex = make_particle_texture();
    memset(g_particles, 0, sizeof(g_particles));
    g_particleHead = 0;

    ballTex = make_tech_ball_texture();
    ballModel = LoadModelFromMesh(GenMeshSphere(BALL_R, 32, 48));
    ballModel.materials[0].shader = lit;
    ballModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = ballTex;

    pads[0] = V3(-78, 0, -(ARENA_L - 24)); pads[1] = V3(78, 0, -(ARENA_L - 24));
    pads[2] = V3(-78, 0,  (ARENA_L - 24)); pads[3] = V3(78, 0,  (ARENA_L - 24));
    pads[4] = V3(-82, 0, 0);               pads[5] = V3(82, 0, 0);

    /* team sizes from the mode; bots get a mix of the other cars */
#define SETUP_CARS() do { int ts_ = g_set.mode == 0 ? 1 : g_set.mode; \
        nCars = g_set.mode == 0 ? 1 : 2 * ts_; botSkill = g_set.botSkill; \
        for (i = 0; i < nCars; i++) { \
            team[i] = i < ts_ ? 0 : 1; \
            if (i > 0) { int mdl_ = (g_set.car + 3 * i) % CAR_COUNT; \
                if (carModel[i] != mdl_ && load_car(CAR_NAMES[mdl_], &cars[i], &crs[i], lit, 1)) carModel[i] = mdl_; } \
        } } while (0)
#define KICKOFF() do { int ts_ = nCars == 1 ? 1 : nCars / 2; \
        for (i = 0; i < nCars; i++) { Vector3 p_; float y_; \
            kickoff_spot(i % ts_, team[i], ts_, &p_, &y_); car_reset(&cars[i], p_, y_); } \
        memset(bots, 0, sizeof(bots)); ball_reset(&ball); \
        state = ST_COUNTDOWN; stateTimer = 3.0f; acc = 0.0f; pendingJump = 0; camSnap = 1; demoBannerTimer = 0.0f; } while (0)
#define NEW_MATCH() do { SETUP_CARS(); scoreBlue = scoreOrange = 0; matchLen = MATCH_LENGTHS[g_set.matchIdx]; matchTime = matchLen; \
                         for (i = 0; i < PAD_COUNT; i++) { padTimer[i] = 0.0f; } \
                         memset(g_particles, 0, sizeof(g_particles)); g_particleHead = 0; KICKOFF(); } while (0)

    carModel[0] = g_set.car;

    float demoBannerTimer = 0.0f;
    char demoBannerText[64] = { 0 };
    Color demoBannerColor = ORANGE;
    float camShake = 0.0f;
    float matchFoundTimer = 0.0f;       /* "MATCH FOUND" banner after joining a quick match */
    int carAdj = 0;                     /* garage car change, applied after drawing */

    NEW_MATCH();
    for (i = 0; i < MAX_CARS; i++) rcars[i] = cars[i];
    rball = ball;
    cam.position = V3(0, 6, -KICKOFF_Z - 12);
    cam.target = cars[0].pos;
    cam.up = V3(0, 1, 0);
    cam.fovy = fov;
    cam.projection = CAMERA_PERSPECTIVE;
    if (shotMode == 1) screen = SCR_GAME;
    /* start searching the selected playlist on the configured server */
#define START_SEARCH() do { char host_[64]; int port_; \
        split_host_port(g_set.server, host_, sizeof(host_), &port_); \
        onlineMsg[0] = 0; settings_save(); \
        net_client_connect(&netClient, host_, port_, g_set.name, g_set.car, g_set.skin, g_set.playlist); \
        screen = SCR_SEARCHING; } while (0)
    /* back to offline state after an online match (restores our car in slot 0 for the menus) */
#define LEAVE_ONLINE() do { isOnline = 0; \
        if (carModel[0] != g_set.car && load_car(CAR_NAMES[g_set.car], &cars[0], &crs[0], lit, 1)) carModel[0] = g_set.car; \
        carModel[0] = g_set.car; NEW_MATCH(); camSnap = 1; } while (0)
    if (autoConnect) START_SEARCH();

    enum { UA_NONE, UA_GO_MENU, UA_GO_PLAY, UA_GO_ONLINE, UA_GO_OFFLINE, UA_GO_GARAGE, UA_GO_SETTINGS, UA_QUIT,
           UA_FIND_MATCH, UA_CANCEL_SEARCH, UA_START_OFFLINE, UA_RESUME, UA_PAUSE_SETTINGS, UA_RESTART,
           UA_LEAVE_MATCH, UA_REQUEUE, UA_POST_MENU };
    while (!quit && !WindowShouldClose()) {
        float dt = fminf(GetFrameTime(), 0.1f);
        Screen screenAtStart = screen;
        Nav nav = read_nav();
        int act = -1, adj = 0, uiAct = UA_NONE;

        /* ================= game update ================= */
        g_navTextMode = 0;   /* re-armed by ui_end() if a text box still has focus */
        if (isOnline) {
            net_client_poll(&netClient, dt);
            if (netClient.state == NET_DISCONNECTED) {
                /* match over (server sent everyone back) or connection lost */
                snprintf(onlineMsg, sizeof(onlineMsg), "%s", netClient.disconnectReason == DISC_MATCH_OVER
                         ? "Match complete. Ready for another one?" : netClient.statusMsg);
                LEAVE_ONLINE();
                screen = SCR_ONLINE;
            }
        }
        if (screen == SCR_SEARCHING) {
            net_client_poll(&netClient, dt);
            if (netClient.state == NET_DISCONNECTED) {
                snprintf(onlineMsg, sizeof(onlineMsg), "%s", netClient.statusMsg);
                screen = SCR_ONLINE;
            } else if (netClient.state == NET_CONNECTED) {
                /* MATCH FOUND: switch the world over to the server's match */
                int mySlot = netClient.localSlot;
                isOnline = 1;
                online_reset();
                for (i = 0; i < SARP_MAX_CLIENTS; i++) {
                    if (i == mySlot) continue;
                    cars[i].pos = V3(0, -500, 0);
                    cars[i].vel = V3(0, 0, 0);
                    cars[i].rot = (Quaternion){ 0, 0, 0, 1 };
                    cars[i].demolished = 0;
                }
                if (carModel[mySlot] != g_set.car) load_car(CAR_NAMES[g_set.car], &cars[mySlot], &crs[mySlot], lit, 1);
                carModel[mySlot] = g_set.car;
                team[mySlot] = netClient.localTeam;
                {
                    Vector3 kp; float ky;
                    kickoff_spot(mySlot % (netClient.playlist > 0 ? netClient.playlist : 1), team[mySlot], netClient.playlist, &kp, &ky);
                    car_reset(&cars[mySlot], kp, ky);
                }
                for (i = 0; i < SARP_MAX_CLIENTS; i++) { prevPos[i] = cars[i].pos; prevRot[i] = cars[i].rot; rcars[i] = cars[i]; }
                ball_reset(&ball);
                prevBallPos = ball.pos; prevBallRot = ball.rot; rball = ball;
                for (i = 0; i < PAD_COUNT; i++) padTimer[i] = 0.0f;
                memset(g_particles, 0, sizeof(g_particles)); g_particleHead = 0;
                scoreBlue = scoreOrange = 0;
                matchLen = MATCH_TIME; matchTime = MATCH_TIME;
                state = ST_COUNTDOWN; stateTimer = 5.0f;
                acc = 0.0f; pendingJump = 0; camSnap = 1; demoBannerTimer = 0.0f; camShake = 0.0f;
                matchFoundTimer = 2.5f;
                screen = SCR_GAME;
            }
        }
        if (isOnline) {
            int me = netClient.localSlot;
            /* (re)load models for remote players before their state is applied */
            for (i = 0; i < SARP_MAX_CLIENTS; i++) {
                if (!netClient.players[i].active) continue;
                team[i] = netClient.players[i].team;
                if (i != me && carModel[i] != netClient.players[i].car_model % CAR_COUNT) {
                    carModel[i] = netClient.players[i].car_model % CAR_COUNT;
                    load_car(CAR_NAMES[carModel[i]], &cars[i], &crs[i], lit, 1);
                    cars[i].pos = V3(0, -500, 0);   /* force a clean snap on the next snapshot */
                    g_on.lastSnap = netClient.snapCount - 1;
                }
            }
            if (netClient.snapCount != g_on.lastSnap) {
                g_on.lastSnap = netClient.snapCount;
                online_apply_snapshot(&netClient, cars, &ball, prevPos, prevRot, &prevBallPos, &prevBallRot, &camSnap);
            }

            /* match state comes from the server */
            scoreBlue = netClient.scoreBlue;
            scoreOrange = netClient.scoreOrange;
            matchTime = netClient.serverMatchTime;
            stateTimer = netClient.serverStateTimer;
            state = netClient.serverGameState == 0 ? ST_COUNTDOWN :
                    netClient.serverGameState == 2 ? ST_GOAL :
                    netClient.serverGameState == 3 ? ST_OVER : ST_PLAY;

            if (netClient.hasGoalEvent) {
                netClient.hasGoalEvent = 0;
                lastScorer = netClient.goalTeam == 0 ? 1 : 2;
                particles_goal_explosion(ball.pos, lastScorer);
                camShake = 1.0f;
            }
            if (netClient.hasDemoEvent) {
                int v = netClient.demoVictimId, k = netClient.demoKillerId;
                netClient.hasDemoEvent = 0;
                if (v < SARP_MAX_CLIENTS) particles_demolition_explosion(cars[v].pos);
                camShake = fmaxf(camShake, (v == me || k == me) ? 0.8f : 0.3f);
            }
        }

        if (screen == SCR_GAME || (isOnline && (screen == SCR_PAUSE || screen == SCR_SETTINGS))) {
            Input frame = read_input();
            int frozen;


            if (demoBannerTimer > 0.0f) demoBannerTimer -= dt;
            if (camShake > 0.0f) camShake = fmaxf(0.0f, camShake - 2.5f * dt);

            if (screen == SCR_GAME && (IsKeyPressed(KEY_ESCAPE) || IsKeyPressed(KEY_P) ||
                (IsGamepadAvailable(0) && IsGamepadButtonPressed(0, GAMEPAD_BUTTON_MIDDLE_RIGHT)))) {
                screen = SCR_PAUSE;
            }
            if (shotMode) {
                if (frameNo == 0) { state = ST_PLAY; cars[0].boost = BOOST_MAX; }
                frame.throttle = 1.0f;
                frame.boost = frameNo > 60 && frameNo < 200;
                frame.steer = frameNo > 150 && frameNo < 175 ? 1.0f : 0.0f;
                if (frameNo == 80) TakeScreenshot("shot_ball.png");
                if (frameNo == 172) TakeScreenshot("shot_kickoff.png");
                if (++frameNo == 330) { TakeScreenshot("shot.png"); break; }
            }
            /* (online: paused players keep simulating with neutral input) */
            frozen = state == ST_COUNTDOWN || state == ST_OVER || screen != SCR_GAME;

            if (screen == SCR_GAME && (IsKeyPressed(KEY_C) || (IsGamepadAvailable(0) && IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_FACE_UP))))
                ballCam = !ballCam;
            if (IsKeyPressed(KEY_R) || (IsGamepadAvailable(0) && IsGamepadButtonPressed(0, GAMEPAD_BUTTON_MIDDLE_LEFT))) {
                if (!isOnline) {
                    if (state == ST_OVER) NEW_MATCH(); else KICKOFF();
                }
            }

            /* edge-triggered input must survive frames with zero physics ticks */
            pendingJump |= frame.jumpPressed;
            in = frame;
            if (frozen) { memset(&in, 0, sizeof(in)); pendingJump = 0; }

            /* --- fixed-step simulation ----------------------------------- */
            acc += dt;
            while (acc >= 1.0f / PHYS_HZ) {
                float h = 1.0f / PHYS_HZ;
                Input tick = in;
                int activeCars = isOnline ? SARP_MAX_CLIENTS : nCars;
                int mySlot = isOnline ? netClient.localSlot : 0;
                for (i = 0; i < activeCars; i++) { prevPos[i] = cars[i].pos; prevRot[i] = cars[i].rot; }
                prevBallPos = ball.pos; prevBallRot = ball.rot;
                tick.jumpPressed = pendingJump;
                pendingJump = 0;
                if (isOnline) {
                    /* simulate with exactly what the server will receive (bit-identical) */
                    NetInput netIn = ns_input_to_net(&tick);
                    tick = ns_input_from_net(&netIn);
                    net_client_send_tick(&netClient, &netIn);
                }

                if (!isOnline) {
                    /* Demolition respawn timers */
                    for (i = 0; i < nCars; i++) {
                        if (cars[i].demolished) {
                            cars[i].demoTimer -= h;
                            if (cars[i].demoTimer <= 0.0f) {
                                float spawnX = (p_randf() < 0.5f) ? -65.0f : 65.0f;
                                float spawnZ = (team[i] == 0) ? -(ARENA_L - 28.0f) : (ARENA_L - 28.0f);
                                float spawnYaw = (team[i] == 0) ? 0.0f : PI;
                                car_reset(&cars[i], V3(spawnX, 0, spawnZ), spawnYaw);
                                cars[i].boost = BOOST_START;
                                cars[i].demolished = 0;
                                cars[i].demoTimer = 0.0f;
                                if (i == 0) camSnap = 1;
                                particles_impact_burst(cars[i].pos, V3(0, 1, 0), 12.0f);
                            }
                        }
                    }

                    if (!cars[0].demolished) car_step(&cars[0], &tick, h);
                    if (nCars > 1) bot_world_update(&ball, h, pads, padTimer, PAD_COUNT);
                    for (i = 1; i < nCars; i++) {
                        if (cars[i].demolished) continue;
                        Input bi = bot_think(i, cars, team, nCars, &ball, &bots[i], botSkill, h);
                        if (frozen) memset(&bi, 0, sizeof(bi));
                        car_step(&cars[i], &bi, h);
                    }
                    ball_step(&ball, h);
                    for (i = 0; i < nCars; i++) {
                        if (cars[i].demolished) continue;
                        Vector3 prevBVel = ball.vel;
                        if (car_ball_collide(&cars[i], &ball)) {
                            float hitDelta = Vector3Distance(ball.vel, prevBVel);
                            if (hitDelta > 3.0f) {
                                Vector3 hitPoint = Vector3Lerp(cars[i].pos, ball.pos, 0.5f);
                                Vector3 hitNorm = Vector3Normalize(Vector3Subtract(ball.pos, cars[i].pos));
                                particles_impact_burst(hitPoint, hitNorm, hitDelta);
                            }
                        }
                    }
                    for (i = 0; i < nCars; i++) {
                        if (cars[i].demolished) continue;
                        for (j = i + 1; j < nCars; j++) {
                            if (cars[j].demolished) continue;
                            Vector3 velDiff = Vector3Subtract(cars[j].vel, cars[i].vel);
                            float relSpeed = Vector3Length(velDiff);

                            int demo = check_demolition(&cars[i], &cars[j], team[i], team[j]);
                            if (demo != 0) {
                                if (demo == 1) {
                                    demolish_car(j, i, cars, carModel, &demoBannerTimer, demoBannerText, sizeof(demoBannerText), &demoBannerColor, &camShake);
                                } else if (demo == 2) {
                                    demolish_car(i, j, cars, carModel, &demoBannerTimer, demoBannerText, sizeof(demoBannerText), &demoBannerColor, &camShake);
                                } else if (demo == 3) {
                                    demolish_car(i, j, cars, carModel, &demoBannerTimer, demoBannerText, sizeof(demoBannerText), &demoBannerColor, &camShake);
                                    demolish_car(j, i, cars, carModel, &demoBannerTimer, demoBannerText, sizeof(demoBannerText), &demoBannerColor, &camShake);
                                }
                                continue;
                            }

                            if (car_car_collide(&cars[i], &cars[j])) {
                                if (relSpeed > 6.0f) {
                                    Vector3 hitPoint = Vector3Lerp(cars[i].pos, cars[j].pos, 0.5f);
                                    Vector3 hitNorm = Vector3Normalize(velDiff);
                                    particles_impact_burst(hitPoint, hitNorm, relSpeed);
                                }
                            }
                        }
                    }

                    for (i = 0; i < PAD_COUNT; i++) {
                        padTimer[i] -= h;
                        for (j = 0; j < nCars && padTimer[i] <= 0.0f; j++) {
                            if (cars[j].demolished) continue;
                            float dx = cars[j].pos.x - pads[i].x, dz = cars[j].pos.z - pads[i].z;
                            if (dx*dx + dz*dz < PAD_RADIUS * PAD_RADIUS && cars[j].pos.y < 4.0f) {
                                cars[j].boost = BOOST_MAX;
                                padTimer[i] = PAD_RESPAWN;
                                particles_pad_pickup(pads[i]);
                            }
                        }
                    }

                    if (state == ST_PLAY) {
                        if (fabsf(ball.pos.z) > ARENA_L + BALL_R) {
                            lastScorer = ball.pos.z > 0 ? 1 : 2;
                            if (lastScorer == 1) scoreBlue++; else scoreOrange++;
                            state = ST_GOAL; stateTimer = 3.0f;
                            particles_goal_explosion(ball.pos, lastScorer);
                            camShake = 1.0f;

                            /* Shockwave blast impulse on nearby cars */
                            for (i = 0; i < nCars; i++) {
                                if (cars[i].demolished) continue;
                                Vector3 toCar = Vector3Subtract(cars[i].pos, ball.pos);
                                float d = Vector3Length(toCar);
                                if (d < 65.0f) {
                                    float p = (1.0f - d / 65.0f);
                                    Vector3 dir = d > 0.1f ? Vector3Scale(toCar, 1.0f / d) : V3(0, 1, 0);
                                    dir.y = fmaxf(dir.y, 0.45f);
                                    dir = Vector3Normalize(dir);
                                    cars[i].vel = Vector3Add(cars[i].vel, Vector3Scale(dir, p * 45.0f + 10.0f));
                                    cars[i].angVel = Vector3Add(cars[i].angVel, p_randv3(-8.0f, 8.0f));
                                }
                            }
                        }
                        if (matchLen > 0.0f) {
                            matchTime -= h;
                            if (matchTime <= 0.0f) { matchTime = 0.0f; state = ST_OVER; }
                        }
                    }
                } else {
                    /* ONLINE: predict the whole world one tick (our car with our input,
                     * remote cars with their last known input, the ball), then record
                     * our input + state so the next snapshot can rewind and replay. */
                    online_step_world(cars, &ball, &netClient, &tick, h, 1);
                    online_record(&netClient, &tick, &cars[mySlot]);
                }

                for (i = 0; i < activeCars; i++) {
                    if (isOnline && !netClient.players[i].active) continue;
                    if (cars[i].demolished) continue;
                    if (cars[i].boosting && screen == SCR_GAME) {
                        particles_boost_emit(&cars[i], team[i] == 0);
                    }
                    Vector3 cr = car_right(&cars[i]);
                    float latSpeed = fabsf(Vector3DotProduct(cars[i].vel, cr));
                    int isSliding = (i == mySlot) ? (in.slide) : 0;
                    particles_drift_emit(&cars[i], latSpeed, isSliding);
                    particles_supersonic_emit(&cars[i]);
                }
                particles_update(h);
                acc -= h;
            }

            /* --- match state machine ------------------------------------- */
            if (screen == SCR_GAME && !isOnline) {
                stateTimer -= dt;
                if (state == ST_COUNTDOWN && stateTimer <= 0.0f) state = ST_PLAY;
                if (state == ST_GOAL && stateTimer <= 0.0f) {
                    int over = matchLen > 0.0f && matchTime <= 0.0f;
                    KICKOFF();
                    if (over) state = ST_OVER;
                }
            }

            /* --- render state: interpolate between the last two physics ticks so motion
             * is smooth at any frame rate (teleports such as kickoffs snap) --------- */
            {
                float alpha = clampf(acc * PHYS_HZ, 0.0f, 1.0f);
                int activeCars = isOnline ? SARP_MAX_CLIENTS : nCars;
                int mySlot = isOnline ? netClient.localSlot : 0;
                if (isOnline) online_decay_visuals(dt, mySlot);
                for (i = 0; i < activeCars; i++) {
                    if (isOnline && i != mySlot && !netClient.players[i].active) {
                        rcars[i].pos = V3(0, -500, 0);
                        continue;
                    }
                    rcars[i] = cars[i];
                    if (Vector3Distance(prevPos[i], cars[i].pos) < 8.0f) {
                        rcars[i].pos = Vector3Lerp(prevPos[i], cars[i].pos, alpha);
                        rcars[i].rot = QuaternionSlerp(prevRot[i], cars[i].rot, alpha);
                    }
                    if (isOnline) {
                        rcars[i].pos = Vector3Add(rcars[i].pos, g_on.visOfs[i]);
                        rcars[i].rot = QuaternionNormalize(QuaternionMultiply(g_on.visRot[i], rcars[i].rot));
                    }
                }
                rball = ball;
                if (Vector3Distance(prevBallPos, ball.pos) < 8.0f) {
                    rball.pos = Vector3Lerp(prevBallPos, ball.pos, alpha);
                    rball.rot = QuaternionSlerp(prevBallRot, ball.rot, alpha);
                }
                if (isOnline) {
                    rball.pos = Vector3Add(rball.pos, g_on.ballVisOfs);
                    rball.rot = QuaternionNormalize(QuaternionMultiply(g_on.ballVisRot, rball.rot));
                }
            }

            /* --- chase camera: rigidly attached at a smoothed heading -------------- */
            if (screen == SCR_GAME) {
                int mySlot = isOnline ? netClient.localSlot : 0;
                const Car *pc = &rcars[mySlot];
                Vector3 f = car_fwd(pc), dirv, want, look;
                float kd = 1.0f - expf(-8.0f * dt);
                if (IsKeyDown(KEY_LEFT_BRACKET))  g_set.camDist = fmaxf(3.0f,  g_set.camDist - 4.0f * dt);
                if (IsKeyDown(KEY_RIGHT_BRACKET)) g_set.camDist = fminf(14.0f, g_set.camDist + 4.0f * dt);
                if (ballCam) { dirv = Vector3Subtract(rball.pos, pc->pos); }
                else         { dirv = pc->wheelsOnGround >= 3 ? f : pc->vel; if (Vector3Length(dirv) < 2.0f) dirv = f; }
                dirv.y = 0.0f;
                if (Vector3Length(dirv) < 0.01f) dirv = V3(0, 0, 1);
                dirv = Vector3Normalize(dirv);
                /* only the heading is smoothed; the distance to the car stays fixed */
                if (Vector3Length(camDir) < 0.5f || camSnap) { camDir = dirv; camY = pc->pos.y; camSnap = 0; }
                camDir = Vector3Normalize(Vector3Lerp(camDir, dirv, kd));
                camY = Lerp(camY, pc->pos.y, 1.0f - expf(-12.0f * dt));   /* soften jump/landing bob */
                want = V3(pc->pos.x - camDir.x * g_set.camDist, camY + g_set.camDist * g_set.camHeight, pc->pos.z - camDir.z * g_set.camDist);
                want.x = clampf(want.x, -ARENA_W + 1, ARENA_W - 1);
                want.y = clampf(want.y, 0.5f, ARENA_H - 1);
                want.z = clampf(want.z, -ARENA_L - GOAL_D + 1, ARENA_L + GOAL_D - 1);
                look = ballCam ? Vector3Lerp(V3(pc->pos.x, camY + 1.0f, pc->pos.z), rball.pos, 0.25f)
                               : V3(pc->pos.x + camDir.x * 2.0f, camY + 0.9f, pc->pos.z + camDir.z * 2.0f);
                cam.position = want;
                cam.target   = look;
                if (camShake > 0.01f) {
                    cam.position = Vector3Add(cam.position, p_randv3(-camShake * 0.45f, camShake * 0.45f));
                    cam.target   = Vector3Add(cam.target,   p_randv3(-camShake * 0.25f, camShake * 0.25f));
                }
                if (shotMode == 1 && frameNo > 30 && frameNo < 150) {   /* jitter metric: car-to-camera distance spread */
                    static float dmin = 1e9f, dmax = 0.0f, dsum = 0.0f, dprev = -1.0f, jmax = 0.0f;
                    float dcc = Vector3Distance(cam.position, pc->pos);
                    dmin = fminf(dmin, dcc); dmax = fmaxf(dmax, dcc);
                    if (dprev >= 0.0f) { dsum += fabsf(dcc - dprev); jmax = fmaxf(jmax, fabsf(dcc - dprev)); }
                    dprev = dcc;
                    if (frameNo == 149) printf("camera-car distance: %.3f..%.3f m, mean frame change %.4f m, max %.4f m\n", dmin, dmax, dsum / 118.0f, jmax);
                }
                /* raylib fovy is VERTICAL: 60 deg ~= 95 deg horizontal at 16:9 */
                fov = Lerp(fov, g_set.fov + (pc->boosting && g_set.boostFov ? 8.0f : 0.0f), 1.0f - expf(-4.0f * dt));
                cam.fovy = fov;
            }
        } else if (screen != SCR_PAUSE && !(screen == SCR_SETTINGS && settingsFrom == SCR_PAUSE)) {
            /* menu backdrop: parked car, slow orbit, framed right of centre so the menus sit on the left */
            float a;
            Vector3 fwd_, side_;
            menuT += dt;
            a = menuT * 0.18f;
            car_reset(&cars[0], V3(0, 0, -KICKOFF_Z), -PI / 2.0f + menuT * 0.35f);
            cam.target   = Vector3Add(cars[0].pos, V3(0, 0.7f, 0));
            cam.position = Vector3Add(cars[0].pos, V3(sinf(a) * 7.5f, 2.4f, cosf(a) * 7.5f));
            fwd_  = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
            side_ = Vector3Normalize(Vector3CrossProduct(fwd_, V3(0, 1, 0)));
            if (screen != SCR_SEARCHING) {
                cam.position = Vector3Subtract(cam.position, Vector3Scale(side_, 2.6f));
                cam.target   = Vector3Subtract(cam.target, Vector3Scale(side_, 2.6f));
            }
            cam.fovy = fov = 50.0f;
            for (i = 0; i < nCars; i++) rcars[i] = cars[i];
            rball = ball;
            if (shotMode == 2 && ++frameNo % 60 == 0) {   /* --menushot: one capture per front-end screen */
                static const int shotScr[] = { SCR_MENU, SCR_PLAY, SCR_ONLINE, SCR_OFFLINE, SCR_GARAGE };
                static const char *shotName[] = { "menu.png", "menu_play.png", "menu_online.png", "menu_offline.png", "menu_garage.png" };
                int k = frameNo / 60 - 1;
                TakeScreenshot(shotName[k]);
                if (k + 1 < 5) screen = shotScr[k + 1]; else quit = 1;
            }
            if (shotMode == 3) {   /* --carshot: 4 views, 90 deg apart */
                int view = frameNo / 20;
                car_reset(&cars[0], V3(0, 0, -KICKOFF_Z), 0.0f);
                a = view * PI / 2.0f + 0.6f;
                cam.target   = Vector3Add(cars[0].pos, V3(0, 0.3f, 0));
                cam.position = Vector3Add(cars[0].pos, V3(sinf(a) * 5.0f, 1.4f, cosf(a) * 5.0f));
                if (frameNo % 20 == 19) TakeScreenshot(TextFormat("car_%d.png", view));
                if (++frameNo >= 80) quit = 1;
            }
            if (shotMode == 4) {   /* --ballshot: 4 views of the tech ball up close */
                int view = frameNo / 20;
                a = view * PI / 2.0f + 0.4f;
                cam.target   = ball.pos;
                cam.position = Vector3Add(ball.pos, V3(sinf(a) * 4.0f, 1.8f, cosf(a) * 4.0f));
                if (frameNo % 20 == 19) TakeScreenshot(TextFormat("ball_%d.png", view));
                if (++frameNo >= 80) quit = 1;
            }
        }

        /* ================= draw ================= */
        gfx_resize();
        /* dynamic lights: goal glows, live boost pads, boosting cars */
        G.ptCount = 0;
        gfx_light(V3(0, 5.0f, -ARENA_L - 3.0f), 55.0f, V3(0.5f, 1.3f, 4.0f));
        gfx_light(V3(0, 5.0f,  ARENA_L + 3.0f), 55.0f, V3(4.0f, 1.6f, 0.4f));
        for (i = 0; i < PAD_COUNT; i++)
            if (padTimer[i] <= 0.0f) gfx_light(V3(pads[i].x, 1.6f, pads[i].z), 10.0f, V3(3.0f, 1.5f, 0.35f));
        for (i = 0; i < nCars; i++)
            if (rcars[i].boosting && !rcars[i].demolished && screen == SCR_GAME)
                gfx_light(Vector3Add(rcars[i].pos, car_to_world(&rcars[i], V3(rcars[i].boxMin.x - 0.9f, 0.3f, 0))),
                          9.0f, team[i] == 0 ? V3(0.8f, 2.2f, 5.0f) : V3(5.0f, 2.2f, 0.6f));

        if (g_set.shadows) {
            /* near cascade sits a bit ahead of the player, where the camera looks */
            Vector3 look = Vector3Subtract(cam.target, cam.position), focus;
            int k;
            int mySlot = isOnline ? netClient.localSlot : 0;
            int activeCars = isOnline ? SARP_MAX_CLIENTS : nCars;
            look.y = 0.0f;
            focus = Vector3Add(rcars[mySlot].pos, Vector3Scale(Vector3Normalize(look), SHADOW_NEAR_SIZE * 0.25f));
            for (k = 0; k < 2; k++) {
                Shader keep = ballModel.materials[0].shader;
                gfx_shadow_begin(k, focus);
                if (haveArena) arena_render_draw(&ar, 0, &G.depth);
                for (i = 0; i < activeCars; i++) {
                    if (isOnline && !netClient.players[i].active) continue;
                    if (!rcars[i].demolished)
                        car_render_draw(&crs[i], &rcars[i], team[i], i == mySlot ? g_set.skin : (isOnline ? netClient.players[i].skin : 0), &G.depth);
                }
                ballModel.materials[0].shader = G.depth;
                ballModel.transform = QuaternionToMatrix(rball.rot);
                DrawModel(ballModel, rball.pos, 1.0f, WHITE);
                ballModel.materials[0].shader = keep;
                gfx_shadow_end();
            }
        }

        gfx_scene_begin(cam, g_set.shadows);
            if (haveArena) {
                arena_render_draw(&ar, 0, NULL);
            } else {
                gfx_mat(0.2f, 24.0f, 0.15f, 0.0f);
                DrawModel(field, V3(0, 0, 0), 1.0f, WHITE);
                gfx_emissive_begin(1.0f);   /* unlit vertex colours: just sRGB -> linear for the HDR target */
                draw_arena_shell();
                EndShaderMode();
            }
            gfx_emissive_begin(1.6f);
            for (i = 0; i < PAD_COUNT; i++) {
                Color pc = padTimer[i] <= 0.0f ? (Color){ 255, 170, 40, 255 } : (Color){ 90, 70, 40, 255 };
                DrawCylinder(V3(pads[i].x, 0.06f, pads[i].z), PAD_RADIUS, PAD_RADIUS, 0.12f, 24, pc);
            }
            EndShaderMode();
            gfx_emissive_begin(4.0f);   /* deep orange: brighter/yellower tints clip to flat yellow in the tonemap */
            for (i = 0; i < PAD_COUNT; i++)
                if (padTimer[i] <= 0.0f)
                    DrawSphere(V3(pads[i].x, 1.6f + 0.3f * sinf((float)GetTime() * 3.0f), pads[i].z), 0.7f, (Color){ 255, 120, 25, 255 });
            EndShaderMode();
            if (!g_set.shadows) {   /* blob shadows only when real shadows are off */
                int activeCars = isOnline ? SARP_MAX_CLIENTS : nCars;
                DrawCylinder(V3(rball.pos.x, 0.07f, rball.pos.z), BALL_R * 0.9f, BALL_R * 0.9f, 0.01f, 24, (Color){ 0, 0, 0, 90 });
                for (i = 0; i < activeCars; i++) {
                    if (isOnline && !netClient.players[i].active) continue;
                    if (!rcars[i].demolished)
                        DrawCylinder(V3(rcars[i].pos.x, 0.07f, rcars[i].pos.z), 1.3f, 1.3f, 0.01f, 24, (Color){ 0, 0, 0, 90 });
                }
            }

            gfx_mat(0.60f, 64.0f, 0.25f, 0.05f);
            ballModel.transform = QuaternionToMatrix(rball.rot);
            DrawModel(ballModel, rball.pos, 1.0f, WHITE);
            {
                int activeCars = isOnline ? SARP_MAX_CLIENTS : nCars;
                int mySlot = isOnline ? netClient.localSlot : 0;
                for (i = 0; i < activeCars; i++) {
                    if (isOnline && !netClient.players[i].active) continue;
                    if (!rcars[i].demolished)
                        car_render_draw(&crs[i], &rcars[i], team[i], i == mySlot ? g_set.skin : (isOnline ? netClient.players[i].skin : 0), NULL);
                }
            }
            /* boost flame: additive, no depth writes (the core sits inside the outer cone);
             * team-colored outer plume (blue plasma vs orange fire) and bright white-hot core */
            rlDrawRenderBatchActive();
            rlDisableDepthMask();
            BeginBlendMode(BLEND_ADDITIVE);
            for (j = 0; j < 2; j++) {
                int activeCars = isOnline ? SARP_MAX_CLIENTS : nCars;
                gfx_emissive_begin(j == 0 ? 2.5f : 5.5f);
                for (i = 0; i < activeCars; i++) {
                    if (isOnline && !netClient.players[i].active) continue;
                    Car *c = &rcars[i];
                    if (c->boosting && !c->demolished && screen == SCR_GAME) {
                        float t = (float)GetTime() * 40.0f + (float)i * 1.7f;
                        Vector3 back = Vector3Add(c->pos, car_to_world(c, V3(c->boxMin.x - 0.2f, 0.12f, 0)));
                        Vector3 bk = Vector3Scale(car_fwd(c), -1.0f);
                        int isBlue = (team[i] == 0);
                        if (j == 0) {
                            Vector3 tail = Vector3Add(back, Vector3Scale(bk, 1.7f + 0.45f * sinf(t)));
                            Color outerCol = isBlue ? (Color){ 30, 140, 255, 210 } : (Color){ 255, 85, 15, 210 };
                            DrawCylinderEx(back, tail, 0.32f, 0.03f, 12, outerCol);
                        } else {
                            Vector3 core = Vector3Add(back, Vector3Scale(bk, 0.9f + 0.25f * sinf(t * 1.3f + 1.0f)));
                            Color coreCol = isBlue ? (Color){ 180, 235, 255, 240 } : (Color){ 255, 220, 130, 240 };
                            DrawCylinderEx(back, core, 0.18f, 0.02f, 10, coreCol);
                        }
                    }
                }
                EndShaderMode();   /* flushes, so each layer gets its own intensity */
            }
            EndBlendMode();

            /* --- Particles: alpha smoke then additive emissive sparks & flames --- */
            if (screen == SCR_GAME || screen == SCR_PAUSE || (screen == SCR_SETTINGS && settingsFrom == SCR_PAUSE)) {
                rlDisableBackfaceCulling();
                BeginBlendMode(BLEND_ALPHA);
                gfx_emissive_begin(1.0f);
                particles_draw_pass(cam, g_particleTex, 0);
                EndShaderMode();
                EndBlendMode();

                BeginBlendMode(BLEND_ADDITIVE);
                gfx_emissive_begin(3.0f);
                particles_draw_pass(cam, g_particleTex, 1);
                EndShaderMode();
                EndBlendMode();
                rlEnableBackfaceCulling();
            }

            rlEnableDepthMask();
            if (haveArena) {
                gfx_emissive_begin(2.5f);
                draw_goal_nets();
                EndShaderMode();
                arena_render_draw(&ar, 1, NULL);
            } else {
                gfx_emissive_begin(1.0f);
                draw_goals();
                EndShaderMode();
            }
        gfx_scene_end();
        {
            int mySlot = isOnline ? netClient.localSlot : 0;
            gfx_post(g_set.bloom, screen == SCR_GAME && rcars[mySlot].boosting ? 1.0f : 0.0f);
        }

        BeginDrawing();
        ClearBackground(BLACK);
        gfx_present();

        /* --- HUD (in game and behind the pause menu) --------------------- */
        if (screen == SCR_GAME || screen == SCR_PAUSE || (screen == SCR_SETTINGS && settingsFrom == SCR_PAUSE)) {
            int sw = GetScreenWidth(), sh = GetScreenHeight();
            int mySlot = isOnline ? netClient.localSlot : 0;
            float speed = Vector3Length(rcars[mySlot].vel);

            draw_hud_scoreboard(sw, scoreBlue, scoreOrange, matchTime, matchLen, state);
            draw_hud_boost_and_speed(sw, sh, rcars[mySlot].boost, speed, rcars[mySlot].boosting);
            draw_hud_tactical_badges(sw, sh, ballCam,
                                     CAR_NAMES[isOnline ? netClient.players[mySlot].car_model : g_set.car],
                                     isOnline ? PLAYLIST_LABELS[netClient.playlist % 4] : MODE_NAMES[g_set.mode],
                                     isOnline ? (team[mySlot] == 0 ? "BLUE TEAM" : "ORANGE TEAM") : SKILL_NAMES[botSkill],
                                     g_set.showFps, g_set.showHints && screen == SCR_GAME);

            if (!isOnline) {
                for (i = 1; i < nCars; i++) {
                    if (rcars[i].demolished) continue;
                    Vector3 above = Vector3Add(rcars[i].pos, V3(0, 2.2f, 0));
                    Vector3 toC = Vector3Subtract(above, cam.position);
                    Vector2 sp;
                    const char *tag = TextFormat("BOT %s", CAR_NAMES[carModel[i] < 0 ? 0 : carModel[i]]);
                    if (Vector3DotProduct(toC, Vector3Subtract(cam.target, cam.position)) <= 0.0f) continue;   /* behind the camera */
                    sp = GetWorldToScreen(above, cam);
                    DrawText(tag, (int)sp.x - MeasureText(tag, 16) / 2, (int)sp.y, 16,
                             team[i] == 0 ? (Color){ 120, 180, 255, 230 } : (Color){ 255, 170, 90, 230 });
                }
            } else {
                Vector3 cpos[SARP_MAX_CLIENTS];
                int cdemo[SARP_MAX_CLIENTS];
                for (i = 0; i < SARP_MAX_CLIENTS; i++) {
                    cpos[i] = rcars[i].pos;
                    cdemo[i] = rcars[i].demolished;
                }
                net_client_draw_nameplates(&netClient, cpos, team, cdemo, cam);
                net_client_draw_hud(&netClient, sw, sh);
                if (IsKeyDown(KEY_TAB)) {
                    net_client_draw_scoreboard(&netClient, sw, sh);
                }
            }

            if (screen == SCR_GAME) {
                if (demoBannerTimer > 0.0f) {
                    float alpha = fminf(demoBannerTimer * 2.5f, 1.0f);
                    int bw = MeasureText(demoBannerText, 38);
                    int cardW = bw + 80, cardH = 54;
                    int bx = sw/2 - cardW/2, by = sh/2 - 135;
                    DrawRectangleRounded((Rectangle){ (float)bx + 2, (float)by + 3, (float)cardW, (float)cardH }, 0.35f, 6, (Color){ 0, 0, 0, (unsigned char)(160 * alpha) });
                    DrawRectangleRounded((Rectangle){ (float)bx, (float)by, (float)cardW, (float)cardH }, 0.35f, 6, (Color){ 18, 12, 12, (unsigned char)(235 * alpha) });
                    DrawRectangleRoundedLinesEx((Rectangle){ (float)bx, (float)by, (float)cardW, (float)cardH }, 0.35f, 6, 2.0f, (Color){ 255, 70, 30, (unsigned char)(255 * alpha) });
                    DrawText("▲", bx + 16, by + cardH/2 - 10, 20, (Color){ 255, 80, 40, (unsigned char)(255 * alpha) });
                    DrawText("▲", bx + cardW - 30, by + cardH/2 - 10, 20, (Color){ 255, 80, 40, (unsigned char)(255 * alpha) });
                    DrawText(demoBannerText, sw/2 - bw/2 + 2, by + 10 + 2, 38, (Color){ 0, 0, 0, (unsigned char)(220 * alpha) });
                    DrawText(demoBannerText, sw/2 - bw/2, by + 10, 38, (Color){ demoBannerColor.r, demoBannerColor.g, demoBannerColor.b, (unsigned char)(255 * alpha) });
                }
                if (cars[mySlot].demolished) {
                    const char *respawnMsg = TextFormat("RESPAWNING IN %d...", (int)ceilf(fmaxf(0.01f, cars[mySlot].demoTimer)));
                    int rw = MeasureText(respawnMsg, 28);
                    int rCardW = rw + 50, rCardH = 44;
                    int rx = sw/2 - rCardW/2, ry = sh/2 - 65;
                    DrawRectangleRounded((Rectangle){ (float)rx, (float)ry, (float)rCardW, (float)rCardH }, 0.35f, 6, (Color){ 10, 14, 22, 220 });
                    DrawRectangleRoundedLinesEx((Rectangle){ (float)rx, (float)ry, (float)rCardW, (float)rCardH }, 0.35f, 6, 1.5f, (Color){ 220, 60, 40, 220 });
                    DrawText(respawnMsg, sw/2 - rw/2, ry + 8, 28, (Color){ 245, 245, 250, 245 });
                }

                if (matchFoundTimer > 0.0f) {
                    float al = fminf(1.0f, matchFoundTimer * 1.5f);
                    const char *mf = "MATCH FOUND";
                    matchFoundTimer -= dt;
                    DrawRectangle(0, sh / 2 - 270, sw, 76, (Color){ 6, 10, 18, (unsigned char)(200 * al) });
                    DrawRectangle(0, sh / 2 - 270, sw, 3, ColorAlpha(UI_ACCENT, al));
                    DrawRectangle(0, sh / 2 - 197, sw, 3, ColorAlpha(UI_ACCENT, al));
                    DrawText(mf, sw / 2 - MeasureText(mf, 52) / 2, sh / 2 - 258, 52, ColorAlpha(UI_ACCENT_HI, al));
                }
                if (state == ST_COUNTDOWN) {
                    const char *t = TextFormat("%d", (int)ceilf(stateTimer));
                    if (isOnline) {
                        const char *pl = TextFormat("QUICK MATCH  -  %s", PLAYLIST_LABELS[netClient.playlist % 4]);
                        ui_text_c(pl, sw * 0.5f, sh / 2 - 182, 22, UI_TEXT);
                    }
                    int tw = MeasureText(t, 84);
                    int cdR = 58;
                    DrawCircle(sw/2 + 2, sh/2 - 95 + 3, cdR, (Color){ 0, 0, 0, 140 });
                    DrawCircle(sw/2, sh/2 - 95, cdR, (Color){ 12, 16, 26, 235 });
                    DrawCircleLines(sw/2, sh/2 - 95, cdR, (Color){ 255, 175, 45, 240 });
                    float tFrac = fminf(1.0f, fmaxf(0.0f, stateTimer - floorf(stateTimer)));
                    DrawRing((Vector2){ (float)(sw/2), (float)(sh/2 - 95) }, cdR - 5, cdR, 0, 360.0f * tFrac, 36, (Color){ 255, 210, 80, 255 });
                    DrawText(t, sw/2 - tw/2, sh/2 - 95 - 42, 84, RAYWHITE);
                } else if (state == ST_GOAL) {
                    const char *t = lastScorer == 1 ? "BLUE SCORES!" : "ORANGE SCORES!";
                    int gw = MeasureText(t, 58);
                    int bW = gw + 90, bH = 72;
                    int bx = sw/2 - bW/2, by = sh/2 - 120;
                    Color teamGlow = lastScorer == 1 ? (Color){ 30, 120, 255, 255 } : (Color){ 255, 120, 30, 255 };
                    Color teamBorder = lastScorer == 1 ? (Color){ 90, 180, 255, 255 } : (Color){ 255, 170, 50, 255 };
                    DrawRectangleRounded((Rectangle){ (float)bx + 3, (float)by + 4, (float)bW, (float)bH }, 0.25f, 6, (Color){ 0, 0, 0, 150 });
                    DrawRectangleRounded((Rectangle){ (float)bx, (float)by, (float)bW, (float)bH }, 0.25f, 6, (Color){ 12, 16, 26, 240 });
                    DrawRectangleRoundedLinesEx((Rectangle){ (float)bx, (float)by, (float)bW, (float)bH }, 0.25f, 6, 2.5f, teamBorder);
                    DrawRectangle(bx + 20, by + bH - 5, bW - 40, 3, teamBorder);
                    DrawText(t, sw/2 - gw/2 + 2, by + 8 + 2, 58, (Color){ 0, 0, 0, 220 });
                    DrawText(t, sw/2 - gw/2, by + 8, 58, teamGlow);
                }
            }
        }

        /* --- front-end screens (immediate mode: drawn and hit-tested together) --- */
        if (screen != screenAtStart) memset(&nav, 0, sizeof(nav));   /* don't let the key that opened a menu also act in it */
        {
            int sw = GetScreenWidth(), sh = GetScreenHeight();
            const char *paintName = g_set.skin ? CAR_SKIN_NAMES[g_set.car] : "Team colours";
            const char *hints = "ARROWS / WASD / D-PAD  Move     ENTER / A  Select     ESC / B  Back";

            if (screen == SCR_MENU && shotMode != 3 && shotMode != 4) {
                float bx = 64, by = sh * 0.37f, bw = 440, bh = 72, gap = 14;
                ui_begin(SCR_MENU, nav, 0);
                DrawRectangleGradientH(0, 0, (int)(sw * 0.6f), sh, (Color){ 4, 6, 12, 220 }, (Color){ 4, 6, 12, 0 });
                ui_chrome(NULL, NULL, hints);
                ui_text("SUPERSONIC ACROBATIC", 66, 52, 28, UI_ACCENT);
                ui_text("ROCKET-POWERED", 64, 86, 58, UI_TEXT);
                ui_text("BATTLE-CARS", 64, 146, 58, UI_TEXT);
                DrawRectangle(66, 214, 300, 3, UI_ACCENT);
                DrawText("CHAMPIONSHIP EDITION", 66, 226, 18, (Color){ 180, 215, 255, 230 });
                if (ui_button((Rectangle){ bx, by, bw, bh }, "PLAY", "Quick Match  -  Exhibition  -  Free Play", 1)) uiAct = UA_GO_PLAY;
                if (ui_button((Rectangle){ bx, by + (bh + gap), bw, bh }, "GARAGE", "Choose your car and paint", 0)) uiAct = UA_GO_GARAGE;
                if (ui_button((Rectangle){ bx, by + 2 * (bh + gap), bw, bh }, "SETTINGS", "Camera, controls and graphics", 0)) uiAct = UA_GO_SETTINGS;
                if (ui_button((Rectangle){ bx, by + 3 * (bh + gap), bw, 56 }, "EXIT", NULL, 0)) uiAct = UA_QUIT;
                ui_profile(g_set.name, CAR_NAMES[g_set.car], paintName);
                ui_end();
            } else if (screen == SCR_PLAY) {
                float cw = fminf(440.0f, (sw - 200) * 0.5f), ch = 320, cy = sh * 0.5f - 170, cx = sw * 0.5f - cw - 22;
                ui_begin(SCR_PLAY, nav, 0);
                DrawRectangle(0, 0, sw, sh, (Color){ 0, 0, 0, 110 });
                ui_chrome("PLAY", "Choose how you want to play", hints);
                if (ui_card((Rectangle){ cx, cy, cw, ch }, "QUICK MATCH", "ONLINE",
                            "Get matched with other players in\n1v1, 2v2 or 3v3. If nobody else\nis around, bots fill the empty seats.", 1, 2))
                    uiAct = UA_GO_ONLINE;
                if (ui_card((Rectangle){ cx + cw + 44, cy, cw, ch }, "EXHIBITION", "OFFLINE",
                            "Play a match against bots of your\nchosen difficulty, or warm up in\nfree play.", 0, 1))
                    uiAct = UA_GO_OFFLINE;
                if (ui_button((Rectangle){ 64, (float)sh - 150, 220, 52 }, "BACK", NULL, 0) || nav.back) uiAct = UA_GO_MENU;
                ui_profile(g_set.name, CAR_NAMES[g_set.car], paintName);
                ui_end();
            } else if (screen == SCR_ONLINE) {
                static const char *plTitle[] = { "", "DUEL", "DOUBLES", "STANDARD" };
                static const char *plTag[]   = { "", "1v1", "2v2", "3v3" };
                static const char *plDesc[]  = { "", "Just you and one opponent.\nNo teammates, no excuses.",
                                                     "Team up with a partner.\nThe classic way to play.",
                                                     "Three players per side.\nFull-team chaos." };
                float cw = fminf(380.0f, (sw - 128 - 48) / 3.0f), ch = 270, cy = 150, x0 = 64;
                float ry = cy + ch + 34;
                int p;
                ui_begin(SCR_ONLINE, nav, g_set.playlist - 1);
                DrawRectangle(0, 0, sw, sh, (Color){ 0, 0, 0, 110 });
                ui_chrome("QUICK MATCH", "Online  -  pick a playlist", hints);
                for (p = 1; p <= 3; p++) {
                    if (ui_card((Rectangle){ x0 + (p - 1) * (cw + 24), cy, cw, ch }, plTitle[p], plTag[p], plDesc[p], g_set.playlist == p, p)) {
                        if (g_set.playlist == p) uiAct = UA_FIND_MATCH;   /* confirm an already selected playlist */
                        g_set.playlist = p;
                    }
                }
                ui_textfield((Rectangle){ x0, ry, 380, 64 }, "SERVER  (host or host:port)", g_set.server, (int)sizeof(g_set.server), 1);
                ui_textfield((Rectangle){ x0 + 400, ry, 300, 64 }, "PLAYER NAME", g_set.name, 20, 0);
                if (ui_button((Rectangle){ x0 + 720, ry, fmaxf(260.0f, 3 * cw + 48 - 720), 64 }, "FIND MATCH",
                              TextFormat("%s  -  %s", plTag[g_set.playlist], CAR_NAMES[g_set.car]), 1))
                    uiAct = UA_FIND_MATCH;
                if (onlineMsg[0]) ui_text(onlineMsg, (int)x0, (int)ry + 82, 20, (Color){ 255, 205, 110, 255 });
                DrawText("If nobody else is searching, bots fill the empty seats after a few minutes. "
                         "Players who leave a match are replaced by bots.", (int)x0, (int)ry + 114, 15, UI_TEXT_DIM);
                if (ui_button((Rectangle){ 64, (float)sh - 150, 220, 52 }, "BACK", NULL, 0) || nav.back) uiAct = UA_GO_PLAY;
                ui_profile(g_set.name, CAR_NAMES[g_set.car], paintName);
                ui_end();
            } else if (screen == SCR_SEARCHING) {
                Rectangle pr = { sw * 0.5f - 320, sh * 0.5f - 210, 640, 420 };
                int queued = netClient.state == NET_QUEUED;
                int secs = (int)netClient.queueSearchSec, need = netClient.queueNeeded > 0 ? netClient.queueNeeded : 2 * g_set.playlist;
                int have = netClient.queueInQueue > 0 ? netClient.queueInQueue : 1, k;
                ui_begin(SCR_SEARCHING, nav, 0);
                DrawRectangle(0, 0, sw, sh, (Color){ 0, 0, 0, 120 });
                ui_chrome("QUICK MATCH", PLAYLIST_LABELS[g_set.playlist], NULL);
                ui_panel(pr, 0.06f, UI_PANEL, UI_PANEL_LINE);
                DrawRectangleRounded((Rectangle){ pr.x + 2, pr.y + 2, pr.width - 4, 8 }, 1.0f, 4, UI_ACCENT);
                ui_spinner_anim((Vector2){ pr.x + 70, pr.y + 78 }, 30, g_ui.t);
                ui_text(queued ? "SEARCHING FOR MATCH" : "CONNECTING TO SERVER", (int)pr.x + 122, (int)pr.y + 44, 30, UI_TEXT);
                DrawText(queued ? PLAYLIST_LABELS[g_set.playlist] : g_set.server, (int)pr.x + 124, (int)pr.y + 82, 18, UI_ACCENT);
                ui_text_c(TextFormat("%d:%02d", secs / 60, secs % 60), pr.x + pr.width * 0.5f, (int)pr.y + 126, 64, UI_TEXT);
                /* seats: filled = players searching this playlist */
                for (k = 0; k < need; k++) {
                    float sx = pr.x + pr.width * 0.5f + (k - (need - 1) * 0.5f) * 46.0f, sy = pr.y + 226;
                    int filled = queued && k < have;
                    Color c = k < need / 2 ? UI_BLUE : UI_ORANGE;
                    DrawCircle((int)sx, (int)sy, 16, filled ? c : (Color){ 30, 38, 55, 255 });
                    DrawCircleLines((int)sx, (int)sy, 16, ColorAlpha(c, 0.8f));
                }
                ui_text_c(queued ? TextFormat("%d of %d players searching", have, need) : netClient.statusMsg,
                          pr.x + pr.width * 0.5f, (int)pr.y + 252, 18, UI_TEXT_DIM);
                if (queued && netClient.queueBotFillSec >= 0.0f) {
                    float left = netClient.queueBotFillSec, frac = netClient.queueSearchSec / fmaxf(1.0f, netClient.queueSearchSec + left);
                    Rectangle bar = { pr.x + 60, pr.y + 290, pr.width - 120, 10 };
                    DrawRectangleRounded(bar, 1.0f, 4, (Color){ 30, 38, 55, 255 });
                    DrawRectangleRounded((Rectangle){ bar.x, bar.y, bar.width * fminf(1.0f, frac), bar.height }, 1.0f, 4, UI_ACCENT);
                    ui_text_c(left > 0.5f ? TextFormat("Bots fill empty seats in %d:%02d", (int)left / 60, (int)left % 60)
                                          : "Starting with bots...", pr.x + pr.width * 0.5f, (int)pr.y + 306, 16, UI_TEXT_DIM);
                }
                if (queued)
                    DrawText(TextFormat("%d online  -  %d match%s in progress  -  ping %.0f ms", netClient.queuePlayersOnline,
                                        netClient.queueMatches, netClient.queueMatches == 1 ? "" : "es", netClient.pingMs),
                             (int)pr.x + 24, (int)(pr.y + pr.height - 30), 14, UI_TEXT_DIM);
                if (ui_button((Rectangle){ pr.x + pr.width - 204, pr.y + pr.height - 80, 180, 52 }, "CANCEL", NULL, 0) || nav.back)
                    uiAct = UA_CANCEL_SEARCH;
                ui_end();
            } else if (screen == SCR_OFFLINE) {
                static const char *modeDesc[] = { "Free play: just you and the ball. Practise shots, aerials and recoveries.",
                                                  "1v1 against a bot.", "2v2: you and a bot teammate against two bots.",
                                                  "3v3: you and two bot teammates against three bots." };
                float x = 64, y = 170, w = 520, h = 58, gap = 12;
                int d;
                ui_begin(SCR_OFFLINE, nav, 3);
                DrawRectangleGradientH(0, 0, (int)(sw * 0.55f), sh, (Color){ 4, 6, 12, 210 }, (Color){ 4, 6, 12, 0 });
                ui_chrome("EXHIBITION", "Offline", hints);
                if ((d = ui_spinner((Rectangle){ x, y, w, h }, "MODE", MODE_NAMES[g_set.mode]))) { g_set.mode = (g_set.mode + d + 4) % 4; settings_save(); }
                if ((d = ui_spinner((Rectangle){ x, y + (h + gap), w, h }, "BOT DIFFICULTY", SKILL_NAMES[g_set.botSkill]))) { g_set.botSkill = (g_set.botSkill + d + 3) % 3; settings_save(); }
                if ((d = ui_spinner((Rectangle){ x, y + 2 * (h + gap), w, h }, "MATCH LENGTH", MATCH_NAMES[g_set.matchIdx]))) { g_set.matchIdx = (g_set.matchIdx + d + 4) % 4; settings_save(); }
                DrawText(modeDesc[g_set.mode], (int)x, (int)(y + 3 * (h + gap) + 4), 16, UI_TEXT_DIM);
                if (ui_button((Rectangle){ x, y + 3 * (h + gap) + 34, w, 70 }, "START MATCH", MODE_NAMES[g_set.mode], 1)) uiAct = UA_START_OFFLINE;
                if (ui_button((Rectangle){ 64, (float)sh - 150, 220, 52 }, "BACK", NULL, 0) || nav.back) uiAct = UA_GO_PLAY;
                ui_profile(g_set.name, CAR_NAMES[g_set.car], paintName);
                ui_end();
            } else if (screen == SCR_GARAGE) {
                float x = 64, y = 170, w = 520, h = 58, gap = 12;
                int d;
                ui_begin(SCR_GARAGE, nav, 0);
                DrawRectangleGradientH(0, 0, (int)(sw * 0.55f), sh, (Color){ 4, 6, 12, 210 }, (Color){ 4, 6, 12, 0 });
                ui_chrome("GARAGE", "Car and paint", hints);
                if ((d = ui_spinner((Rectangle){ x, y, w, h }, "CAR", CAR_NAMES[g_set.car]))) carAdj = d;
                if ((d = ui_spinner((Rectangle){ x, y + (h + gap), w, h }, "PAINT", paintName))) { g_set.skin = (g_set.skin + d + 2) % 2; settings_save(); }
                DrawText("Your car and paint are used offline and in online matches.", (int)x, (int)(y + 2 * (h + gap) + 4), 16, UI_TEXT_DIM);
                if (ui_button((Rectangle){ 64, (float)sh - 150, 220, 52 }, "BACK", NULL, 0) || nav.back) uiAct = UA_GO_MENU;
                {
                    const char *cn = TextFormat("%s", CAR_NAMES[g_set.car]);
                    char up[32]; int c;
                    for (c = 0; cn[c] && c < 31; c++) up[c] = (char)(cn[c] >= 'a' && cn[c] <= 'z' ? cn[c] - 32 : cn[c]);
                    up[c] = 0;
                    ui_text(up, sw - 64 - MeasureText(up, 64), sh - 200, 64, UI_TEXT);
                    DrawText(paintName, sw - 64 - MeasureText(paintName, 20), sh - 130, 20, UI_ACCENT);
                }
                ui_end();
            } else if (screen == SCR_PAUSE) {
                Rectangle pr = { sw * 0.5f - 250, sh * 0.5f - 230, 500, 460 };
                float bx = pr.x + 30, bw = pr.width - 60, by = pr.y + 110, bh = 54, gap = 12;
                ui_begin(SCR_PAUSE, nav, 0);
                DrawRectangle(0, 0, sw, sh, (Color){ 0, 0, 0, 140 });
                ui_panel(pr, 0.06f, UI_PANEL, UI_PANEL_LINE);
                DrawRectangleRounded((Rectangle){ pr.x + 2, pr.y + 2, pr.width - 4, 8 }, 1.0f, 4, UI_ACCENT);
                ui_text_c("PAUSED", pr.x + pr.width * 0.5f, (int)pr.y + 30, 40, UI_TEXT);
                DrawText(isOnline ? "The match keeps going while you're paused" : MODE_NAMES[g_set.mode],
                         (int)(pr.x + pr.width * 0.5f - MeasureText(isOnline ? "The match keeps going while you're paused" : MODE_NAMES[g_set.mode], 15) * 0.5f),
                         (int)pr.y + 76, 15, UI_TEXT_DIM);
                if (ui_button((Rectangle){ bx, by, bw, bh }, "RESUME", NULL, 1) || nav.back) uiAct = UA_RESUME;
                if (ui_button((Rectangle){ bx, by + (bh + gap), bw, bh }, "SETTINGS", NULL, 0)) uiAct = UA_PAUSE_SETTINGS;
                if (isOnline) {
                    if (ui_button((Rectangle){ bx, by + 2 * (bh + gap), bw, bh }, "LEAVE MATCH", "A bot will take your place", 0)) uiAct = UA_LEAVE_MATCH;
                } else {
                    if (ui_button((Rectangle){ bx, by + 2 * (bh + gap), bw, bh }, "RESTART MATCH", NULL, 0)) uiAct = UA_RESTART;
                }
                if (ui_button((Rectangle){ bx, by + 3 * (bh + gap), bw, bh }, "MAIN MENU", NULL, 0)) uiAct = UA_POST_MENU;
                if (ui_button((Rectangle){ bx, by + 4 * (bh + gap), bw, bh }, "EXIT GAME", NULL, 0)) uiAct = UA_QUIT;
                ui_end();
            } else if (screen == SCR_SETTINGS) {
                MenuItem it[13];
                memset(it, 0, sizeof(it));
                it[0].label = "CAMERA DISTANCE";  snprintf(it[0].value, 48, "%.1f m", g_set.camDist);
                it[1].label = "CAMERA HEIGHT";    snprintf(it[1].value, 48, "%.2f", g_set.camHeight);
                it[2].label = "FIELD OF VIEW";    snprintf(it[2].value, 48, "%.0f", g_set.fov);
                it[3].label = "BOOST FOV KICK";   snprintf(it[3].value, 48, "%s", g_set.boostFov ? "On" : "Off");
                it[4].label = "MATCH LENGTH";     snprintf(it[4].value, 48, "%s", MATCH_NAMES[g_set.matchIdx]);
                it[5].label = "INVERT AIR PITCH"; snprintf(it[5].value, 48, "%s", g_set.invertPitch ? "On" : "Off");
                it[6].label = "FULLSCREEN";       snprintf(it[6].value, 48, "%s", g_set.fullscreen ? "On" : "Off");
                it[7].label = "SHOW FPS";         snprintf(it[7].value, 48, "%s", g_set.showFps ? "On" : "Off");
                it[8].label = "SHOW CONTROLS";    snprintf(it[8].value, 48, "%s", g_set.showHints ? "On" : "Off");
                it[9].label = "SHADOWS";          snprintf(it[9].value, 48, "%s", g_set.shadows ? "On" : "Off");
                it[10].label = "BLOOM";           snprintf(it[10].value, 48, "%s", g_set.bloom ? "On" : "Off");
                it[11].label = "USE ORIGINAL CAMERA"; snprintf(it[11].value, 48, "5.4 m / 59");
                it[12].label = "BACK";
                menu_run("SETTINGS", it, 13, &setSel, nav, &act, &adj);
                if (setSel == 4)
                    DrawText("match length applies to offline matches", sw / 2 - MeasureText("match length applies to offline matches", 18) / 2,
                             sh - 70, 18, (Color){ 255, 190, 120, 255 });
            } else if (screen == SCR_GAME && state == ST_OVER) {
                /* --- post-match: result, rosters, next step --------------------------- */
                int me = isOnline ? netClient.localSlot : 0, myTeam = team[me];
                int won = myTeam == 0 ? scoreBlue > scoreOrange : scoreOrange > scoreBlue, draw = scoreBlue == scoreOrange;
                Rectangle pr = { sw * 0.5f - 360, sh * 0.5f - 250, 720, 500 };
                const char *res = draw ? "DRAW" : won ? "VICTORY" : "DEFEAT";
                Color rc = draw ? UI_TEXT : won ? (Color){ 255, 205, 80, 255 } : (Color){ 230, 90, 80, 255 };
                int t, row;
                ui_begin(200, nav, 0);
                DrawRectangle(0, 0, sw, sh, (Color){ 0, 0, 0, 110 });
                ui_panel(pr, 0.05f, UI_PANEL, UI_PANEL_LINE);
                DrawRectangleRounded((Rectangle){ pr.x + 2, pr.y + 2, pr.width - 4, 8 }, 1.0f, 4, rc);
                ui_text_c(res, pr.x + pr.width * 0.5f, (int)pr.y + 26, 60, rc);
                ui_text(TextFormat("%d", scoreBlue), (int)(pr.x + pr.width * 0.5f - 70 - MeasureText(TextFormat("%d", scoreBlue), 48)), (int)pr.y + 96, 48, UI_BLUE);
                ui_text_c("-", pr.x + pr.width * 0.5f, (int)pr.y + 96, 48, UI_TEXT_DIM);
                ui_text(TextFormat("%d", scoreOrange), (int)(pr.x + pr.width * 0.5f + 70), (int)pr.y + 96, 48, UI_ORANGE);
                for (t = 0; t < 2; t++) {
                    float cx = pr.x + 30 + t * (pr.width * 0.5f - 15);
                    float cw = pr.width * 0.5f - 45;
                    DrawRectangleRounded((Rectangle){ cx, pr.y + 166, cw, 34 }, 0.3f, 4, t == 0 ? (Color){ 24, 72, 165, 240 } : (Color){ 195, 75, 18, 240 });
                    DrawText(t == 0 ? "BLUE" : "ORANGE", (int)cx + 14, (int)pr.y + 174, 18, RAYWHITE);
                    row = 0;
                    for (i = 0; i < (isOnline ? SARP_MAX_CLIENTS : nCars); i++) {
                        const char *nm;
                        int isMe = i == me, bot;
                        if (isOnline ? (!netClient.players[i].active || netClient.players[i].team != t) : team[i] != t) continue;
                        bot = isOnline ? netClient.players[i].isBot : i != 0;
                        nm = isOnline ? netClient.players[i].name : (i == 0 ? g_set.name : TextFormat("Bot %d", i));
                        DrawRectangleRounded((Rectangle){ cx, pr.y + 208 + row * 38, cw, 32 }, 0.3f, 4,
                                             isMe ? (Color){ 40, 60, 90, 240 } : (Color){ 18, 24, 36, 200 });
                        DrawText(nm, (int)cx + 14, (int)(pr.y + 215 + row * 38), 18, isMe ? UI_ACCENT_HI : UI_TEXT);
                        if (bot) DrawText("BOT", (int)(cx + cw - 50), (int)(pr.y + 217 + row * 38), 14, UI_TEXT_DIM);
                        else if (isMe) DrawText("YOU", (int)(cx + cw - 50), (int)(pr.y + 217 + row * 38), 14, UI_ACCENT);
                        row++;
                    }
                }
                if (isOnline) {
                    if (ui_button((Rectangle){ pr.x + 30, pr.y + pr.height - 86, 320, 60 }, "QUEUE AGAIN", PLAYLIST_LABELS[netClient.playlist % 4], 1)) uiAct = UA_REQUEUE;
                    if (ui_button((Rectangle){ pr.x + pr.width - 350, pr.y + pr.height - 86, 320, 60 }, "MAIN MENU", NULL, 0)) uiAct = UA_POST_MENU;
                    DrawText(TextFormat("Returning to the menu in %d", (int)ceilf(fmaxf(0.0f, stateTimer))),
                             (int)(pr.x + pr.width * 0.5f - MeasureText(TextFormat("Returning to the menu in %d", (int)ceilf(fmaxf(0.0f, stateTimer))), 14) * 0.5f),
                             (int)(pr.y + pr.height - 112), 14, UI_TEXT_DIM);
                } else {
                    if (ui_button((Rectangle){ pr.x + 30, pr.y + pr.height - 86, 320, 60 }, "REMATCH", MODE_NAMES[g_set.mode], 1)) uiAct = UA_RESTART;
                    if (ui_button((Rectangle){ pr.x + pr.width - 350, pr.y + pr.height - 86, 320, 60 }, "MAIN MENU", NULL, 0)) uiAct = UA_POST_MENU;
                }
                ui_end();
            }
        }
        EndDrawing();

        /* --- apply menu actions ------------------------------------------ */
        switch (uiAct) {
        case UA_GO_MENU:     screen = SCR_MENU; break;
        case UA_GO_PLAY:     screen = SCR_PLAY; break;
        case UA_GO_ONLINE:   screen = SCR_ONLINE; onlineMsg[0] = 0; break;
        case UA_GO_OFFLINE:  screen = SCR_OFFLINE; break;
        case UA_GO_GARAGE:   screen = SCR_GARAGE; break;
        case UA_GO_SETTINGS: settingsFrom = SCR_MENU; setSel = 0; screen = SCR_SETTINGS; break;
        case UA_QUIT:        quit = 1; break;
        case UA_FIND_MATCH:  START_SEARCH(); break;
        case UA_CANCEL_SEARCH:
            net_client_disconnect(&netClient);
            onlineMsg[0] = 0;
            screen = SCR_ONLINE;
            break;
        case UA_START_OFFLINE:
            isOnline = 0;
            NEW_MATCH();
            cam.position = Vector3Add(cars[0].pos, V3(-g_set.camDist, 2.0f, 0));
            screen = SCR_GAME;
            break;
        case UA_RESUME:          screen = SCR_GAME; break;
        case UA_PAUSE_SETTINGS:  settingsFrom = SCR_PAUSE; setSel = 0; screen = SCR_SETTINGS; break;
        case UA_RESTART:         NEW_MATCH(); screen = SCR_GAME; break;
        case UA_LEAVE_MATCH:
            net_client_disconnect(&netClient);
            LEAVE_ONLINE();
            snprintf(onlineMsg, sizeof(onlineMsg), "You left the match. A bot took your place.");
            screen = SCR_ONLINE;
            break;
        case UA_REQUEUE:
            net_client_disconnect(&netClient);
            LEAVE_ONLINE();
            START_SEARCH();
            break;
        case UA_POST_MENU:
            if (isOnline) { net_client_disconnect(&netClient); LEAVE_ONLINE(); }
            screen = SCR_MENU;
            break;
        default: break;
        }
        if (carAdj) {
            int prev = g_set.car;
            g_set.car = (g_set.car + carAdj + CAR_COUNT) % CAR_COUNT;
            if (!load_car(CAR_NAMES[g_set.car], &cars[0], &crs[0], lit, 1)) { g_set.car = prev; load_car(CAR_NAMES[prev], &cars[0], &crs[0], lit, 1); }
            carModel[0] = g_set.car;
            settings_save();
            carAdj = 0;
        }
        if (screen == SCR_SETTINGS) {
            if (adj) {
                switch (setSel) {
                case 0: g_set.camDist   = clampf(g_set.camDist + 0.5f * adj, 3.0f, 14.0f); break;
                case 1: g_set.camHeight = clampf(g_set.camHeight + 0.02f * adj, 0.10f, 0.80f); break;
                case 2: g_set.fov       = clampf(g_set.fov + 1.0f * adj, 45.0f, 90.0f); break;
                case 3: g_set.boostFov    = !g_set.boostFov; break;
                case 4: g_set.matchIdx    = (g_set.matchIdx + adj + 4) % 4; break;
                case 5: g_set.invertPitch = !g_set.invertPitch; break;
                case 6: g_set.fullscreen  = !g_set.fullscreen; ToggleBorderlessWindowed(); break;
                case 7: g_set.showFps     = !g_set.showFps; break;
                case 8: g_set.showHints   = !g_set.showHints; break;
                case 9: g_set.shadows     = !g_set.shadows; break;
                case 10: g_set.bloom      = !g_set.bloom; break;
                default: break;
                }
            }
            if (act == 11) { g_set.camDist = 5.4f; g_set.camHeight = 0.11f; g_set.fov = 59.0f; }
            if (act == 12 || nav.back) { settings_save(); screen = settingsFrom; }
        }
    }
#undef KICKOFF
#undef NEW_MATCH
#undef SETUP_CARS

    if (netClient.state == NET_CONNECTED) {
        net_client_disconnect(&netClient);
    }
    net_client_shutdown();

    settings_save();
    for (i = 0; i < MAX_CARS; i++) car_render_unload(&crs[i]);
    UnloadTexture(fieldTex);
    UnloadTexture(ballTex);
    if (g_particleTex.id) UnloadTexture(g_particleTex);
    gfx_unload();
    CloseWindow();
    return 0;
}

#ifndef SARPBC_DLL
int main(int argc, char **argv)
{
    return sarpbc_main(argc, argv);
}
#endif
