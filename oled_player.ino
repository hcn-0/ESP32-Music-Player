/*
 * ESP32 OLED Lyric Visualizer  -  firmware (protocol v2)
 * Board : ESP32 DevKit V1        Display: SSD1306 128x64 I2C (SDA=21, SCL=22)
 * Libs  : Adafruit SSD1306, Adafruit GFX, Wire
 *
 * Screen = big lyrics (words pop in one by one) over a beat-reactive animated
 * background. No other text. The background style changes with every lyric line
 * (keywords like rain / star / love / fire pick a matching style).
 *
 * Serial protocol v2 (115200 8N1), one ASCII line per frame:
 *
 *     $<payload>*<HH>\n        HH = XOR of all payload bytes (2 hex digits)
 *
 *   PC  -> ESP32   H|2                       hello / keep-alive   (we answer A|2)
 *   ESP32 -> PC    A|2                       hello acknowledged
 *   ESP32 -> PC    R|2                       sent once at boot (PC resends state)
 *   PC  -> ESP32   T|<title>|<durationMs>    new song (resets the screen)
 *   PC  -> ESP32   L|<line>|<lineDurMs>|<elapsedMs>   current lyric line (UTF-8)
 *   PC  -> ESP32   P|<posMs>|<durMs>|<state>          state 0=stop 1=play 2=pause
 *   PC  -> ESP32   E|<level 0-100>|<beat 0/1>         audio energy + beat flag (20 Hz)
 *
 * Non-blocking: no delay(); all timing uses millis().
 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ------------------------------- Hardware ---------------------------------
#define OLED_SDA     21
#define OLED_SCL     22
#define OLED_ADDR    0x3C          // try 0x3D if the display is not found
#define SCREEN_W     128
#define SCREEN_H     64
#define STATUS_LED   2

// ------------------------------- Protocol ---------------------------------
#define SERIAL_BAUD        115200
#define PROTOCOL_VERSION   2
#define RX_BUF_SIZE        512
#define MAX_FIELDS         6
#define LINE_MAX           104
#define LINK_TIMEOUT_MS    3500
#define MAX_WORDS          32
#define NUM_STYLES         7

static const uint32_t FRAME_MS = 30;     // ~33 fps

// Latin-1 (U+00C0..U+00FF) -> ASCII fold table, used when the PC sends raw UTF-8
static const char LATIN1_FOLD[] =
  "AAAAAAACEEEEIIIIDNOOOOOxOUUUUYTs"
  "aaaaaaaceeeeiiiidnooooo/ouuuuyty";
static_assert(sizeof(LATIN1_FOLD) == 65, "LATIN1_FOLD must have 64 entries");

// style keywords (first match wins): lyric word -> background style
struct Keyword { const char* word; uint8_t style; };
static const Keyword KEYWORDS[] = {
  {"rain", 4}, {"tear", 4}, {"cry", 4}, {"storm", 4},
  {"star", 2}, {"night", 2}, {"sky", 2}, {"dream", 2}, {"space", 2},
  {"love", 0}, {"heart", 0}, {"kiss", 0},
  {"fire", 6}, {"burn", 6}, {"light", 6}, {"electric", 6}, {"sun", 6},
  {"dance", 1}, {"beat", 1}, {"turn it up", 1}, {"loud", 1}, {"party", 1},
  {"wave", 3}, {"sea", 3}, {"ocean", 3}, {"fly", 3}, {"wind", 3}, {"slow", 3},
  {"flash", 5}, {"city", 5}, {"neon", 5}, {"pixel", 5}, {"grid", 5},
};

// ------------------------------- Globals ----------------------------------
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1, 400000UL, 400000UL);
static bool oledOk = false;

// serial receive
static char     rxBuf[RX_BUF_SIZE];
static uint16_t rxLen = 0;
static bool     rxOverflow = false;

// link / player state
static uint8_t  playState = 0;
static uint32_t lastValidRxAt = 0, lastEAt = 0, ledOffAt = 0;
static bool     everLinked = false, wasAlive = false;

// lyric line + word layout
struct Word { uint8_t off, len, row, col; uint16_t at; };
static char     lineText[LINE_MAX];
static Word     words[MAX_WORDS];
static uint8_t  nWords = 0, nRows = 0, txtSize = 2, rowW[6], shownWords = 0;
static float    lineElapsed = 0;                 // ms since the line started

// background / energy
static uint8_t  styleIdx = 0;
static float    lvl = 0.15f, lvlTarget = 0.15f, pulse = 0, phase = 0;
static uint32_t flashUntil = 0, nextFakeBeat = 0, lastRenderAt = 0;
static uint16_t beatCount = 0;
static bool     invState = false;

struct Star { float x; uint8_t y, z; };
struct Drop { float y; uint8_t x, speed, len; };
static float   ringR[4];
static bool    ringOn[4];
static Star    stars[40];
static Drop    drops[28];
static float   barH[16];

// ------------------------------- Small helpers ----------------------------
static void copyStr(char* dst, const char* src, size_t cap) {
  size_t i = 0;
  while (i + 1 < cap && src[i]) { dst[i] = src[i]; i++; }
  dst[i] = 0;
}

static bool linkAlive() {
  return everLinked && (uint32_t)(millis() - lastValidRxAt) < LINK_TIMEOUT_MS;
}

// ------------------------- UTF-8 -> printable ASCII -----------------------
static char mapCodepoint(uint32_t cp) {
  if (cp >= 0xC0 && cp <= 0xFF) return LATIN1_FOLD[cp - 0xC0];
  switch (cp) {
    case 0xA0:   return ' ';
    case 0xA1:   return '!';
    case 0xBF:   return '?';
    case 0x2018: case 0x2019: case 0x201A: return '\'';
    case 0x201C: case 0x201D: case 0x201E: return '"';
    case 0x2013: case 0x2014: return '-';
    case 0x2022: return '*';
    case 0x2026: return '.';
    default:     return '?';
  }
}

static void utf8ToDisplay(const char* in, char* out, size_t cap) {
  size_t o = 0;
  const uint8_t* p = (const uint8_t*)in;
  while (*p && o + 1 < cap) {
    uint8_t c = *p;
    if (c < 0x80) { out[o++] = (c >= 32 && c != 127) ? (char)c : ' '; p++; continue; }
    uint32_t cp; int extra;
    if      ((c & 0xE0) == 0xC0) { cp = c & 0x1F; extra = 1; }
    else if ((c & 0xF0) == 0xE0) { cp = c & 0x0F; extra = 2; }
    else if ((c & 0xF8) == 0xF0) { cp = c & 0x07; extra = 3; }
    else { out[o++] = '?'; p++; continue; }
    p++;
    int i = 0;
    for (; i < extra && (*p & 0xC0) == 0x80; i++, p++) cp = (cp << 6) | (*p & 0x3F);
    out[o++] = (i < extra) ? '?' : mapCodepoint(cp);
  }
  out[o] = 0;
}

// ------------------------------- Serial TX --------------------------------
static void sendFrame(const char* payload) {
  uint8_t cs = 0;
  for (const char* p = payload; *p; p++) cs ^= (uint8_t)*p;
  Serial.write('$');
  Serial.print(payload);
  Serial.write('*');
  if (cs < 16) Serial.write('0');
  Serial.print(cs, HEX);
  Serial.write('\n');
}

// ------------------------------- Effects state ----------------------------
static void spawnRing() {
  for (uint8_t i = 0; i < 4; i++) if (!ringOn[i]) { ringOn[i] = true; ringR[i] = 2; return; }
}

static void triggerBeat(bool flash) {
  pulse = 1.0f;
  beatCount++;
  spawnRing();
  if (flash && lvl > 0.3f) flashUntil = millis() + 55;     // brief screen invert
}

static void initEffects() {
  for (uint8_t i = 0; i < 4; i++) ringOn[i] = false;
  for (uint8_t i = 0; i < 40; i++) stars[i] = { (float)random(0, SCREEN_W), (uint8_t)random(0, SCREEN_H), (uint8_t)random(1, 4) };
  for (uint8_t i = 0; i < 28; i++) drops[i] = { (float)random(-64, SCREEN_H), (uint8_t)random(0, SCREEN_W), (uint8_t)random(60, 140), (uint8_t)random(3, 7) };
  for (uint8_t i = 0; i < 16; i++) barH[i] = 0;
}

// ------------------------------- Lyric layout -----------------------------
static uint8_t pickStyle(const char* s) {
  char low[LINE_MAX];
  size_t n = 0;
  for (; s[n] && n + 1 < LINE_MAX; n++) low[n] = (s[n] >= 'A' && s[n] <= 'Z') ? s[n] + 32 : s[n];
  low[n] = 0;
  for (uint8_t i = 0; i < sizeof(KEYWORDS) / sizeof(KEYWORDS[0]); i++)
    if (strstr(low, KEYWORDS[i].word)) return KEYWORDS[i].style;
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < n; i++) h = (h ^ (uint8_t)low[i]) * 16777619u;
  return (styleIdx + 1 + (h % (NUM_STYLES - 1))) % NUM_STYLES;     // always a different style
}

// greedy word wrap; long words are split. Returns true if everything fit.
static bool layoutWords(const char* s, uint8_t cols, uint8_t maxRows) {
  nWords = 0; nRows = 0;
  for (uint8_t r = 0; r < 6; r++) rowW[r] = 0;
  uint8_t row = 0, col = 0;
  size_t i = 0;
  bool fit = true;
  while (s[i] && fit) {
    while (s[i] == ' ') i++;
    if (!s[i]) break;
    size_t st = i;
    while (s[i] && s[i] != ' ') i++;
    size_t wl = i - st;
    while (wl > 0) {
      uint8_t take = wl > cols ? cols : (uint8_t)wl;
      if (col > 0 && col + 1 + take > cols) { row++; col = 0; }
      else if (col > 0) col++;
      if (row >= maxRows || nWords >= MAX_WORDS) { fit = false; break; }
      words[nWords++] = { (uint8_t)st, take, row, col, 0 };
      col += take; rowW[row] = col;
      st += take; wl -= take;
    }
  }
  nRows = nWords ? words[nWords - 1].row + 1 : 0;
  return fit;
}

static void setLine(const char* s, uint32_t durMs, uint32_t elapsedMs) {
  copyStr(lineText, s, LINE_MAX);
  txtSize = 2;
  if (!layoutWords(lineText, 10, 4)) { txtSize = 1; layoutWords(lineText, 21, 6); }

  uint32_t span = durMs ? durMs * 7 / 10 : 2500;      // words appear over 70% of the line
  if (span < 250) span = 250;
  if (span > 4500) span = 4500;
  uint32_t total = strlen(lineText);
  if (total == 0) total = 1;
  for (uint8_t i = 0; i < nWords; i++) words[i].at = (uint16_t)(span * words[i].off / total);

  lineElapsed = (float)elapsedMs;
  shownWords = 0;
  while (shownWords < nWords && words[shownWords].at <= elapsedMs) shownWords++;

  styleIdx = pickStyle(lineText);
  pulse = pulse > 0.8f ? pulse : 0.8f;                 // new line = light burst
  spawnRing();
}

static void resetToIdle() {
  lineText[0] = 0; nWords = 0; nRows = 0; shownWords = 0;
  playState = 0;
}

// ------------------------------- Frame handling ---------------------------
static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static bool verifyAndSplit(char* line, uint16_t len, char* fields[], uint8_t& n) {
  if (len < 4 || line[0] != '$') return false;
  char* star = strrchr(line, '*');
  if (!star || (line + len) - star != 3) return false;
  int hi = hexVal(star[1]), lo = hexVal(star[2]);
  if (hi < 0 || lo < 0) return false;
  uint8_t calc = 0;
  for (char* p = line + 1; p < star; p++) calc ^= (uint8_t)*p;
  if (calc != (uint8_t)((hi << 4) | lo)) return false;
  *star = 0;
  n = 0;
  char* p = line + 1;
  fields[n++] = p;
  for (; *p; p++) {
    if (*p == '|') {
      *p = 0;
      if (n < MAX_FIELDS) fields[n++] = p + 1;
    }
  }
  return true;
}

static void handleFrame(char* f[], uint8_t n) {
  uint32_t now = millis();
  switch (f[0][0]) {
    case 'H': {
      char reply[16];
      snprintf(reply, sizeof(reply), "A|%d", PROTOCOL_VERSION);
      sendFrame(reply);
      break;
    }
    case 'T':
      resetToIdle();
      break;
    case 'L':
      if (n >= 4) {
        char c[LINE_MAX];
        utf8ToDisplay(f[1], c, LINE_MAX);
        setLine(c, strtoul(f[2], nullptr, 10), strtoul(f[3], nullptr, 10));
      }
      break;
    case 'P':
      if (n >= 4) {
        uint32_t st = strtoul(f[3], nullptr, 10);
        playState = (st > 2) ? 0 : (uint8_t)st;
      }
      break;
    case 'E':
      if (n >= 3) {
        uint32_t v = strtoul(f[1], nullptr, 10);
        lvlTarget = (v > 100 ? 100 : v) / 100.0f;
        lastEAt = now;
        if (f[2][0] == '1') triggerBeat(true);
      }
      break;
    default: break;
  }
}

static void handleLine(char* line, uint16_t len) {
  char* fields[MAX_FIELDS];
  uint8_t n = 0;
  if (!verifyAndSplit(line, len, fields, n)) return;     // bad checksum: drop
  lastValidRxAt = millis();
  everLinked = true;
  digitalWrite(STATUS_LED, HIGH);
  ledOffAt = lastValidRxAt + 30;
  handleFrame(fields, n);
}

static void pollSerial() {
  uint16_t budget = 512;
  while (budget-- && Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;
    if (c == '\n') {
      if (!rxOverflow && rxLen > 0) { rxBuf[rxLen] = 0; handleLine(rxBuf, rxLen); }
      rxLen = 0; rxOverflow = false;
    } else if (c == '\r' || rxOverflow) {
      // ignore
    } else if (rxLen < RX_BUF_SIZE - 1) {
      rxBuf[rxLen++] = (char)c;
    } else {
      rxOverflow = true;
    }
  }
}

// ------------------------------- Backgrounds ------------------------------
// 50% dither fill: looks "dim" on a 1-bit display so text stays readable
static void ditherRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t par) {
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > SCREEN_W) w = SCREEN_W - x;
  if (y + h > SCREEN_H) h = SCREEN_H - y;
  for (int16_t yy = y; yy < y + h; yy++)
    for (int16_t xx = x + ((x + yy + par) & 1); xx < x + w; xx += 2) display.drawPixel(xx, yy, WHITE);
}

static void bgRings(float dt) {
  static uint32_t lastSpawn = 0;
  uint32_t now = millis();
  if (now - lastSpawn > 900) { lastSpawn = now; spawnRing(); }
  for (uint8_t i = 0; i < 4; i++) {
    if (!ringOn[i]) continue;
    ringR[i] += dt * (60.0f + 80.0f * lvl);
    if (ringR[i] > 80) { ringOn[i] = false; continue; }
    display.drawCircle(64, 32, (int16_t)ringR[i], WHITE);
    if (ringR[i] > 6) display.drawCircle(64, 32, (int16_t)ringR[i] - 3, WHITE);
  }
  display.fillCircle(64, 32, 2 + (int16_t)(pulse * 7), WHITE);
}

static void bgBars(float dt) {
  for (uint8_t i = 0; i < 16; i++) {
    float t = 0.35f + 0.65f * fabsf(sinf(phase * 1.7f + i * 0.9f));
    float target = (lvl * 0.75f + pulse * 0.45f) * 60.0f * t;
    if (barH[i] < target) { float k = dt * 18.0f; barH[i] += (target - barH[i]) * (k > 1 ? 1 : k); }
    else { barH[i] -= 70.0f * dt; if (barH[i] < 0) barH[i] = 0; }
    int16_t h = (int16_t)barH[i];
    if (h > 0) {
      ditherRect(i * 8 + 1, SCREEN_H - h, 6, h, 0);
      display.drawFastHLine(i * 8 + 1, SCREEN_H - h, 6, WHITE);
    }
  }
}

static void bgStars(float dt) {
  for (uint8_t i = 0; i < 40; i++) {
    Star& s = stars[i];
    s.x -= (18.0f + lvl * 110.0f + pulse * 140.0f) / s.z * dt;
    if (s.x < 0) { s.x = SCREEN_W - 1; s.y = random(0, SCREEN_H); s.z = random(1, 4); }
    int16_t px = (int16_t)s.x;
    if (pulse > 0.55f && s.z < 3) display.drawFastHLine(px, s.y, 7 - s.z * 2, WHITE);   // warp streak
    else if (s.z == 1) display.fillRect(px, s.y, 2, 2, WHITE);
    else display.drawPixel(px, s.y, WHITE);
  }
}

static void bgWaves(float dt) {
  (void)dt;
  float amp = 5.0f + lvl * 18.0f + pulse * 10.0f;
  for (uint8_t w = 0; w < 2; w++) {
    float k = w ? 0.11f : 0.07f, sp = w ? -1.3f : 1.0f;
    int16_t px = 0, py = 32 + (int16_t)(amp * sinf(phase * sp));
    for (int16_t x = 2; x < SCREEN_W; x += 2) {
      int16_t y = 32 + (int16_t)(amp * sinf(x * k + phase * 2.0f * sp));
      display.drawLine(px, py, x, y, WHITE);
      px = x; py = y;
    }
  }
}

static void bgRain(float dt) {
  for (uint8_t i = 0; i < 28; i++) {
    Drop& d = drops[i];
    d.y += d.speed * (0.8f + lvl * 1.6f) * dt;
    if (d.y > SCREEN_H) { d.y = -d.len; d.x = random(0, SCREEN_W); d.speed = random(60, 140); }
    display.drawFastVLine(d.x, (int16_t)d.y, d.len, WHITE);
  }
  if (pulse > 0.35f)                                   // splash on the beat
    for (int16_t x = 0; x < SCREEN_W; x += 4)
      display.drawFastVLine(x + (x & 3), SCREEN_H - 1 - (int16_t)(pulse * (4 + (x % 3) * 3)), (int16_t)(pulse * 8), WHITE);
}

static void bgChecker(float dt) {
  (void)dt;
  float scan = fmodf(phase * 14.0f, 12.0f);
  for (float fy = scan; fy < SCREEN_H; fy += 12.0f)
    for (int16_t x = 0; x < SCREEN_W; x += 2) display.drawPixel(x, (int16_t)fy, WHITE);
  for (float fx = scan * 2; fx < SCREEN_W; fx += 24.0f)
    for (int16_t y = 0; y < SCREEN_H; y += 2) display.drawPixel((int16_t)fx, y, WHITE);
  if (pulse > 0.5f) ditherRect(0, 0, SCREEN_W, SCREEN_H, beatCount & 1);   // checker flash
}

static void bgRays(float dt) {
  (void)dt;
  const uint8_t n = 14;
  float a0 = phase * 0.5f, r1 = 16.0f + lvl * 26.0f + pulse * 30.0f;
  for (uint8_t k = 0; k < n; k++) {
    float a = a0 + k * (6.2832f / n);
    float c = cosf(a), s = sinf(a) * 0.8f;
    display.drawLine(64 + (int16_t)(c * 8), 32 + (int16_t)(s * 8), 64 + (int16_t)(c * r1), 32 + (int16_t)(s * r1), WHITE);
  }
  display.fillCircle(64, 32, 3 + (int16_t)(pulse * 6), WHITE);
}

static void drawBackground(float dt) {
  switch (styleIdx) {
    case 0: bgRings(dt);   break;
    case 1: bgBars(dt);    break;
    case 2: bgStars(dt);   break;
    case 3: bgWaves(dt);   break;
    case 4: bgRain(dt);    break;
    case 5: bgChecker(dt); break;
    default: bgRays(dt);   break;
  }
}

// ------------------------------- Lyrics drawing ---------------------------
static void drawWord(const char* s, uint8_t len, int16_t x, int16_t y, uint8_t size, bool highlight) {
  int16_t cw = 6 * size;
  if (highlight) {                                     // inverted flash when a word appears
    display.fillRect(x - 2, y - 1, len * cw + 3, 8 * size + 2, WHITE);
    for (uint8_t i = 0; i < len; i++) display.drawChar(x + i * cw, y, s[i], BLACK, BLACK, size);
    return;
  }
  static const int8_t O[4][2] = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };
  for (uint8_t o = 0; o < 4; o++)                      // black outline keeps text readable
    for (uint8_t i = 0; i < len; i++) display.drawChar(x + i * cw + O[o][0], y + O[o][1], s[i], BLACK, BLACK, size);
  for (uint8_t i = 0; i < len; i++) display.drawChar(x + i * cw, y, s[i], WHITE, WHITE, size);
}

static void drawNote(uint32_t now) {                   // instrumental part: big bouncing note
  int16_t y = 16 + (int16_t)(sinf(now / 170.0f) * 4.0f - pulse * 6.0f);
  static const int8_t O[4][2] = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };
  for (uint8_t o = 0; o < 4; o++) display.drawChar(52 + O[o][0], y + O[o][1], 14, BLACK, BLACK, 4);
  display.drawChar(52, y, 14, WHITE, WHITE, 4);
}

static void drawLyric(uint32_t now, bool playing) {
  if (nWords == 0) { if (playing) drawNote(now); return; }
  int16_t pitch = (txtSize == 2) ? (nRows <= 3 ? 16 : 15) : 10;
  int16_t total = nRows * pitch - 1;
  int16_t startY = (SCREEN_H - total) / 2;
  int16_t sx = 0, sy = 0;
  if (pulse > 0.7f) { sx = random(-1, 2); sy = random(-1, 2); }    // beat shake
  uint8_t vis = 0;
  int16_t cw = 6 * txtSize;
  for (uint8_t i = 0; i < nWords; i++) {
    if (lineElapsed < (float)words[i].at) break;
    vis++;
    float age = lineElapsed - (float)words[i].at;
    int16_t yoff = 0;
    if (age < 200.0f) { float q = 1.0f - age / 200.0f; yoff = -(int16_t)(q * q * 8.0f); }   // drop-in
    int16_t x = (SCREEN_W - rowW[words[i].row] * cw) / 2 + words[i].col * cw + sx;
    int16_t y = startY + words[i].row * pitch + yoff + sy;
    drawWord(lineText + words[i].off, words[i].len, x, y, txtSize, age < 90.0f);
  }
  if (vis > shownWords && playing) { pulse = pulse > 0.45f ? pulse : 0.45f; spawnRing(); }  // light up per word
  shownWords = vis;
}

// ------------------------------- Render -----------------------------------
static void updateEnergy(uint32_t now, float dt, bool alive, bool playing) {
  pulse -= dt / 0.28f;
  if (pulse < 0) pulse = 0;
  phase += dt * (1.5f + lvl * 4.0f);

  bool fresh = (uint32_t)(now - lastEAt) < 600;
  if (playing && !fresh) {                              // no beat data yet: steady 120 BPM pulse
    lvlTarget = 0.5f + 0.2f * sinf(now / 300.0f);
    if ((int32_t)(now - nextFakeBeat) >= 0) { nextFakeBeat = now + 500; triggerBeat(false); }
  } else if (!playing) {
    lvlTarget = alive ? 0.1f : 0.2f;
    if (!alive && (int32_t)(now - nextFakeBeat) >= 0) { nextFakeBeat = now + 1400; triggerBeat(false); }
  }
  float k = dt * 10.0f;
  lvl += (lvlTarget - lvl) * (k > 1 ? 1 : k);
}

static void render(uint32_t now) {
  float dt = (now - lastRenderAt) / 1000.0f;
  lastRenderAt = now;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.08f) dt = 0.08f;

  bool alive = linkAlive();
  if (wasAlive && !alive) resetToIdle();
  wasAlive = alive;
  bool playing = alive && playState == 1;

  updateEnergy(now, dt, alive, playing);
  if (playing) lineElapsed += dt * 1000.0f;
  if (!alive) styleIdx = 0;

  display.clearDisplay();
  drawBackground(dt);
  if (alive) drawLyric(now, playing);
  display.display();

  bool inv = (int32_t)(flashUntil - now) > 0;           // beat flash
  if (inv != invState) { display.invertDisplay(inv); invState = inv; }
}

// ------------------------------- Arduino entry ----------------------------
void setup() {
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  Serial.setRxBufferSize(1024);                         // must be called before begin()
  Serial.begin(SERIAL_BAUD);

  Wire.begin(OLED_SDA, OLED_SCL);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, true, false);

  initEffects();
  lineText[0] = 0;
  if (oledOk) {
    display.clearDisplay();
    display.display();
    char hello[16];
    snprintf(hello, sizeof(hello), "R|%d", PROTOCOL_VERSION);
    sendFrame(hello);
  }
}

void loop() {
  uint32_t now = millis();

  if (!oledOk) {                                        // fatal: blink LED and report
    static uint32_t lastMsg = 0;
    if ((uint32_t)(now - lastMsg) >= 1000) {
      lastMsg = now;
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      Serial.println("ERROR: SSD1306 not found (check wiring / address 0x3C or 0x3D)");
    }
    return;
  }

  pollSerial();

  if (ledOffAt && (int32_t)(now - ledOffAt) >= 0) {
    digitalWrite(STATUS_LED, LOW);
    ledOffAt = 0;
  }

  if ((uint32_t)(now - lastRenderAt) >= FRAME_MS) render(now);
  yield();
}
