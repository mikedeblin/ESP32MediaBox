/**
 * ============================================================================
 *  ESP32-S3 Audio Player v2.0
 *  ESP32-S3 N16R8 + PCM5102A (I2S) + ST7735 1.8" TFT + OLED 128x32 + microSD
 *  Источники: SD-карта и Synology (DLNA / UPnP ContentDirectory)
 * ============================================================================
 *
 *  Arduino IDE → Инструменты:
 *    Board:            ESP32S3 Dev Module
 *    Flash Size:       16MB
 *    Flash Mode:       QIO 80MHz
 *    PSRAM:            OPI PSRAM
 *    Partition Scheme: 16M Flash (2MB APP/12.5MB FATFS)  (прошивка ~1.44MB)
 *    USB CDC On Boot:  Disabled (консоль через порт UART)
 *
 *  Библиотеки: arduino-audio-tools, arduino-libhelix, LovyanGFX, U8g2
 *  Подробности — README.md / README.ru.md
 *
 *  ── Пины ────────────────────────────────────────────────────────────────────
 *    ST7735 (FSPI):  SCK 18  MOSI 9  CS 10  DC 13  RST 4  LED 2
 *    microSD (SPI3): SCK 40  MISO 42  MOSI 41  CS 5
 *    PCM5102A (I2S): BCK 14  LRCK 12  DIN 11  XSMT 39
 *    I2C (OLED):     SDA 21  SCL 47
 *    Энкодер:        A 15  B 16  SW 17
 *    Кнопки:         PREV 1  NEXT 6  PLAY 7  MODE 8   (на GND, внутр. подтяжка)
 *
 *  ── Управление ──────────────────────────────────────────────────────────────
 *                  Плеер                          Браузер
 *    Энкодер       громкость                      листать
 *    Энк. клик     play / pause                   открыть / играть
 *    Энк. двойной  repeat one вкл/выкл            назад
 *    Энк. долгий   открыть браузер                закрыть браузер
 *    PREV          пред. трек (>3с — в начало)    назад
 *    PREV долгий   стоп                           —
 *    NEXT          след. трек                     открыть / играть
 *    NEXT долгий   repeat one вкл/выкл            —
 *    PLAY          play / pause                   play / pause
 *    PLAY долгий   стоп                           стоп
 *    MODE          экран: карточка → VU полосы    закрыть браузер
 *                  → VU стрелки → обложка
 *    MODE долгий   открыть браузер                закрыть браузер
 *
 *  ── Архитектура ─────────────────────────────────────────────────────────────
 *    Core 0 — audioTask: AudioPlayer, ВСЕ операции с плеером (команды через очередь)
 *    Core 1 — loop(): кнопки, энкодер, дисплеи, браузер, DLNA, обложки, NVS
 * ============================================================================
 */

// ============================ НАСТРОЙКИ ======================================
// WiFi и DLNA — в secrets.h рядом с этим файлом (его не перезаписываем при обновлениях).
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#if !defined(WIFI_SSID) || !defined(WIFI_PASS)
#error "Скопируй secrets.example.h в secrets.h и заполни (WiFi и DLNA) / Copy secrets.example.h to secrets.h"
#endif
#ifndef DLNA_CONTROL_URL
#define DLNA_CONTROL_URL  "http://192.168.1.10:50001/ContentDirectory/control"
#endif
#ifndef DLNA_NAME
#define DLNA_NAME         "DLNA"
#endif

// Буфер сетевого потока в PSRAM (переживает провалы WiFi). 256 КБ ≈ 6-16 с звука
#define NET_BUFFER_KB     256
#define NET_PREFILL_KB    24         // сколько накопить перед стартом трека

#define TFT_BRIGHTNESS    200
#define VU_REF_DBFS       -12.0f     // уровень RMS, который стрелочный VU показывает как 0 VU

// Громкость: шкала в dB. VOL 100 = VOL_MAX_DB, каждый шаг энкодера = VOL_RANGE_DB/99.
// Защиту от перегруза выхода дают компрессор и лимитер ниже.
#define VOL_MAX_DB        0.0f
#define VOL_RANGE_DB      60.0f

// Компрессор, ручки как у ReaComp. Стоит ПОСЛЕ громкости: порог — это уровень на выходе ЦАП,
// поэтому на тихой громкости он не работает вообще, а на громкой не даёт выходу перегружаться.
// По умолчанию — «ReaComp в режиме лимитера»: порог -9, ratio ∞, RMS 5 мс.
#define COMP_ENABLE        1
#define COMP_THRESH_DB     -9.0f
#define COMP_RATIO         0.0f       // 0 = ∞ (лимитер); 4 = 4:1 и т.д.
#define COMP_KNEE_DB       0.0f
#define COMP_ATTACK_MS     3.0f
#define COMP_RELEASE_MS    150.0f
#define COMP_AUTO_RELEASE  0
#define COMP_RMS_MS        5.0f       // 0 = пиковый детектор
#define COMP_PRECOMP_MS    0.0f       // упреждение
#define COMP_MAKEUP_DB     0.0f

// Страховочный лимитер с упреждением на выходе (после громкости), ручки как у ReaLimit:
//   всё выше THRESHOLD лимитируется, затем весь сигнал сдвигается на (CEILING − THRESHOLD),
//   так что пики на выходе = CEILING. Ниже порога звук не трогается.
//   Пример: THRESH 0, CEIL -9 → лимит выше 0 dBFS и весь трек на -9 dB.
#define LIM_THRESH_DB     -1.5f      // порог
#define LIM_CEIL_DB       -1.5f      // потолок на выходе (не выше 0)
#define LIM_LOOKAHEAD_MS  3.0f       // упреждение = время атаки
#define LIM_RELEASE_MS    120.0f     // восстановление
#define DIAG_LEVELS       1          // 1 = печатать [LVL] в консоль раз в секунду

// LED-кольцо энкодера (16 LED = 4 катода × 4 анода) через MCP23017, порт A, I²C 0x20.
// Кольцо: пины 1–4 — катоды, 5–8 — аноды. Распайка: аноды на GPA0–3, катоды на GPA4–7.
#define RING_ENABLE       1
#define RING_ADDR         0x20
#define RING_CATH_BIT     4          // первый бит порта A с катодами (4 подряд)
#define RING_ANODE_BIT    0          // первый бит порта A с анодами (4 подряд)
#define RING_TEST         0          // 1 = при старте зажечь все 16 по очереди и напечатать порядок
// Порядок по кругу: RING_ORDER[позиция] = номер в матрице (катод*4 + анод), как печатает тест.
// Заполни по результату теста, чтобы шкала шла ровно по кругу, по часовой.
#define RING_ORDER        { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }
#define RING_REVERSE      0          // 1 = шкала идёт в обратную сторону

// ============================ БИБЛИОТЕКИ =====================================
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>
#include <math.h>
#include "esp_heap_caps.h"
#include "limiter.h"
#include "compressor.h"

#include "AudioTools.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"
#include "AudioTools/Communication/AudioHttp.h"

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <U8g2lib.h>

// ============================ ПИНЫ ===========================================
#define IO_TFT_SCK    18
#define IO_TFT_MOSI    9
#define IO_TFT_CS     10
#define IO_TFT_DC     13
#define IO_TFT_RST     4
#define IO_TFT_BL      2

#define IO_SD_SCK     40
#define IO_SD_MISO    42
#define IO_SD_MOSI    41
#define IO_SD_CS       5

#define IO_I2S_BCK    14
#define IO_I2S_LRCK   12
#define IO_I2S_DOUT   11
#define IO_DAC_XSMT   39

#define IO_I2C_SDA    21
#define IO_I2C_SCL    47

#define IO_ENC_A      15
#define IO_ENC_B      16
#define IO_ENC_SW     17

#define IO_BTN_PREV    1
#define IO_BTN_NEXT    6
#define IO_BTN_PLAY    7
#define IO_BTN_MODE    8

// ============================ ЛИМИТЫ =========================================
#define LIST_MAX        256      // треков в плейлисте / элементов в папке
#define PATH_LEN        192
#define NAME_LEN         96
#define ID_LEN           64
#define URL_LEN         192
#define META_LEN         96
#define NAV_STACK_DEPTH  10
#define COVER_MAX   (600*1024)   // максимальный размер JPEG обложки
#define LONG_PRESS_MS   700

#define SCR_W 160
#define SCR_H 128

// ============================ ТИПЫ ===========================================
enum Source : uint8_t { SRC_NONE = 0, SRC_SD = 1, SRC_NET = 2 };
enum PlayState : uint8_t { ST_STOP = 0, ST_PLAY, ST_PAUSE };

struct NetTrack {
  char     url[URL_LEN];
  char     title[META_LEN];
  char     artist[META_LEN];
  char     album[META_LEN];
  char     art[URL_LEN];
  uint32_t durSec;
  uint16_t kbps;
};

// Метаданные текущего трека: пишет Core 0 (trackOpen), читает Core 1 (UI)
struct TrackMeta {
  char     title[META_LEN];
  char     artist[META_LEN];
  char     album[META_LEN];
  char     file[NAME_LEN];
  uint32_t durSec;
  uint16_t kbps;
  int      index;
  int      count;
  Source   src;
};

// Запрос обложки: Core 0 говорит «где лежит», Core 1 грузит и декодирует
struct CoverReq {
  Source   src;
  char     path[PATH_LEN];
  uint32_t off;
  uint32_t len;
  char     url[URL_LEN];
};

enum NavType : uint8_t { NT_ROOT_SD, NT_ROOT_NET, NT_DIR, NT_FILE, NT_NETDIR, NT_NETITEM };
struct NavItem {
  char    name[NAME_LEN];
  char    id[ID_LEN];
  int16_t ref;
  NavType type;
};
struct NavStackEntry {
  char path[PATH_LEN];
  char title[NAME_LEN];
  int  index;
};

enum CmdType : uint8_t { CMD_TOGGLE, CMD_STOP, CMD_NEXT, CMD_PREV, CMD_LOAD, CMD_VOLUME };
struct Cmd { CmdType t; int i; float f; };

// ============================ УТИЛИТЫ ========================================
static void* psAlloc(size_t n) {
  void* p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p) p = malloc(n);
  return p;
}

static inline uint32_t be32(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static inline uint32_t syncsafe32(const uint8_t* p) {
  return ((uint32_t)(p[0] & 0x7F) << 21) | ((uint32_t)(p[1] & 0x7F) << 14) |
         ((uint32_t)(p[2] & 0x7F) << 7) | (p[3] & 0x7F);
}

static size_t utf8Put(char* o, uint32_t cp) {
  if (cp < 0x80)    { o[0] = (char)cp; return 1; }
  if (cp < 0x800)   { o[0] = 0xC0 | (cp >> 6); o[1] = 0x80 | (cp & 0x3F); return 2; }
  if (cp < 0x10000) { o[0] = 0xE0 | (cp >> 12); o[1] = 0x80 | ((cp >> 6) & 0x3F);
                      o[2] = 0x80 | (cp & 0x3F); return 3; }
  o[0] = 0xF0 | (cp >> 18); o[1] = 0x80 | ((cp >> 12) & 0x3F);
  o[2] = 0x80 | ((cp >> 6) & 0x3F); o[3] = 0x80 | (cp & 0x3F); return 4;
}

// Обрезает недописанную UTF-8 последовательность в конце строки
static void utf8Trim(char* s) {
  int n = strlen(s);
  if (!n) return;
  int i = n - 1, cont = 0;
  while (i >= 0 && ((uint8_t)s[i] & 0xC0) == 0x80 && cont < 3) { i--; cont++; }
  if (i < 0) { s[0] = 0; return; }
  uint8_t lead = (uint8_t)s[i];
  int need = (lead < 0x80) ? 1 : ((lead & 0xE0) == 0xC0) ? 2 : ((lead & 0xF0) == 0xE0) ? 3 : ((lead & 0xF8) == 0xF0) ? 4 : 1;
  if (cont + 1 < need) s[i] = 0;
}

static void strTrim(char* s) {
  int n = strlen(s);
  while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\r' || s[n - 1] == '\n' || s[n - 1] == '\t')) s[--n] = 0;
  int i = 0;
  while (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n') i++;
  if (i) memmove(s, s + i, n - i + 1);
}

static void fmtTime(char* b, size_t cap, uint32_t sec) {
  sec %= 360000;                                 // не больше 99:59:59
  if (sec >= 3600) snprintf(b, cap, "%u:%02u:%02u", (unsigned)(sec / 3600), (unsigned)((sec / 60) % 60), (unsigned)(sec % 60));
  else             snprintf(b, cap, "%u:%02u", (unsigned)(sec / 60), (unsigned)(sec % 60));
}

static bool isAudioFile(const char* name) {
  const char* ext = strrchr(name, '.');
  return ext && strcasecmp(ext + 1, "mp3") == 0;
}

// ============================ VU-ОТВОД =======================================
// Встаёт между плеером и I2S: пропускает звук дальше и попутно считает
// пики/RMS по каналам и количество отыгранных сэмплов (для времени трека).
class VuTap : public AudioStream {
 public:
  explicit VuTap(AudioStream& out) : p_out(&out) {}

  // Здесь весь выходной тракт: громкость (плавно) → компрессор → лимитер → сдвиг к потолку → ЦАП
  size_t write(const uint8_t* data, size_t len) override {
    if (limRate != rate) {
      lim.setup(rate, LIM_THRESH_DB, LIM_LOOKAHEAD_MS, LIM_RELEASE_MS);
      comp.setup(rate, COMP_THRESH_DB, COMP_RATIO, COMP_KNEE_DB, COMP_ATTACK_MS, COMP_RELEASE_MS,
                 COMP_RMS_MS, COMP_PRECOMP_MS, COMP_AUTO_RELEASE, COMP_MAKEUP_DB);
      float ceil = LIM_CEIL_DB > 0.f ? 0.f : LIM_CEIL_DB;
      makeup = powf(10.f, (ceil - LIM_THRESH_DB) / 20.f);
      limRate = rate;
    }
    float vd = volDb;
    if (vd != lastVol) {
      lastVol = vd;
      gainTarget = vd <= -100.f ? 0.f : powf(10.f, vd / 20.f);
    }
    const int16_t* in = (const int16_t*)data;
    const int c = ch >= 2 ? 2 : 1;
    size_t n = len / 2;
    n -= n % c;
    size_t done = 0;
    while (done < n) {
      size_t k = n - done;
      if (k > GBUF) k = GBUF;
      k -= k % c;
      size_t frames = k / c;
      float g0 = gainCur, g1 = gainTarget, dg = (g1 - g0) / (float)frames;
      float gmin = 1.f, cmin = 1.f;
      for (size_t f = 0; f < frames; f++) {
        float g = (g0 == g1) ? g1 : g0 + dg * (float)(f + 1);
        float l = (float)in[done + f * c] * g;
        float r = (c == 2) ? (float)in[done + f * c + 1] * g : l;
        if (COMP_ENABLE) {
          float cg = comp.process(l, r, l, r);
          if (cg < cmin) cmin = cg;
        }
        float ol, orr;
        float lg = lim.process(l, r, ol, orr);
        if (lg < gmin) gmin = lg;
        gbuf[f * c] = sat16(ol * makeup);
        if (c == 2) gbuf[f * c + 1] = sat16(orr * makeup);
      }
      gainCur = g1;
      // Отдаём в I2S целиком: состояние лимитера уже продвинуто, повторно эти сэмплы не придут
      size_t off = 0;
      int tries = 0;
      while (off < k * 2 && tries < 500) {
        size_t w = p_out->write((const uint8_t*)gbuf + off, k * 2 - off);
        if (w == 0) { vTaskDelay(1); tries++; continue; }
        off += w;
      }
      analyze((const uint8_t*)gbuf, off);
      portENTER_CRITICAL(&mux);
      if (gmin < grMinDiag) grMinDiag = gmin;
      if (cmin < crMinDiag) crMinDiag = cmin;
      if (gmin * cmin < grMinUi) grMinUi = gmin * cmin;
      portEXIT_CRITICAL(&mux);
      done += k;
    }
    return n * 2;
  }
  void setVolumeDb(float db) { volDb = db; }
  // Минимальное усиление лимитера с прошлого вызова (1.0 = не работал)
  float limiterUi() {
    portENTER_CRITICAL(&mux);
    float g = grMinUi; grMinUi = 1.f;
    portEXIT_CRITICAL(&mux);
    return g;
  }
  int  availableForWrite() override { return p_out->availableForWrite(); }
  void flush() override { p_out->flush(); }
  void setAudioInfo(AudioInfo ai) override {
    AudioStream::setAudioInfo(ai);
    p_out->setAudioInfo(ai);
    if (ai.sample_rate) rate = ai.sample_rate;
    if (ai.channels)    ch = ai.channels;
  }

  // Для UI: забрать накопленное и обнулить
  void fetch(uint16_t& pkL, uint16_t& pkR, float& rmsL, float& rmsR) {
    portENTER_CRITICAL(&mux);
    pkL = accPkL; pkR = accPkR;
    rmsL = accN ? sqrtf(accSqL / accN) : 0;
    rmsR = accN ? sqrtf(accSqR / accN) : 0;
    accPkL = accPkR = 0; accSqL = accSqR = 0; accN = 0;
    portEXIT_CRITICAL(&mux);
  }
  // Диагностика: пик и число «упёршихся» сэмплов с прошлого вызова
  void diag(uint16_t& pk, uint32_t& clips, uint32_t& n, float& grDb, float& crDb) {
    portENTER_CRITICAL(&mux);
    pk = dPk; clips = dClip; n = dN;
    float g = grMinDiag, cr = crMinDiag;
    dPk = 0; dClip = 0; dN = 0; grMinDiag = 1.f; crMinDiag = 1.f;
    portEXIT_CRITICAL(&mux);
    grDb = 20.f * log10f(g > 1e-6f ? g : 1e-6f);
    crDb = 20.f * log10f(cr > 1e-6f ? cr : 1e-6f);
  }
  uint32_t frames()     { return framesOut; }
  void     resetFrames() { framesOut = 0; lim.reset(); comp.reset(); }   // новый трек — без хвоста старого
  uint32_t sampleRate() { return rate; }

 private:
  AudioStream* p_out;
  static constexpr size_t GBUF = 512;
  int16_t gbuf[GBUF];
  Limiter lim;
  uint32_t limRate = 0;
  volatile float volDb = 0.f;
  float lastVol = 1e9f, gainTarget = 1.f, gainCur = 1.f, makeup = 1.f;
  float grMinDiag = 1.f, crMinDiag = 1.f, grMinUi = 1.f;
  ReaStyleComp comp;
  static inline int16_t sat16(float v) {
    int32_t x = (int32_t)lroundf(v);
    return (int16_t)(x > 32767 ? 32767 : (x < -32768 ? -32768 : x));
  }
  volatile uint32_t rate = 44100;
  volatile uint8_t  ch = 2;
  volatile uint32_t framesOut = 0;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  uint16_t accPkL = 0, accPkR = 0;
  float    accSqL = 0, accSqR = 0;
  uint32_t accN = 0;
  uint16_t dPk = 0;
  uint32_t dClip = 0, dN = 0;

  void analyze(const uint8_t* d, size_t len) {
    const int16_t* s = (const int16_t*)d;
    size_t n = len / 2;
    if (!n) return;
    uint16_t pL = 0, pR = 0;
    float sqL = 0, sqR = 0;
    uint32_t fr, clip = 0;
    for (size_t i = 0; i < n; i++) if (s[i] >= 32766 || s[i] <= -32767) clip++;
    if (ch >= 2) {
      fr = n / 2;
      for (size_t i = 0; i + 1 < n; i += 2) {
        int32_t l = s[i], r = s[i + 1];
        uint16_t al = (uint16_t)(l < 0 ? -l : l), ar = (uint16_t)(r < 0 ? -r : r);
        if (al > pL) pL = al;
        if (ar > pR) pR = ar;
        sqL += (float)l * l; sqR += (float)r * r;
      }
    } else {
      fr = n;
      for (size_t i = 0; i < n; i++) {
        int32_t v = s[i];
        uint16_t a = (uint16_t)(v < 0 ? -v : v);
        if (a > pL) pL = a;
        sqL += (float)v * v;
      }
      pR = pL; sqR = sqL;
    }
    portENTER_CRITICAL(&mux);
    if (pL > accPkL) accPkL = pL;
    if (pR > accPkR) accPkR = pR;
    accSqL += sqL; accSqR += sqR; accN += fr;
    uint16_t pk = pL > pR ? pL : pR;
    if (pk > dPk) dPk = pk;
    dClip += clip; dN += n;
    portEXIT_CRITICAL(&mux);
    framesOut += fr;
  }
};

// ============================ СЕТЕВОЙ ПОТОК С БУФЕРОМ ========================
// URLStream + кольцевой буфер в PSRAM + задача-подкачка на Core 0.
// Плеер читает только из буфера и не ждёт сеть. Провалы WiFi гасятся буфером.
// (Библиотечный URLStreamBuffered выделял память во внутренней RAM — WiFi умирал.)
class NetStream : public AudioStream {
 public:
  bool init(size_t bytes) {
    cap = bytes;
    ring = (uint8_t*)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ring) return false;
    return xTaskCreatePinnedToCore(taskFn, "netfill", 6144, this, 3, &task, 0) == pdPASS;
  }

  // Вызывается из аудио-задачи
  bool open(const char* url) {
    close();
    head = tail = 0;
    eof = false;
    prebuffering = true;
    if (!url_.begin(url, "audio/mpeg")) { url_.end(); return false; }
    contentLen = url_.contentLength();
    active = true;
    xTaskNotifyGive(task);
    // Ждём предзаполнения ЗДЕСЬ, до передачи потока плееру: AudioPlayer не сбрасывает
    // таймер автоперехода при смене трека, и пустое первое чтение = мгновенный skip.
    uint32_t t0 = millis();
    size_t need = (size_t)NET_PREFILL_KB * 1024;
    while (used() < need && !eof && millis() - t0 < 8000) vTaskDelay(pdMS_TO_TICKS(10));
    prebuffering = false;
    Serial.printf("[NET] prefill %u bytes in %u ms (len %d)%s\n", (unsigned)used(),
                  (unsigned)(millis() - t0), contentLen, used() ? "" : "  <-- НЕТ ДАННЫХ");
    if (!used()) { close(); return false; }
    return true;
  }
  void close() {
    active = false;
    uint32_t t0 = millis();
    while (busy && millis() - t0 < 2000) vTaskDelay(1);   // дождаться, пока задача отпустит сокет
    url_.end();
    head = tail = 0;
  }

  size_t readBytes(uint8_t* data, size_t len) override {
    size_t fill = used();
    if (len > fill) len = fill;
    size_t t = tail;
    for (size_t i = 0; i < len; i++) { data[i] = ring[t]; t = (t + 1) % cap; }
    __sync_synchronize();
    tail = t;
    return len;
  }
  int available() override { return (int)used(); }
  int read() override { uint8_t b; return readBytes(&b, 1) == 1 ? b : -1; }
  int peek() override { return used() ? ring[tail] : -1; }
  operator bool() override { return active || used() > 0; }
  size_t write(const uint8_t*, size_t) override { return 0; }

  // Для UI: заполнение буфера в процентах
  int fillPercent() { return cap ? (int)(used() * 100 / cap) : 0; }

 private:
  URLStream url_{1024};
  uint8_t* ring = nullptr;
  size_t cap = 0;
  volatile size_t head = 0, tail = 0;
  volatile bool active = false, busy = false, eof = false, prebuffering = true;
  int contentLen = -1;
  TaskHandle_t task = nullptr;

  size_t used() { size_t h = head, t = tail; return (h + cap - t) % cap; }
  size_t space() { return cap - 1 - used(); }

  static void taskFn(void* arg) { ((NetStream*)arg)->loopTask(); }
  void loopTask() {
    static uint8_t tmp[1460];
    for (;;) {
      if (!active) { busy = false; ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50)); continue; }
      busy = true;
      if (!active) continue;
      size_t sp = space();
      if (sp < sizeof(tmp)) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
      int av = url_.available();
      if (av <= 0) {
        if (contentLen > 0 && (int)url_.totalRead() >= contentLen) eof = true;
        vTaskDelay(pdMS_TO_TICKS(3));
        continue;
      }
      size_t want = sizeof(tmp);
      if ((size_t)av < want) want = av;
      size_t n = url_.readBytes(tmp, want);
      size_t h = head;
      for (size_t i = 0; i < n; i++) { ring[h] = tmp[i]; h = (h + 1) % cap; }
      __sync_synchronize();
      head = h;
    }
  }
};

// ============================ ДИСПЛЕЙ ST7735 =================================
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7735S _panel;
  lgfx::Bus_SPI       _bus;
  lgfx::Light_PWM     _light;
 public:
  LGFX() {
    { auto cfg = _bus.config();
      cfg.spi_host = SPI2_HOST; cfg.spi_mode = 0;
      cfg.freq_write = 16000000;   // 27 МГц даёт помехи на длинных проводах макета cfg.freq_read = 16000000;
      cfg.spi_3wire = false; cfg.use_lock = true; cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = IO_TFT_SCK; cfg.pin_mosi = IO_TFT_MOSI; cfg.pin_miso = -1; cfg.pin_dc = IO_TFT_DC;
      _bus.config(cfg); _panel.setBus(&_bus); }
    { auto cfg = _panel.config();
      cfg.pin_cs = IO_TFT_CS; cfg.pin_rst = IO_TFT_RST; cfg.pin_busy = -1;
      cfg.panel_width = 128; cfg.panel_height = 160;
      cfg.memory_width = 128; cfg.memory_height = 160;
      cfg.offset_x = 0; cfg.offset_y = 0; cfg.offset_rotation = 0;
      cfg.readable = false; cfg.invert = false; cfg.rgb_order = true;   // панель BGR: без этого золото на обложках синее
      cfg.dlen_16bit = false; cfg.bus_shared = false;   // SD на отдельной шине
      _panel.config(cfg); }
    { auto cfg = _light.config();
      cfg.pin_bl = IO_TFT_BL; cfg.invert = false; cfg.freq = 44100; cfg.pwm_channel = 7;
      _light.config(cfg); _panel.setLight(&_light); }
    setPanel(&_panel);
  }
};

// Цвета RGB565
static constexpr uint16_t c565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
static constexpr uint16_t C_BG      = c565(8, 10, 14);
static constexpr uint16_t C_BAR     = c565(26, 30, 40);
static constexpr uint16_t C_PANEL   = c565(34, 40, 52);
static constexpr uint16_t C_WHITE   = c565(240, 240, 240);
static constexpr uint16_t C_TEXT    = c565(210, 214, 222);
static constexpr uint16_t C_DIM     = c565(120, 128, 140);
static constexpr uint16_t C_ACCENT  = c565(0, 190, 220);
static constexpr uint16_t C_ARTIST  = c565(255, 196, 80);
static constexpr uint16_t C_SEL     = c565(0, 90, 120);
static constexpr uint16_t C_TRACK   = c565(50, 56, 70);
static constexpr uint16_t C_GREEN   = c565(40, 220, 90);
static constexpr uint16_t C_YELLOW  = c565(250, 210, 40);
static constexpr uint16_t C_RED     = c565(250, 60, 50);
static constexpr uint16_t C_GREEN_D = c565(12, 50, 22);
static constexpr uint16_t C_YELL_D  = c565(55, 48, 10);
static constexpr uint16_t C_RED_D   = c565(60, 16, 14);
static constexpr uint16_t C_FACE    = c565(236, 224, 188);
static constexpr uint16_t C_INK     = c565(30, 26, 20);
static constexpr uint16_t C_NEEDLE  = c565(20, 18, 16);
static constexpr uint16_t C_LEDOFF  = c565(90, 40, 36);
static constexpr uint16_t C_FOLDER  = c565(240, 190, 70);
static constexpr uint16_t C_NOTE    = c565(90, 200, 240);
static constexpr uint16_t C_TOAST   = c565(0, 70, 95);

// Шрифты. lgfxJapanGothic включает Latin-1 и кириллицу (А–я, Ёё).
// Если кириллица не нужна и хочется сэкономить flash — &fonts::Font2 / &fonts::Font0.
#include "fonts.h"   // PT Sans, сглаженные VLW
static lgfx::PointerWrapper fwTitle, fwText, fwSmall;
static lgfx::VLWfont        vfTitle, vfText, vfSmall;
#define FONT_TITLE  (&vfTitle)   // PT Sans Bold 17px
#define FONT_TEXT   (&vfText)    // PT Sans 13px
#define FONT_SMALL  (&vfSmall)   // PT Sans 11px

static void loadFonts() {
  fwTitle.set(ptsans_bold_17, sizeof(ptsans_bold_17)); vfTitle.loadFont(&fwTitle);
  fwText.set(ptsans_13, sizeof(ptsans_13));            vfText.loadFont(&fwText);
  fwSmall.set(ptsans_11, sizeof(ptsans_11));           vfSmall.loadFont(&fwSmall);
}

// ============================ ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ==============================
static LGFX        tft;
static LGFX_Sprite frame(&tft);
static LGFX_Sprite coverBig(&tft);
static LGFX_Sprite coverSmall(&tft);
static LGFX_Sprite vuFaceL(&tft);
static LGFX_Sprite vuFaceR(&tft);

static U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE, IO_I2C_SCL, IO_I2C_SDA);

static SPIClass    sdSPI(HSPI);
static Preferences prefs;

static AudioSourceCallback source;
static I2SStream           i2s;
static VuTap               vuTap(i2s);
static MP3DecoderHelix     decoder;
static AudioPlayer         player(source, vuTap, decoder);
static File                trackFile;
static NetStream           netStream;
static bool                netOpen = false;

// ---- Плейлист (активный — Core 0; pending — готовит Core 1 для CMD_LOAD) ----
static char     (*sdList)[PATH_LEN] = nullptr;
static NetTrack*  netList = nullptr;
static int        trackCount = 0;
static int        trackIndex = 0;
static Source     playSrc = SRC_NONE;

static char     (*sdPend)[PATH_LEN] = nullptr;
static NetTrack*  netPend = nullptr;
static int        pendCount = 0;
static Source     pendSrc = SRC_NONE;
static volatile bool pendBusy = false;

// ---- Межъядерный обмен ----
static QueueHandle_t      cmdQ;
static volatile PlayState playState = ST_STOP;
static volatile bool      repeatOne = false;
static volatile bool      manualNav = false;
static volatile uint8_t   audioError = 0;       // 1 = файл, 2 = поток
static volatile uint32_t  pauseStart = 0;

static portMUX_TYPE       shareMux = portMUX_INITIALIZER_UNLOCKED;
static TrackMeta          metaShared;
static volatile uint32_t  metaGen = 0;
static CoverReq           coverShared;
static volatile uint32_t  coverGen = 0;

// ---- Браузер (только Core 1) ----
static NavItem*   navItems = nullptr;
static int        navCount = 0;
static int        navIndex = 0;
static bool       navMode = false;
static Source     navSrc = SRC_NONE;              // NONE = корень «Library»
static char       navPath[PATH_LEN] = "/";        // SD: путь, NET: ObjectID
static char       navTitle[NAME_LEN] = "";
static NavStackEntry navStack[NAV_STACK_DEPTH];   // только для NET
static int        navStackTop = 0;
static NetTrack*  netBrowse = nullptr;
static int        netBrowseCount = 0;

// ---- UI (только Core 1) ----
enum Screen : uint8_t { SCR_CARD = 0, SCR_VU_BARS, SCR_VU_NEEDLE, SCR_COVER, SCR_COUNT };
static uint8_t    screenMode = SCR_CARD;
static TrackMeta  ui;
static uint32_t   uiMetaGen = 0;
static uint32_t   uiCoverGen = 0;
static bool       coverOk = false;
static int        volume = 50;
static bool       sdReady = false;
static char       activeSdDir[PATH_LEN] = "/";
static char       activeNetCon[ID_LEN] = "0";
static char       activeNetTitle[NAME_LEN] = DLNA_NAME;
static char       toastMsg[48] = "";
static uint32_t   toastUntil = 0;
static uint32_t   volShowUntil = 0;
static bool       nvsDirty = false;
static uint32_t   nvsTimer = 0;

// ============================ ПАРСЕР ID3 / MP3 ===============================
// Win-1251 → Unicode (только кириллица, остальное '?')
static uint32_t cp1251(uint8_t b) {
  if (b >= 0xC0) return 0x410 + (b - 0xC0);
  if (b == 0xA8) return 0x401;
  if (b == 0xB8) return 0x451;
  return '?';
}

// Строка ID3 → UTF-8. enc: 0=Latin-1 (или CP1251 — угадываем), 1=UTF-16+BOM, 2=UTF-16BE, 3=UTF-8
static void decodeID3Text(char* dst, int cap, const uint8_t* s, int n, uint8_t enc) {
  int o = 0;
  dst[0] = 0;
  auto put = [&](uint32_t cp) -> bool {
    if (cp == 0) return false;
    char tmp[4];
    size_t k = utf8Put(tmp, cp);
    if (o + (int)k >= cap) return false;
    memcpy(dst + o, tmp, k);
    o += k;
    return true;
  };
  if (enc == 1 || enc == 2) {
    bool be = (enc == 2);
    int i = 0;
    if (n >= 2) {
      if (s[0] == 0xFF && s[1] == 0xFE) { be = false; i = 2; }
      else if (s[0] == 0xFE && s[1] == 0xFF) { be = true; i = 2; }
    }
    for (; i + 1 < n; i += 2) {
      uint32_t u = be ? ((uint32_t)s[i] << 8 | s[i + 1]) : ((uint32_t)s[i + 1] << 8 | s[i]);
      if (u == 0) break;
      if (u >= 0xD800 && u <= 0xDBFF && i + 3 < n) {
        uint32_t lo = be ? ((uint32_t)s[i + 2] << 8 | s[i + 3]) : ((uint32_t)s[i + 3] << 8 | s[i + 2]);
        if (lo >= 0xDC00 && lo <= 0xDFFF) { u = 0x10000 + ((u - 0xD800) << 10) + (lo - 0xDC00); i += 2; }
      }
      if (!put(u)) break;
    }
  } else if (enc == 3) {
    for (int i = 0; i < n && s[i] && o < cap - 1; i++) dst[o++] = s[i];
  } else {
    // Latin-1 или CP1251? Если «высоких» байт больше половины от ASCII-букв — это кириллица
    int hi = 0, letters = 0;
    for (int i = 0; i < n && s[i]; i++) {
      if (s[i] >= 0x80) hi++;
      else if (isalpha(s[i])) letters++;
    }
    bool useCp = hi > 0 && hi * 2 > letters;
    for (int i = 0; i < n && s[i]; i++) {
      uint8_t b = s[i];
      uint32_t cp = b < 0x80 ? b : (useCp ? cp1251(b) : b);
      if (!put(cp)) break;
    }
  }
  dst[o] = 0;
  utf8Trim(dst);
  strTrim(dst);
}

// Читает ID3v2: TIT2/TPE1/TALB + положение JPEG в APIC. Возвращает начало аудио.
static uint32_t readID3(File& f, TrackMeta& m, uint32_t& apicOff, uint32_t& apicLen) {
  apicOff = apicLen = 0;
  f.seek(0);
  uint8_t h[10];
  if (f.read(h, 10) != 10 || memcmp(h, "ID3", 3) != 0) return 0;
  uint8_t  ver = h[3], flags = h[5];
  uint32_t tagSize = syncsafe32(h + 6);
  uint32_t total = 10 + tagSize + ((flags & 0x10) ? 10 : 0);
  if (ver < 3 || ver > 4) return total;          // v2.2 — только пропускаем

  uint32_t pos = 10, tagEnd = 10 + tagSize;
  if (flags & 0x40) {                            // extended header
    uint8_t e[4];
    if (f.read(e, 4) != 4) return total;
    uint32_t es = (ver == 4) ? syncsafe32(e) : be32(e) + 4;
    pos = 10 + es;
    f.seek(pos);
  }

  static uint8_t buf[512];
  bool apicFront = false;
  while (pos + 10 <= tagEnd) {
    uint8_t fh[10];
    if (f.read(fh, 10) != 10) break;
    pos += 10;
    if (fh[0] == 0) break;                       // padding
    uint32_t fs = (ver == 4) ? syncsafe32(fh + 4) : be32(fh + 4);
    if (fs == 0 || pos + fs > tagEnd) break;

    bool tit = !memcmp(fh, "TIT2", 4), tpe = !memcmp(fh, "TPE1", 4);
    bool tal = !memcmp(fh, "TALB", 4), apic = !memcmp(fh, "APIC", 4);

    if (tit || tpe || tal) {
      uint32_t rl = fs < sizeof(buf) ? fs : sizeof(buf);
      int got = f.read(buf, rl);
      if (got > 1) {
        char* d = tit ? m.title : (tpe ? m.artist : m.album);
        decodeID3Text(d, META_LEN, buf + 1, got - 1, buf[0]);
      }
    } else if (apic && !apicFront) {
      uint32_t rl = fs < 256 ? fs : 256;
      int got = f.read(buf, rl);
      uint8_t enc = buf[0];
      int i = 1;
      while (i < got && buf[i]) i++;            // MIME
      i++;
      uint8_t ptype = (i < got) ? buf[i] : 0;
      i++;
      if (enc == 1 || enc == 2) { while (i + 1 < got && !(buf[i] == 0 && buf[i + 1] == 0)) i += 2; i += 2; }
      else                      { while (i < got && buf[i]) i++; i++; }
      if (i + 1 < got && buf[i] == 0xFF && buf[i + 1] == 0xD8) {   // JPEG
        apicOff = pos + i;
        apicLen = fs - i;
        if (ptype == 3) apicFront = true;       // front cover — лучший вариант
      }
    }
    pos += fs;
    f.seek(pos);
  }
  return total;
}

// Длительность и битрейт по первому MPEG-кадру (Xing/Info/VBRI или CBR)
static bool parseMp3Info(File& f, uint32_t start, uint32_t fsize, uint32_t& dur, uint16_t& kbps) {
  static const uint16_t BR1[16] = {0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 0};
  static const uint16_t BR2[16] = {0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160, 0};
  static const uint32_t SR[3][3] = {{44100, 48000, 32000}, {22050, 24000, 16000}, {11025, 12000, 8000}};
  static uint8_t mb[2048];
  dur = 0; kbps = 0;
  if (start >= fsize) return false;
  f.seek(start);
  int n = f.read(mb, sizeof(mb));
  for (int i = 0; i + 4 < n; i++) {
    if (mb[i] != 0xFF || (mb[i + 1] & 0xE0) != 0xE0) continue;
    uint8_t vb = (mb[i + 1] >> 3) & 3, layer = (mb[i + 1] >> 1) & 3;
    uint8_t bri = mb[i + 2] >> 4, sri = (mb[i + 2] >> 2) & 3, mode = mb[i + 3] >> 6;
    if (vb == 1 || layer != 1 || bri == 0 || bri == 15 || sri == 3) continue;
    bool mpeg1 = (vb == 3);
    int vi = mpeg1 ? 0 : (vb == 2 ? 1 : 2);
    uint32_t sr = SR[vi][sri];
    uint16_t br = mpeg1 ? BR1[bri] : BR2[bri];
    uint32_t spf = mpeg1 ? 1152 : 576;
    int xoff = mpeg1 ? (mode == 3 ? 21 : 36) : (mode == 3 ? 13 : 21);
    uint32_t frames = 0;
    if (i + xoff + 12 < n && (!memcmp(mb + i + xoff, "Xing", 4) || !memcmp(mb + i + xoff, "Info", 4))) {
      if (be32(mb + i + xoff + 4) & 1) frames = be32(mb + i + xoff + 8);
    } else if (i + 36 + 18 < n && !memcmp(mb + i + 36, "VBRI", 4)) {
      frames = be32(mb + i + 36 + 14);
    }
    uint32_t audioBytes = fsize - (start + i);
    if (frames) {
      dur = (uint32_t)((uint64_t)frames * spf / sr);
      kbps = dur ? (uint16_t)((uint64_t)audioBytes * 8 / 1000 / dur) : br;
    } else {
      kbps = br;
      dur = br ? (uint32_t)((uint64_t)audioBytes * 8 / ((uint32_t)br * 1000)) : 0;
    }
    return true;
  }
  return false;
}

// ============================ ОТКРЫТИЕ ТРЕКОВ (Core 0) ========================
static void publishMeta(const TrackMeta& m, const CoverReq& c) {
  portENTER_CRITICAL(&shareMux);
  metaShared = m;  metaGen = metaGen + 1;
  coverShared = c; coverGen = coverGen + 1;
  portEXIT_CRITICAL(&shareMux);
}

static void closeStreams() {
  if (trackFile) trackFile.close();
  if (netOpen) { netStream.close(); netOpen = false; }
}

static Stream* trackOpen(int idx) {
  if (trackCount <= 0) return nullptr;
  if (idx >= trackCount) idx = 0;
  if (idx < 0) idx = trackCount - 1;
  trackIndex = idx;
  vuTap.resetFrames();

  static TrackMeta m;
  static CoverReq  c;
  memset(&m, 0, sizeof(m));
  memset(&c, 0, sizeof(c));
  m.index = idx; m.count = trackCount; m.src = playSrc;
  c.src = playSrc;

  if (playSrc == SRC_SD) {
    closeStreams();
    const char* path = sdList[idx];
    const char* fn = strrchr(path, '/');
    strlcpy(m.file, fn ? fn + 1 : path, NAME_LEN);
    trackFile = SD.open(path);
    if (!trackFile) {
      Serial.printf("[TRACK] open failed: %s\n", path);
      audioError = 1;
      publishMeta(m, c);
      return nullptr;
    }
    uint32_t apicOff, apicLen;
    uint32_t audioStart = readID3(trackFile, m, apicOff, apicLen);
    parseMp3Info(trackFile, audioStart, trackFile.size(), m.durSec, m.kbps);
    trackFile.seek(0);
    if (apicLen) { strlcpy(c.path, path, PATH_LEN); c.off = apicOff; c.len = apicLen; }
    publishMeta(m, c);
    Serial.printf("[TRACK] SD %d/%d %s (%us, %ukbps)\n", idx + 1, trackCount, path, (unsigned)m.durSec, m.kbps);
    return &trackFile;
  }

  if (playSrc == SRC_NET) {
    const NetTrack& t = netList[idx];
    strlcpy(m.title, t.title, META_LEN);
    strlcpy(m.artist, t.artist, META_LEN);
    strlcpy(m.album, t.album, META_LEN);
    m.durSec = t.durSec; m.kbps = t.kbps;
    strlcpy(c.url, t.art, URL_LEN);
    publishMeta(m, c);                           // UI покажет теги сразу, пока идёт коннект
    closeStreams();
    Serial.printf("[TRACK] NET %d/%d %s\n", idx + 1, trackCount, t.url);
    if (!netStream.open(t.url)) {
      Serial.println("[TRACK] stream open failed");
      audioError = 2;
      return nullptr;
    }
    netOpen = true;
    return &netStream;
  }
  return nullptr;
}

// Колбэки AudioSourceCallback
static Stream* cbNextStream(int offset) {
  if (!manualNav && repeatOne && offset == 1) offset = 0;   // repeat one для автоперехода
  manualNav = false;
  return trackOpen(trackIndex + offset);
}
static Stream* cbSelectStream(int idx) {
  manualNav = false;
  if (idx < 0) return nullptr;
  return trackOpen(idx);
}

// ============================ АУДИО-ЗАДАЧА (Core 0) ===========================
static void updateStateFromPlayer() {
  playState = player.isActive() ? ST_PLAY : ST_STOP;
}

static void handleCmd(const Cmd& c) {
  switch (c.t) {
    case CMD_LOAD: {
      if (player.isActive()) player.setActive(false);
      closeStreams();
      playSrc = pendSrc;
      trackCount = pendCount;
      if (playSrc == SRC_SD) memcpy(sdList, sdPend, (size_t)pendCount * PATH_LEN);
      else                   memcpy(netList, netPend, (size_t)pendCount * sizeof(NetTrack));
      pendBusy = false;
      source.setTimeoutAutoNext(playSrc == SRC_NET ? 2500 : 600);
      int start = c.i;
      if (start < 0 || start >= trackCount) start = 0;
      manualNav = true;
      player.begin(start);
      updateStateFromPlayer();
      break;
    }
    case CMD_TOGGLE:
      if (trackCount == 0) break;
      if (playState == ST_PLAY) {
        player.setActive(false);
        playState = ST_PAUSE;
        pauseStart = millis();
      } else if (playState == ST_PAUSE &&
                 !(playSrc == SRC_NET && millis() - pauseStart > 30000)) {
        player.setActive(true);
        playState = ST_PLAY;
      } else {
        // STOP, или долгая пауза в сети (соединение наверняка умерло) — трек заново
        manualNav = true;
        player.setIndex(trackIndex);
        updateStateFromPlayer();
      }
      break;
    case CMD_STOP:
      if (player.isActive()) player.setActive(false);
      playState = ST_STOP;
      vuTap.resetFrames();
      break;
    case CMD_NEXT:
      if (trackCount == 0) break;
      manualNav = true;
      player.setIndex((trackIndex + 1) % trackCount);
      updateStateFromPlayer();
      break;
    case CMD_PREV: {
      if (trackCount == 0) break;
      bool restart = vuTap.sampleRate() && vuTap.frames() / vuTap.sampleRate() > 3;
      manualNav = true;
      player.setIndex(restart ? trackIndex : (trackIndex - 1 + trackCount) % trackCount);
      updateStateFromPlayer();
      break;
    }
    case CMD_VOLUME:                             // громкость теперь в vuTap.setVolumeDb()
      break;
  }
}

static void audioTask(void*) {
  Cmd c;
  for (;;) {
    while (xQueueReceive(cmdQ, &c, 0) == pdTRUE) handleCmd(c);
    if (playState == ST_PLAY) {
      player.copy();
      if (!player.isActive()) playState = ST_STOP;   // поток не открылся / конец списка
      vTaskDelay(1);
    } else if (xQueueReceive(cmdQ, &c, pdMS_TO_TICKS(20)) == pdTRUE) {
      handleCmd(c);
    }
  }
}

static void sendCmd(CmdType t, int i = 0, float f = 0) {
  Cmd c{t, i, f};
  xQueueSend(cmdQ, &c, pdMS_TO_TICKS(100));
}

// ============================ ОТРИСОВКА: ОБЩЕЕ ================================
struct Scroller {
  int off = 0;
  uint32_t last = 0, holdUntil = 0;
  bool atEnd = false;
  void reset() { off = 0; atEnd = false; last = millis(); holdUntil = last + 1500; }
  int update(int tw, int w) {
    if (tw <= w) { off = 0; return 0; }
    uint32_t now = millis();
    if ((int32_t)(now - holdUntil) < 0) return off;
    if (atEnd) { off = 0; atEnd = false; holdUntil = now + 1500; return 0; }
    if (now - last >= 30) {
      last = now;
      off++;
      if (off >= tw - w) { off = tw - w; atEnd = true; holdUntil = now + 1500; }
    }
    return off;
  }
};
static Scroller scTitle, scArtist, scAlbum, scNav, scHeader;

static void drawText(LGFX_Sprite& g, const char* s, int x, int y, const lgfx::IFont* f, uint16_t col,
                     lgfx::textdatum_t datum = lgfx::textdatum_t::top_left) {
  g.setFont(f);
  g.setTextColor(col);
  g.setTextDatum(datum);
  g.drawString(s, x, y);
  g.setTextDatum(lgfx::textdatum_t::top_left);
}

static void drawScroll(LGFX_Sprite& g, const char* s, int x, int y, int w, int h,
                       const lgfx::IFont* f, uint16_t col, Scroller& sc) {
  g.setFont(f);
  g.setTextColor(col);
  g.setTextDatum(lgfx::textdatum_t::top_left);
  int tw = g.textWidth(s);
  int off = sc.update(tw, w);
  g.setClipRect(x, y, w, h);
  g.drawString(s, x - off, y);
  g.clearClipRect();
}

static void drawClipped(LGFX_Sprite& g, const char* s, int x, int y, int w, int h,
                        const lgfx::IFont* f, uint16_t col) {
  g.setFont(f);
  g.setTextColor(col);
  g.setTextDatum(lgfx::textdatum_t::top_left);
  g.setClipRect(x, y, w, h);
  g.drawString(s, x, y);
  g.clearClipRect();
}

static void drawStateIcon(LGFX_Sprite& g, int x, int y, int s, PlayState st, uint16_t col) {
  if (st == ST_PLAY) {
    g.fillTriangle(x, y, x, y + s - 1, x + s - 1, y + s / 2, col);
  } else if (st == ST_PAUSE) {
    int w = s / 3 > 1 ? s / 3 : 2;
    g.fillRect(x, y, w, s, col);
    g.fillRect(x + s - w, y, w, s, col);
  } else {
    g.fillRect(x, y, s, s, col);
  }
}

static uint32_t elapsedSec() {
  uint32_t sr = vuTap.sampleRate();
  return sr ? vuTap.frames() / sr : 0;
}

static const char* uiTitle() {
  if (ui.title[0]) return ui.title;
  if (ui.file[0])  return ui.file;
  return ui.count ? "Unknown track" : "Nothing playing";
}

static void drawProgress(LGFX_Sprite& g, int x, int y, int w, int h) {
  g.fillRoundRect(x, y, w, h, h / 2, C_TRACK);
  if (ui.durSec) {
    uint32_t e = elapsedSec();
    int fw = (int)((uint64_t)w * (e > ui.durSec ? ui.durSec : e) / ui.durSec);
    if (fw > 0) g.fillRoundRect(x, y, fw, h, h / 2, C_ACCENT);
  }
}

static void drawRssi(LGFX_Sprite& g, int x, int y, uint16_t col) {
  if (WiFi.status() != WL_CONNECTED) { drawText(g, "x", x, y - 2, FONT_SMALL, C_RED); return; }
  int r = WiFi.RSSI();
  int bars = r > -60 ? 4 : r > -68 ? 3 : r > -76 ? 2 : 1;
  for (int i = 0; i < 4; i++) {
    int h = 2 + i * 2;
    g.fillRect(x + i * 3, y + 8 - h, 2, h, i < bars ? col : C_TRACK);
  }
}

static uint32_t limShowUntil = 0;
static void drawStatusBar(LGFX_Sprite& g) {
  g.fillRect(0, 0, SCR_W, 14, C_BAR);
  drawStateIcon(g, 4, 3, 8, playState, C_ACCENT);
  const char* src = ui.src == SRC_NET ? "NET" : ui.src == SRC_SD ? "SD" : "--";
  drawText(g, src, 17, 1, FONT_SMALL, C_TEXT);
  int x = 17 + g.textWidth(src) + 4;
  if (ui.src == SRC_NET) { drawRssi(g, x, 3, C_TEXT); x += 15; }
  if (repeatOne) drawText(g, "RPT", x + 2, 1, FONT_SMALL, C_ARTIST);
  char v[12];
  snprintf(v, sizeof(v), "VOL %d", volume);
  drawText(g, v, SCR_W - 4, 1, FONT_SMALL, C_TEXT, lgfx::textdatum_t::top_right);
  if ((int32_t)(limShowUntil - millis()) > 0) {   // лимитер сейчас работает
    int vx = SCR_W - 4 - g.textWidth(v) - 6;
    drawText(g, "LIM", vx, 1, FONT_SMALL, C_RED, lgfx::textdatum_t::top_right);
  }
}

static void drawTimeRow(LGFX_Sprite& g, int y) {
  char a[12], b[12];
  fmtTime(a, sizeof(a), elapsedSec());
  drawText(g, a, 4, y, FONT_SMALL, C_TEXT);
  if (ui.durSec) {
    fmtTime(b, sizeof(b), ui.durSec);
    drawText(g, b, SCR_W - 4, y, FONT_SMALL, C_TEXT, lgfx::textdatum_t::top_right);
  }
}

// ============================ ЭКРАН: КАРТОЧКА ТРЕКА ===========================
static void drawCard() {
  frame.fillScreen(C_BG);
  drawStatusBar(frame);

  int tx = 4;
  if (coverOk) { coverSmall.pushSprite(&frame, 4, 18); tx = 53; }
  else {
    frame.fillRoundRect(4, 18, 44, 44, 4, C_PANEL);
    frame.fillCircle(21, 47, 5, C_DIM);
    frame.fillRect(25, 27, 2, 21, C_DIM);
    frame.fillRect(25, 27, 9, 3, C_DIM);
    tx = 53;
  }
  int tw = SCR_W - tx - 4;
  drawScroll(frame, ui.artist[0] ? ui.artist : "Unknown artist", tx, 17, tw, 16, FONT_TEXT, C_ARTIST, scArtist);
  drawScroll(frame, ui.album[0] ? ui.album : "—", tx, 33, tw, 16, FONT_TEXT, C_DIM, scAlbum);
  if (ui.count) {
    char t[24];
    snprintf(t, sizeof(t), "%d / %d", ui.index + 1, ui.count);
    drawText(frame, t, tx, 50, FONT_SMALL, C_DIM);
  }

  drawScroll(frame, uiTitle(), 4, 65, SCR_W - 8, 21, FONT_TITLE, C_WHITE, scTitle);

  drawProgress(frame, 4, 90, SCR_W - 8, 5);
  drawTimeRow(frame, 98);

  char info[48];
  uint32_t sr = vuTap.sampleRate();
  if (ui.kbps) snprintf(info, sizeof(info), "MP3  %uk  %u.%ukHz", ui.kbps, (unsigned)(sr / 1000), (unsigned)((sr % 1000) / 100));
  else         snprintf(info, sizeof(info), "MP3  %u.%ukHz", (unsigned)(sr / 1000), (unsigned)((sr % 1000) / 100));
  drawText(frame, info, 4, 113, FONT_SMALL, C_DIM);
}

// ============================ УРОВНИ VU =======================================
static float    barL = -60, barR = -60, holdL = -60, holdR = -60;
static uint32_t holdTL = 0, holdTR = 0;
static float    needleL = 0, needleR = 0;          // линейный RMS 0..1
static uint32_t peakLedL = 0, peakLedR = 0;
static uint32_t lastLevelMs = 0;

static float toDb(float lin) { return lin > 1e-6f ? 20.f * log10f(lin) : -90.f; }

static void updateLevels() {
  uint32_t now = millis();
  float dt = (now - lastLevelMs) / 1000.f;
  if (dt > 0.2f) dt = 0.2f;
  lastLevelMs = now;
  uint16_t pkL, pkR;
  float rmsL, rmsR;
  vuTap.fetch(pkL, pkR, rmsL, rmsR);
  if (vuTap.limiterUi() < 0.944f) limShowUntil = now + 400;   // ослабление больше 0.5 dB
  float pL = toDb(pkL / 32768.f), pR = toDb(pkR / 32768.f);

  // Полосы: мгновенная атака, спад 24 dB/с
  barL = pL > barL ? pL : barL - 24.f * dt;
  barR = pR > barR ? pR : barR - 24.f * dt;
  if (barL < -60) barL = -60;
  if (barR < -60) barR = -60;
  // Пиковое удержание 1с, потом спад 12 dB/с
  if (pL >= holdL) { holdL = pL; holdTL = now; } else if (now - holdTL > 1000) holdL -= 12.f * dt;
  if (pR >= holdR) { holdR = pR; holdTR = now; } else if (now - holdTR > 1000) holdR -= 12.f * dt;
  // Стрелки: интегрирование ~300 мс, как у настоящего VU
  float a = 1.f - expf(-dt / 0.3f);
  needleL += (rmsL / 32768.f - needleL) * a;
  needleR += (rmsR / 32768.f - needleR) * a;
  if (pL > -1.f) peakLedL = now + 300;
  if (pR > -1.f) peakLedR = now + 300;
}

// ============================ ЭКРАН: VU ПОЛОСЫ ================================
static void drawVuBar(int y, int h, float level, float hold) {
  const int x0 = 16, segW = 3, gap = 1, N = 34;
  const float lo = -48.f;
  for (int i = 0; i < N; i++) {
    float segDb = lo + (i + 1) * (-lo) / N;
    bool lit = level >= segDb - (-lo) / N * 0.5f;
    uint16_t col;
    if (segDb > -3.f)       col = lit ? C_RED : C_RED_D;
    else if (segDb > -12.f) col = lit ? C_YELLOW : C_YELL_D;
    else                    col = lit ? C_GREEN : C_GREEN_D;
    frame.fillRect(x0 + i * (segW + gap), y, segW, h, col);
  }
  if (hold > lo) {
    int hi = (int)((hold - lo) / (-lo) * N);
    if (hi > N - 1) hi = N - 1;
    if (hi >= 0) frame.fillRect(x0 + hi * (segW + gap), y, segW, h, C_WHITE);
  }
}

static void drawVuBars() {
  frame.fillScreen(C_BG);
  drawStatusBar(frame);
  drawText(frame, "L", 5, 22, FONT_SMALL, C_TEXT);
  drawText(frame, "R", 5, 60, FONT_SMALL, C_TEXT);
  drawVuBar(20, 18, barL, holdL);
  drawVuBar(58, 18, barR, holdR);
  // Шкала dB между полосами
  static const int marks[] = {-40, -30, -20, -12, -6, -3, 0};
  for (int m : marks) {
    int x = 16 + (int)((m + 48.f) / 48.f * 136.f);
    frame.drawFastVLine(x - 1, 40, 3, C_DIM);
    char t[8];
    snprintf(t, sizeof(t), "%d", m < 0 ? -m : m);
    drawText(frame, t, x - 1, 43, FONT_SMALL, m >= -3 ? C_RED : C_DIM, lgfx::textdatum_t::top_center);
  }
  drawScroll(frame, uiTitle(), 4, 80, SCR_W - 8, 16, FONT_TEXT, C_WHITE, scTitle);
  drawProgress(frame, 4, 99, SCR_W - 8, 4);
  drawTimeRow(frame, 108);
}

// ============================ ЭКРАН: VU СТРЕЛКИ ===============================
static const int VU_W = 156, VU_H = 61, VU_PX = 78, VU_PY = 88;

static float vuPos(float vu) {                     // шкала VU линейна по напряжению
  const float pmin = 0.1f, pmax = 1.4125f;         // -20 VU ... +3 VU
  return (powf(10.f, vu / 20.f) - pmin) / (pmax - pmin);
}
static float vuDeg(float pos) { return 230.f + pos * 80.f; }   // 0° = 3 часа, по часовой

static void polar(float deg, float r, int& x, int& y) {
  float rad = deg * (float)M_PI / 180.f;
  x = VU_PX + (int)lroundf(r * cosf(rad));
  y = VU_PY + (int)lroundf(r * sinf(rad));
}

static void buildVuFace(LGFX_Sprite& f, const char* chName) {
  f.setColorDepth(16);
  f.setPsram(true);
  f.createSprite(VU_W, VU_H);
  f.fillScreen(C_BG);
  f.fillRoundRect(0, 0, VU_W, VU_H, 5, C_FACE);
  f.fillArc(VU_PX, VU_PY, 67, 68, vuDeg(vuPos(-20)), vuDeg(vuPos(0)), C_INK);
  f.fillArc(VU_PX, VU_PY, 64, 68, vuDeg(vuPos(0)), vuDeg(vuPos(3)), C_RED);
  static const float ticks[] = {-20, -10, -7, -5, -3, -2, -1, 0, 1, 2, 3};
  for (float v : ticks) {
    float d = vuDeg(vuPos(v));
    bool major = (v == -20 || v == -10 || v == -5 || v == -3 || v == 0 || v == 3);
    int x0, y0, x1, y1;
    polar(d, major ? 60 : 63, x0, y0);
    polar(d, 68, x1, y1);
    f.drawLine(x0, y0, x1, y1, v > 0 ? C_RED : C_INK);
    if (major) {
      char t[5];
      snprintf(t, sizeof(t), v > 0 ? "+%d" : "%d", (int)fabsf(v));
      int lx, ly;
      polar(d, 76, lx, ly);
      f.setFont(FONT_SMALL);
      f.setTextColor(v > 0 ? C_RED : C_INK);
      f.setTextDatum(lgfx::textdatum_t::middle_center);
      f.drawString(t, lx, ly);
    }
  }
  f.setFont(FONT_TEXT);
  f.setTextColor(C_INK);
  f.setTextDatum(lgfx::textdatum_t::middle_center);
  f.drawString("VU", VU_PX, 38);
  f.setFont(FONT_SMALL);
  f.setTextDatum(lgfx::textdatum_t::top_left);
  f.drawString(chName, 6, VU_H - 12);
}

static void drawNeedle(int fx, int fy, float lin, uint32_t ledUntil) {
  float vu = toDb(lin) - VU_REF_DBFS;              // 0 VU = VU_REF_DBFS RMS
  float pos = vuPos(vu);
  if (pos < -0.03f) pos = -0.03f;
  if (pos > 1.05f)  pos = 1.05f;
  float d = vuDeg(pos) * (float)M_PI / 180.f;
  int tx = fx + VU_PX + (int)lroundf(70.f * cosf(d));
  int ty = fy + VU_PY + (int)lroundf(70.f * sinf(d));
  frame.setClipRect(fx + 1, fy + 1, VU_W - 2, VU_H - 2);
  frame.drawWideLine(fx + VU_PX, fy + VU_PY, tx, ty, 0.8f, C_NEEDLE);
  frame.clearClipRect();
  frame.fillCircle(fx + VU_W - 10, fy + 9, 3, (int32_t)(ledUntil - millis()) > 0 ? C_RED : C_LEDOFF);
}

static void drawVuNeedles() {
  frame.fillScreen(C_BG);
  vuFaceL.pushSprite(&frame, 2, 1);
  vuFaceR.pushSprite(&frame, 2, 65);
  drawNeedle(2, 1, needleL, peakLedL);
  drawNeedle(2, 65, needleR, peakLedR);
}

// ============================ ЭКРАН: ОБЛОЖКА ==================================
static void drawCover() {
  frame.fillScreen(C_BG);
  if (coverOk) {
    coverBig.pushSprite(&frame, 16, 0);
  } else {
    frame.fillRoundRect(16, 0, 128, 128, 6, C_PANEL);
    frame.fillCircle(70, 72, 12, C_DIM);
    frame.fillRect(79, 30, 4, 42, C_DIM);
    frame.fillRect(79, 30, 20, 6, C_DIM);
    drawText(frame, "No cover", 80, 100, FONT_TEXT, C_DIM, lgfx::textdatum_t::top_center);
  }
  drawStateIcon(frame, 3, 4, 9, playState, C_ACCENT);
  if (repeatOne) drawText(frame, "R", 4, 16, FONT_SMALL, C_ARTIST);
  // Прогресс вертикально справа
  frame.fillRect(151, 2, 4, 124, C_TRACK);
  if (ui.durSec) {
    uint32_t e = elapsedSec();
    int h = (int)((uint64_t)124 * (e > ui.durSec ? ui.durSec : e) / ui.durSec);
    frame.fillRect(151, 2 + 124 - h, 4, h, C_ACCENT);
  }
}

// ============================ ЭКРАН: БРАУЗЕР ==================================
static void drawFolderIcon(int x, int y, uint16_t col) {
  frame.fillRect(x, y + 1, 5, 2, col);
  frame.fillRoundRect(x, y + 3, 12, 8, 1, col);
}
static void drawNoteIcon(int x, int y, uint16_t col) {
  frame.fillCircle(x + 3, y + 9, 2, col);
  frame.fillRect(x + 5, y + 1, 2, 9, col);
  frame.fillRect(x + 5, y + 1, 6, 2, col);
}

static void drawBrowser() {
  frame.fillScreen(C_BG);
  frame.fillRect(0, 0, SCR_W, 15, C_BAR);
  const char* tag = navSrc == SRC_SD ? "SD" : navSrc == SRC_NET ? "NET" : "";
  int hx = 4;
  if (tag[0]) { drawText(frame, tag, 4, 2, FONT_SMALL, C_ACCENT); hx = 4 + frame.textWidth(tag) + 5; }
  const char* hdr = navSrc == SRC_NONE ? "Library" : (navSrc == SRC_SD ? navPath : navTitle);
  drawScroll(frame, hdr, hx, 0, SCR_W - hx - 3, 15, FONT_TEXT, C_WHITE, scHeader);

  const int rowH = 16, rows = 6, y0 = 16;
  if (navCount == 0) {
    drawText(frame, navSrc == SRC_NET ? "Nothing here" : "Empty folder", SCR_W / 2, 58,
             FONT_TEXT, C_DIM, lgfx::textdatum_t::top_center);
  }
  int start = navIndex - 2;
  if (start + rows > navCount) start = navCount - rows;
  if (start < 0) start = 0;
  for (int r = 0; r < rows && start + r < navCount; r++) {
    int i = start + r, y = y0 + r * rowH;
    const NavItem& it = navItems[i];
    bool sel = (i == navIndex);
    if (sel) frame.fillRoundRect(1, y, SCR_W - 7, rowH - 1, 3, C_SEL);
    bool dir = it.type == NT_DIR || it.type == NT_NETDIR || it.type == NT_ROOT_SD || it.type == NT_ROOT_NET;
    if (dir) drawFolderIcon(4, y + 2, it.type == NT_ROOT_NET ? C_ACCENT : C_FOLDER);
    else     drawNoteIcon(4, y + 2, C_NOTE);
    if (sel) drawScroll(frame, it.name, 19, y, SCR_W - 7 - 21, 16, FONT_TEXT, C_WHITE, scNav);
    else     drawClipped(frame, it.name, 19, y, SCR_W - 7 - 21, 16, FONT_TEXT, C_TEXT);
  }
  if (navCount > rows) {                         // полоса прокрутки
    int th = rows * rowH;
    int bh = th * rows / navCount;
    if (bh < 6) bh = 6;
    int by = y0 + (th - bh) * navIndex / (navCount - 1);
    frame.fillRect(SCR_W - 4, y0, 3, th, C_TRACK);
    frame.fillRect(SCR_W - 4, by, 3, bh, C_ACCENT);
  }
  char pos[24];
  snprintf(pos, sizeof(pos), "%d/%d", navCount ? navIndex + 1 : 0, navCount);
  drawText(frame, "OK: open   DBL: back", 3, 114, FONT_SMALL, C_DIM);
  drawText(frame, pos, SCR_W - 3, 114, FONT_SMALL, C_DIM, lgfx::textdatum_t::top_right);
}

// ============================ ОВЕРЛЕИ ========================================
static void toast(const char* msg, uint32_t ms = 1500) {
  strlcpy(toastMsg, msg, sizeof(toastMsg));
  toastUntil = millis() + ms;
  Serial.printf("[UI] %s\n", msg);
}

static void drawOverlays() {
  uint32_t now = millis();
  if ((int32_t)(volShowUntil - now) > 0 && !navMode) {
    frame.fillRoundRect(20, 50, 120, 28, 5, C_TOAST);
    char t[16];
    snprintf(t, sizeof(t), "Volume %d", volume);
    drawText(frame, t, 80, 53, FONT_TEXT, C_WHITE, lgfx::textdatum_t::top_center);
    frame.fillRect(28, 69, 104, 4, C_TRACK);
    frame.fillRect(28, 69, 104 * volume / 100, 4, C_ACCENT);
  } else if ((int32_t)(toastUntil - now) > 0) {
    frame.fillRoundRect(10, 98, 140, 20, 5, C_TOAST);
    drawText(frame, toastMsg, 80, 100, FONT_TEXT, C_WHITE, lgfx::textdatum_t::top_center);
  }
}

static void showBusy(const char* msg) {
  frame.fillScreen(C_BG);
  frame.fillRoundRect(10, 44, 140, 40, 6, C_PANEL);
  drawText(frame, msg, 80, 57, FONT_TEXT, C_WHITE, lgfx::textdatum_t::top_center);
  frame.pushSprite(0, 0);
}

static void renderTFT() {
  if (navMode) drawBrowser();
  else switch (screenMode) {
    case SCR_VU_BARS:   drawVuBars();    break;
    case SCR_VU_NEEDLE: drawVuNeedles(); break;
    case SCR_COVER:     drawCover();     break;
    default:            drawCard();      break;
  }
  drawOverlays();
  frame.pushSprite(0, 0);
}

// ============================ OLED 128x32 =====================================
static void renderOLED() {
  // Только состояние: крупная иконка + слово. Перерисовываем лишь при смене.
  static int last = -1;
  int key = (int)playState | (repeatOne ? 0x10 : 0);
  if (last == key) return;
  last = key;
  oled.clearBuffer();
  const char* word;
  switch (playState) {
    case ST_PLAY:
      oled.drawTriangle(4, 4, 4, 27, 24, 16);
      word = "PLAY";
      break;
    case ST_PAUSE:
      oled.drawBox(4, 4, 7, 24);
      oled.drawBox(17, 4, 7, 24);
      word = "PAUSE";
      break;
    default:
      oled.drawBox(4, 5, 22, 22);
      word = "STOP";
      break;
  }
  oled.setFont(u8g2_font_logisoso22_tr);
  oled.drawStr(32, 27, word);
  if (repeatOne) {                               // ↻1 — закольцованная стрелка
    oled.drawCircle(114, 16, 10);
    oled.drawCircle(114, 16, 9);
    oled.setDrawColor(0);
    oled.drawBox(114, 3, 13, 8);                 // разрыв кольца справа сверху
    oled.setDrawColor(1);
    oled.drawTriangle(114, 1, 114, 11, 120, 6);  // стрелка по часовой
    oled.setFont(u8g2_font_5x7_tr);
    oled.drawStr(112, 20, "1");
  }
  oled.sendBuffer();
}

// ============================ LED-КОЛЬЦО (MCP23017) ===========================
// Мультиплексирование: 4 группы (катоды) по очереди, по 2 мс → 125 Гц на кадр.
// Отдельная задача: если делать это в loop(), кольцо мерцает, пока рисуется TFT (~20 мс).
// Wire держит мьютекс на транзакцию, поэтому шину можно делить с OLED.
static bool              ringOk = false;
static volatile uint16_t ringMask = 0;       // бит n = светодиод на позиции n по кругу
static volatile int      ringTestIdx = -1;   // >=0 — тест: горит этот номер матрицы
static const uint8_t     ringOrder[16] = RING_ORDER;

static bool mcpWrite(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(RING_ADDR);
  Wire.write(reg);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

static uint8_t ringPortByte(int group, uint8_t anodes4) {
  uint8_t cath = 0x0F;                         // все катоды HIGH = выключены
  if (anodes4) cath &= ~(1 << group);          // активная группа LOW
  return (uint8_t)((cath << RING_CATH_BIT) | ((anodes4 & 0x0F) << RING_ANODE_BIT));
}

static void ringTask(void*) {
  int group = 0;
  for (;;) {
    uint8_t an[4] = {0, 0, 0, 0};
    int t = ringTestIdx;
    if (t >= 0) {
      an[t / 4] |= 1 << (t % 4);
    } else {
      uint16_t m = ringMask;
      for (int pos = 0; pos < 16; pos++)
        if (m & (1 << pos)) {
          uint8_t k = ringOrder[RING_REVERSE ? 15 - pos : pos] & 15;
          an[k / 4] |= 1 << (k % 4);
        }
    }
    mcpWrite(0x14, ringPortByte(group, an[group]));   // OLATA
    group = (group + 1) & 3;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void ringInit() {
  if (!RING_ENABLE) return;
  Wire.beginTransmission(RING_ADDR);
  if (Wire.endTransmission() != 0) { Serial.printf("[RING] MCP23017 not found at 0x%02X\n", RING_ADDR); return; }
  mcpWrite(0x14, ringPortByte(0, 0));          // сначала всё выключить
  mcpWrite(0x00, 0x00);                        // IODIRA: порт A на выход
  ringOk = true;
  xTaskCreatePinnedToCore(ringTask, "ring", 3072, nullptr, 2, nullptr, 1);
  Serial.printf("[RING] MCP23017 OK at 0x%02X\n", RING_ADDR);
  if (RING_TEST) {
    Serial.println("[RING] test: запиши, в каком порядке по кругу (по часовой) загораются номера");
    for (int k = 0; k < 16; k++) {
      ringTestIdx = k;
      Serial.printf("[RING]   #%2d  (катод %d, анод %d)\n", k, k / 4, k % 4);
      delay(600);
    }
    ringTestIdx = -1;
  }
}

static uint16_t ringBar(int n) {               // n светодиодов подряд от позиции 0
  if (n <= 0) return 0;
  if (n >= 16) return 0xFFFF;
  return (uint16_t)((1u << n) - 1);
}

// Что показывает кольцо: браузер — точка позиции в списке; плеер — громкость.
// Громкость — светодиоды 1–15 (VOL 100 = все 15). Пауза или громкость 0 — только 16-й (красный).
// Стоп — одна точка на уровне громкости.
static void ringUpdate(uint32_t) {
  if (!ringOk) return;
  uint16_t m;
  if (navMode) {
    int pos = navCount > 1 ? (navIndex * 15 + (navCount - 1) / 2) / (navCount - 1) : 0;
    m = (uint16_t)(1u << pos);
  } else {
    int n = volume <= 0 ? 0 : (volume * 15 + 99) / 100;   // 1–15 для громкости, 16-й — пауза/mute
    if (playState == ST_PAUSE || n == 0) m = (uint16_t)(1u << 15);
    else if (playState == ST_STOP) m = (uint16_t)(1u << (n - 1));
    else m = ringBar(n);
  }
  ringMask = m;
}

// ============================ ОБЛОЖКИ (Core 1) ================================
static size_t httpGetToBuf(const char* url, uint8_t** out) {
  *out = nullptr;
  HTTPClient http;
  http.setConnectTimeout(3000);
  http.setTimeout(4000);
  if (!http.begin(url)) return 0;
  int code = http.GET();
  if (code != 200) { http.end(); return 0; }
  int total = http.getSize();
  size_t got = 0;
  uint8_t* b = nullptr;
  if (total > 0) {
    if (total > COVER_MAX) { http.end(); return 0; }
    b = (uint8_t*)psAlloc(total);
    if (!b) { http.end(); return 0; }
    WiFiClient* s = http.getStreamPtr();
    uint32_t t0 = millis();
    while (got < (size_t)total && millis() - t0 < 5000) {
      int av = s->available();
      if (av > 0) {
        size_t want = (size_t)total - got;
        if ((size_t)av < want) want = av;
        got += s->readBytes(b + got, want);
        t0 = millis();
      } else if (!s->connected()) break;
      else delay(2);
    }
    if (got < (size_t)total) { free(b); b = nullptr; got = 0; }
  } else {                                       // chunked — через String (бинарно-безопасно)
    String s = http.getString();
    if (s.length() > 0 && s.length() <= COVER_MAX) {
      b = (uint8_t*)psAlloc(s.length());
      if (b) { memcpy(b, s.c_str(), s.length()); got = s.length(); }
    }
  }
  http.end();
  *out = b;
  return got;
}

static void processCover() {
  if (coverGen == uiCoverGen) return;
  static CoverReq r;
  portENTER_CRITICAL(&shareMux);
  r = coverShared;
  uiCoverGen = coverGen;
  portEXIT_CRITICAL(&shareMux);

  coverOk = false;
  uint8_t* buf = nullptr;
  size_t len = 0;
  if (r.src == SRC_SD && r.len > 0 && r.len <= COVER_MAX) {
    File f = SD.open(r.path);
    if (f) {
      buf = (uint8_t*)psAlloc(r.len);
      if (buf && f.seek(r.off)) len = f.read(buf, r.len);
      f.close();
    }
  } else if (r.src == SRC_NET && r.url[0] && WiFi.status() == WL_CONNECTED) {
    len = httpGetToBuf(r.url, &buf);
  }
  if (buf && len > 0) {
    coverBig.fillScreen(C_BG);
    coverSmall.fillScreen(C_BG);
    bool ok1 = coverBig.drawJpg(buf, len, 0, 0, 128, 128, 0, 0, 0.0f, 0.0f, lgfx::datum_t::middle_center);
    bool ok2 = coverSmall.drawJpg(buf, len, 0, 0, 44, 44, 0, 0, 0.0f, 0.0f, lgfx::datum_t::middle_center);
    coverOk = ok1 && ok2;
    Serial.printf("[COVER] %u bytes, %s\n", (unsigned)len, coverOk ? "ok" : "decode failed (progressive?)");
  }
  free(buf);
}

// ============================ WiFi / DLNA (Core 1) ============================
static const char* wlStatusName(wl_status_t st) {
  switch (st) {
    case WL_IDLE_STATUS:     return "IDLE";
    case WL_NO_SSID_AVAIL:   return "NO_SSID (сеть не найдена)";
    case WL_CONNECTED:       return "CONNECTED";
    case WL_CONNECT_FAILED:  return "CONNECT_FAILED (пароль?)";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED:    return "DISCONNECTED";
    default:                 return "?";
  }
}

static bool wifiEnsure(uint32_t timeoutMs = 15000) {
  if (WiFi.status() == WL_CONNECTED) return true;
  showBusy("Connecting WiFi...");
  Serial.printf("[WIFI] connecting to \"%s\"...\n", WIFI_SSID);
  Serial.printf("[WIFI] internal heap free %u, largest block %u\n",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  Serial.flush();
  WiFi.persistent(false);
  Serial.println("[WIFI] mode(STA)..."); Serial.flush();
  bool mOk = WiFi.mode(WIFI_STA);
  Serial.printf("[WIFI] mode -> %d\n", mOk); Serial.flush();
  wl_status_t bst = WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WIFI] begin -> %s\n", wlStatusName(bst)); Serial.flush();
  WiFi.setSleep(false);                          // без power save — иначе поток рвётся
  WiFi.setAutoReconnect(true);
  uint32_t t0 = millis(), tLog = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) {
    if (millis() - tLog >= 1000) {
      tLog = millis();
      Serial.printf("[WIFI] %us: %s\n", (unsigned)((millis() - t0) / 1000), wlStatusName(WiFi.status()));
    }
    delay(50);
  }
  bool ok = WiFi.status() == WL_CONNECTED;
  if (ok) {
    Serial.printf("[WIFI] %s, RSSI %d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.printf("[WIFI] FAILED: %s\n", wlStatusName(WiFi.status()));
    WiFi.disconnect(true);
    toast("WiFi failed", 2500);
  }
  return ok;
}

// XML-сущности → текст (&lt; &gt; &amp; &quot; &apos; &#NN; &#xNN;)
static size_t xmlDecode(const char* s, size_t n, char* out, size_t cap) {
  size_t o = 0;
  for (size_t i = 0; i < n && o + 5 < cap;) {
    if (s[i] == '&') {
      if (!strncmp(s + i, "&lt;", 4))   { out[o++] = '<';  i += 4; continue; }
      if (!strncmp(s + i, "&gt;", 4))   { out[o++] = '>';  i += 4; continue; }
      if (!strncmp(s + i, "&amp;", 5))  { out[o++] = '&';  i += 5; continue; }
      if (!strncmp(s + i, "&quot;", 6)) { out[o++] = '"';  i += 6; continue; }
      if (!strncmp(s + i, "&apos;", 6)) { out[o++] = '\''; i += 6; continue; }
      if (i + 1 < n && s[i + 1] == '#') {
        size_t j = i + 2;
        bool hex = false;
        if (j < n && (s[j] == 'x' || s[j] == 'X')) { hex = true; j++; }
        uint32_t cp = 0;
        while (j < n && s[j] != ';' && j - i < 12) {
          char d = s[j];
          if (hex) cp = cp * 16 + (isdigit(d) ? d - '0' : (tolower(d) - 'a' + 10));
          else     cp = cp * 10 + (d - '0');
          j++;
        }
        if (j < n && s[j] == ';' && cp) { o += utf8Put(out + o, cp); i = j + 1; continue; }
      }
    }
    out[o++] = s[i++];
  }
  out[o] = 0;
  return o;
}

static const char* findIn(const char* p, const char* e, const char* needle) {
  const char* r = strstr(p, needle);
  return (r && r < e) ? r : nullptr;
}

static bool tagText(const char* p, const char* e, const char* tag, char* out, size_t cap) {
  out[0] = 0;
  char open[40], close[40];
  snprintf(open, sizeof(open), "<%s", tag);
  snprintf(close, sizeof(close), "</%s>", tag);
  size_t ol = strlen(open);
  const char* s = p;
  while ((s = findIn(s, e, open))) {
    char nc = s[ol];
    if (nc == '>' || nc == ' ' || nc == '\t' || nc == '\n' || nc == '\r') break;
    s++;
  }
  if (!s) return false;
  const char* gt = findIn(s, e, ">");
  if (!gt || gt[-1] == '/') return false;
  const char* ce = findIn(gt, e, close);
  if (!ce) return false;
  xmlDecode(gt + 1, ce - gt - 1, out, cap);
  utf8Trim(out);
  strTrim(out);
  return true;
}

static bool attrVal(const char* ts, const char* te, const char* name, char* out, size_t cap) {
  out[0] = 0;
  char pat[40];
  snprintf(pat, sizeof(pat), " %s=\"", name);
  const char* s = findIn(ts, te, pat);
  if (!s) return false;
  s += strlen(pat);
  const char* q = findIn(s, te, "\"");
  if (!q) return false;
  xmlDecode(s, q - s, out, cap);
  utf8Trim(out);
  return true;
}

static uint32_t parseDuration(const char* d) {
  unsigned h = 0, m = 0, s = 0;
  if (sscanf(d, "%u:%u:%u", &h, &m, &s) == 3) return h * 3600 + m * 60 + s;
  return 0;
}

static int intBetween(const char* r, const char* tag) {
  const char* p = strstr(r, tag);
  return p ? atoi(p + strlen(tag)) : 0;
}

// Один Browse-запрос (страница). Добавляет элементы в navItems / netBrowse.
static bool dlnaBrowsePage(const char* objectId, int start, int count, int& returned, int& total) {
  returned = total = 0;
  static const char SOAP[] =
    "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
    "<s:Envelope xmlns:s=\"http://schemas.xmlsoap.org/soap/envelope/\" "
    "s:encodingStyle=\"http://schemas.xmlsoap.org/soap/encoding/\"><s:Body>"
    "<u:Browse xmlns:u=\"urn:schemas-upnp-org:service:ContentDirectory:1\">"
    "<ObjectID>%s</ObjectID><BrowseFlag>BrowseDirectChildren</BrowseFlag>"
    "<Filter>*</Filter><StartingIndex>%d</StartingIndex><RequestedCount>%d</RequestedCount>"
    "<SortCriteria></SortCriteria></u:Browse></s:Body></s:Envelope>";
  char idEsc[ID_LEN * 5];
  size_t k = 0;
  for (const char* c = objectId; *c && k + 6 < sizeof(idEsc); c++) {
    if (*c == '&')      { memcpy(idEsc + k, "&amp;", 5); k += 5; }
    else if (*c == '<') { memcpy(idEsc + k, "&lt;", 4);  k += 4; }
    else idEsc[k++] = *c;
  }
  idEsc[k] = 0;
  static char body[1200];
  snprintf(body, sizeof(body), SOAP, idEsc, start, count);

  HTTPClient http;
  http.setConnectTimeout(4000);
  http.setTimeout(8000);
  if (!http.begin(DLNA_CONTROL_URL)) return false;
  http.addHeader("Content-Type", "text/xml; charset=\"utf-8\"");
  http.addHeader("SOAPAction", "\"urn:schemas-upnp-org:service:ContentDirectory:1#Browse\"");
  int code = http.POST((uint8_t*)body, strlen(body));
  if (code != 200) {
    Serial.printf("[DLNA] HTTP %d\n", code);
    http.end();
    return false;
  }
  String resp = http.getString();
  http.end();

  const char* r = resp.c_str();
  returned = intBetween(r, "<NumberReturned>");
  total    = intBetween(r, "<TotalMatches>");
  const char* rs = strstr(r, "<Result>");
  const char* re = rs ? strstr(rs, "</Result>") : nullptr;
  if (!rs || !re) return false;
  rs += 8;
  size_t n = re - rs;
  char* didl = (char*)psAlloc(n + 1);
  if (!didl) return false;
  xmlDecode(rs, n, didl, n + 1);                  // первый слой экранирования (SOAP)

  const char* p = didl;
  static char tmp[200];
  while (navCount < LIST_MAX) {
    const char* c  = strstr(p, "<container");
    const char* it = strstr(p, "<item");
    if (!c && !it) break;
    bool isC = c && (!it || c < it);
    const char* s = isC ? c : it;
    const char* e = strstr(s, isC ? "</container>" : "</item>");
    if (!e) break;
    const char* te = strchr(s, '>');
    NavItem& ni = navItems[navCount];
    memset(&ni, 0, sizeof(ni));
    tagText(s, e, "dc:title", tmp, sizeof(tmp));

    if (isC) {
      attrVal(s, te, "id", ni.id, ID_LEN);
      strlcpy(ni.name, tmp[0] ? tmp : "?", NAME_LEN);
      utf8Trim(ni.name);
      ni.type = NT_NETDIR;
      ni.ref = -1;
      navCount++;
    } else if (netBrowseCount < LIST_MAX) {
      NetTrack& t = netBrowse[netBrowseCount];
      memset(&t, 0, sizeof(t));
      bool found = false;
      const char* q = s;
      while ((q = findIn(q, e, "<res"))) {
        const char* qe = findIn(q, e, ">");
        if (!qe) break;
        char pi[128];
        attrVal(q, qe, "protocolInfo", pi, sizeof(pi));
        if (strstr(pi, "audio/mpeg")) {
          const char* ce = findIn(qe, e, "</res>");
          if (ce) { xmlDecode(qe + 1, ce - qe - 1, t.url, URL_LEN); strTrim(t.url); }
          char d[24];
          if (attrVal(q, qe, "duration", d, sizeof(d))) t.durSec = parseDuration(d);
          char br[16];
          if (attrVal(q, qe, "bitrate", br, sizeof(br))) t.kbps = (uint16_t)(atol(br) * 8 / 1000);  // bytes/s → kbps
          found = t.url[0] != 0;
          break;
        }
        q = qe;
      }
      if (found) {
        strlcpy(t.title, tmp, META_LEN);
        if (!tagText(s, e, "upnp:artist", t.artist, META_LEN)) tagText(s, e, "dc:creator", t.artist, META_LEN);
        tagText(s, e, "upnp:album", t.album, META_LEN);
        tagText(s, e, "upnp:albumArtURI", t.art, URL_LEN);
        // Synology дописывает к названию "/Альбом" или "/Исполнитель" — срезаем
        for (const char* suffix : {t.album, t.artist}) {
          size_t tl = strlen(t.title), sl = strlen(suffix);
          if (sl && tl > sl + 1 && t.title[tl - sl - 1] == '/' && !strcmp(t.title + tl - sl, suffix))
            t.title[tl - sl - 1] = 0;
        }
        strlcpy(ni.name, t.title[0] ? t.title : "Track", NAME_LEN);
        utf8Trim(ni.name);
        ni.type = NT_NETITEM;
        ni.ref = netBrowseCount++;
        navCount++;
      }
    }
    p = e + 1;
  }
  free(didl);
  return true;
}

static bool dlnaBrowse(const char* objectId) {
  if (!wifiEnsure()) return false;
  showBusy("Loading...");
  navCount = 0;
  netBrowseCount = 0;
  int start = 0;
  for (int page = 0; page < 10; page++) {
    int ret, total;
    if (!dlnaBrowsePage(objectId, start, 50, ret, total)) {
      if (page == 0) { toast("DLNA error"); return false; }
      break;
    }
    start += ret;
    if (ret <= 0 || start >= total || navCount >= LIST_MAX) break;
  }
  Serial.printf("[DLNA] %s: %d items (%d tracks)\n", objectId, navCount, netBrowseCount);
  return true;
}

// ============================ SD: СКАНИРОВАНИЕ ================================
static int cmpNav(const void* a, const void* b) {
  const NavItem* x = (const NavItem*)a;
  const NavItem* y = (const NavItem*)b;
  if (x->type != y->type) return x->type == NT_DIR ? -1 : 1;
  return strcasecmp(x->name, y->name);
}
static int cmpPath(const void* a, const void* b) {
  return strcasecmp((const char*)a, (const char*)b);
}

static void joinPath(char* out, size_t cap, const char* dir, const char* name) {
  if (!strcmp(dir, "/")) snprintf(out, cap, "/%s", name);
  else                   snprintf(out, cap, "%s/%s", dir, name);
}

static void sdScanDir(const char* path) {
  navCount = 0;
  File dir = SD.open(path);
  if (!dir || !dir.isDirectory()) { Serial.printf("[NAV] cannot open %s\n", path); return; }
  File e;
  while ((e = dir.openNextFile()) && navCount < LIST_MAX) {
    const char* nm = e.name();
    const char* base = strrchr(nm, '/');
    base = base ? base + 1 : nm;
    bool isDir = e.isDirectory();
    if (base[0] != '.' && strcmp(base, "System Volume Information") != 0 && (isDir || isAudioFile(base))) {
      NavItem& ni = navItems[navCount++];
      memset(&ni, 0, sizeof(ni));
      strlcpy(ni.name, base, NAME_LEN);
      utf8Trim(ni.name);
      ni.type = isDir ? NT_DIR : NT_FILE;
      ni.ref = -1;
    }
    e.close();
  }
  dir.close();
  qsort(navItems, navCount, sizeof(NavItem), cmpNav);
}

// Собирает отсортированный список mp3 папки в dst
static int sdBuildList(const char* path, char (*dst)[PATH_LEN]) {
  int n = 0;
  File dir = SD.open(path);
  if (!dir || !dir.isDirectory()) return 0;
  File e;
  while ((e = dir.openNextFile()) && n < LIST_MAX) {
    const char* nm = e.name();
    const char* base = strrchr(nm, '/');
    base = base ? base + 1 : nm;
    if (!e.isDirectory() && base[0] != '.' && isAudioFile(base)) joinPath(dst[n++], PATH_LEN, path, base);
    e.close();
  }
  dir.close();
  qsort(dst, n, PATH_LEN, cmpPath);
  return n;
}

// ============================ ЗАГРУЗКА ПЛЕЙЛИСТА ==============================
static bool waitPend() {
  uint32_t t0 = millis();
  while (pendBusy && millis() - t0 < 2000) delay(5);
  return !pendBusy;
}

static bool loadSdFolder(const char* dir, int start) {
  if (!waitPend()) return false;
  int n = sdBuildList(dir, sdPend);
  if (n == 0) return false;
  pendSrc = SRC_SD;
  pendCount = n;
  pendBusy = true;
  strlcpy(activeSdDir, dir, PATH_LEN);
  sendCmd(CMD_LOAD, start < n ? start : 0);
  nvsDirty = true; nvsTimer = millis();
  return true;
}

static bool loadNetBrowse(const char* containerId, const char* title, int start) {
  if (netBrowseCount == 0 || !waitPend()) return false;
  memcpy(netPend, netBrowse, (size_t)netBrowseCount * sizeof(NetTrack));
  pendSrc = SRC_NET;
  pendCount = netBrowseCount;
  pendBusy = true;
  strlcpy(activeNetCon, containerId, ID_LEN);
  strlcpy(activeNetTitle, title, NAME_LEN);
  sendCmd(CMD_LOAD, start < netBrowseCount ? start : 0);
  nvsDirty = true; nvsTimer = millis();
  return true;
}

// ============================ БРАУЗЕР: НАВИГАЦИЯ ==============================
static void navRoot() {
  navSrc = SRC_NONE;
  navStackTop = 0;
  strlcpy(navPath, "/", PATH_LEN);
  navCount = 2;
  memset(navItems, 0, 2 * sizeof(NavItem));
  strlcpy(navItems[0].name, sdReady ? "SD card" : "SD card (no card)", NAME_LEN);
  navItems[0].type = NT_ROOT_SD;
  strlcpy(navItems[1].name, DLNA_NAME " (DLNA)", NAME_LEN);
  navItems[1].type = NT_ROOT_NET;
  navIndex = 0;
}

static void selectByName(const char* name) {
  for (int i = 0; i < navCount; i++)
    if (!strcmp(navItems[i].name, name)) { navIndex = i; return; }
}

static void browserOpen() {
  navMode = true;
  navIndex = 0;
  if (ui.src == SRC_SD && sdReady) {
    navSrc = SRC_SD;
    strlcpy(navPath, activeSdDir, PATH_LEN);
    sdScanDir(navPath);
    if (ui.file[0]) selectByName(ui.file);
  } else if (ui.src == SRC_NET) {
    navSrc = SRC_NET;
    navStackTop = 0;
    strlcpy(navPath, activeNetCon, PATH_LEN);
    strlcpy(navTitle, activeNetTitle, NAME_LEN);
    if (dlnaBrowse(navPath)) {
      for (int i = 0; i < navCount; i++)
        if (navItems[i].type == NT_NETITEM && navItems[i].ref == ui.index) { navIndex = i; break; }
    } else navRoot();
  } else {
    navRoot();
  }
  scNav.reset();
  scHeader.reset();
}

static void browserBack() {
  if (navSrc == SRC_NONE) { navMode = false; return; }
  if (navSrc == SRC_SD) {
    if (!strcmp(navPath, "/")) { navRoot(); return; }
    char child[NAME_LEN];
    const char* sl = strrchr(navPath, '/');
    strlcpy(child, sl ? sl + 1 : navPath, NAME_LEN);
    if (sl == navPath) navPath[1] = 0;           // "/abc" → "/"
    else if (sl) navPath[sl - navPath] = 0;
    sdScanDir(navPath);
    navIndex = 0;
    selectByName(child);
  } else {                                        // NET — по стеку
    if (navStackTop == 0) { navRoot(); navIndex = 1; return; }
    navStackTop--;
    strlcpy(navPath, navStack[navStackTop].path, PATH_LEN);
    strlcpy(navTitle, navStack[navStackTop].title, NAME_LEN);
    int idx = navStack[navStackTop].index;
    if (!dlnaBrowse(navPath)) { navRoot(); navIndex = 1; return; }
    navIndex = idx < navCount ? idx : 0;
  }
  scNav.reset();
  scHeader.reset();
}

static void browserActivate() {
  if (navCount == 0) return;
  NavItem it = navItems[navIndex];               // копия: navItems будет перезаписан
  switch (it.type) {
    case NT_ROOT_SD:
      if (!sdReady) { toast("No SD card"); return; }
      navSrc = SRC_SD;
      strlcpy(navPath, "/", PATH_LEN);
      sdScanDir(navPath);
      navIndex = 0;
      break;
    case NT_ROOT_NET:
      navStackTop = 0;
      if (!dlnaBrowse("0")) { navRoot(); navIndex = 1; return; }
      navSrc = SRC_NET;
      strlcpy(navPath, "0", PATH_LEN);
      strlcpy(navTitle, DLNA_NAME, NAME_LEN);
      navIndex = 0;
      break;
    case NT_DIR: {
      char np[PATH_LEN];
      joinPath(np, PATH_LEN, navPath, it.name);
      strlcpy(navPath, np, PATH_LEN);
      sdScanDir(navPath);
      navIndex = 0;
      break;
    }
    case NT_NETDIR: {
      if (navStackTop < NAV_STACK_DEPTH) {
        strlcpy(navStack[navStackTop].path, navPath, PATH_LEN);
        strlcpy(navStack[navStackTop].title, navTitle, NAME_LEN);
        navStack[navStackTop].index = navIndex;
        navStackTop++;
      }
      char prevPath[PATH_LEN];
      strlcpy(prevPath, navPath, PATH_LEN);
      if (!dlnaBrowse(it.id)) {                  // не вышло — возвращаем как было
        if (navStackTop) navStackTop--;
        dlnaBrowse(prevPath);
        return;
      }
      strlcpy(navPath, it.id, PATH_LEN);
      strlcpy(navTitle, it.name, NAME_LEN);
      navIndex = 0;
      break;
    }
    case NT_FILE: {
      // Браузер отсортирован так же, как плейлист: индекс = число файлов до курсора
      int start = 0;
      for (int i = 0; i < navIndex; i++) if (navItems[i].type == NT_FILE) start++;
      if (loadSdFolder(navPath, start)) navMode = false;
      else toast("Cannot play");
      break;
    }
    case NT_NETITEM:
      if (loadNetBrowse(navPath, navTitle, it.ref)) navMode = false;
      else toast("Cannot play");
      break;
  }
  scNav.reset();
  scHeader.reset();
}

// ============================ ВВОД ===========================================
// Энкодер Ruichi: state machine по обоим фронтам A и B (как в v1)
static volatile int     encoderPos = 0;
static volatile uint8_t encState = 0;
static const int8_t ENC_TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
static void IRAM_ATTR encHandler() {
  uint8_t curr = ((uint8_t)digitalRead(IO_ENC_A) << 1) | (uint8_t)digitalRead(IO_ENC_B);
  encState = ((encState << 2) | curr) & 0x0F;
  encoderPos += ENC_TABLE[encState];
}

struct Button {
  uint8_t  pin;
  bool     st = false, lastRaw = false, longFired = false;
  uint32_t tChange = 0, tDown = 0;
  enum Ev { NONE, SHORT, LONG };
  explicit Button(uint8_t p) : pin(p) {}
  Ev update(uint32_t now) {
    bool raw = digitalRead(pin) == LOW;
    if (raw != lastRaw) { lastRaw = raw; tChange = now; }
    if (now - tChange < 25) return NONE;          // антидребезг
    if (raw != st) {
      st = raw;
      if (st) { tDown = now; longFired = false; }
      else if (!longFired) return SHORT;
    }
    if (st && !longFired && now - tDown >= LONG_PRESS_MS) { longFired = true; return LONG; }
    return NONE;
  }
};
static Button bPrev(IO_BTN_PREV), bNext(IO_BTN_NEXT), bPlay(IO_BTN_PLAY), bMode(IO_BTN_MODE);

static float volumeDb() {
  if (volume <= 0) return -120.f;
  return VOL_MAX_DB - (100 - volume) * (VOL_RANGE_DB / 99.f);
}
static void applyVolume() { vuTap.setVolumeDb(volumeDb()); }

static void toggleRepeat() {
  repeatOne = !repeatOne;
  toast(repeatOne ? "Repeat one: ON" : "Repeat one: OFF");
  nvsDirty = true; nvsTimer = millis();
}

static void nextScreen() {
  screenMode = (screenMode + 1) % SCR_COUNT;
  static const char* names[] = {"Track", "VU bars", "VU meters", "Cover"};
  toast(names[screenMode], 800);
  nvsDirty = true; nvsTimer = millis();
}

static void handleInput(uint32_t now) {
  // --- Вращение ---
  static int lastEncPos = 0;
  int pos = encoderPos;
  int delta = pos - lastEncPos;
  if (delta) {
    lastEncPos = pos;
    if (navMode) {
      if (navCount > 0) {
        navIndex += delta;
        if (navIndex < 0) navIndex = 0;
        if (navIndex >= navCount) navIndex = navCount - 1;
        scNav.reset();
      }
    } else {
      volume += delta;
      if (volume < 0) volume = 0;
      if (volume > 100) volume = 100;
      applyVolume();
      volShowUntil = now + 1200;
      nvsDirty = true; nvsTimer = now;
    }
  }

  // --- Кнопка энкодера: одиночное / двойное / долгое (логика из v1) ---
  static bool     swPrev = false, swWaitDouble = false, swWasLong = false;
  static uint32_t swPressTime = 0, swFirstUp = 0;
  bool swDown = digitalRead(IO_ENC_SW) == LOW;
  if (swDown != swPrev) {
    swPrev = swDown;
    if (swDown) {
      if (swWaitDouble) {                         // ДВОЙНОЕ
        swWaitDouble = false; swPressTime = 0; swWasLong = false;
        if (navMode) browserBack(); else toggleRepeat();
      } else {
        swPressTime = now; swWasLong = false;
      }
    } else if (!swWasLong && swPressTime && now - swPressTime >= 30) {
      swFirstUp = now; swWaitDouble = true; swPressTime = 0;
    }
  }
  if (swDown && !swWasLong && swPressTime && now - swPressTime >= LONG_PRESS_MS) {  // ДОЛГОЕ
    swWasLong = true; swWaitDouble = false; swPressTime = 0;
    if (navMode) navMode = false; else browserOpen();
  }
  if (swWaitDouble && now - swFirstUp >= 150) {   // ОДИНОЧНОЕ
    swWaitDouble = false;
    if (navMode) browserActivate(); else sendCmd(CMD_TOGGLE);
  }

  // --- Кнопки ---
  Button::Ev e;
  if ((e = bPrev.update(now)) == Button::SHORT) { if (navMode) browserBack(); else sendCmd(CMD_PREV); }
  else if (e == Button::LONG && !navMode) sendCmd(CMD_STOP);

  if ((e = bNext.update(now)) == Button::SHORT) { if (navMode) browserActivate(); else sendCmd(CMD_NEXT); }
  else if (e == Button::LONG && !navMode) toggleRepeat();

  if ((e = bPlay.update(now)) == Button::SHORT) sendCmd(CMD_TOGGLE);
  else if (e == Button::LONG) sendCmd(CMD_STOP);

  if ((e = bMode.update(now)) == Button::SHORT) { if (navMode) navMode = false; else nextScreen(); }
  else if (e == Button::LONG) { if (navMode) navMode = false; else browserOpen(); }
}

// ============================ NVS ============================================
static void saveNvs() {
  prefs.begin("player", false);
  prefs.putInt("vol", volume);
  prefs.putBool("rpt", repeatOne);
  prefs.putUChar("scr", screenMode);
  if (ui.src != SRC_NONE) {
    prefs.putUChar("src", ui.src);
    if (ui.src == SRC_SD) { prefs.putString("sdDir", activeSdDir); prefs.putInt("sdIdx", ui.index); }
    else { prefs.putString("netCon", activeNetCon); prefs.putString("netTtl", activeNetTitle); prefs.putInt("netIdx", ui.index); }
  }
  prefs.end();
  Serial.println("[NVS] saved");
}

static void syncMeta(uint32_t now) {
  if (metaGen == uiMetaGen) return;
  portENTER_CRITICAL(&shareMux);
  ui = metaShared;
  uiMetaGen = metaGen;
  portEXIT_CRITICAL(&shareMux);
  scTitle.reset(); scArtist.reset(); scAlbum.reset();
  nvsDirty = true;
  nvsTimer = now;
}

// ============================ СТАРТ: ВОССТАНОВЛЕНИЕ ============================
static void resumeLast() {
  prefs.begin("player", true);
  uint8_t src = prefs.getUChar("src", SRC_SD);
  String sdDir = prefs.getString("sdDir", "/");
  int sdIdx = prefs.getInt("sdIdx", 0);
  String netCon = prefs.getString("netCon", "");
  String netTtl = prefs.getString("netTtl", DLNA_NAME);
  int netIdx = prefs.getInt("netIdx", 0);
  prefs.end();

  if (src == SRC_NET && netCon.length()) {
    if (dlnaBrowse(netCon.c_str()) && loadNetBrowse(netCon.c_str(), netTtl.c_str(), netIdx)) {
      Serial.printf("[RESUME] NET %s #%d\n", netCon.c_str(), netIdx);
      return;
    }
  }
  if (!sdReady) { browserOpen(); return; }
  if (loadSdFolder(sdDir.c_str(), sdIdx)) {
    Serial.printf("[RESUME] SD %s #%d\n", sdDir.c_str(), sdIdx);
    return;
  }
  // Папки больше нет — ищем первую папку с mp3 в корне
  if (loadSdFolder("/", 0)) return;
  sdScanDir("/");
  for (int i = 0; i < navCount; i++) {
    if (navItems[i].type != NT_DIR) continue;
    char p[PATH_LEN];
    joinPath(p, PATH_LEN, "/", navItems[i].name);
    if (loadSdFolder(p, 0)) return;
  }
  browserOpen();
}

// ============================ SETUP ==========================================
static void memLog(const char* step) {
  Serial.printf("[MEM] %-12s internal free %6u  largest %6u  | PSRAM free %u\n", step,
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                (unsigned)ESP.getFreePsram());
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[PLAYER] ESP32-S3 Audio Player v2.0");
  memLog("boot");

  // WiFi-драйвер поднимаем первым, пока внутренняя RAM свободна (подключаемся позже)
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  memLog("wifi init");

  pinMode(IO_DAC_XSMT, OUTPUT);
  digitalWrite(IO_DAC_XSMT, LOW);               // мьют до стабилизации I2S

  // Большие буферы — в PSRAM
  sdList    = (char (*)[PATH_LEN])psAlloc((size_t)LIST_MAX * PATH_LEN);
  sdPend    = (char (*)[PATH_LEN])psAlloc((size_t)LIST_MAX * PATH_LEN);
  netList   = (NetTrack*)psAlloc((size_t)LIST_MAX * sizeof(NetTrack));
  netPend   = (NetTrack*)psAlloc((size_t)LIST_MAX * sizeof(NetTrack));
  netBrowse = (NetTrack*)psAlloc((size_t)LIST_MAX * sizeof(NetTrack));
  navItems  = (NavItem*)psAlloc((size_t)LIST_MAX * sizeof(NavItem));
  memLog("buffers");
  if (!sdList || !sdPend || !netList || !netPend || !netBrowse || !navItems)
    Serial.println("[MEM] allocation FAILED — PSRAM включена? (Инструменты → PSRAM: OPI PSRAM)");
  Serial.printf("[MEM] PSRAM free: %u\n", (unsigned)ESP.getFreePsram());

  // Настройки
  prefs.begin("player", true);
  volume     = prefs.getInt("vol", 40);
  repeatOne  = prefs.getBool("rpt", false);
  screenMode = prefs.getUChar("scr", SCR_CARD) % SCR_COUNT;
  prefs.end();

  // TFT + спрайты
  loadFonts();
  tft.init();
  tft.setRotation(1);
  tft.setBrightness(TFT_BRIGHTNESS);
  frame.setColorDepth(16);  frame.setPsram(true);  frame.createSprite(SCR_W, SCR_H);
  coverBig.setColorDepth(16); coverBig.setPsram(true); coverBig.createSprite(128, 128);
  coverSmall.setColorDepth(16); coverSmall.setPsram(true); coverSmall.createSprite(44, 44);
  buildVuFace(vuFaceL, "L");
  buildVuFace(vuFaceR, "R");
  memLog("tft");
  showBusy("Starting...");

  // OLED
  Wire.begin(IO_I2C_SDA, IO_I2C_SCL, 400000);
  oled.setBusClock(400000);
  oled.begin();
  oled.clearBuffer();
  oled.setFont(u8g2_font_7x13B_tf);
  oled.drawStr(0, 13, "Audio Player v2");
  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(0, 29, "starting...");
  oled.sendBuffer();
  memLog("oled");
  ringInit();

  // Ввод
  pinMode(IO_ENC_A, INPUT_PULLUP);
  pinMode(IO_ENC_B, INPUT_PULLUP);
  pinMode(IO_ENC_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IO_ENC_A), encHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IO_ENC_B), encHandler, CHANGE);
  pinMode(IO_BTN_PREV, INPUT_PULLUP);
  pinMode(IO_BTN_NEXT, INPUT_PULLUP);
  pinMode(IO_BTN_PLAY, INPUT_PULLUP);
  pinMode(IO_BTN_MODE, INPUT_PULLUP);

  // SD на своей шине SPI3
  sdSPI.begin(IO_SD_SCK, IO_SD_MISO, IO_SD_MOSI, IO_SD_CS);
  sdReady = SD.begin(IO_SD_CS, sdSPI, 20000000);
  Serial.printf("[SD] %s\n", sdReady ? "OK" : "FAILED");
  memLog("sd");

  // I2S → PCM5102A. ВНИМАНИЕ: AudioTools берёт count*size как dma_frame_num,
  // а ESP-IDF выделяет ещё ×6 дескрипторов во ВНУТРЕННЕЙ RAM:
  // 8*512 → 6 × 4 КБ = 24 КБ (~140 мс звука). 16*1024 съедало 96 КБ и WiFi не стартовал.
  // Провалы сети гасит буфер в PSRAM (NET_BUFFER_KB), большой I2S не нужен.
  auto cfg = i2s.defaultConfig(TX_MODE);
  cfg.pin_bck = IO_I2S_BCK;
  cfg.pin_ws = IO_I2S_LRCK;
  cfg.pin_data = IO_I2S_DOUT;
  cfg.sample_rate = 44100;
  cfg.channels = 2;
  cfg.bits_per_sample = 16;
  cfg.buffer_count = 8;
  cfg.buffer_size = 512;
  if (!i2s.begin(cfg)) Serial.println("[I2S] FAILED");
  memLog("i2s");
  vuTap.AudioStream::setAudioInfo(cfg);          // стартовый формат для fade/VU, I2S не трогаем

  if (!netStream.init((size_t)NET_BUFFER_KB * 1024)) Serial.println("[NET] buffer init FAILED");
  memLog("netbuf");

  // Плеер
  source.setCallbackNextStream(cbNextStream);
  source.setCallbackSelectStream(cbSelectStream);
  player.setAutoNext(true);
  player.setDelayIfOutputFull(2);
  player.setVolume(1.0f);                       // библиотека — без ослабления, громкость в vuTap
  applyVolume();

  cmdQ = xQueueCreate(16, sizeof(Cmd));
  xTaskCreatePinnedToCore(audioTask, "audio", 16384, nullptr, 2, nullptr, 0);
  memLog("audio task");

  resumeLast();

  delay(300);
  digitalWrite(IO_DAC_XSMT, HIGH);              // снимаем мьют
  lastLevelMs = millis();
  Serial.printf("[PLAYER] ready, internal RAM free %u (largest %u)\n",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
}

// ============================ LOOP (Core 1) ===================================
void loop() {
  uint32_t now = millis();

  handleInput(now);

  if (audioError) {
    uint8_t e = audioError;
    audioError = 0;
    toast(e == 1 ? "File error" : "Stream error");
  }

  syncMeta(now);
  processCover();
  updateLevels();

  if (nvsDirty && now - nvsTimer > 2000) { nvsDirty = false; saveNvs(); }

  // Диагностика уровня на входе ЦАП, раз в секунду во время игры
  static uint32_t lastDiag = 0;
  if (DIAG_LEVELS && playState == ST_PLAY && now - lastDiag >= 1000) {
    lastDiag = now;
    uint16_t pk; uint32_t clips, n; float gr, cr;
    vuTap.diag(pk, clips, n, gr, cr);
    Serial.printf("[LVL] VOL %d (%.1f dB)  peak %6.1f dBFS  comp %5.1f dB  limiter %5.1f dB  clipped %u\n",
                  volume, volumeDb(), toDb(pk / 32768.f), cr, gr, (unsigned)clips);
  }

  static uint32_t lastTft = 0, lastOled = 0;
  uint32_t interval = (!navMode && (screenMode == SCR_VU_BARS || screenMode == SCR_VU_NEEDLE)) ? 25 : 40;
  if (now - lastTft >= interval) { lastTft = now; renderTFT(); }
  if (now - lastOled >= 200)     { lastOled = now; renderOLED(); }
  ringUpdate(now);

  delay(1);
}
