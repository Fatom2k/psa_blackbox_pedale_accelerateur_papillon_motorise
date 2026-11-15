/*
  ============================================================================
   BLACKBOX pédale d'accélérateur + papillon motorisé — PSA (calculateur MM6LP)
   Cible : LilyGO TTGO T-Display v1.1 + 3 x ADS1115 + lecteur microSD
  ============================================================================
   Enregistre en continu, au niveau du calculateur :
     - +5 V pédale, S1, S2, masse capteur pédale
     - +5 V papillon, piste 1, piste 2, masse capteur papillon
     - + permanent et + après relais du calculateur, masse puissance calculateur
   Deux chaînes de mesure :
     - « précise » : 3 x ADS1115 16 bits, ~120 mesures/s par voie
     - « rapide »  : ADC de l'ESP32 sur les 6 voies 5 V / pistes, 2 kHz, pour
                     les pointes et microcoupures (min/max dans chaque ligne)
   Fichiers sur la carte SD (numéro N = numéro de démarrage) :
     bbNNNN_PP.csv        journal continu, 100 lignes/s, nouveau fichier / 30 min
     bbNNNN_events.csv    événements (début / fin / marqueur), avec durée
     bbNNNN_cap_KKK.csv   capture 2 kHz des voies rapides, 1 s avant / 0,5 s après
  ============================================================================
*/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <Adafruit_ADS1X15.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_timer.h>
#include <esp_partition.h>
#include <WiFi.h>
#include <WebServer.h>
#include "config.h"
#include "web_page.h"

// ============================================================================
// Événements
// ============================================================================
enum EvType : uint8_t { EV_MARK, EV_PEDAL, EV_RANGE, EV_V5, EV_V12, EV_GND, EV_JUMP,
                        EV_ADSERR, EV_PAUSE, EV_RESUME, EV_COUNT };
static const char *const EV_NAME[EV_COUNT] = {
  "MARQUEUR", "PEDALE_INCOH", "HORS_PLAGE", "ALIM_5V", "ALIM_12V", "MASSE", "GLITCH",
  "ADS_ERREUR", "PAUSE", "REPRISE" };
enum EvPhase : uint8_t { PH_POINT, PH_START, PH_END };
static const char *const PH_NAME[3] = { "point", "debut", "fin" };

struct Event {
  uint32_t t_ms;
  uint8_t  type, phase, ch;     // ch : 0..5 voie rapide, 100+i voie ADS i
  float    value;               // valeur la plus défavorable
  uint32_t dur_ms;              // durée (phase fin)
  uint32_t capId;               // numéro de capture rapide associée (0 = aucune)
};

static QueueHandle_t evQ;
static volatile uint32_t evFlags = 0;      // bits par type, remis à zéro à chaque ligne
static volatile uint32_t evTotal = 0, evLost = 0;
static volatile bool ecuArmed = false;     // calculateur alimenté depuis > 1 s

static const char *chName(uint8_t ch) {
  if (ch < FAST_N) return FAST_NAME[ch];
  if (ch >= 100 && ch < 100 + ADS_N) return ADS_DEF[ch - 100].name;
  return "-";
}

// ============================================================================
// Capture rapide (anneau 2 kHz des voies rapides)
// ============================================================================
struct FastRec { uint32_t t10us; uint16_t mv[FAST_N]; };   // 16 octets
#define RING (CAP_PRE + CAP_POST)
static FastRec *ring = nullptr;
static volatile uint32_t ringCount = 0;
static portMUX_TYPE capMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint8_t  capState = 0;     // 0 libre, 1 post-déclenchement, 2 prête à écrire
static volatile uint32_t capEnd = 0, capId = 0, capCounter = 0;

static uint32_t requestCapture() {
  uint32_t id = 0;
  portENTER_CRITICAL(&capMux);
  if (capState == 0 && ring) {
    capEnd = ringCount + CAP_POST;
    id = ++capCounter;
    capId = id;
    capState = 1;
  }
  portEXIT_CRITICAL(&capMux);
  return id;
}

static void postEvent(uint8_t type, uint8_t phase, uint8_t ch, float value, uint32_t dur, uint32_t t_ms) {
  Event e;
  e.t_ms = t_ms; e.type = type; e.phase = phase; e.ch = ch;
  e.value = value; e.dur_ms = dur;
  e.capId = (phase != PH_END && type != EV_PAUSE && type != EV_RESUME) ? requestCapture() : 0;
  __atomic_fetch_or(&evFlags, 1u << type, __ATOMIC_RELAXED);
  evTotal++;
  if (xQueueSend(evQ, &e, 0) != pdTRUE) evLost++;
}

// Condition avec début / fin et valeur la plus défavorable
struct Cond { bool active = false; uint32_t t0 = 0, lastBad = 0; float worst = 0, worstDev = 0; };

static void condEval(Cond &c, bool bad, float value, float dev, uint8_t type, uint8_t ch, uint32_t now) {
  if (bad) {
    c.lastBad = now;
    if (!c.active) {
      c.active = true; c.t0 = now; c.worst = value; c.worstDev = dev;
      postEvent(type, PH_START, ch, value, 0, now);
    } else if (dev > c.worstDev) { c.worst = value; c.worstDev = dev; }
  } else if (c.active && now - c.lastBad >= END_MS) {
    c.active = false;
    postEvent(type, PH_END, ch, c.worst, c.lastBad - c.t0, now);
  }
}

// ============================================================================
// Chaîne précise : 3 x ADS1115
// ============================================================================
static Adafruit_ADS1115 ads[3];
static bool adsOk[3] = { false, false, false };
static const uint8_t ADS_ADDR[3] = { 0x48, 0x49, 0x4A };
static const uint16_t ADS_MUX[4] = { ADS1X15_REG_CONFIG_MUX_SINGLE_0, ADS1X15_REG_CONFIG_MUX_SINGLE_1,
                                     ADS1X15_REG_CONFIG_MUX_SINGLE_2, ADS1X15_REG_CONFIG_MUX_SINGLE_3 };
static volatile float adsNode[ADS_N];      // tension sur l'entrée ADS
static volatile float adsPhys[ADS_N];      // tension réelle sur le fil
static float zeroOff[ADS_N] = { 0 };       // zéro des voies de masse (Preferences)
static volatile uint32_t adsRate = 0;

static void adsTask(void *) {
  Cond cV12[2], cGnd[3];
  const uint8_t gndCh[3] = { A_GNDP, A_GNDT, A_GNDECU };
  uint32_t cnt = 0, tRate = millis();

  for (;;) {
    bool anyOk = adsOk[0] || adsOk[1] || adsOk[2];
    if (!anyOk) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }

    for (int m = 0; m < 4; m++) {
      for (int i = 0; i < 3; i++) if (adsOk[i]) ads[i].startADCReading(ADS_MUX[m], false);
      vTaskDelay(1);
      for (int i = 0; i < 3; i++) {
        if (!adsOk[i]) continue;
        uint32_t t0 = micros();
        bool ok = true;
        while (!ads[i].conversionComplete()) {
          if (micros() - t0 > 5000) { ok = false; break; }
          delayMicroseconds(100);
        }
        if (ok) adsNode[i * 4 + m] = ads[i].computeVolts(ads[i].getLastConversionResults());
        else    postEvent(EV_ADSERR, PH_POINT, 100 + i * 4 + m, 0, 0, millis());
      }
    }

    const float vref = adsNode[A_REF];
    for (int c = 0; c < ADS_N; c++) {
      const AdsDef &d = ADS_DEF[c];
      float v;
      switch (d.kind) {
        case K_BIAS: v = d.gain * (adsNode[c] - vref) - zeroOff[c]; break;
        case K_REF:  v = adsNode[c]; break;
        default:     v = adsNode[c] * d.gain; break;
      }
      adsPhys[c] = v;
    }

    if (ecuArmed) {
      const uint32_t now = millis();
      const uint8_t v12[2] = { A_12VBAT, A_12VREL };
      for (int k = 0; k < 2; k++) {
        float v = adsPhys[v12[k]];
        condEval(cV12[k], v < V12_MIN || v > V12_MAX, v, fabsf(v - 13.5f), EV_V12, 100 + v12[k], now);
      }
      for (int k = 0; k < 3; k++) {
        float v = adsPhys[gndCh[k]];
        condEval(cGnd[k], fabsf(v) > GND_MAX, v, fabsf(v), EV_GND, 100 + gndCh[k], now);
      }
    }

    cnt++;
    if (millis() - tRate >= 1000) { adsRate = cnt; cnt = 0; tRate = millis(); }
  }
}

// ============================================================================
// Chaîne rapide : ADC1 de l'ESP32, 2 kHz, cœur 0 dédié
// ============================================================================
static uint16_t lut[4096];                 // brut -> mV sur la broche (étalonnage eFuse)
static const adc1_channel_t FAST_ADC[FAST_N] = {
  ADC1_CHANNEL_0 /*36*/, ADC1_CHANNEL_1 /*37*/, ADC1_CHANNEL_2 /*38*/,
  ADC1_CHANNEL_3 /*39*/, ADC1_CHANNEL_4 /*32*/, ADC1_CHANNEL_5 /*33*/ };

struct FastAgg { uint16_t mn[FAST_N], mx[FAST_N]; uint32_t n; };
static FastAgg agg[2];
static volatile uint8_t aggCur = 0;
static volatile bool aggSwapReq = false;
static volatile uint16_t fastNow[FAST_N];
static volatile uint16_t webMn[FAST_N], webMx[FAST_N];   // min/max depuis la dernière requête web
static volatile bool webResetReq = true;
static volatile uint32_t fastRate = 0;

static void aggReset(FastAgg &a) {
  for (int i = 0; i < FAST_N; i++) { a.mn[i] = 0xFFFF; a.mx[i] = 0; }
  a.n = 0;
}

static void fastTask(void *) {
  uint32_t acc[FAST_N] = { 0 }, nacc = 0;
  uint16_t prev[FAST_N] = { 0 };
  bool havePrev = false;
  Cond cPed, c5[2], cRange[4];
  const uint8_t sigCh[4] = { F_S1, F_S2, F_T1, F_T2 };
  uint32_t lastJump[4] = { 0 };
  uint32_t okSince = 0, lowSince = 0, cnt = 0, tRate = millis();
  int64_t tNext = esp_timer_get_time() + FAST_PERIOD_US;

  for (;;) {
    for (int i = 0; i < FAST_N; i++) acc[i] += adc1_get_raw(FAST_ADC[i]);
    nacc++;
    const int64_t nowUs = esp_timer_get_time();
    if (nowUs < tNext) continue;
    tNext += FAST_PERIOD_US;
    if (nowUs - tNext > 10000) tNext = nowUs + FAST_PERIOD_US;   // rattrapage

    uint16_t mv[FAST_N];
    for (int i = 0; i < FAST_N; i++) {
      float v = lut[acc[i] / nacc] * FAST_GAIN[i];
      mv[i] = (uint16_t)(v > 65535.0f ? 65535.0f : v);
      acc[i] = 0;
    }
    nacc = 0;

    // agrégat min/max pour la ligne de journal en cours
    if (aggSwapReq) { aggReset(agg[aggCur ^ 1]); aggCur ^= 1; aggSwapReq = false; }
    FastAgg &a = agg[aggCur];
    for (int i = 0; i < FAST_N; i++) {
      if (mv[i] < a.mn[i]) a.mn[i] = mv[i];
      if (mv[i] > a.mx[i]) a.mx[i] = mv[i];
      fastNow[i] = mv[i];
    }
    a.n++;
    if (webResetReq) {
      for (int i = 0; i < FAST_N; i++) { webMn[i] = mv[i]; webMx[i] = mv[i]; }
      webResetReq = false;
    } else {
      for (int i = 0; i < FAST_N; i++) {
        if (mv[i] < webMn[i]) webMn[i] = mv[i];
        if (mv[i] > webMx[i]) webMx[i] = mv[i];
      }
    }

    // anneau de capture
    if (capState != 2 && ring) {
      FastRec &r = ring[ringCount % RING];
      r.t10us = (uint32_t)(nowUs / 10);
      memcpy(r.mv, mv, sizeof mv);
      ringCount++;
      if (capState == 1 && ringCount >= capEnd) capState = 2;
    }

    // ---- surveillance ----
    const uint32_t now = (uint32_t)(nowUs / 1000);
    const float v5p = mv[F_5VP] * 1e-3f, v5t = mv[F_5VT] * 1e-3f;

    // armement : calculateur alimenté (les deux 5 V présents depuis 1 s),
    // désarmement après 2 s sans aucun 5 V (coupure contact)
    if (v5p > 4.5f && v5t > 4.5f) { if (!ecuArmed && now - okSince > 1000) ecuArmed = true; }
    else okSince = now;
    if (v5p > 1.0f || v5t > 1.0f) lowSince = now;
    else if (ecuArmed && now - lowSince > 2000) ecuArmed = false;

    if (ecuArmed) {
      condEval(c5[0], v5p < V5_MIN || v5p > V5_MAX, v5p, fabsf(v5p - 5.0f), EV_V5, F_5VP, now);
      condEval(c5[1], v5t < V5_MIN || v5t > V5_MAX, v5t, fabsf(v5t - 5.0f), EV_V5, F_5VT, now);

      bool bad[4];
      for (int k = 0; k < 4; k++) {
        const float v = mv[sigCh[k]] * 1e-3f;
        bad[k] = v < SIG_MIN || v > SIG_MAX;
        condEval(cRange[k], bad[k], v, bad[k] ? (v < SIG_MIN ? SIG_MIN - v : v - SIG_MAX) : 0,
                 EV_RANGE, sigCh[k], now);
      }

      // cohérence pédale, vue depuis la masse capteur du calculateur
      const float gp = adsPhys[A_GNDP];
      const float s1 = mv[F_S1] * 1e-3f - gp, s2 = mv[F_S2] * 1e-3f - gp;
      const float ref = (v5p - gp > 3.0f) ? v5p - gp : 5.0f;
      const float e = (s1 - PEDAL_K * s2) / ref * 100.0f;
      condEval(cPed, !bad[0] && !bad[1] && fabsf(e) > PEDAL_ERR_PCT, e, fabsf(e), EV_PEDAL, F_S1, now);

      // sauts brutaux (microcoupure, faux contact)
      if (havePrev) {
        for (int k = 0; k < 4; k++) {
          const int c = sigCh[k];
          const float d = ((int)mv[c] - (int)prev[c]) * 1e-3f;
          if (fabsf(d) > JUMP_V && now - lastJump[k] > 100) {
            lastJump[k] = now;
            postEvent(EV_JUMP, PH_POINT, c, d, 0, now);
          }
        }
      }
    }
    memcpy(prev, mv, sizeof mv);
    havePrev = true;

    cnt++;
    if (now - tRate >= 1000) { fastRate = cnt; cnt = 0; tRate = now; }
  }
}

// ============================================================================
// Journal : une ligne toutes les 10 ms
// ============================================================================
struct Row {
  uint32_t t_ms, flags;
  float ads[ADS_N];
  uint16_t mn[FAST_N], mx[FAST_N];
};
static QueueHandle_t rowQ;
static volatile bool logging = true;
static volatile uint32_t rowsDropped = 0;

static void loggerTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / LOG_HZ));
    Row r;
    r.t_ms = millis();
    r.flags = __atomic_exchange_n(&evFlags, 0u, __ATOMIC_RELAXED);
    for (int c = 0; c < ADS_N; c++) r.ads[c] = adsPhys[c];

    aggSwapReq = true;
    for (int w = 0; w < 3 && aggSwapReq; w++) vTaskDelay(1);
    if (!aggSwapReq) {
      const FastAgg &a = agg[aggCur ^ 1];
      for (int i = 0; i < FAST_N; i++) {
        r.mn[i] = a.n ? a.mn[i] : 0;
        r.mx[i] = a.n ? a.mx[i] : 0;
      }
    } else {
      aggSwapReq = false;                       // voie rapide figée : on le signale
      for (int i = 0; i < FAST_N; i++) { r.mn[i] = 0xFFFF; r.mx[i] = 0xFFFF; }
    }
    if (xQueueSend(rowQ, &r, 0) != pdTRUE) rowsDropped++;
  }
}

// ============================================================================
// Écriture SD (seule tâche qui touche la carte)
// ============================================================================
static SPIClass sdSPI(HSPI);
static bool sdOk = false;
static SemaphoreHandle_t sdMtx;            // la carte est partagée entre l'écriture et le serveur web
#define SD_LOCK()   xSemaphoreTake(sdMtx, portMAX_DELAY)
#define SD_UNLOCK() xSemaphoreGive(sdMtx)
static uint32_t bootNo = 0;
static File fLog, fEv;
static char logName[32] = "-";
static volatile uint32_t bytesWritten = 0, capsWritten = 0;
static volatile int8_t pauseReq = 0;

// derniers événements pour l'écran
#define NLAST 20                            // 7 affichés à l'écran, 20 sur la page web
#define NLAST_LCD 7
static Event lastEv[NLAST];
static int lastEvN = 0;
static SemaphoreHandle_t lastMtx;

static void openLog(int part) {
  if (!sdOk) return;
  snprintf(logName, sizeof logName, "/bb%04lu_%02d.csv", (unsigned long)bootNo, part);
  fLog = SD.open(logName, FILE_WRITE);
  if (!fLog) { sdOk = false; return; }
  fLog.printf("# Blackbox PSA pedale/papillon - demarrage %lu - partie %d - %d lignes/s\n",
              (unsigned long)bootNo, part, LOG_HZ);
  fLog.print("# tensions en volts au niveau du calculateur, par rapport a la masse du boitier (borne - batterie)\n");
  fLog.print("# _min/_max : extremes de la voie rapide (2 kHz) pendant la ligne ; flags : bits des evenements\n");
  fLog.print("t_ms");
  for (int c = 0; c < ADS_N; c++) { fLog.print(','); fLog.print(ADS_DEF[c].name); }
  for (int i = 0; i < FAST_N; i++) fLog.printf(",%s_min,%s_max", FAST_NAME[i], FAST_NAME[i]);
  fLog.print(",flags\n");
}

static size_t formatRow(char *p, size_t room, const Row &r) {
  int n = snprintf(p, room, "%lu", (unsigned long)r.t_ms);
  for (int c = 0; c < ADS_N; c++) n += snprintf(p + n, room - n, ",%.4f", r.ads[c]);
  for (int i = 0; i < FAST_N; i++) {
    if (r.mn[i] == 0xFFFF && r.mx[i] == 0xFFFF) n += snprintf(p + n, room - n, ",,");
    else n += snprintf(p + n, room - n, ",%.3f,%.3f", r.mn[i] * 1e-3f, r.mx[i] * 1e-3f);
  }
  n += snprintf(p + n, room - n, ",%lu\n", (unsigned long)r.flags);
  return n;
}

static void writeEvent(const Event &e) {
  if (xSemaphoreTake(lastMtx, pdMS_TO_TICKS(10)) == pdTRUE) {
    memmove(&lastEv[1], &lastEv[0], sizeof(Event) * (NLAST - 1));
    lastEv[0] = e;
    if (lastEvN < NLAST) lastEvN++;
    xSemaphoreGive(lastMtx);
  }
  Serial.printf("EV %lu %s %s %s %.3f %lums cap%lu\n", (unsigned long)e.t_ms, EV_NAME[e.type],
                PH_NAME[e.phase], chName(e.ch), e.value, (unsigned long)e.dur_ms, (unsigned long)e.capId);
  if (!sdOk || !fEv) return;
  SD_LOCK();
  fEv.printf("%lu,%s,%s,%s,%.4f,%lu,%lu,%s\n", (unsigned long)e.t_ms, EV_NAME[e.type], PH_NAME[e.phase],
             chName(e.ch), e.value, (unsigned long)e.dur_ms, (unsigned long)e.capId, logName);
  SD_UNLOCK();
}

static void dumpCapture() {
  const uint32_t end = capEnd, id = capId;
  const uint32_t start = (end > RING) ? end - RING : 0;
  if (sdOk) {
    SD_LOCK();
    char name[40];
    snprintf(name, sizeof name, "/bb%04lu_cap_%03lu.csv", (unsigned long)bootNo, (unsigned long)id);
    File f = SD.open(name, FILE_WRITE);
    if (f) {
      const FastRec &trig = ring[(end - CAP_POST) % RING];
      f.printf("# capture %lu - declenchement a t_ms=%.2f - voies rapides 2 kHz (V)\n",
               (unsigned long)id, trig.t10us / 100.0f);
      f.print("t_ms");
      for (int i = 0; i < FAST_N; i++) { f.print(','); f.print(FAST_NAME[i]); }
      f.print('\n');
      static char buf[2048];
      size_t len = 0;
      for (uint32_t k = start; k < end; k++) {
        const FastRec &r = ring[k % RING];
        int n = snprintf(buf + len, sizeof buf - len, "%.2f", r.t10us / 100.0f);
        for (int i = 0; i < FAST_N; i++) n += snprintf(buf + len + n, sizeof buf - len - n, ",%.3f", r.mv[i] * 1e-3f);
        n += snprintf(buf + len + n, sizeof buf - len - n, "\n");
        len += n;
        if (len > sizeof buf - 120) { f.write((uint8_t *)buf, len); bytesWritten += len; len = 0; }
      }
      if (len) { f.write((uint8_t *)buf, len); bytesWritten += len; }
      f.close();
      capsWritten++;
    }
    SD_UNLOCK();
  }
  portENTER_CRITICAL(&capMux);
  capState = 0;
  portEXIT_CRITICAL(&capMux);
}


// ============================================================================
// Enregistrement en boucle dans la flash interne (partition « rolllog »)
//   secteur 0 : en-tête (état, déclenchement) ; secteurs 1..N : anneau de RollRec
// ============================================================================
#define FLAG_FLASH (1u << 15)                    // bit des lignes pendant une écriture flash

struct __attribute__((packed)) RollRec {
  uint32_t seq;                                  // numéro d'ordre dans le cycle (0xFFFFFFFF = vide)
  uint32_t t_ms;
  uint16_t run;                                  // numéro de cycle (change à chaque réarmement)
  uint16_t flags;
  int16_t  ads[ADS_N];                           // mV
  uint16_t mn[FAST_N], mx[FAST_N];               // mV, 0xFFFF = indisponible
  uint8_t  pad[4];
};
static_assert(sizeof(RollRec) == 64, "RollRec doit faire 64 octets");
#define ROLL_PER_SECT (4096 / sizeof(RollRec))

enum RollState : uint8_t { RS_OFF, RS_RUN, RS_POST, RS_FROZEN };
static const char *const RS_NAME[4] = { "ABSENT", "ENREG", "APRES-DECL", "FIGE" };

struct RollHdr {
  uint32_t magic;                                // 'ROL1'
  uint16_t recSize, run;
  uint32_t boot;
  uint8_t  state, trigType, trigCh, rollHz;
  float    trigValue;
  uint32_t trigSeq, trigMs, lastSeq;
};
#define ROLL_MAGIC 0x314C4F52u

static const esp_partition_t *rollPart = nullptr;
static uint32_t rollSectors = 0, rollCap = 0;
static volatile uint8_t rollState = RS_OFF;
static RollHdr rollHdr;
static RollRec rollBuf[ROLL_PER_SECT];
static uint32_t rollBufN = 0;
static volatile uint32_t rollSeq = 0;           // prochain numéro à écrire
static uint32_t rollPostEnd = 0;
static SemaphoreHandle_t flashMtx;
static volatile bool rollRearmReq = false;

static void rollWriteHdr() {                    // appelé sous flashMtx
  __atomic_fetch_or(&evFlags, FLAG_FLASH, __ATOMIC_RELAXED);
  esp_partition_erase_range(rollPart, 0, 4096);
  esp_partition_write(rollPart, 0, &rollHdr, sizeof rollHdr);
}

static void rollFlushSector() {                 // appelé sous flashMtx
  if (!rollBufN) return;
  const uint32_t sect = (rollBuf[0].seq / ROLL_PER_SECT) % rollSectors;
  const size_t off = (1 + sect) * 4096;
  __atomic_fetch_or(&evFlags, FLAG_FLASH, __ATOMIC_RELAXED);
  esp_partition_erase_range(rollPart, off, 4096);
  esp_partition_write(rollPart, off, rollBuf, rollBufN * sizeof(RollRec));
  rollBufN = 0;
}

static void rollStart() {
  const uint16_t run = (rollHdr.magic == ROLL_MAGIC) ? rollHdr.run + 1 : 1;
  memset(&rollHdr, 0, sizeof rollHdr);
  rollHdr.magic = ROLL_MAGIC; rollHdr.recSize = sizeof(RollRec); rollHdr.run = run;
  rollHdr.boot = bootNo; rollHdr.state = RS_RUN; rollHdr.rollHz = ROLL_HZ;
  rollSeq = 0; rollBufN = 0;
  xSemaphoreTake(flashMtx, portMAX_DELAY);
  rollWriteHdr();
  xSemaphoreGive(flashMtx);
  rollState = RS_RUN;
}

// Dernier enregistrement valide du cycle en cours (après une coupure en phase « après déclenchement »)
static uint32_t rollScanLast() {
  RollRec r;
  int64_t best = -1;
  uint32_t bestSect = 0;
  for (uint32_t s = 0; s < rollSectors; s++) {
    esp_partition_read(rollPart, (1 + s) * 4096, &r, sizeof r);
    if (r.seq != 0xFFFFFFFFu && r.run == rollHdr.run && (int64_t)r.seq > best) { best = r.seq; bestSect = s; }
  }
  if (best < 0) return 0;
  uint32_t last = (uint32_t)best;
  for (uint32_t k = 1; k < ROLL_PER_SECT; k++) {
    esp_partition_read(rollPart, (1 + bestSect) * 4096 + k * sizeof(RollRec), &r, sizeof r);
    if (r.seq == 0xFFFFFFFFu || r.run != rollHdr.run) break;
    last = r.seq;
  }
  return last;
}

static void rollInit() {
  if (!ROLL_ENABLE) return;
  rollPart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x40, "rolllog");
  if (!rollPart || rollPart->size < 3 * 4096) { rollPart = nullptr; return; }
  rollSectors = rollPart->size / 4096 - 1;
  rollCap = rollSectors * ROLL_PER_SECT;
  esp_partition_read(rollPart, 0, &rollHdr, sizeof rollHdr);
  const bool valid = rollHdr.magic == ROLL_MAGIC && rollHdr.recSize == sizeof(RollRec);
  if (valid && (rollHdr.state == RS_FROZEN || rollHdr.state == RS_POST)) {
    if (rollHdr.state == RS_POST) {             // coupure avant la fin : on fige ce qui a été écrit
      rollHdr.lastSeq = rollScanLast();
      rollHdr.state = RS_FROZEN;
      rollWriteHdr();
    }
    rollSeq = rollHdr.lastSeq + 1;
    rollState = RS_FROZEN;
  } else {
    if (!valid) rollHdr.magic = 0;
    rollStart();
  }
}

static int16_t toMv16(float v) {
  float m = v * 1000.0f;
  if (m > 32767.0f) m = 32767.0f;
  if (m < -32768.0f) m = -32768.0f;
  return (int16_t)lrintf(m);
}

// Reçoit chaque ligne de 10 ms ; regroupe LOG_HZ/ROLL_HZ lignes (min/max fusionnés)
static void rollFeed(const Row &r) {
  static Row acc;
  static int accN = 0;
  if (!rollPart || (rollState != RS_RUN && rollState != RS_POST)) { accN = 0; return; }
  const bool rValid = !(r.mn[0] == 0xFFFF && r.mx[0] == 0xFFFF);
  if (accN == 0) acc = r;
  else {
    const bool aValid = !(acc.mn[0] == 0xFFFF && acc.mx[0] == 0xFFFF);
    if (rValid && !aValid) { memcpy(acc.mn, r.mn, sizeof acc.mn); memcpy(acc.mx, r.mx, sizeof acc.mx); }
    else if (rValid) {
      for (int i = 0; i < FAST_N; i++) {
        if (r.mn[i] < acc.mn[i]) acc.mn[i] = r.mn[i];
        if (r.mx[i] > acc.mx[i]) acc.mx[i] = r.mx[i];
      }
    }
    acc.flags |= r.flags;
    acc.t_ms = r.t_ms;
    memcpy(acc.ads, r.ads, sizeof acc.ads);
  }
  if (++accN < LOG_HZ / ROLL_HZ) return;
  accN = 0;

  RollRec &w = rollBuf[rollBufN++];
  w.seq = rollSeq++;
  w.t_ms = acc.t_ms;
  w.run = rollHdr.run;
  w.flags = (uint16_t)((acc.flags & 0x7FFF) | ((acc.flags & FLAG_FLASH) ? FLAG_FLASH : 0));
  for (int c = 0; c < ADS_N; c++) w.ads[c] = toMv16(acc.ads[c]);
  memcpy(w.mn, acc.mn, sizeof w.mn);
  memcpy(w.mx, acc.mx, sizeof w.mx);
  memset(w.pad, 0, sizeof w.pad);
  if (rollBufN == ROLL_PER_SECT) {
    xSemaphoreTake(flashMtx, portMAX_DELAY);
    rollFlushSector();
    xSemaphoreGive(flashMtx);
  }
}

// Machine d'état : déclenchement, fin de la période « après », réarmement
static void rollOnEvent(const Event &e) {
  if (rollState != RS_RUN || e.phase == PH_END || !(ROLL_TRIG_MASK & (1u << e.type))) return;
  rollHdr.state = RS_POST;
  rollHdr.trigType = e.type; rollHdr.trigCh = e.ch; rollHdr.trigValue = e.value;
  rollHdr.trigSeq = rollSeq; rollHdr.trigMs = e.t_ms;
  rollPostEnd = millis() + ROLL_POST_S * 1000UL;
  xSemaphoreTake(flashMtx, portMAX_DELAY);
  rollWriteHdr();
  xSemaphoreGive(flashMtx);
  rollState = RS_POST;
}

static void rollPoll() {
  if (!rollPart) return;
  if (rollState == RS_POST && (int32_t)(millis() - rollPostEnd) >= 0) {
    xSemaphoreTake(flashMtx, portMAX_DELAY);
    rollFlushSector();
    rollHdr.state = RS_FROZEN;
    rollHdr.lastSeq = rollSeq ? rollSeq - 1 : 0;
    rollWriteHdr();
    xSemaphoreGive(flashMtx);
    rollState = RS_FROZEN;
  }
  if (rollRearmReq) {
    rollRearmReq = false;
    if (rollState == RS_FROZEN) rollStart();
  }
}

static uint32_t rollSecondsStored() {
  if (!rollPart) return 0;
  uint32_t n = (rollState == RS_FROZEN) ? rollHdr.lastSeq + 1 : rollSeq;
  if (n > rollCap) n = rollCap;
  return n / ROLL_HZ;
}

static void writerTask(void *) {
  static char buf[8192];
  size_t len = 0;
  int part = 0;
  uint32_t lastFlush = millis(), partStart = millis();

  if (sdOk) {
    SD_LOCK();
    char name[32];
    snprintf(name, sizeof name, "/bb%04lu_events.csv", (unsigned long)bootNo);
    fEv = SD.open(name, FILE_WRITE);
    if (fEv) fEv.print("t_ms,type,phase,voie,valeur,duree_ms,capture,fichier_journal\n");
    openLog(part);
    SD_UNLOCK();
  }

  auto flushBuf = [&]() {
    if (len && sdOk && fLog) {
      SD_LOCK();
      fLog.write((uint8_t *)buf, len);
      SD_UNLOCK();
      bytesWritten += len;
    }
    len = 0;
  };

  for (;;) {
    Row r;
    if (xQueueReceive(rowQ, &r, pdMS_TO_TICKS(50)) == pdTRUE) {
      do {
        rollFeed(r);
        if (logging && sdOk) {
          len += formatRow(buf + len, sizeof buf - len, r);
          if (len > sizeof buf - 512) flushBuf();
        }
      } while (xQueueReceive(rowQ, &r, 0) == pdTRUE);
    }

    Event e;
    while (xQueueReceive(evQ, &e, 0) == pdTRUE) { writeEvent(e); rollOnEvent(e); }
    rollPoll();

    if (capState == 2) dumpCapture();

    const uint32_t now = millis();
    if (now - lastFlush >= FLUSH_MS) {
      flushBuf();
      SD_LOCK();
      if (sdOk && fLog) fLog.flush();
      if (sdOk && fEv) fEv.flush();
      SD_UNLOCK();
      lastFlush = now;
    }
    if (logging && now - partStart >= ROTATE_MIN * 60000UL) {
      flushBuf();
      SD_LOCK();
      if (fLog) fLog.close();
      openLog(++part);
      SD_UNLOCK();
      partStart = now;
    }
    if (pauseReq) {
      pauseReq = 0;
      if (logging) {
        logging = false;
        flushBuf();
        SD_LOCK();
        if (fLog) fLog.close();
        if (fEv) fEv.flush();
        SD_UNLOCK();
        postEvent(EV_PAUSE, PH_POINT, 255, 0, 0, now);
      } else {
        SD_LOCK();
        openLog(++part);
        SD_UNLOCK();
        partStart = now;
        logging = true;
        postEvent(EV_RESUME, PH_POINT, 255, 0, 0, now);
      }
    }
  }
}

// ============================================================================
// Affichage
// ============================================================================
static TFT_eSPI tft;
static TFT_eSprite spr(&tft);
static int screen = 0;
static const int NSCREENS = 5;

#define C_BG   TFT_BLACK
#define C_TXT  TFT_WHITE
#define C_DIM  0x7BEF
#define C_OK   TFT_GREEN
#define C_WARN TFT_ORANGE
#define C_BAD  TFT_RED

static void hms(uint32_t ms, char *out, size_t n, bool tenths = false) {
  uint32_t s = ms / 1000;
  if (tenths) snprintf(out, n, "%02lu:%02lu:%02lu.%lu", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60),
                       (unsigned long)(s % 60), (unsigned long)(ms / 100 % 10));
  else snprintf(out, n, "%02lu:%02lu:%02lu", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60),
                (unsigned long)(s % 60));
}

static void header(const char *title) {
  spr.fillRect(0, 0, 240, 18, 0x18E3);
  spr.setTextFont(2);
  spr.setTextDatum(ML_DATUM);
  spr.setTextColor(C_TXT);
  spr.drawString(title, 4, 9);
  const char *st; uint16_t col;
  if (!sdOk)         { st = "PAS DE SD"; col = C_BAD; }
  else if (!logging) { st = "PAUSE";     col = C_WARN; }
  else               { st = "REC";       col = C_BAD; }
  int w = spr.textWidth(st) + 10;
  spr.fillRoundRect(238 - w, 1, w, 16, 4, col);
  spr.setTextColor(TFT_BLACK);
  spr.setTextDatum(MC_DATUM);
  spr.drawString(st, 238 - w / 2, 9);
  spr.setTextDatum(TL_DATUM);
}

static void valLine(int y, const char *label, float v, const char *unit, uint16_t col) {
  char b[40];
  spr.setTextFont(2);
  spr.setTextColor(C_DIM);
  spr.drawString(label, 4, y);
  spr.setTextColor(col);
  snprintf(b, sizeof b, "%7.3f %s", v, unit);
  spr.drawString(b, 96, y);
}

static uint16_t c5v(float v)  { return (v < V5_MIN || v > V5_MAX) ? C_BAD : C_OK; }
static uint16_t cSig(float v) { return (v < SIG_MIN || v > SIG_MAX) ? C_BAD : C_TXT; }
static uint16_t cGnd(float v) { return fabsf(v) > GND_MAX ? C_BAD : (fabsf(v) > GND_MAX / 2 ? C_WARN : C_OK); }
static uint16_t c12(float v)  { return (v < V12_MIN || v > V12_MAX) ? C_BAD : C_OK; }

static void eventText(const Event &e, char *b, size_t n) {
  char t[16];
  hms(e.t_ms, t, sizeof t, true);
  snprintf(b, n, "%s %s%s %s %.2f", t, EV_NAME[e.type],
           e.phase == PH_START ? "+" : (e.phase == PH_END ? "-" : ""), chName(e.ch), e.value);
}

static void drawScreen() {
  char b[64];
  spr.fillSprite(C_BG);
  switch (screen) {
    case 0: {
      header("BLACKBOX");
      spr.setTextFont(2);
      spr.setTextColor(C_TXT);
      hms(millis(), b, sizeof b);
      spr.drawString(b, 4, 22);
      spr.setTextColor(C_DIM);
      spr.drawString(logName, 96, 22);
      spr.setTextColor(ecuArmed ? C_OK : C_WARN);
      spr.drawString(ecuArmed ? "Calculateur: ALIMENTE" : "Calculateur: en attente", 4, 40);
      spr.setTextColor(C_TXT);
      snprintf(b, sizeof b, "Evenements %lu   captures %lu", (unsigned long)evTotal, (unsigned long)capsWritten);
      spr.drawString(b, 4, 58);
      spr.setTextColor((rowsDropped || evLost) ? C_BAD : C_DIM);
      snprintf(b, sizeof b, "%.1f Mo  perdus %lu/%lu  ADS %d%d%d", bytesWritten / 1048576.0f,
               (unsigned long)rowsDropped, (unsigned long)evLost, adsOk[0], adsOk[1], adsOk[2]);
      spr.drawString(b, 4, 76);
      {
        const uint8_t rs = rollState;
        spr.setTextColor(rs == RS_FROZEN ? TFT_CYAN : (rs == RS_POST ? C_WARN : (rs == RS_RUN ? C_OK : C_DIM)));
        if (rs == RS_FROZEN)
          snprintf(b, sizeof b, "FLASH FIGEE: %s %s", EV_NAME[rollHdr.trigType], chName(rollHdr.trigCh));
        else
          snprintf(b, sizeof b, "Flash %s  %lu s / %lu s", RS_NAME[rs], (unsigned long)rollSecondsStored(),
                   (unsigned long)(rollCap / ROLL_HZ));
        spr.drawString(b, 4, 94);
      }
      if (xSemaphoreTake(lastMtx, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (lastEvN) {
          eventText(lastEv[0], b, sizeof b);
          spr.setTextColor(lastEv[0].type == EV_MARK ? TFT_CYAN : C_WARN);
          spr.drawString(b, 4, 112);
        }
        xSemaphoreGive(lastMtx);
      }
      break;
    }
    case 1: {
      header("PEDALE (48V NR)");
      const float v5 = adsPhys[A_5VP], s1 = adsPhys[A_S1], s2 = adsPhys[A_S2], g = adsPhys[A_GNDP];
      valLine(20, "+5V  G1", v5, "V", c5v(v5));
      valLine(37, "S1   F2", s1, "V", cSig(s1));
      valLine(54, "S2   G2", s2, "V", cSig(s2));
      valLine(71, "masse F1", g, "V", cGnd(g));
      const float ref = (v5 - g > 3.0f) ? v5 - g : 5.0f;
      const float e = ((s1 - g) - PEDAL_K * (s2 - g)) / ref * 100.0f;
      valLine(88, "ecart S1-2S2", e, "%", fabsf(e) > PEDAL_ERR_PCT ? C_BAD : C_OK);
      spr.setTextColor(C_DIM);
      snprintf(b, sizeof b, "rapide: S1 %.2f  S2 %.2f  5V %.2f", fastNow[F_S1] * 1e-3f, fastNow[F_S2] * 1e-3f,
               fastNow[F_5VP] * 1e-3f);
      spr.drawString(b, 4, 112);
      break;
    }
    case 2: {
      header("PAPILLON (32V GR)");
      const float v5 = adsPhys[A_5VT], t1 = adsPhys[A_T1], t2 = adsPhys[A_T2], g = adsPhys[A_GNDT];
      valLine(20, "+5V  A1", v5, "V", c5v(v5));
      valLine(37, "piste1 B1", t1, "V", cSig(t1));
      valLine(54, "piste2 B4", t2, "V", cSig(t2));
      valLine(71, "masse A4", g, "V", cGnd(g));
      valLine(88, "T1 + T2", t1 + t2 - 2 * g, "V", C_TXT);
      spr.setTextColor(C_DIM);
      snprintf(b, sizeof b, "rapide: T1 %.2f  T2 %.2f  5V %.2f", fastNow[F_T1] * 1e-3f, fastNow[F_T2] * 1e-3f,
               fastNow[F_5VT] * 1e-3f);
      spr.drawString(b, 4, 112);
      break;
    }
    case 3: {
      header("ALIM CALCULATEUR");
      const float vb = adsPhys[A_12VBAT], vr = adsPhys[A_12VREL], g = adsPhys[A_GNDECU];
      valLine(20, "+BAT  M3", vb, "V", c12(vb));
      valLine(37, "+relais", vr, "V", c12(vr));
      valLine(54, "masse calc", g, "V", cGnd(g));
      valLine(71, "12V vu calc", vr - g, "V", c12(vr - g));
      spr.setTextColor(C_DIM);
      snprintf(b, sizeof b, "ref %.3fV  ADS %luHz  rapide %luHz", adsPhys[A_REF], (unsigned long)adsRate,
               (unsigned long)fastRate);
      spr.drawString(b, 4, 94);
      spr.drawString("Zero masses: GPIO0 maintenu au demarrage", 4, 112);
      break;
    }
    case 4: {
      header("EVENEMENTS");
      spr.setTextFont(2);
      if (xSemaphoreTake(lastMtx, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (!lastEvN) { spr.setTextColor(C_DIM); spr.drawString("aucun", 4, 24); }
        for (int i = 0; i < lastEvN && i < NLAST_LCD; i++) {
          eventText(lastEv[i], b, sizeof b);
          spr.setTextColor(lastEv[i].type == EV_MARK ? TFT_CYAN : (lastEv[i].phase == PH_END ? C_DIM : C_WARN));
          spr.drawString(b, 2, 20 + i * 16);
        }
        xSemaphoreGive(lastMtx);
      }
      break;
    }
  }
  spr.pushSprite(0, 0);
}

// ============================================================================
// Zéro des voies de masse (contact coupé, rien ne consomme)
// ============================================================================
static Preferences prefs;

static void zeroGrounds() {
  tft.fillScreen(C_BG);
  tft.setTextColor(C_WARN, C_BG);
  tft.drawString("ZERO DES MASSES", 10, 20, 4);
  tft.setTextColor(C_TXT, C_BG);
  tft.drawString("Contact coupe ? garder GPIO0 3 s", 10, 60, 2);
  uint32_t t0 = millis();
  while (millis() - t0 < 3000) {
    if (digitalRead(BTN_NEXT) != LOW) return;
    delay(20);
  }
  tft.drawString("Mesure...                         ", 10, 60, 2);
  double sum[ADS_N] = { 0 };
  int n = 0;
  t0 = millis();
  while (millis() - t0 < 2000) {
    for (int c = 0; c < ADS_N; c++) sum[c] += adsPhys[c];
    n++;
    delay(10);
  }
  for (int c = 0; c < ADS_N; c++) {
    if (ADS_DEF[c].kind != K_BIAS) continue;
    zeroOff[c] += (float)(sum[c] / n);
    char key[8];
    snprintf(key, sizeof key, "z%d", c);
    prefs.putFloat(key, zeroOff[c]);
  }
  tft.drawString("Enregistre.                       ", 10, 60, 2);
  delay(1000);
}


// ============================================================================
// Serveur web (point d'accès Wi-Fi, http://192.168.4.1)
// ============================================================================
static WebServer server(80);

static void handleLive() {
  static char b[2048];
  int n = snprintf(b, sizeof b,
    "{\"t\":%lu,\"armed\":%d,\"rec\":%d,\"sd\":%d,\"file\":\"%s\",\"ev\":%lu,\"caps\":%lu,\"mb\":%.2f,"
    "\"drop\":%lu,\"evlost\":%lu,\"adsHz\":%lu,\"fastHz\":%lu,"
    "\"roll\":{\"st\":\"%s\",\"sec\":%lu,\"cap\":%lu,\"run\":%u,\"trig\":\"%s %s %.3f\",\"trigT\":%lu,\"boot\":%lu},\"a\":{",
    (unsigned long)millis(), ecuArmed ? 1 : 0, logging ? 1 : 0, sdOk ? 1 : 0, logName,
    (unsigned long)evTotal, (unsigned long)capsWritten, bytesWritten / 1048576.0f,
    (unsigned long)rowsDropped, (unsigned long)evLost, (unsigned long)adsRate, (unsigned long)fastRate,
    RS_NAME[rollState], (unsigned long)rollSecondsStored(), (unsigned long)(rollCap / ROLL_HZ), rollHdr.run,
    rollState >= RS_POST ? EV_NAME[rollHdr.trigType] : "-", rollState >= RS_POST ? chName(rollHdr.trigCh) : "",
    rollState >= RS_POST ? rollHdr.trigValue : 0.0f, (unsigned long)rollHdr.trigMs, (unsigned long)rollHdr.boot);
  for (int c = 0; c < ADS_N; c++)
    n += snprintf(b + n, sizeof b - n, "%s\"%s\":%.4f", c ? "," : "", ADS_DEF[c].name, (float)adsPhys[c]);
  n += snprintf(b + n, sizeof b - n, "},\"f\":{");
  uint16_t mn[FAST_N], mx[FAST_N], now[FAST_N];
  for (int i = 0; i < FAST_N; i++) { mn[i] = webMn[i]; mx[i] = webMx[i]; now[i] = fastNow[i]; }
  webResetReq = true;
  for (int i = 0; i < FAST_N; i++)
    n += snprintf(b + n, sizeof b - n, "%s\"%s\":[%.3f,%.3f,%.3f]", i ? "," : "", FAST_NAME[i],
                  now[i] * 1e-3f, mn[i] * 1e-3f, mx[i] * 1e-3f);
  snprintf(b + n, sizeof b - n,
    "},\"th\":{\"v5min\":%.2f,\"v5max\":%.2f,\"v12min\":%.1f,\"v12max\":%.1f,\"gnd\":%.2f,"
    "\"smin\":%.2f,\"smax\":%.2f,\"k\":%.2f,\"ped\":%.1f}}",
    V5_MIN, V5_MAX, V12_MIN, V12_MAX, GND_MAX, SIG_MIN, SIG_MAX, PEDAL_K, PEDAL_ERR_PCT);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", b);
}

static void handleEvents() {
  String s = "[";
  if (xSemaphoreTake(lastMtx, pdMS_TO_TICKS(20)) == pdTRUE) {
    char b[200];
    for (int i = 0; i < lastEvN; i++) {
      const Event &e = lastEv[i];
      snprintf(b, sizeof b, "%s{\"t\":%lu,\"type\":\"%s\",\"ph\":\"%s\",\"ch\":\"%s\",\"v\":%.4f,\"d\":%lu,\"cap\":%lu}",
               i ? "," : "", (unsigned long)e.t_ms, EV_NAME[e.type], PH_NAME[e.phase], chName(e.ch), e.value,
               (unsigned long)e.dur_ms, (unsigned long)e.capId);
      s += b;
    }
    xSemaphoreGive(lastMtx);
  }
  s += "]";
  server.send(200, "application/json", s);
}

static void handleFiles() {
  String s = "[";
  if (sdOk) {
    SD_LOCK();
    File root = SD.open("/");
    bool first = true;
    for (File f = root.openNextFile(); f; f = root.openNextFile()) {
      if (!f.isDirectory()) {
        const char *nm = f.name();
        if (*nm == '/') nm++;
        char b[96];
        snprintf(b, sizeof b, "%s{\"n\":\"%s\",\"s\":%lu}", first ? "" : ",", nm, (unsigned long)f.size());
        s += b;
        first = false;
      }
      f.close();
    }
    root.close();
    SD_UNLOCK();
  }
  s += "]";
  server.send(200, "application/json", s);
}

static void handleDownload() {
  String nm = server.arg("f");
  for (size_t i = 0; i < nm.length(); i++) {               // noms simples uniquement
    char c = nm[i];
    if (!(isalnum((unsigned char)c) || c == '_' || c == '.' || c == '-') || nm.indexOf("..") >= 0) {
      server.send(400, "text/plain", "nom invalide");
      return;
    }
  }
  if (!sdOk || nm.length() == 0) { server.send(404, "text/plain", "pas de carte"); return; }
  if (logging && ("/" + nm) == String(logName)) {
    server.send(409, "text/plain; charset=utf-8", "Fichier en cours d'ecriture : mettre l'enregistrement en pause.");
    return;
  }
  SD_LOCK();
  File f = SD.open("/" + nm, FILE_READ);
  const size_t size = f ? f.size() : 0;
  SD_UNLOCK();
  if (!f) { server.send(404, "text/plain", "introuvable"); return; }

  server.setContentLength(size);
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + nm + "\"");
  server.send(200, "text/csv", "");
  static uint8_t buf[4096];
  WiFiClient cl = server.client();
  for (size_t done = 0; done < size && cl.connected();) {
    SD_LOCK();                                             // morceau par morceau : l'enregistrement continue
    const int n = f.read(buf, sizeof buf);
    SD_UNLOCK();
    if (n <= 0) break;
    cl.write(buf, n);
    done += n;
  }
  SD_LOCK();
  f.close();
  SD_UNLOCK();
}


// Export CSV de l'anneau flash, dans l'ordre chronologique (même colonnes que le journal SD)
static void handleRollCsv() {
  if (!rollPart) { server.send(404, "text/plain", "pas de partition rolllog"); return; }
  uint32_t last;
  xSemaphoreTake(flashMtx, portMAX_DELAY);
  if (rollState == RS_FROZEN) last = rollHdr.lastSeq;
  else last = (rollSeq > rollBufN) ? rollSeq - rollBufN - 1 : 0;   // seulement ce qui est en flash
  const RollHdr h = rollHdr;
  xSemaphoreGive(flashMtx);
  const uint32_t first = (last + 1 > rollCap) ? last + 1 - rollCap : 0;

  char fname[48];
  snprintf(fname, sizeof fname, "attachment; filename=\"roll_run%u.csv\"", h.run);
  server.sendHeader("Content-Disposition", fname);
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");

  static char out[4096];
  int n = snprintf(out, sizeof out,
    "# Blackbox PSA - memoire flash en boucle - cycle %u - demarrage %lu - etat %s - %d lignes/s\n"
    "# declenchement: %s voie %s valeur %.4f a t_ms=%lu (ligne %lu)\n"
    "# valeurs ADS arrondies au mV ; flags bit 15 = ecriture flash en cours (chaine rapide suspendue)\n"
    "t_ms",
    h.run, (unsigned long)h.boot, RS_NAME[rollState], ROLL_HZ,
    h.state >= RS_POST ? EV_NAME[h.trigType] : "-", h.state >= RS_POST ? chName(h.trigCh) : "-",
    h.trigValue, (unsigned long)h.trigMs, (unsigned long)h.trigSeq);
  for (int c = 0; c < ADS_N; c++) n += snprintf(out + n, sizeof out - n, ",%s", ADS_DEF[c].name);
  for (int i = 0; i < FAST_N; i++) n += snprintf(out + n, sizeof out - n, ",%s_min,%s_max", FAST_NAME[i], FAST_NAME[i]);
  n += snprintf(out + n, sizeof out - n, ",flags\n");
  server.sendContent(out, n);
  n = 0;

  static RollRec sect[ROLL_PER_SECT];
  uint32_t loaded = 0xFFFFFFFFu;
  for (uint32_t seq = first; seq <= last && rollCap; seq++) {
    const uint32_t idx = seq % rollCap, s = idx / ROLL_PER_SECT;
    if (s != loaded) {
      xSemaphoreTake(flashMtx, portMAX_DELAY);
      esp_partition_read(rollPart, (1 + s) * 4096, sect, sizeof sect);
      xSemaphoreGive(flashMtx);
      loaded = s;
    }
    const RollRec &r = sect[idx % ROLL_PER_SECT];
    if (r.seq != seq || r.run != h.run) continue;              // écrasé ou vide
    n += snprintf(out + n, sizeof out - n, "%lu", (unsigned long)r.t_ms);
    for (int c = 0; c < ADS_N; c++) n += snprintf(out + n, sizeof out - n, ",%.3f", r.ads[c] * 1e-3f);
    for (int i = 0; i < FAST_N; i++) {
      if (r.mn[i] == 0xFFFF && r.mx[i] == 0xFFFF) n += snprintf(out + n, sizeof out - n, ",,");
      else n += snprintf(out + n, sizeof out - n, ",%.3f,%.3f", r.mn[i] * 1e-3f, r.mx[i] * 1e-3f);
    }
    n += snprintf(out + n, sizeof out - n, ",%u\n", r.flags);
    if (n > (int)sizeof out - 300) {
      if (!server.client().connected()) return;
      server.sendContent(out, n);
      n = 0;
    }
  }
  if (n) server.sendContent(out, n);
  server.sendContent("");                                      // fin du transfert « chunked »
}

static void webTask(void *) {
  for (;;) { server.handleClient(); vTaskDelay(2); }
}

static void startWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", WEB_PAGE); });
  server.on("/api/live", HTTP_GET, handleLive);
  server.on("/api/events", HTTP_GET, handleEvents);
  server.on("/api/files", HTTP_GET, handleFiles);
  server.on("/dl", HTTP_GET, handleDownload);
  server.on("/api/mark", HTTP_POST, []() { postEvent(EV_MARK, PH_POINT, 255, 0, 0, millis()); server.send(200, "text/plain", "ok"); });
  server.on("/api/pause", HTTP_POST, []() { pauseReq = 1; server.send(200, "text/plain", "ok"); });
  server.on("/roll.csv", HTTP_GET, handleRollCsv);
  server.on("/api/roll/rearm", HTTP_POST, []() { rollRearmReq = true; server.send(200, "text/plain", "ok"); });
  server.begin();
  xTaskCreatePinnedToCore(webTask, "web", 8192, nullptr, 1, nullptr, 1);
}

// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_MARK, INPUT);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(C_BG);
  spr.setColorDepth(8);
  spr.createSprite(240, 135);
  tft.setTextColor(C_TXT, C_BG);
  tft.drawString("Blackbox PSA", 10, 10, 4);

  prefs.begin("bb", false);
  bootNo = prefs.getUInt("boot", 0) + 1;
  prefs.putUInt("boot", bootNo);
  for (int c = 0; c < ADS_N; c++) {
    char key[8];
    snprintf(key, sizeof key, "z%d", c);
    zeroOff[c] = prefs.getFloat(key, 0.0f);
  }

  // ADC rapide
  adc1_config_width(ADC_WIDTH_BIT_12);
  for (int i = 0; i < FAST_N; i++) adc1_config_channel_atten(FAST_ADC[i], ADC_ATTEN_DB_11);
  esp_adc_cal_characteristics_t chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &chars);
  for (int i = 0; i < 4096; i++) lut[i] = esp_adc_cal_raw_to_voltage(i, &chars);
  aggReset(agg[0]); aggReset(agg[1]);
  ring = (FastRec *)malloc(sizeof(FastRec) * RING);

  // ADS1115
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  for (int i = 0; i < 3; i++) {
    adsOk[i] = ads[i].begin(ADS_ADDR[i], &Wire);
    if (adsOk[i]) { ads[i].setGain(GAIN_ONE); ads[i].setDataRate(RATE_ADS1115_860SPS); }
  }

  // Carte SD
  sdSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  sdOk = SD.begin(PIN_SD_CS, sdSPI, 20000000);

  char b[64];
  snprintf(b, sizeof b, "Demarrage %lu  SD:%s  ADS:%d%d%d", (unsigned long)bootNo, sdOk ? "ok" : "ABSENTE",
           adsOk[0], adsOk[1], adsOk[2]);
  tft.drawString(b, 10, 50, 2);
  tft.drawString(ring ? "Capture rapide: ok" : "Capture rapide: memoire insuffisante", 10, 70, 2);
  delay(1500);

  evQ     = xQueueCreate(64, sizeof(Event));
  rowQ    = xQueueCreate(200, sizeof(Row));    // 2 s de tampon si la carte est lente
  lastMtx = xSemaphoreCreateMutex();
  sdMtx   = xSemaphoreCreateMutex();
  flashMtx = xSemaphoreCreateMutex();
  rollInit();

  disableCore0WDT();                           // la voie rapide occupe le cœur 0 en continu
  xTaskCreatePinnedToCore(fastTask,   "rapide",  4096, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(adsTask,    "ads",     4096, nullptr, 4, nullptr, 1);
  xTaskCreatePinnedToCore(loggerTask, "journal", 4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(writerTask, "sd",      8192, nullptr, 2, nullptr, 1);

  if (WIFI_ENABLE) startWeb();

  delay(300);
  if (digitalRead(BTN_NEXT) == LOW) zeroGrounds();
}

void loop() {
  static uint32_t tDraw = 0, markDown = 0;
  static bool lastNext = false, lastMark = false, pauseDone = false;

  static uint32_t nextDown = 0;
  static bool rearmDone = false;
  const bool next = digitalRead(BTN_NEXT) == LOW;
  if (next && !lastNext) { nextDown = millis(); rearmDone = false; }
  if (next && !rearmDone && millis() - nextDown > 2000) { rollRearmReq = true; rearmDone = true; }
  if (!next && lastNext && !rearmDone && millis() - nextDown > 30) screen = (screen + 1) % NSCREENS;
  lastNext = next;

  const bool mark = digitalRead(BTN_MARK) == LOW;
  if (mark && !lastMark && millis() - markDown > 300) {   // appui : marqueur immédiat (anti-rebond)
    markDown = millis();
    pauseDone = false;
    postEvent(EV_MARK, PH_POINT, 255, 0, 0, millis());
  }
  if (mark && !pauseDone && millis() - markDown > 2000) { pauseReq = 1; pauseDone = true; }
  lastMark = mark;

  if (millis() - tDraw >= 100) { tDraw = millis(); drawScreen(); }
  delay(10);
}
