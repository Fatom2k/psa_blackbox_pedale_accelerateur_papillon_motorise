// ============================================================================
//  Configuration de la blackbox — tout ce qui est réglable est ici
// ============================================================================
#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Brochage TTGO T-Display v1.1
// (écran : 4, 5, 16, 18, 19, 23 — réservés)
// ---------------------------------------------------------------------------
#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22
#define PIN_SD_SCK    25
#define PIN_SD_MOSI   26
#define PIN_SD_MISO   27
#define PIN_SD_CS     13
#define BTN_NEXT      0     // bouton gauche : écran suivant (maintenu au démarrage : zéro des masses)
#define BTN_MARK      35    // bouton droit  : MARQUEUR « panne ressentie » (+ bouton déporté en parallèle)
                            // appui long 2 s : pause / reprise de l'enregistrement

// ---------------------------------------------------------------------------
// Voies rapides : ADC de l'ESP32, ~2 kHz, pour attraper les pointes brèves.
// Elles lisent LES MÊMES nœuds que les voies ADS1115 correspondantes.
// ---------------------------------------------------------------------------
#define FAST_N 6
enum FastCh : uint8_t { F_5VP, F_S1, F_S2, F_5VT, F_T1, F_T2 };
static const uint8_t FAST_PIN[FAST_N]  = { 36, 37, 38, 39, 32, 33 };
static const char *const FAST_NAME[FAST_N] = { "V5P", "S1", "S2", "V5T", "T1", "T2" };
// Rapport du pont d'entrée (100k au point de dérivation + 100k à la masse = /2)
static const float FAST_GAIN[FAST_N]   = { 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f };

// ---------------------------------------------------------------------------
// Voies précises : 3 x ADS1115 (16 bits), ~120 mesures/s par voie
// ---------------------------------------------------------------------------
#define ADS_N 12
enum AdsCh : uint8_t {
  A_5VP, A_S1, A_S2, A_GNDP,          // ADS #1 adresse 0x48 (ADDR -> GND) : pédale
  A_5VT, A_T1, A_T2, A_GNDT,          // ADS #2 adresse 0x49 (ADDR -> VDD) : papillon
  A_12VBAT, A_12VREL, A_GNDECU, A_REF // ADS #3 adresse 0x4A (ADDR -> SDA) : alim calculateur
};
enum ChKind : uint8_t { K_DIV, K_BIAS, K_REF };
struct AdsDef { const char *name; ChKind kind; float gain; };

// K_DIV  : V = Vadc x gain                      (pont 100k / 100k -> 2 ; 100k / 15k -> 7.667)
// K_BIAS : V = gain x (Vadc - Vref) - zéro       (réseau polarisé 3 x 100k -> gain 3, mesure +/-)
// K_REF  : nœud de référence du réseau polarisé (identique, entrée reliée à la masse du boîtier)
// Corrigez les gains après étalonnage au multimètre (voir README §6).
static const AdsDef ADS_DEF[ADS_N] = {
  { "V5P",    K_DIV,  2.0f   },  // +5 V pédale            48V NR G1  fil 1380
  { "S1",     K_DIV,  2.0f   },  // piste 1 pédale         48V NR F2  fil 1377
  { "S2",     K_DIV,  2.0f   },  // piste 2 pédale         48V NR G2  fil 1378
  { "GNDP",   K_BIAS, 3.0f   },  // masse capteur pédale   48V NR F1  fil 1379
  { "V5T",    K_DIV,  2.0f   },  // +5 V papillon          32V GR A1  fil 1219
  { "T1",     K_DIV,  2.0f   },  // piste 1 papillon       32V GR B1  fil 1218
  { "T2",     K_DIV,  2.0f   },  // piste 2 papillon       32V GR B4  fil 1367
  { "GNDT",   K_BIAS, 3.0f   },  // masse capteur papillon 32V GR A4  fil 6784
  { "V12BAT", K_DIV,  7.667f },  // + permanent calculateur 48V NR M3 fil 140F
  { "V12REL", K_DIV,  7.667f },  // + après relais calculateur (à confirmer, voir README)
  { "GNDECU", K_BIAS, 3.0f   },  // masse puissance calculateur 48V NR L1/M1/L2 (MC136-138)
  { "REF",    K_REF,  1.0f   },  // référence interne du réseau polarisé
};

// ---------------------------------------------------------------------------
// Interface web : point d'accès Wi-Fi du boîtier -> http://192.168.4.1
// ---------------------------------------------------------------------------
#define WIFI_ENABLE   1             // 0 pour couper le Wi-Fi (moins de bruit, moins de conso)
#define WIFI_SSID     "BLACKBOX-PSA"
#define WIFI_PASS     "psa12345"    // 8 caractères minimum

// ---------------------------------------------------------------------------
// Enregistrement en boucle dans la flash de la carte (partition « rolllog », ~2,4 Mo)
// Tourne en permanence ; au premier événement déclencheur, continue ROLL_POST_S
// secondes puis se FIGE (conservé après coupure) jusqu'au réarmement.
// ---------------------------------------------------------------------------
#define ROLL_ENABLE     1
#define ROLL_HZ         50      // lignes/s (64 octets) -> ~13 min d'historique
#define ROLL_POST_S     10      // secondes enregistrées après le déclenchement
// Déclencheurs (types d'événements, voir EvType dans main.cpp) :
#define ROLL_TRIG_MASK  ((1u << EV_PEDAL) | (1u << EV_RANGE) | (1u << EV_MARK))

// ---------------------------------------------------------------------------
// Enregistrement sur carte SD
// ---------------------------------------------------------------------------
#define LOG_HZ          100     // lignes/s dans le journal continu
#define ROTATE_MIN      30      // nouveau fichier toutes les N minutes
#define FLUSH_MS        1000    // écriture forcée sur la carte (perte max à la coupure)
#define FAST_PERIOD_US  500     // 2 kHz par voie rapide (moyenne des lectures dans la période)
#define CAP_PRE         2000    // capture rapide : 1,0 s avant l'événement
#define CAP_POST        1000    //                  0,5 s après

// ---------------------------------------------------------------------------
// Seuils de déclenchement (réglages de la blackbox, pas des valeurs PSA)
// ---------------------------------------------------------------------------
#define V5_MIN        4.75f
#define V5_MAX        5.25f
#define V12_MIN       9.0f     // creux d'alimentation
#define V12_MAX       16.0f
#define GND_MAX       0.30f    // décalage de masse toléré (en valeur absolue)
#define SIG_MIN       0.10f    // piste en dessous : coupure / court à la masse
#define SIG_MAX       4.90f    // piste au-dessus  : court au +5 V
#define PEDAL_K       2.0f     // S1 = K x S2
#define PEDAL_ERR_PCT 6.0f     // incohérence pédale (en % du 5 V)
#define JUMP_V        0.50f    // saut d'une piste en 0,5 ms -> « glitch »
#define END_MS        20       // une condition est terminée après N ms de retour à la normale
