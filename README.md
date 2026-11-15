# Blackbox PSA — pédale d'accélérateur + papillon motorisé

Enregistreur embarqué à base de **LilyGO TTGO T-Display v1.1** pour attraper le moment exact où le calculateur **MM6LP (1320)** rejette l'information pédale (P0121 / P0122, mode dégradé).

Il mesure **au niveau du calculateur** l'alimentation de celui-ci, ses masses, les 5 V capteurs et les pistes de la pédale et du papillon. Le but est de trancher entre trois causes : **pédale/faisceau**, **perturbation** ou **calculateur**.

> Projet compagnon : [psa_banc_test_pedale_accelerateur](https://github.com/Fatom2k/psa_banc_test_pedale_accelerateur), pour valider les pédales sur table avant de les monter.

---

## 1. Principe

| Chaîne | Matériel | Voies | Cadence | Rôle |
|---|---|---|---|---|
| Précise | 3 × ADS1115 (16 bits) | 12 | ~120 mesures/s/voie | valeurs justes, décalages de masse au mV |
| Rapide | ADC de l'ESP32 | 6 (5 V et pistes) | 2 kHz | microcoupures, pointes, faux contacts |

Sur la carte microSD :

| Fichier | Contenu |
|---|---|
| `bbNNNN_PP.csv` | journal continu, **100 lignes/s** : les 12 voies précises + **min/max** des voies rapides sur chaque ligne de 10 ms (aucune pointe n'échappe au journal). Nouveau fichier toutes les 30 min. |
| `bbNNNN_events.csv` | chaque anomalie avec **début, fin, durée** et valeur la plus défavorable, plus les **marqueurs** posés au bouton |
| `bbNNNN_cap_KKK.csv` | **capture 2 kHz** des voies rapides, de 1 s avant à 0,5 s après l'événement qui l'a déclenchée |

`NNNN` est le numéro de démarrage du boîtier ; il s'incrémente à chaque mise sous tension. Le temps `t_ms` est compté depuis la mise sous tension.

## 2. Points de mesure au calculateur

D'après les schémas du projet (calculateur 1320, pédale 1261, papillon 1262) :

| Voie | Signal | Connecteur calc. | Broche | Fil | Entrée ADS | Voie rapide |
|---|---|---|---|---|---|---|
| V5P | +5 V pédale | 48V NR | G1 | 1380 | ADS#1 A0 | GPIO36 |
| S1 | piste 1 pédale | 48V NR | F2 | 1377 | ADS#1 A1 | GPIO37 |
| S2 | piste 2 pédale | 48V NR | G2 | 1378 | ADS#1 A2 | GPIO38 |
| GNDP | masse capteur pédale | 48V NR | F1 | 1379 | ADS#1 A3 | — |
| V5T | +5 V papillon | 32V GR | A1 | 1219 | ADS#2 A0 | GPIO39 |
| T1 | piste 1 papillon | 32V GR | B1 | 1218 | ADS#2 A1 | GPIO32 |
| T2 | piste 2 papillon | 32V GR | B4 | 1367 | ADS#2 A2 | GPIO33 |
| GNDT | masse capteur papillon | 32V GR | A4 | 6784 | ADS#2 A3 | — |
| V12BAT | + permanent calculateur (via F10) | 48V NR | M3 (ou M4) | 140F (140G) | ADS#3 A0 | — |
| V12REL | + après relais calculateur | 48V NR | **à confirmer** : L4, A3 ou F4 | 1203E / 1226E / 1229E | ADS#3 A1 | — |
| GNDECU | masse puissance calculateur | 48V NR | L1 / M1 / L2 | MC136 / MC137 / MC138 | ADS#3 A2 | — |
| REF | référence interne (pas de fil) | — | — | — | ADS#3 A3 | — |

**À vérifier avant de dériver** : ces affectations viennent de ma lecture des schémas. Contrôle chaque broche au multimètre :
- les 5 V doivent être présents contact mis ;
- `140F` doit être présent en permanence ;
- le bon « + après relais » est **0 V contact coupé et ~12 V contact mis**.

Pour info, le moteur du papillon est sur 32V GR **G1 / G2** (fils 1334 / 1335). Il n'est pas mesuré dans cette version, car c'est un pont en H piloté en PWM ±12 V.

## 3. Étage d'entrée

Toutes les tensions sont mesurées par rapport à **la masse du boîtier**. Celle-ci est reliée par un fil dédié à la **borne − de la batterie**, la référence commune. Les masses du calculateur et des capteurs sont mesurées elles aussi, pour voir leur décalage.

**Règle n° 1 : la première résistance de 100 kΩ se soude au point de dérivation**, côté calculateur, sous gaine thermo. Un court-circuit dans le câble ou dans le boîtier ne peut alors rien injecter dans le calculateur, et la charge ajoutée sur chaque fil reste ≥ 200 kΩ.

```
 Pistes et 5 V (V5P, S1, S2, V5T, T1, T2) — rapport /2, 0..5 V -> 0..2,5 V
   fil calc ──[100k]~~~~~câble~~~~~┬────────┬──► ADS Ax ──► (et GPIO voie rapide)
   (au point de dérivation)        [100k]   [10nF]
                                    │        │
                                   GND      GND

 +12 V (V12BAT, V12REL) — rapport /7,67, 0..20 V -> 0..2,6 V
   fil calc ──[100k]~~~~~câble~~~~~┬────────┬──► ADS Ax
                                   [15k]    [10nF]
                                    │        │
                                   GND      GND

 Masses (GNDP, GNDT, GNDECU) — réseau polarisé, mesure de -1 V à +2 V
                                  3V3
                                   │
                                 [100k]
   fil calc ──[100k]~~~~~câble~~~~~┼────────┬──► ADS Ax
                                 [100k]   [10nF]
                                   │        │
                                  GND      GND

 Référence (REF) — même réseau, entrée reliée à la masse du boîtier
   GND ──[100k]──┬── 3V3 via [100k]
                 ├── GND via [100k]
                 ├── [10nF] ── GND
                 └──► ADS#3 A3
```

Le firmware calcule `masse = 3 × (V_nœud − V_REF)`. Les variations du 3,3 V s'annulent ainsi. Les petites dispersions des résistances sont compensées par le **zéro des masses** (§ 6).

Côté câble : utiliser un **multiconducteur blindé** (blindage relié à la masse du boîtier **d'un seul côté**, côté boîtier). Le faire passer loin des bobines et des faisceaux d'allumage, et le garder aussi court que possible. Un câble mal posé crée lui-même des perturbations.

## 4. Matériel et câblage du boîtier

**Nomenclature**
- 1 × TTGO T-Display v1.1
- 3 × module ADS1115
- 1 × module lecteur microSD SPI, avec une carte ≤ 32 Go formatée en **FAT32**
- 1 × **convertisseur 12 V → 5 V isolé** d'au moins **3 W** (le Wi-Fi consomme par pointes), alimenté par la prise allume-cigare via un fusible de 1 A
- Résistances (1 %) : 26 × 100 kΩ, 2 × 15 kΩ ; 12 × condensateurs de 10 nF
- 1 bouton poussoir déporté pour le volant ou la console (**marqueur**)
- Câble blindé multiconducteur, gaine thermo

**Pourquoi un convertisseur isolé ?** La masse du boîtier doit être reliée **uniquement** à la borne − de la batterie, par le fil de référence. Un convertisseur non isolé relierait aussi cette masse à la carrosserie par la prise allume-cigare. Le courant de masse du véhicule passerait alors en partie par le fil de référence, ce qui fausserait les mesures de masse.

**Liaisons**

| T-Display | vers |
|---|---|
| 5V / GND | sortie du convertisseur isolé |
| GND | **fil dédié vers la borne − de la batterie** (référence) |
| 3V3 | VDD des 3 ADS1115, haut des réseaux polarisés |
| GPIO21 / GPIO22 | SDA / SCL des 3 ADS1115 (en parallèle) |
| ADS#1 ADDR → GND (0x48), ADS#2 ADDR → VDD (0x49), ADS#3 ADDR → SDA (0x4A) | adresses I2C |
| GPIO25 / 26 / 27 / 13 | SD : SCK / MOSI / MISO / CS (module SD alimenté en 5 V s'il a son propre régulateur) |
| GPIO36, 37, 38, 39, 32, 33 | nœuds V5P, S1, S2, V5T, T1, T2 (les mêmes que les entrées ADS) |
| GPIO35 + GND | bouton marqueur déporté (en parallèle du bouton droit), câble torsadé |

## 5. Compilation

PlatformIO : ouvrir le dossier puis lancer *Upload*. L'écran et les bibliothèques sont configurés dans `platformio.ini`, sur la plateforme `espressif32@6.9.0` (Arduino ESP32 2.0.17).

Avec l'Arduino IDE : installer le core ESP32 **2.0.17**, ainsi que les bibliothèques TFT_eSPI (activer `Setup25_TTGO_T_Display.h`) et Adafruit ADS1X15. Copier `src/` dans un dossier `blackbox/` et renommer `main.cpp` en `blackbox.ino`. Choisir le schéma de partition **« Huge APP (3MB No OTA) »**, car le Wi-Fi alourdit le programme.

Tous les réglages se trouvent dans **`src/config.h`** : brochage, rapports des ponts, seuils, cadence.

## 6. Mise en service

1. **Avant tout montage sur la voiture**, sur table : alimenter le boîtier et appliquer des tensions connues (une pile ou une alimentation réglable). Vérifier chaque voie à l'écran face au multimètre. Corriger si besoin les `gain` dans `config.h`, car l'impédance d'entrée des ADS1115 donne une erreur de gain de l'ordre de 1 %.
2. Monter le boîtier. **Contact coupé**, maintenir le bouton gauche (GPIO0) à la mise sous tension pendant 3 s. Le boîtier fait alors le **zéro des masses** et le garde en mémoire.
3. Mettre le contact. L'écran 1 doit passer à « Calculateur : ALIMENTE ». La surveillance automatique ne s'active qu'à ce moment, pour éviter de déclencher des alertes contact coupé.

## 7. Utilisation

| Bouton | Action |
|---|---|
| Gauche (GPIO0) | écran suivant : BLACKBOX → PÉDALE → PAPILLON → ALIM → ÉVÉNEMENTS |
| Droit (GPIO35) / bouton déporté | **MARQUEUR** : à presser dès que la panne est ressentie |
| Droit maintenu 2 s | pause / reprise. **Mettre en pause avant de retirer la carte SD** |

Le journal est écrit sur la carte au moins une fois par seconde. Une coupure brutale fait donc perdre au plus la dernière seconde.

**Déclenchements automatiques** (seuils dans `config.h`, ce sont des réglages de la blackbox et non des valeurs PSA) :

| Événement | Condition |
|---|---|
| `PEDALE_INCOH` | \|S1 − 2·S2\| > 6 % du 5 V, calculé par rapport à la masse capteur |
| `HORS_PLAGE` | une piste < 0,10 V ou > 4,90 V |
| `ALIM_5V` | un 5 V capteur hors de 4,75–5,25 V |
| `ALIM_12V` | + calculateur < 9 V ou > 16 V |
| `MASSE` | décalage d'une masse > 0,30 V |
| `GLITCH` | saut d'une piste de plus de 0,5 V en 0,5 ms |

Chaque début d'événement lance une capture rapide, sauf si une capture est déjà en cours d'écriture.

## 8. Interface web (temps réel sur téléphone)

Le boîtier crée son propre réseau Wi-Fi, sans box ni Internet :

- réseau **`BLACKBOX-PSA`**, mot de passe **`psa12345`** (modifiable dans `config.h`) ;
- puis ouvrir **http://192.168.4.1** dans le navigateur. Si le téléphone signale « pas d'Internet », choisir de rester connecté.

| Onglet | Contenu |
|---|---|
| Temps réel | toutes les voies avec code couleur, rafraîchies 10 fois par seconde. Courbes sur 30 s de la pédale (S1 et 2×S2), du papillon et des deux 5 V. Les bandes claires montrent le **min/max à 2 kHz** entre deux rafraîchissements, donc une microcoupure reste visible |
| Événements | les 20 derniers, mis à jour dès qu'il en arrive un |
| Fichiers | liste de la carte SD et **téléchargement direct** des journaux et captures, sans retirer la carte. L'enregistrement continue pendant le téléchargement. Le fichier en cours d'écriture demande une pause |

Les boutons **MARQUEUR** et **Pause** de la page font la même chose que les boutons du boîtier. Un passager peut ainsi poser le marqueur depuis son téléphone.

**Remarque** : le Wi-Fi ajoute un peu de bruit sur l'ADC de l'ESP32 et quelques dizaines de mA de consommation. Pour une session de mesure « propre », on peut le couper avec `WIFI_ENABLE 0` dans `config.h`.

## 9. Analyse

```bash
pip install pandas matplotlib
python tools/analyse.py events  bb0012_events.csv
python tools/analyse.py journal bb0012_00.csv --events bb0012_events.csv --t 754321 --fenetre 5
python tools/analyse.py capture bb0012_cap_003.csv
```

Le tracé du journal comporte six panneaux : pistes pédale avec bandes min/max, écart de cohérence, papillon, les deux 5 V, les 12 V, les masses. Les événements apparaissent en rouge et les marqueurs en cyan.

**Lecture du résultat au moment de la panne**

| Ce qu'on voit au calculateur | Piste probable |
|---|---|
| S1/S2 incohérents, 5 V pédale stable, masses stables | **pédale ou faisceau pédale** (fils 1377/1378/1379, connecteur). Repasser la pédale au banc de test |
| 5 V **pédale seul** qui chute ou saute | court ou faux contact sur le fil 1380 côté pédale |
| **Les deux** 5 V qui chutent ensemble | alimentation du calculateur ou son régulateur interne. Regarder les 12 V et la masse calculateur au même instant |
| Masse calculateur ou masse capteur qui décolle | point de masse (MC13…) ou fil de masse |
| 12 V « vu par le calculateur » qui plonge | batterie, relais double, fusibles, masse |
| Pointes brèves sur plusieurs voies en même temps, sans cause d'alimentation | **perturbation** (parasite). Noter dans quelle situation de conduite elles apparaissent |
| Tous les signaux parfaits au calculateur, mais le défaut est quand même enregistré | **calculateur** (étage d'entrée) fortement suspect |

## 10. Évolutions possibles

- Mesure du pilotage moteur du papillon (G1/G2) par opto-coupleurs, pour voir l'instant où le calculateur coupe la commande.
- Régime moteur, via le signal du capteur 1313 ou le CAN, pour corréler les pointes avec l'allumage.
- Horloge DS3231 sur le bus I2C, pour horodater en heure réelle.
