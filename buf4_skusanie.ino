#include <Wire.h>
#include <TFT_eSPI.h>
#include <Adafruit_BME280.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <PID_v1.h>
#include "FS.h"
#include "SPIFFS.h"
#include <vector>
#include <WiFi.h>
#include <driver/ledc.h>
using namespace fs;
#include "max6675.h"


float merajPrud();
void emergencyShutdown();

// ====== KONSTANTY ======
#define LEVELS 10
#define MIN_VENT_OTACKY 100
//#define ZHAVENIE_CAS 30000
//#define CERPADLO_START_CAS 15000
//#define CERPADLO_PULZ 20
#define TEPLOTA_RYCHLE_START 60.0
//#define TEPLOTA_NASTUP_NAHRIVANIE 70.0
//#define TEPLOTA_VYPNOUT_CHLADENIE 50.0
#define TEPLOTA_MAX 230.0
//#define VENTILATOR_VYPNUTY_CAS 30000
//#define VENT_CISTENIE 128
//#define CHLADENIE_CAS 30000
//#define KONTROLA_SENZOROV_CAS 60000
#define SPOTREBA_NA_PULZ 0.000025
//#define REFERENCNA_TEPLOTA 20.0
//#define KOEF_TEPLOTA 0.05
//#define WDT_TIMEOUT 10
#define EMERGENCY_SHUTDOWN_TEMP 230.0
#define PID_KP 1.0   // alebo aj 0.7
#define PID_KI 0.1   // veľmi opatrne s I
#define PID_KD 2.0   // ak chceš tlmenie (alebo 0.5)
#define TEPLOTA_MAX 230.0              // A1: hranica prehriatia komory
#define TEPLOTA_BEZPECNA 180.0             // A1: hranica bezpečnej teploty pri dochladzovaní
#define DOCHLADENIE_MS   60000UL    // A1: minimálny čas dochladenia ventilátorom (1 min)
#define MAX_RESTART_POKUSY 3           // A2: max. auto-reštarty pri zhasnutí plameňa
#define PLAMEN_TIMEOUT_MS 5000UL       // A2: timeout pre vyhodnotenie zhasnutia plameňa
#define SPOTREBA_NA_PULZ 0.000025
//#define EMERGENCY_SHUTDOWN_TEMP 230.0

#define R1    46900.0     // Tvoj delič – odpor na +3.3V
#define R0    50000.0     // NTC odpor pri 25°C
#define BETA  3650.0      // Beta koeficient tvojho NTC (skontroluj datasheet, ale 3950 je klasika)
#define T0    298.15      // Absolútna teplota v Kelvinoch (25°C = 298.15 K)




// ====== FÁZY KÚRENIA ======
enum FazaKurenia {
  FAZA_NEZAPNUTE,
  FAZA_NAHRIEVANIE,
  FAZA_KURENIE,
  FAZA_CHLADENIE,
  FAZA_EMERGENCY_CHLADENIE,
  FAZA_LOCKOUT
};

// ====== A1/A2/A3 PREMENNÉ ======
bool stavZablokovany = false;       // A3: blokovanie po neúspešných štartoch
bool plamenDetekovany = false;      // A2: senzor plameňa
unsigned long plamenNaposledy = 0;  // A2: čas poslednej detekcie plameňa
int restartPokusy = 0;              // A2: počítadlo auto-reštartov
unsigned long emergencyStartMs = 0; // A1: čas spustenia emergency chladenia


FazaKurenia aktualnaFaza = FAZA_NEZAPNUTE;
unsigned long casZaciatkuFazy = 0;
int urovenKurenia = 1;                   // Začína na 1, ale nahradí sa poslednou hodnotou
bool chybaSystemu = false;
String popisChyby = "";
bool jeOznaceneSpat = false;             // Sleduje, či je označené "SPAŤ"
Preferences prefs;                       // Ukladanie poslednej úrovne


bool wifiScanInProgress = false;
bool wifiScanDone = false;


double pidInput, pidOutput, pidSetpoint;
PID pid(&pidInput, &pidOutput, &pidSetpoint, PID_KP, PID_KI, PID_KD, DIRECT);

// ====== PID pre ventilátor ======
double pidVentInput, pidVentOutput, pidVentSetpoint;
PID pidVent(&pidVentInput, &pidVentOutput, &pidVentSetpoint, PID_KP, PID_KI, PID_KD, DIRECT);

// Výstupné hodnoty PWM pre ventilátor podľa úrovne (zodpovedajú cca 25–100 % výkonu)
int ventilatorPWM[10] = {64, 84, 105, 127, 148, 168, 189, 211, 232, 255};

// Cieľové teploty pre jednotlivé úrovne kúrenia (v stupňoch Celzia)
double cieloveTeploty[10] = { 130, 140, 150, 160, 170, 180, 190, 200, 210 };




// ====== FARBY DISPLEJA ======
#define ZELENA   0x07E0
#define ORANZOVA 0xFD20
#define CERVENA  0xF800
#define MODRA    0x001F
#define ZLTA     0xFFE0
#define BIELA    0xFFFF
#define CIERNA   0x0000

// ====== PINY ======
#define PWM_ZHAVENIE     25
#define PWM_CERPADLO      2
#define PWM_VENTILATOR   27
#define VENT_PWM_CH       0
#define ACS712_PIN       33
#define BTN_UP            0  // používateľské tlačidlo
#define BTN_OK           35  // boot tlačidlo
#define PIN_NTC          32
#define SDA_BME          22
#define SCL_BME          21

int soPin = 12;    // SO (DO)
int csPin = 13;    // CS
int sckPin = 17;   // SCK (CLK)

MAX6675 thermocouple(sckPin, csPin, soPin);

// ====== KONFIGURACIA MERANIA PRUDU ======
#define ACS712_SENZITIVITA 0.066
#define ADC_REFERENCIA 3.3
#define ADC_ROZLISENIE 4095.0



// ====== PREMENNE ======
TFT_eSPI tft = TFT_eSPI();
TwoWire I2C_AHT = TwoWire(0);
Adafruit_BME280 bme;  // I2C

unsigned long casPrechodu = 0;         // Čas začiatku prechodu
const unsigned long DOBA_PRECHAZU = 5000; // 5 sekúnd pre plynulý prechod
int cieloveVentPWM = 0;               // Cieľové PWM pre ventilátor
int startVentPWM = 0;                 // Počiatočné PWM na začiatku prechodu
// Plynulé prechody
struct PlynulyPrechod {
  unsigned long casZaciatku;
  int startHodnota;
  int cielovaHodnota;
  bool aktivny;
};

PlynulyPrechod prechodVentilator = {0, 0, 0, false};
PlynulyPrechod prechodCerpadlo = {0, 0, 0, false};



int vypocitajPlynuluHodnotu(PlynulyPrechod prechod) {
  if (!prechod.aktivny) return prechod.cielovaHodnota;

  unsigned long elapsed = millis() - prechod.casZaciatku;
  float progress = min(1.0f, (float)elapsed / DOBA_PRECHAZU);

  // Ease-in-out krivka pre prirodzenejší prechod
  progress = progress < 0.5 ? 2 * progress * progress : 1 - pow(-2 * progress + 2, 2) / 2;

  return prechod.startHodnota + (prechod.cielovaHodnota - prechod.startHodnota) * progress;
}


// --- Kalibrácia ACS712
bool trebaKalibrovat = true;
unsigned long casStartu = 0;
const float KALIBRACNY_PRUD = 0; // nastav si podľa reálneho kľudového prúdu v A

int ACS712_FLIP = 1; // 1 = normálne, -1 = preklopiť

float tlakBME = 0.0;
float offsetNapetie = 0.0;
bool kalibrovane = false;

enum StavKurenia { KREADY, KPROCESS, KFAULT };
StavKurenia stav_kurenia = KREADY;

enum StavProcesu { VYPNUTE, NAHRIEVANIE, KURENIE, CHLADENIE };
StavProcesu proces = VYPNUTE;

enum MenuPage {
  PAGE_INDEX,
  PAGE_SPOTREBA,
  PAGE_INFO,
  PAGE_START_KURENIE,
  PAGE_NASTAVENIE_UROVNE,
  PAGE_PREDPOVED,
  PAGE_NASTAVENIA,
  PAGE_WIFI_SELECT,
  PAGE_WIFI_PASSWORD
};


MenuPage currentPage = PAGE_INDEX;
const char* menuItems[] = {
  "Start kurenia",
  "Predpoved",
  "Spotreba",
  "Informacie",
  "Nadstavenia"
};

int aktualneVentPWM = 70; // Začiatočná hodnota podľa tvojho setupu

const char* nastaveniaMenu[] = {
  "Objem nadrze",
  "WiFi siet",
  "Natankovane"
};
String ssidList[10];        // Zoznam nájdených sietí
int foundNetworks = 0;      // Počet nájdených sietí
int selectedWiFi = 0;       // Index aktuálne vybranej siete
bool wifiSelectMode = false;// Režim výberu siete v menu
bool wifiPasswordMode = false;
String wifiPassword = "";

bool kurenieAktivne = false;
unsigned long posledneVypnuteCas = 0;

unsigned long poslednyLogCas = 0;
const unsigned long LOG_INTERVAL = 30 * 60 * 1000UL; // 30 minút v milisekundách

enum NastaveniaItem {
  SET_NADRZ,
  SET_WIFI,
  SET_NATANKOVANE,
  POCET_NASTAVENI
};
int aktualneNastavenie = 0;
bool editMode = false;

float objemNadrze = 5.0;          // nastavený objem nádrže (default 5 l)
float aktualnyStavNadrze = 5.0;   // aktuálne v nádrži (mení sa automaticky)
unsigned long posledneDoliatieCas = 0; // čas posledného natankovania

bool zvolenaPolozkaNadrz = true;  // true = nastavujem objem, false = natanko


float vonkajsiaTeplota = 20.0;
unsigned long PulzInterval = 500;
unsigned long lastPumpPulse = 0;
bool cerpadloPulzuje = false;
float teplotaCielStart = 60.0;
float teplotaCielUdrzanie = 90.0;

int indexSelect = 0, menuSelect = 0, levelSelect = 0;
int curLevel = 0, ventLevels[LEVELS] = {100, 150, 200, 230, 255};
int pumpLevels[LEVELS] = {800, 600, 400, 300, 200};
int startLevel = 0;

float tAHT = NAN, hAHT = NAN, tBufik = NAN, tNTC = NAN;

float predpovedMinTeplota = 8.0;    // Očakávaná minimálna teplota (noc)
float predpovedMaxTeplota = 16.0;   // Očakávaná maximálna teplota (deň)
int predpovedStavIndex = 0;         // Index stavu počasia (pozri nižšie)
const char* predpovedStavy[] = { "Jasno", "Polooblacno", "Oblacno", "Dazd", "Sneh" };


int startPumpInterval = 2000;    // Počiatočná hodnota, neskôr meníš adaptívne
int minPumpInterval = 333;
int pokusyZapalit = 0;
bool uspesnyStart = false;


unsigned long procesStart = 0, lastTeplotaUpdate = 0, lastSenzorCheck = 0;
unsigned long casNahrievania = 0;
bool pumpOn = false, confirmationShown = false;
bool zhavicZapnuty = false, prvych3PulzovHotovo = false;
bool chybaStartu = false, cerpadlo15sHotovo = false;
unsigned long zhavicStart = 0, startPulzCas = 0;

hw_timer_t *watchdogTimer = NULL;

volatile unsigned long pulzyZaInterval = 0;        // Pulzy za posledný interval (napr. 5 sekúnd)
unsigned long poslednyResetPulzov = 0;             // Kedy sa naposledy resetovalo počítadlo pulzov
float momentalnaSpotrebaLh = 0.0;                  // Momentálna spotreba v L/h
const unsigned long INTERVAL_MERANIA = 5000;       // Interval merania (5 sekúnd)

struct SpotrebaData {
  unsigned long startTime;
  unsigned long duration;
  float spotreba;
  float priemernaTeplota;
  int pulzy;
};
SpotrebaData spotrebaHistory[3];
int currentSpotrebaIndex = 0;
unsigned long totalPulzy = 0;
float totalSpotreba = 0;
float runningAvgTemp = 0;
unsigned long tempSamples = 0;


//==================================================================================================================== logujDataDoSPIFFS ====================================================
void logujDataDoSPIFFS() {
  File file = SPIFFS.open("/data_log.txt", FILE_APPEND);
  if (!file) {
    Serial.println("❌ Nepodarilo sa otvoriť súbor na zápis");
    return;
  }

  unsigned long teraz = millis() / 1000;

  // POZOR: všetky tieto premenné MUSÍŠ mať predvolené!
  file.printf("%lu,%.2f,%.1f,%.0f,%.1f,%d,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%lu,%d,%d,%d,%d\n",
              teraz, tAHT, hAHT, 1013.0,
              pidOutput, urovenKurenia,
              SPOTREBA_NA_PULZ * totalPulzy,
              pid.GetKp(), pid.GetKi(), pid.GetKd(),          // pumpa
              pidVent.GetKp(), pidVent.GetKi(), pidVent.GetKd(), // ventilátor
              casNahrievania,
              startPumpInterval,
              minPumpInterval,
              pokusyZapalit,
              uspesnyStart ? 1 : 0
             );

  file.close();
  Serial.println("✅ Dáta uložené do SPIFFS");
}

struct ZaznamLogu {
  unsigned long cas;
  float teplota;
  float vlhkost;
  float tlak;
  float pid;
  int uroven;
  float spotreba;
  float Kp, Ki, Kd;          // pumpa
  float KpVent, KiVent, KdVent; // ventilátor
  unsigned long casNahrievania;
  int startPumpInterval;
  int minPumpInterval;
  int pokusyZapalit;
  bool uspesnyStart;
};

// *** TOTO MUSÍ BYŤ VONKU, NIE VO FUNKCII! ***
std::vector<ZaznamLogu> zaznamy;




//==================================================================================================================== nacitajLogyZoSPIFFS ====================================================
void nacitajLogyZoSPIFFS() {
  File file = SPIFFS.open("/data_log.txt", FILE_READ);
  if (!file) {
    Serial.println("❌ Nepodarilo sa otvoriť súbor na čítanie");
    return;
  }

  zaznamy.clear();
  while (file.available()) {
    String line = file.readStringUntil('\n');
    ZaznamLogu zaznam;
    int uspesnyStartInt = 0;
    int scanned = sscanf(line.c_str(), "%lu,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f,%f,%lu,%d,%d,%d,%d",
                         &zaznam.cas, &zaznam.teplota, &zaznam.vlhkost,
                         &zaznam.tlak, &zaznam.pid, &zaznam.uroven,
                         &zaznam.spotreba,
                         &zaznam.Kp, &zaznam.Ki, &zaznam.Kd,          // pumpa
                         &zaznam.KpVent, &zaznam.KiVent, &zaznam.KdVent, // ventilátor
                         &zaznam.casNahrievania,
                         &zaznam.startPumpInterval,
                         &zaznam.minPumpInterval,
                         &zaznam.pokusyZapalit,
                         &uspesnyStartInt);

    zaznam.uspesnyStart = (uspesnyStartInt != 0);

    if (scanned == 17) {
      zaznam.uspesnyStart = (uspesnyStartInt != 0);
      zaznamy.push_back(zaznam);
    }
  }

  file.close();
  Serial.printf("📄 Načítaných %d záznamov zo SPIFFS\n", zaznamy.size());
}



// ================================================================================================================ Proces kurenia ===================================================




// ============================================================================================================== Kontrola Ventilatora  ==============================================
void kontrolaVentilatora() {
  Serial.println("🧪 [Kontrola ventilátora] Spúšťam test na plný výkon (PWM 255)");

  ledcWrite(VENT_PWM_CH, 255);
  delay(1000);

  float prud = merajPrud();
  Serial.print("🔍 Meraný prúd ventilátora: ");
  Serial.print(prud, 2);
  Serial.println(" A");

  if (prud < 0.5) {
    chybaSystemu = true;
    popisChyby = "Chyba ventilátora! Skontroluj napájanie/otáčky ventilátora.";
    Serial.println("❗ Upozornenie: ventilátor neťahá prúd – kúrenie NEBUDE spustené.");
    // *** NEVOLAJ EMERGENCY SHUTDOWN pri predštarte ***
    return;
  }

  ledcWrite(VENT_PWM_CH, 64);
  Serial.println("✅ Ventilátor OK, nastavujem späť na nízke otáčky (PWM 64)");
}
float citajNTCTeplotu(int pin) {
  return thermocouple.readCelsius();
}


// =============================================================================================================== Kontrola cerpada  ===============================================
void kontrolaCerpadla() {
  Serial.println("🧪 [Kontrola čerpadla] Spúšťam 3 testovacie impulzy...");

  for (int i = 0; i < 3; i++) {
    Serial.printf("🔁 Impulz #%d:\n", i + 1);

    digitalWrite(PWM_CERPADLO, HIGH);
    delay(200);
    digitalWrite(PWM_CERPADLO, LOW);
    delay(30);
    pulzyZaInterval++;

    float prudPred = merajPrud();
    Serial.printf("  🔸 Prúd pred impulzom: %.2f A\n", prudPred);

    digitalWrite(PWM_CERPADLO, HIGH);
    delay(10);
    float prudImpulz = merajPrud();
    delay(40);
    digitalWrite(PWM_CERPADLO, LOW);
    pulzyZaInterval++;
    delay(30);
    float rozdiel = prudImpulz - prudPred;
    Serial.printf("  🔸 Prúd počas impulzu: %.2f A (rozdiel: %.2f A)\n", prudImpulz, rozdiel);

    if (rozdiel < 0.1) {
      chybaSystemu = true;
      popisChyby = "Chyba čerpadla!";
      Serial.println("❌ Zlyhala kontrola čerpadla – rozdiel prúdov je príliš malý");
      return;
    }
  }

  Serial.println("✅ Čerpadlo OK – testy úspešné");
}


// =============================================================================================================== Kontrola žhavenia  ==============================================
void kontrolaZhavenia() {
  Serial.println("🧪 [Kontrola žhavenia] Zapínam žhavenie...");

  digitalWrite(PWM_ZHAVENIE, HIGH); // ak máš aktívne HIGH
  delay(1000);

  float prud = merajPrud();
  Serial.printf("🔍 Meraný prúd žhavenia: %.2f A\n", prud);

  if (prud < 1.0) {
    Serial.println("❌ Zlyhala kontrola žhavenia – prúd príliš nízky!");
    Serial.println("💡 Možné príčiny:");
    Serial.println("  1. Porucha relé");
    Serial.println("  2. Porucha žhavenia");
    Serial.println("  3. Chybné pripojenie");
    Serial.println("  4. Chyba merania prúdu");

    chybaSystemu = true;
    popisChyby = "Chyba žhavenia! Nízky prúd";
    digitalWrite(PWM_ZHAVENIE, LOW);
    delay(30);
    return;
  }

  Serial.println("✅ Žhavenie OK – vypínam");
  digitalWrite(PWM_ZHAVENIE, LOW);
}



//================================================================================================================== Faza Nahrievania ================================================
void fazaNahrievania() {
  static bool init = false;
  static unsigned long startNahrievania = 0;
  static int cielovePWM = 0;
  static int startPWM = 200; // rozbeh ventilátora (~3000 rpm)
  static int aktualnyPumpInterval = 1000;
  static int minPumpInterval = 333;
  static unsigned long lastPump = 0;
  static bool plamenPotvrdeny = false;
  static unsigned long casPotvrdeniaPlamena = 0;
  static bool predpulzHotovy = false;
  static unsigned long casPredpulzu = 0;

  // ---- Reset premenných pri vstupe do fázy ----
  static int lastFaza = -1;
  if (aktualnaFaza != lastFaza) {
    init = false;
    startNahrievania = 0;
    cielovePWM = 0;
    startPWM = 200; // začni rozbehom ventilátora
    aktualnyPumpInterval = 1000;
    minPumpInterval = 333;
    lastPump = 0;
    plamenPotvrdeny = false;
    casPotvrdeniaPlamena = 0;
    predpulzHotovy = false;
    casPredpulzu = 0;
    lastFaza = aktualnaFaza;
  }

  const unsigned long DOBA_PREDZHAVENIA = 30000; // 30 sekúnd

  if (!init) {
    cielovePWM = ventilatorPWM[urovenKurenia - 1];
    aktualneVentPWM = startPWM;
    lastPump = millis();
    startNahrievania = millis();
    plamenPotvrdeny = false;
    casPotvrdeniaPlamena = 0;
    predpulzHotovy = false;
    casPredpulzu = 0;
    digitalWrite(PWM_ZHAVENIE, HIGH);
    Serial.println("▶️ Štart nahrievania: rozbeh ventilátora + žhavenie.");
    init = true;
  }

  unsigned long casOdStartu = millis() - startNahrievania;

  // === BEZPEČNÁ PREDŽHAVIACA FÁZA ===
  if (casOdStartu < DOBA_PREDZHAVENIA) {
    digitalWrite(PWM_ZHAVENIE, HIGH);
    digitalWrite(PWM_CERPADLO, LOW);
    ledcWrite(VENT_PWM_CH, aktualneVentPWM);

    // simulácia rozbehu ventilátora → po 5 s začni pád otáčok na ~960 rpm
    if (casOdStartu > 5000 && aktualneVentPWM > 100) {
      aktualneVentPWM -= 1;
      ledcWrite(VENT_PWM_CH, aktualneVentPWM);
    }

    return;
  }

  // === PREDPULZ ČERPADLA (2–3 s na 2,3 Hz) ===
  if (!predpulzHotovy) {
    if (casPredpulzu == 0) casPredpulzu = millis();
    if (millis() - casPredpulzu < 3000) {
      if (millis() - lastPump > 435) { // 2.3 Hz ≈ 435 ms interval
        digitalWrite(PWM_CERPADLO, HIGH);
        delay(50);
        digitalWrite(PWM_CERPADLO, LOW);
        pulzyZaInterval++;
        lastPump = millis();
      }
      return; // ešte stále predpulz
    } else {
      predpulzHotovy = true;
      Serial.println("💧 Predpulz čerpadla dokončený, pokračujem normálnym nahrievaním.");
    }
  }

  // --- Normálne rampovanie ventilátora ---
  if (aktualneVentPWM < cielovePWM) aktualneVentPWM++;
  if (aktualneVentPWM > cielovePWM) aktualneVentPWM--;
  ledcWrite(VENT_PWM_CH, aktualneVentPWM);

  // --- Normálne zrýchľovanie pumpy ---
  if (aktualnyPumpInterval > 333) aktualnyPumpInterval -= 2;
  if (aktualnyPumpInterval < 333) aktualnyPumpInterval = 333;

  // --- Pulz čerpadla ---
  if (millis() - lastPump > aktualnyPumpInterval) {
    digitalWrite(PWM_CERPADLO, HIGH);
    delay(50);
    digitalWrite(PWM_CERPADLO, LOW);
    delay(30);
    pulzyZaInterval++;
    lastPump = millis();
  }

  // --- Detekcia plameňa ---
  if (!plamenPotvrdeny && tBufik > TEPLOTA_RYCHLE_START) {
    plamenPotvrdeny = true;
    casPotvrdeniaPlamena = millis();
    digitalWrite(PWM_ZHAVENIE, LOW);
    delay(30);
    Serial.println("🔥 Plameň potvrdený! Prechod do PID kúrenia.");
    casNahrievania = casPotvrdeniaPlamena - startNahrievania;
    logujDataDoSPIFFS();

    aktualneVentPWM = ventilatorPWM[urovenKurenia - 1];
    ledcWrite(VENT_PWM_CH, aktualneVentPWM);
    prechodVentilator.startHodnota = aktualneVentPWM;
    prechodVentilator.cielovaHodnota = ventilatorPWM[urovenKurenia - 1];
    prechodVentilator.casZaciatku = millis();
    prechodVentilator.aktivny = false;

    init = false;
    aktualnaFaza = FAZA_KURENIE;
    casZaciatkuFazy = millis();
    return;
  }

  // --- Timeout/havária ---
  if (!plamenPotvrdeny && millis() - startNahrievania > 180000) {
    chybaSystemu = true;
    popisChyby = "Neúspešný štart bufíka!";
    digitalWrite(PWM_ZHAVENIE, LOW);
    digitalWrite(PWM_CERPADLO, LOW);
    Serial.println("❌ Timeout: štart bufíka sa nepodaril.");
    init = false;
    emergencyShutdown();
    return;
  }
}



//============================================================================================================== Plynuly Prechod Ventilatora ==============================================

void plynulyPrechodVentilatora() {
  if (!prechodVentilator.aktivny) return;

  unsigned long elapsed = millis() - prechodVentilator.casZaciatku;
  float progress = min(1.0f, (float)elapsed / DOBA_PRECHAZU);

  // Ease-in-out prechod
  progress = progress < 0.5 ? 2 * progress * progress : 1 - pow(-2 * progress + 2, 2) / 2;

  aktualneVentPWM = prechodVentilator.startHodnota +
                    (prechodVentilator.cielovaHodnota - prechodVentilator.startHodnota) * progress;

  ledcWrite(VENT_PWM_CH, aktualneVentPWM);

  if (progress >= 1.0f) {
    prechodVentilator.aktivny = false;
  }
}



//=======================================================================================================================Faza Kurenia=========================================================
void fazaKurenia() {
  static int lastUroven = -1;
  static bool pidInit = false;
  static float poslednaTAHT = -1000;

  // >>> RESET PID PRI VÝRAZNEJ ZMENE TEPLOTY
  if (fabs(tAHT - poslednaTAHT) > 5.0) {   // ak rozdiel vonkajšej teploty > 5°C
    pidInit = false;
    poslednaTAHT = tAHT;
    Serial.println("🌀 Výrazná zmena vonkajšej teploty – resetujem PID adaptáciu!");
  }
  // --- ADAPTÍVNE PID podľa logu (efektivita: casNahrievania * spotreba) ---
  if (!pidInit || urovenKurenia != lastUroven) {
    float bestKp = PID_KP;
    float bestKi = PID_KI;
    float bestKd = PID_KD;
    float bestEfektivita = 99999999.0;
    bool pidProfilFound = false;

    for (const auto& z : zaznamy) {
      if (z.uspesnyStart &&
          fabs(z.teplota - tAHT) < 6.0 &&
          z.uroven == urovenKurenia &&
          z.casNahrievania < 90000) // max 90s štart
      {
        float efektivita = z.casNahrievania * z.spotreba;
        if (efektivita < bestEfektivita) {
          bestKp = z.Kp;
          bestKi = z.Ki;
          bestKd = z.Kd;
          bestEfektivita = efektivita;
          pidProfilFound = true;
        }
      }
    }

     pid.SetTunings(bestKp, bestKi, bestKd);

    // Ak sú v logu aj PID pre ventilátor, použijeme ich
    if (zaznamy.size() > 0) {
      const auto& z = zaznamy.back(); // vezmi posledný záznam
      if (z.KpVent > 0 && z.KiVent >= 0 && z.KdVent >= 0) {
        pidVent.SetTunings(z.KpVent, z.KiVent, z.KdVent);
        Serial.printf("🔧 PID ventilátora nastavené: Kp=%.3f Ki=%.3f Kd=%.3f\n",
                      z.KpVent, z.KiVent, z.KdVent);
      }
    }

    Serial.printf("🔧 PID pumpy nastavené: Kp=%.3f Ki=%.3f Kd=%.3f (profil: %s)\n",
                  bestKp, bestKi, bestKd, pidProfilFound ? "log" : "default");
    pidInit = true;
    lastUroven = urovenKurenia;
  }

  // --- DYNAMICKÁ ÚPRAVA PID (raz za 5 minút podľa spriemerovanej odchýlky a spotreby) ---
  static unsigned long lastAutoTune = 0;
  static float odchylkaSum = 0;
  static int odchylkaCnt = 0;
  static float spotrebaSum = 0;
  static int spotrebaCnt = 0;
  const float desiredOdchylka = 1.2;   // cieľová max priemerná odchýlka teploty od setpointu (v °C)
  const float maxSpotrebaLh = 0.12;    // napr. max cieľová spotreba (upraviť podľa tvojich dát)

  // Zber štatistiky:
  odchylkaSum += fabs(tBufik - cieloveTeploty[urovenKurenia - 1]);
  odchylkaCnt++;
  spotrebaSum += momentalnaSpotrebaLh;
  spotrebaCnt++;

  // --- Adaptívne učenie PID: interval meníme podľa počtu logov a teploty ---
  int autotuneInterval = 30000; // začíname s 30s (rýchle učenie)
  int minSamples = 8;  // Počet záznamov pre prepnutie na dlhšie ladenie

  if (zaznamy.size() >= minSamples) autotuneInterval = 2 * 60 * 1000UL; // 2 minúty
  if (zaznamy.size() >= 20) autotuneInterval = 5 * 60 * 1000UL; // 5 minút

  if (!zaznamy.empty()) {
    float teplotaDiff = 0.0;
    for (const auto& z : zaznamy) teplotaDiff += fabs(z.teplota - tAHT);
    teplotaDiff /= zaznamy.size();
    if (teplotaDiff > 8.0) autotuneInterval = 60000; // 1 minúta ak výrazná zmena vonkajšej teploty
  }


  if (millis() - lastAutoTune > autotuneInterval) {
    float avgOdchylka = odchylkaSum / odchylkaCnt;
    float avgSpotreba = spotrebaSum / spotrebaCnt;
    Serial.printf("➕ 5min AVG odchylka: %.2f °C | AVG spotreba: %.3f L/h\n", avgOdchylka, avgSpotreba);

    // Dynamické ladenie (veľmi jemne, meníš v reálnom čase)
    float Kp = pid.GetKp();
    float Ki = pid.GetKi();
    float Kd = pid.GetKd();

    bool upravene = false;
    if (avgOdchylka > desiredOdchylka) {
      Kp += 0.07; // viac sily keď “nestíha” PID
      upravene = true;
    }
    if (avgSpotreba > maxSpotrebaLh && avgOdchylka < (desiredOdchylka / 2)) {
      Kp -= 0.03; // jemne uber, keď je spotreba zbytočne vysoká ale teplota OK
      if (Kp < 0.08) Kp = 0.08;
      upravene = true;
    }
    if (upravene) {
      pid.SetTunings(Kp, Ki, Kd);
      Serial.printf("⚡ Dynamická úprava PID: Kp=%.3f Ki=%.3f Kd=%.3f\n", Kp, Ki, Kd);
    }
    odchylkaSum = 0; odchylkaCnt = 0;
    spotrebaSum = 0; spotrebaCnt = 0;
    lastAutoTune = millis();
  }

  // Ak sa zmení úroveň kúrenia → začni nový prechod
  if (urovenKurenia != lastUroven) {
    prechodVentilator.startHodnota = aktualneVentPWM;
    prechodVentilator.cielovaHodnota = ventilatorPWM[urovenKurenia - 1];
    prechodVentilator.casZaciatku = millis();
    prechodVentilator.aktivny = true;
    lastUroven = urovenKurenia;
  }

  // Vypočítaj a aplikuj plynulý prechod
  aktualneVentPWM = vypocitajPlynuluHodnotu(prechodVentilator);
  ledcWrite(VENT_PWM_CH, aktualneVentPWM);

  float prudVent = merajPrud();
  if (prudVent < 0.15) {
    chybaSystemu = true;
    popisChyby = "Porucha ventilátora počas kúrenia!";
    emergencyShutdown();
    return;
  }
  // (analogicky čerpadlo)
  float prudCerp = merajPrud();
  if (prudCerp < 0.08) { // prah si nastav podľa reality
    chybaSystemu = true;
    popisChyby = "Porucha čerpadla počas kúrenia!";
    emergencyShutdown();
    return;
  }


  if (!kurenieAktivne) {
    pid.SetMode(MANUAL);
    return;
  } else {
    pid.SetMode(AUTOMATIC);
  }

  pidInput = tBufik;
  pidSetpoint = cieloveTeploty[urovenKurenia - 1];
  pid.Compute();

  // --- PID pre ventilátor (nové) ---
  pidVentInput = tBufik; // alebo neskôr EGT, ak pridáš MAX31855
  pidVentSetpoint = cieloveTeploty[urovenKurenia - 1];
  pidVent.Compute();

  // --- Výstupy ---
  // Čerpadlo
  int maxInterval = 1800;
  int minInterval = 222;
  int interval = map((int)pidOutput, 0, 255, maxInterval, minInterval);
  interval = constrain(interval, minInterval, maxInterval);

  static unsigned long lastPulse = 0;
  if (millis() - lastPulse > interval) {
    digitalWrite(PWM_CERPADLO, HIGH);
    delay(50);
    digitalWrite(PWM_CERPADLO, LOW);
    delay(30);
    pulzyZaInterval++;
    lastPulse = millis();
    Serial.printf("💧 Pulz čerpadla | interval: %d ms\n", interval);
  

  // Ventilátor
  int ventPWM = map((int)pidVentOutput, 0, 255, 70, 255);
  ventPWM = constrain(ventPWM, 70, 255);
  aktualneVentPWM = ventPWM;
  ledcWrite(VENT_PWM_CH, aktualneVentPWM);
}

}



//====================================================================================================================== Faza Chladenia======================================================
void fazaChladenia() {
  static unsigned long chladenieZacalo = 0;
  const int cielovePWM = 127; // ~50% PWM (~3000 rpm)
  const float T_VYPNUT = 85.0; // vypnúť až keď klesne na 85 °C

  // Prvý vstup do chladenia → nastav začiatok
  if (chladenieZacalo == 0) {
    chladenieZacalo = millis();
    Serial.println("❄️ Spúšťam dochladzovanie...");
  }

  // Plynulý prechod PWM (každý cyklus inkrementuj/dekrementuj hodnotu o 1 až po cieľovú)
  if (aktualneVentPWM < cielovePWM) aktualneVentPWM++;
  if (aktualneVentPWM > cielovePWM) aktualneVentPWM--;
  ledcWrite(VENT_PWM_CH, aktualneVentPWM);

  // čerpadlo OFF
  digitalWrite(PWM_CERPADLO, LOW);

  // Žhavenie zapni prvých 60 sekúnd chladenia, potom vypni
  if (millis() - chladenieZacalo < 60000) {
    digitalWrite(PWM_ZHAVENIE, HIGH); // aktívne LOW = žhavenie ON
  } else {
    digitalWrite(PWM_ZHAVENIE, LOW);  // žhavenie OFF
  }

  // Ventilátor vypni až keď teplota klesne pod 85 °C
  if (tBufik <= T_VYPNUT) {
    ledcWrite(VENT_PWM_CH, 0);
    aktualnaFaza = FAZA_NEZAPNUTE;
    kurenieAktivne = false;
    chladenieZacalo = 0;
    Serial.println("✅ Bufík dochladený, ventilátor OFF.");
  }
}




//==================================================================================================================== Kontrola plamena ====================================================
void kontrolaPlamena() {
  static unsigned long poslednyCas = 0;
  static float poslednaTeplota = -1000;
  static unsigned long casBezNarastu = 0;
  static int pocetReStartov = 0;

  // Kontrola len pri aktívnom kúrení alebo nahrievaní
  if (aktualnaFaza == FAZA_NAHRIEVANIE || aktualnaFaza == FAZA_KURENIE) {
    if (millis() - poslednyCas >= 3000) { // každé 3 sekundy
      float deltaT = tBufik - poslednaTeplota;
      Serial.printf("📈 Kontrola plameňa: deltaT = %.2f\n", deltaT);

      if (deltaT < 0.5) {  // teplota nerastie dosť
        casBezNarastu += 3000;
      } else {
        casBezNarastu = 0; // resetuj ak stúpa
        pocetReStartov = 0; // reset počítadla pokusov pri úspechu!
      }

      poslednaTeplota = tBufik;
      poslednyCas = millis();
    }

    // Ak teplota nerástla viac než 30 sekúnd počas kúrenia → chyba
    if (casBezNarastu > 30000) {
      if (tBufik > 120.0) {
        // ... len čakáš
        return;
      }
      // Ak bufík vychladol pod 120°C, spusti žhavenie, ale čerpadlo NEVYPÍNAJ!
      if (pocetReStartov < 3) {
        pocetReStartov++;
        Serial.printf("🔄 Automaticky zapinam spiralu, pokus %d/3\n", pocetReStartov);
        popisChyby = "Automatický pokus o zapálenie";

        digitalWrite(PWM_ZHAVENIE, HIGH); // žhavenie ON
        delay(1000); // nech sa žhavenie naštartuje

        float prudZh = merajPrud();
        if (prudZh < 1.0) {
          chybaSystemu = true;
          popisChyby = "Chyba žhavenia pri reštarte plameňa!";
          digitalWrite(PWM_ZHAVENIE, LOW);
          emergencyShutdown();
          return;
        }

        // čerpadlo NEVYPÍNAJ! (ani ventilátor)
        delay(9000); // zvyšných 9s žhavenia (spolu 10s)
        digitalWrite(PWM_ZHAVENIE, LOW);  // žhavenie OFF

        casBezNarastu = 0;
        poslednaTeplota = tBufik;
        poslednyCas = millis();
      } else {
        chybaSystemu = true;
        popisChyby = "Plameň nebol zistený (blokovanie)";
        Serial.println("❌ CHYBA: 3x neúspešný pokus o zapálenie alebo vysoká teplota, blokujem bufík!");
        pocetReStartov = 0;
        emergencyShutdown();
      }
    }

  } else {
    // mimo kúrenia reset
    casBezNarastu = 0;
    poslednaTeplota = tBufik;
    poslednyCas = millis();
  }
}


//==================================================================================================================== testScanWiFi ====================================================
void testScanWiFi() {
  Serial.println("Testujem scan WiFi ...");
  int n = WiFi.scanNetworks();
  Serial.printf("Nájdených sietí: %d\n", n);
  for (int i = 0; i < n; ++i) {
    Serial.printf("%d: %s (RSSI %d)\n", i, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
  }
}

void signalError(int code) {
  Serial.print("Chyba: ");
  Serial.println(code);
  // tu môžeš neskôr doplniť LED/buzzer
}

void enterLockout(int code, const String& message) {
  stavZablokovany = true;
  kurenieAktivne = false;
  Serial.print("LOCKOUT: ");
  Serial.println(message);
  signalError(code);
}

//==================================================================================================================== Emergency Shutdown ====================================================
void emergencyShutdown() {
  esp_task_wdt_delete(NULL);  // deaktivuj watchdog

  // Vypni žhavenie a čerpadlo – ventilátor ostáva bežať v emergencyChladenie!
  digitalWrite(PWM_ZHAVENIE, LOW);
  digitalWrite(PWM_CERPADLO, LOW);

  // Prejdi do FAZA_EMERGENCY_CHLADENIE
  aktualnaFaza = FAZA_EMERGENCY_CHLADENIE;
  kurenieAktivne = false;

  // Zablokuj všetky ďalšie ovládania (handleUI to kontroluje cez aktualnaFaza)
  // ... nič viac nepotrebuješ meniť, logiku máš v emergencyChladenie!

  Serial.println("‼️ Emergency shutdown aktivovaný");
  Serial.println(popisChyby);
}

//==================================================================================================================== EmergencyChladenie ====================================================

void emergencyChladenie() {
  static unsigned long poslednyRefresh = 0;
  const int ventPWM = 200; // alebo 255

  ledcWrite(VENT_PWM_CH, ventPWM);
  digitalWrite(PWM_ZHAVENIE, LOW);
  digitalWrite(PWM_CERPADLO, LOW);

  if (millis() - poslednyRefresh > 500) {
    tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.println("‼ HAVARIA!");
    tft.setTextSize(1);
    tft.setCursor(10, 40);
    tft.println("Chyba:");
    tft.setCursor(10, 55);
    tft.println(popisChyby);
    tft.setCursor(10, 80);
    tft.println("Teplota bufika:");
    tft.setTextSize(2);
    tft.setCursor(10, 100);
    tft.printf("%.1f C", tBufik);
    tft.setTextSize(1);
    tft.setCursor(10, 130);
    if (stavZablokovany) {
      tft.println("LOCKOUT - odblokuj manualne");
    } else {
      tft.println("Cakaj na ochladenie...");
    }
    poslednyRefresh = millis();
  }

  // Po ochladení rozhodni podľa LOCKOUT stavu
  if (tBufik < 80.0) {
    if (stavZablokovany) {
      aktualnaFaza = FAZA_LOCKOUT;   // novy stav – system ostava zablokovaný
      kurenieAktivne = false;
      // nechaj obrazovku s chybou, kým nepríde manuálny reset
    } else {
      // pôvodné správanie
      aktualnaFaza = FAZA_NEZAPNUTE;
      kurenieAktivne = false;
      tft.fillScreen(TFT_BLACK);
      drawIndexScreen();
      chybaSystemu = false;
      popisChyby = "";

    }
  }
}



//==================================================================================================================== merajPrud ====================================================
float merajPrud() {
  const int priemerovanie = 100;
  float suma = 0;
  int validSamples = 0;

  for (int i = 0; i < priemerovanie; i++) {
    float napatie = analogRead(ACS712_PIN) * (ADC_REFERENCIA / ADC_ROZLISENIE);
    if (napatie > 0.1 && napatie < 3.3) {
      float prud = (napatie - offsetNapetie) / ACS712_SENZITIVITA;
      prud *= ACS712_FLIP;
      suma += prud;
      validSamples++;
    }
    delay(1);
  }

  if (validSamples == 0) return 0.0;
  float vysledok = suma / validSamples;

  // AUTOFLIP: ak je prúd stále záporný, otočí smer po každom reštarte
  static bool flipnuty = false;

  // Ak je prúd stále záporný (a je kludovy), zmeň smer aj po reštarte
  if (!flipnuty && abs(vysledok) < 1.0 && vysledok < -0.02) { // hranicu si dolaď podľa šumu (napr. -0.02 A)
    ACS712_FLIP = -1;
    flipnuty = true;
    Serial.println("🛠️ Preklápam smer ACS712 podľa záporného prúdu.");
    vysledok *= -1; // otoč aj aktuálnu hodnotu
  }
  return vysledok;
}

//==================================================================================================================== merajNapetie ====================================================
float merajNapetie() {
  float napatie = analogRead(ACS712_PIN) * (ADC_REFERENCIA / ADC_ROZLISENIE);
  return napatie;
}

void regulatePump() {
  static unsigned long lastPumpOn = 0;

  if (tBufik < TEPLOTA_RYCHLE_START) {
    digitalWrite(PWM_CERPADLO, LOW);
    return;
  }

  int interval = map(tBufik, TEPLOTA_RYCHLE_START, EMERGENCY_SHUTDOWN_TEMP, 300, 3000);
  interval = constrain(interval, 300, 3000);

  if (millis() - lastPumpPulse > interval) {
    digitalWrite(PWM_CERPADLO, HIGH);
    delay(50);
    digitalWrite(PWM_CERPADLO, LOW);
    lastPumpPulse = millis();
    totalPulzy++;

    pulzyZaInterval++; // Prirátaj pulz pre live spotrebu

    if (!isnan(tBufik)) {
      runningAvgTemp = (runningAvgTemp * tempSamples + tBufik) / (tempSamples + 1);
      tempSamples++;
    }
  }
}


//==================================================================================================================== citajCidla ====================================================
void citajCidla() {
  float novaTAHT = bme.readTemperature();  // C Out (BME280 teplota)
  float novaHAHT = bme.readHumidity();     // RH (vlhkosť)
  float novaTlak = bme.readPressure() / 100.0; // tlak v hPa
  float novaTBufik = citajNTCTeplotu(PIN_NTC);     // C Buf NTC

  float prud = merajPrud();
  float napatie = merajNapetie();

  static float poslednaTAHT = -1000;
  static float poslednaHAHT = -1000;
  static float poslednaTBufik = -1000;
  static float poslednyPrud = -1000;
  static float posledneNapatie = -1000;
  static float poslednyTlak = -1000;

  // FILTER na nerealistické hodnoty!
  if (!isnan(novaTBufik) && novaTBufik > -40.0 && novaTBufik < 500.0) {
    tBufik = novaTBufik;
  } else {
    // Serial.print("⚠️ Ignorujem nerealnu hodnotu C Buf: ");
    Serial.println(novaTBufik);
    // nechaj starú hodnotu tBufik
  }




  //==================================================================================================================== Kontrola zmien ====================================================
  bool zmena =
    (abs(novaTAHT - poslednaTAHT) > 0.1) ||
    (abs(novaHAHT - poslednaHAHT) > 0.5) ||
    (abs(novaTBufik - poslednaTBufik) > 0.1) ||
    (abs(prud - poslednyPrud) > 0.05) ||
    (abs(napatie - posledneNapatie) > 0.01) ||
    (abs(novaTlak - poslednyTlak) > 0.1);

  if (zmena) {
    Serial.printf("C B: %.1f°C | C Out: %.1f°C | RH: %.1f%% | PA: %.1f hPa | A: %.2f A\n",
                  novaTBufik, novaTAHT, novaHAHT, novaTlak, prud);

    poslednaTAHT = novaTAHT;
    poslednaHAHT = novaHAHT;
    poslednaTBufik = novaTBufik;
    poslednyPrud = prud;
    posledneNapatie = napatie;
    poslednyTlak = novaTlak;
  }

  // Zapíš do globálnych premenných ak potrebuješ ďalej
  tAHT = novaTAHT;
  hAHT = novaHAHT;
  tlakBME = novaTlak;
  tBufik = novaTBufik;
}


//==================================================================================================================== wifi vyhladavanie ====================================================
void scanWiFiNetworks() {
  foundNetworks = WiFi.scanNetworks();
  Serial.printf("Nájdených sietí: %d\n", foundNetworks);
  for (int i = 0; i < foundNetworks && i < 10; i++) {
    ssidList[i] = WiFi.SSID(i);
    Serial.printf("SSID[%d]: %s\n", i, ssidList[i].c_str());
  }
  selectedWiFi = 0;
}

void openWiFiSelectionScreen() {
  wifiSelectMode = true;
  wifiScanInProgress = true;  // signalizujem, že treba scanovať v ďalšom cykle
  currentPage = PAGE_WIFI_SELECT;

  // Najprv zobrazím informáciu, že vyhľadávam
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 50);
  tft.println("Vyhladavam siete...");
}

void drawWiFiScreen() {
  Serial.printf("Kreslim WiFi obrazovku, siete: %d\n", foundNetworks);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Vyber WiFi:");

  tft.setTextSize(1);
  int y = 40;
  for (int i = 0; i < foundNetworks && i < 5; i++) {
    tft.setCursor(10, y + i * 20);
    if (selectedWiFi == i)
      tft.setTextColor(TFT_BLACK, TFT_WHITE); // zvýrazni vybrané
    else
      tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.print(ssidList[i]);
  }
  // Spat
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, tft.height() - 20);
  tft.print("<- spat");
}

void drawWiFiSelectScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Vyber WiFi");

  tft.setTextSize(1);
  for (int i = 0; i < foundNetworks && i < 5; i++) {
    int index = (selectedWiFi - 2 + i + foundNetworks) % foundNetworks; // Rolovanie
    if (index == selectedWiFi) tft.setTextColor(TFT_BLACK, TFT_WHITE);
    else tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(10, 40 + i * 20);
    tft.print(ssidList[index]);
  }
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, tft.height() - 20);
  tft.print("<- spat");
}

const char* abeceda = "aAbBcCdDeEfFgGhHiIjJkKlLmMnNoOpPqQrRsStTuUvVwWxXyYzZ0123456789*/+-.";
int aktualnyZnak = 0;

void drawWiFiPasswordScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.print("Zadaj heslo:");

  tft.setTextSize(2);
  tft.setCursor(10, 40);
  tft.print(wifiPassword);

  tft.setTextSize(2);
  tft.setCursor(10, 70);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print(abeceda[aktualnyZnak]);

  tft.setTextSize(1);
  tft.setCursor(10, tft.height() - 20);
  tft.print("[Klik]=Prida, [Dlhy]=Zmaz, [Dvojklik]=Uloz");
}

// vypočet Spotrebu Na 8Hodin

float vypocitajSpotrebuNa8Hodin(float objemNadrzeLitrov) {
  if (zaznamy.empty()) return -1;

  unsigned long teraz = millis() / 1000;
  unsigned long casovyLimit = 3600;  // posledná hodina
  float spotrebaSpolu = 0;
  int pocet = 0;

  for (int i = zaznamy.size() - 1; i >= 0; --i) {
    if (teraz - zaznamy[i].cas > casovyLimit) break;
    spotrebaSpolu += zaznamy[i].spotreba;
    pocet++;
  }

  if (pocet == 0) return -1;

  float priemerZaHodinu = spotrebaSpolu / pocet * 3600.0f;  // konverzia z priemerného záznamu na hodinu
  float predpovedNa8h = priemerZaHodinu * 8;

  Serial.printf("🔍 Odhadovaná spotreba na 8h: %.2f L\n", predpovedNa8h);
  if (predpovedNa8h > objemNadrzeLitrov) {
    Serial.println("⚠️ Upozornenie: NEDOSTATOK nafty na 8 hodín!");
  } else {
    Serial.println("✅ Dostatočný objem nafty.");
  }

  return predpovedNa8h;
}

// Spotreba

void updateSpotrebaStats() {
  if (pumpOn) {
    totalPulzy++;
    totalSpotreba += SPOTREBA_NA_PULZ;
  }
  if (!isnan(tBufik)) {
    runningAvgTemp = (runningAvgTemp * tempSamples + tBufik) / (tempSamples + 1);
    tempSamples++;
  }
}

void saveSpotrebaStats() {
  spotrebaHistory[currentSpotrebaIndex].duration = (millis() - procesStart) / 1000;
  spotrebaHistory[currentSpotrebaIndex].spotreba = totalSpotreba;
  spotrebaHistory[currentSpotrebaIndex].priemernaTeplota = runningAvgTemp;
  spotrebaHistory[currentSpotrebaIndex].pulzy = totalPulzy;
  currentSpotrebaIndex = (currentSpotrebaIndex + 1) % 3;
  totalPulzy = 0;
  totalSpotreba = 0;
  runningAvgTemp = 0;
  tempSamples = 0;
}



// ====== Kreslenie stránky Index ======
void drawIndexScreen() {
  tft.fillScreen(TFT_BLACK); // Vymaže displej (čierna farba pozadia)

  // Nakreslí biely rám okolo displeja
  tft.drawFastHLine(0, 0, tft.width(), TFT_WHITE);                   // horný okraj
  tft.drawFastHLine(0, tft.height() - 1, tft.width(), TFT_WHITE);    // dolný okraj
  tft.drawFastVLine(0, 0, tft.height(), TFT_WHITE);                  // ľavý okraj
  tft.drawFastVLine(tft.width() - 1, 0, tft.height(), TFT_WHITE);    // pravý okraj

  // Nový riadok hore na WiFi stav
  tft.setTextSize(1);
  tft.setCursor(10, 10);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  if (WiFi.status() == WL_CONNECTED) {
    tft.print("WiFi: ");
    tft.print(WiFi.SSID());
  } else {
    tft.print("WiFi: Nie pripojene");
  }

  const int MENU_VISIBLE = 3; // Maximálne 3 riadky menu naraz
  int menuCount = sizeof(menuItems) / sizeof(menuItems[0]); // Koľko položiek je v menu

  // Vypočíta, ktorý index bude ako prvý na obrazovke
  int first = indexSelect - 1;           // Vybraný riadok bude v strede (ak je to možné)
  if (first < 0) first = 0;              // Ak sme na začiatku menu
  if (first > menuCount - MENU_VISIBLE) first = menuCount - MENU_VISIBLE; // Ak sme na konci menu
  if (first < 0) first = 0;              // Ak máme menej ako 3 položky

  tft.setTextSize(2); // Nastaví veľkosť písma pre menu

  // Posunutie menu o 10 px nižšie, aby nebolo pod WiFi riadkom
  for (int i = 0; i < MENU_VISIBLE; i++) {
    int item = first + i;
    if (item >= menuCount) break;        // Ak sme už za poslednou položkou
    tft.setCursor(10, 25 + i * 25);      // Posunutie z 15 na 25 v y osi

    // Zvýrazní vybranú položku (biele pozadie, čierny text)
    if (item == indexSelect) {
      tft.setTextColor(TFT_BLACK, TFT_WHITE);
    } else {
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
    }
    if (item == 0) {
      // Dynamický prvý riadok menu podľa stavu kúrenia
      if (kurenieAktivne)
        tft.print("Vypnut kurenie");
      else
        tft.print("Start kurenia");
    } else {
      tft.print(menuItems[item]);
    }
  }

  // Posunutie spodného riadku o 15 px vyššie
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(10, tft.height() - 15);
  tft.print("faza: ");
  switch (aktualnaFaza) {
    case FAZA_NAHRIEVANIE: tft.print("NAHRIEVANIE"); break;
    case FAZA_KURENIE: tft.print("KURENIE"); break;
    case FAZA_CHLADENIE: tft.print("CHLADENIE"); break;
    default: tft.print("VYPNUTE"); break;
  }
  tft.setCursor(tft.width() / 2, tft.height() - 15);
  tft.print("out: ");
  tft.print(tAHT, 1);
  tft.print("C ");
  tft.print(momentalnaSpotrebaLh, 2);
  tft.print("L/h");
}

// ====== Kreslenie stránky Spotreba ======
void drawSpotrebaScreen() {
  tft.fillScreen(TFT_BLACK);

  // Ram
  tft.drawFastHLine(0, 0, tft.width(), TFT_WHITE);
  tft.drawFastHLine(0, tft.height() - 1, tft.width(), TFT_WHITE);
  tft.drawFastVLine(0, 0, tft.height(), TFT_WHITE);
  tft.drawFastVLine(tft.width() - 1, 0, tft.height(), TFT_WHITE);

  // Nadpis
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(10, 10);
  tft.print("Spotreba");

  // Hodnoty
  tft.setTextSize(1);
  tft.setCursor(10, 40);
  tft.print("Aktualna:   ");
  tft.print(merajPrud() * 220 / 1000, 2);
  tft.print(" L/h");

  tft.setCursor(10, 60);
  tft.print("Posledne kurenie:");

  tft.setCursor(20, 80);
  tft.print("- Trvanie:   ");
  tft.print(spotrebaHistory[0].duration / 60);
  tft.print(" min");

  tft.setCursor(20, 100);
  tft.print("- Spotreba:  ");
  tft.print(spotrebaHistory[0].spotreba, 2);
  tft.print(" L");

  tft.setCursor(20, 120);
  tft.print("- Priemer:   ");
  tft.print(spotrebaHistory[0].spotreba / (spotrebaHistory[0].duration / 3600.0), 2);
  tft.print(" L/h");

  // Tlacidlo spat
  tft.setCursor(tft.width() - 50, tft.height() - 20);
  tft.print("<- spat");
}

// ====== Kreslenie stránky Info ======
void drawInfoScreen() {
  tft.fillScreen(TFT_BLACK);

  // Ram
  tft.drawFastHLine(0, 0, tft.width(), TFT_WHITE);
  tft.drawFastHLine(0, tft.height() - 1, tft.width(), TFT_WHITE);
  tft.drawFastVLine(0, 0, tft.height(), TFT_WHITE);
  tft.drawFastVLine(tft.width() - 1, 0, tft.height(), TFT_WHITE);

  // Nadpis
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(10, 10);
  tft.print("Info");

  // Hodnoty
  tft.setTextSize(1);
  tft.setCursor(10, 30);
  tft.print("Teplota vonkajsia: ");
  tft.print(tAHT, 1);
  tft.print("C");

  tft.setCursor(10, 45);
  tft.print("Vlhkost vonkajsia: ");
  tft.print(hAHT, 1);
  tft.print("%");

  tft.setCursor(10, 60);
  tft.print("Tlak : ");
  tft.print(tlakBME, 1);
  tft.print(" hPa");

  tft.setCursor(10, 75);
  tft.print("Teplota bufik:    ");
  tft.print(tBufik, 1);
  tft.print("C");

  tft.setCursor(10, 90);
  tft.print("Teplota NTC:      ");
  tft.print(tNTC, 1);
  tft.print("C");


  tft.setCursor(10, 100);
  tft.print("Teplota interier: -- C");

  tft.setCursor(10, 115);
  tft.print("Vlhk. interier:   -- %");

  // Tlacidlo spat
  tft.setCursor(tft.width() - 50, tft.height() - 20);
  tft.print("<- spat");
}

// ====== Kreslenie stránky Start Kurenie ======
void drawStartKurenieScreen() {
  tft.fillScreen(TFT_BLACK);

  // Ram
  tft.drawFastHLine(0, 0, tft.width(), TFT_WHITE);
  tft.drawFastHLine(0, tft.height() - 1, tft.width(), TFT_WHITE);
  tft.drawFastVLine(0, 0, tft.height(), TFT_WHITE);
  tft.drawFastVLine(tft.width() - 1, 0, tft.height(), TFT_WHITE);

  // Nadpis
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(10, 10);
  tft.print(kurenieAktivne ? "Vypnut kurenie" : "Start kurenia");

  // Potvrdenie
  tft.setTextSize(2);
  tft.setCursor((tft.width() - 200) / 2, tft.height() / 2 - 10);
  tft.print(kurenieAktivne ? "Potvrdit vypnutie" : "Potvrdit start");

  // Tlacidlo spat
  tft.setTextSize(1);
  tft.setCursor(tft.width() - 50, tft.height() - 20);
  tft.print("<- spat");
}

// ====== Kreslenie stránky Nastavenie urovne ======
void drawNastavenieUrovneScreen() {
  tft.fillScreen(TFT_BLACK);

  // Rámik
  tft.drawFastHLine(0, 0, tft.width(), TFT_WHITE);
  tft.drawFastHLine(0, tft.height() - 1, tft.width(), TFT_WHITE);
  tft.drawFastVLine(0, 0, tft.height(), TFT_WHITE);
  tft.drawFastVLine(tft.width() - 1, 0, tft.height(), TFT_WHITE);

  // Nadpis
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(10, 10);
  tft.print("Nastavenie urovne");

  // Úroveň (veľké číslo)
  tft.setTextSize(3);
  tft.setCursor(tft.width() / 2 - 15, tft.height() / 2 - 15);
  if (!jeOznaceneSpat) {
    tft.print(urovenKurenia); // Zobraz aktuálnu úroveň (1-10)
  }

  // "SPAŤ" v pravom dolnom rohu
  tft.setTextSize(1);
  tft.setCursor(tft.width() - 50, tft.height() - 20);
  if (jeOznaceneSpat) {
    tft.setTextColor(TFT_BLACK, TFT_WHITE); // Zvýraznené
    tft.print("SPAT");
  } else {
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // Normálne
    tft.print("SPAT");
  }
}

// ====== Kreslenie stránky Predpoveď počasia ======
void drawPredpovedScreen() {
  tft.fillScreen(TFT_BLACK);

  // Vykreslenie rámika
  tft.drawFastHLine(0, 0, tft.width(), TFT_WHITE);
  tft.drawFastHLine(0, tft.height() - 1, tft.width(), TFT_WHITE);
  tft.drawFastVLine(0, 0, tft.height(), TFT_WHITE);
  tft.drawFastVLine(tft.width() - 1, 0, tft.height(), TFT_WHITE);

  // Nadpis
  tft.setCursor(10, 10);
  tft.print("Predpoved pocasia");

  // Hodnoty
  tft.setCursor(10, 50);
  tft.print("Min. teplota: ");
  tft.print(predpovedMinTeplota, 1);
  tft.print(" C");

  tft.setCursor(10, 70);
  tft.print("Max. teplota: ");
  tft.print(predpovedMaxTeplota, 1);
  tft.print(" C");

  tft.setCursor(10, 90);
  tft.print("Stav: ");
  tft.print(predpovedStavy[predpovedStavIndex]);

  // Šípka späť
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(tft.width() - 50, tft.height() - 20);
  tft.print("<- spat");
}

// ====== Kreslenie stránky Nastavenie ======
void drawSettingsScreen() {
  tft.fillScreen(TFT_BLACK);

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 10);
  tft.print("Nastavenia");

  tft.setTextSize(1);
  int y = 40;
  for (int i = 0; i < POCET_NASTAVENI; i++) {
    tft.setCursor(10, y);

    // Objem nádrže
    if (i == SET_NADRZ) {
      if (aktualneNastavenie == i && !editMode) {
        // Zvýrazni celý riadok mimo editácie
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        tft.print("Objem nadrze: ");
        tft.print(objemNadrze, 1);
        tft.print(" L (2-20)");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
      } else if (aktualneNastavenie == i && editMode) {
        // V editMode zvýrazni LEN číslo, zvyšok biely
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.print("Objem nadrze: ");
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        tft.print(objemNadrze, 1);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.print(" L (2-20)");
      } else {
        // Bežný riadok, nie je vybraný
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.print("Objem nadrze: ");
        tft.print(objemNadrze, 1);
        tft.print(" L (2-20)");
      }
    } else {
      // Ostatné riadky (WiFi, Natankovane)
      if (aktualneNastavenie == i && !editMode)
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
      else
        tft.setTextColor(TFT_WHITE, TFT_BLACK);

      switch (i) {
        case SET_WIFI:
          tft.print("WiFi siet: ");
          tft.print("…");
          break;
        case SET_NATANKOVANE:
          tft.print("Natankovane");
          break;
      }
    }
    y += 25;
  }

  // Aktuálny stav nádrže pod menu
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, y + 5);
  tft.print("Aktualny stav: ");
  tft.print(aktualnyStavNadrze, 2);
  tft.print(" L");

  // Späť (v pravom dolnom rohu)
  if (aktualneNastavenie == POCET_NASTAVENI)
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
  else
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.setCursor(tft.width() - 50, tft.height() - 20);
  tft.print("SPAT");
}


void handleUI() {

  //================================tlacidka=====================================

  int move = 0;
  bool click = false;

  static bool lastBtnUp = false;
  static bool lastBtnDown = false;

  bool btnUp = digitalRead(BTN_UP) == LOW;
  bool btnDown = digitalRead(BTN_OK) == LOW;

  unsigned long btnOkPressedTime = 0;
  unsigned long btnOkLastRelease = 0;
  bool btnOkWasDown = false;
  bool btnOkLongPressDetected = false;
  int btnOkClickCount = 0;


  // Horné tlačidlo: pohyb v menu
  if (btnUp && !lastBtnUp) {
    move = 1;  // alebo -1 ak chceš meniť smer
  }

  // Spodné tlačidlo: potvrdenie/potvrdiť výber
  if (btnDown && !lastBtnDown) {
    click = true;
  }

  lastBtnUp = btnUp;
  lastBtnDown = btnDown;
  // Uložiť predchádzajúce stavy:

  switch (currentPage) {
    case PAGE_INDEX: {
        int menuCount = sizeof(menuItems) / sizeof(menuItems[0]);
        if (move != 0) {
          indexSelect = (indexSelect + move + menuCount) % menuCount;
          drawIndexScreen();
        }
        if (click) {
          switch (indexSelect) {
            case 0:
              if (kurenieAktivne) {
                // Už kúriš -> možnosť vypnúť
                currentPage = PAGE_START_KURENIE;
                drawStartKurenieScreen();
              } else {
                // Nekúriš -> možnosť spustiť
                currentPage = PAGE_NASTAVENIE_UROVNE;
                drawNastavenieUrovneScreen();
              }
              break;
            case 1: currentPage = PAGE_PREDPOVED; drawPredpovedScreen(); break;
            case 2: currentPage = PAGE_SPOTREBA; drawSpotrebaScreen(); break;
            case 3: currentPage = PAGE_INFO; drawInfoScreen(); break;
            case 4: currentPage = PAGE_NASTAVENIA; drawSettingsScreen(); break;
          }
        }
      } break;

    case PAGE_NASTAVENIE_UROVNE:
      if (move == 1) { // Tlačidlo HORE
        if (!jeOznaceneSpat) {
          if (urovenKurenia < 10) {
            urovenKurenia++;
          } else {
            jeOznaceneSpat = true; // Po 10 označ "SPAŤ"
          }
        } else {
          jeOznaceneSpat = false; // Po "SPAŤ" ide na 1
          urovenKurenia = 1;
        }
        drawNastavenieUrovneScreen();
      }
      if (click) {
        if (jeOznaceneSpat) {
          currentPage = PAGE_INDEX; // Návrat do menu
          drawIndexScreen();
        } else {
          // Ulož a potvrď úroveň
          prefs.begin("kurenie", false);
          prefs.putInt("lastLevel", urovenKurenia);
          prefs.end();

          // --------- BEZPEČNOSTNÉ KONTROLY PRI SPUSTENÍ KÚRENIA ----------
          // if (millis() - posledneVypnuteCas < 60000) {
          //   unsigned long zostava = (60000 - (millis() - posledneVypnuteCas)) / 1000;
          // tft.fillScreen(TFT_BLACK);
          //tft.setTextColor(TFT_RED);
          //tft.setTextSize(2);
          //tft.setCursor(10, 40);
          //tft.println("Start zablokovany!");
          //tft.setTextSize(1);
          //tft.setCursor(10, 70);
          //tft.print("Pockaj: ");
          //tft.print(zostava);
          //tft.println(" s");
          //delay(2000);
          //currentPage = PAGE_INDEX;
          //drawIndexScreen();
          //return;
          //}

          if (isnan(tBufik)) {
            chybaSystemu = true;
            popisChyby = "Chyba senzora bufik!";
            // emergencyShutdown(); // <-- ZMAŽ alebo ZAKOMENTUJ TOTO!
            // ---- ZOBRAZ CHYBU NA DISPLEJI, ČAKAJ NA POTVRDENIE: ----
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_RED);
            tft.setTextSize(2);
            tft.setCursor(10, 30);
            tft.println("Chyba:");
            tft.setTextSize(2);
            tft.setCursor(10, 60);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.println(popisChyby);

            tft.setTextColor(TFT_WHITE, TFT_RED);
            tft.setTextSize(1);
            tft.setCursor(10, 120);
            tft.println("Stlac OK pre navrat");

            while (digitalRead(BTN_OK) == HIGH) delay(10);
            delay(300);
            chybaSystemu = false;
            popisChyby = "";
            currentPage = PAGE_INDEX;
            drawIndexScreen();
            return;
          }

          if (isnan(tAHT) || isnan(hAHT)) {
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_RED);
            tft.setTextSize(2);
            tft.setCursor(10, 40);
            tft.println("Chyba AHT senzora!");
            delay(2000);
            currentPage = PAGE_INDEX;
            drawIndexScreen();
            return;
          }

          // RESETUJ CHYBY PRED KONTROLOU!
          chybaSystemu = false;
          popisChyby = "";

          Serial.println("🟢 Spúšťam kontroly pred kúrením...");
          kontrolaVentilatora();
          if (!chybaSystemu) kontrolaCerpadla();
          if (!chybaSystemu) kontrolaZhavenia();

          if (!chybaSystemu) {
            Serial.println("✅ Všetky kontroly prebehli úspešne!");
            aktualnaFaza = FAZA_NAHRIEVANIE;
            casZaciatkuFazy = millis();
            kurenieAktivne = true;
            currentPage = PAGE_INDEX;
            drawIndexScreen();
          } else {
            // UPOZORNENIE (NIE emergencyShutdown!)
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_RED);
            tft.setTextSize(2);
            tft.setCursor(10, 30);
            tft.println("Chyba: ");
            tft.setTextSize(1);
            tft.setCursor(10, 60);
            tft.println(popisChyby);
            tft.setCursor(10, 120);
            tft.setTextColor(TFT_WHITE, TFT_RED);
            tft.setTextSize(1);
            tft.println("Stlac OK pre navrat");
            // Čakaj kým užívateľ stlačí tlačidlo
            while (digitalRead(BTN_OK) == HIGH) delay(10);
            delay(300); // debounce
            // Vráť menu, resetuj chybu
            chybaSystemu = false;
            popisChyby = "";
            currentPage = PAGE_INDEX;
            drawIndexScreen();
            return;
          }


        }

        break;


      case PAGE_PREDPOVED:
      case PAGE_SPOTREBA:
      case PAGE_INFO:
        if (click) {
          currentPage = PAGE_INDEX;
          drawIndexScreen();
        }
        break;

      case PAGE_START_KURENIE:
        if (click) {
          aktualnaFaza = FAZA_CHLADENIE;
          casZaciatkuFazy = millis();
          kurenieAktivne = false;
          posledneVypnuteCas = millis();
          currentPage = PAGE_INDEX;
          drawIndexScreen();
        }
        break;

      case PAGE_WIFI_SELECT:
        if (move != 0) {
          selectedWiFi += move;
          if (selectedWiFi < 0) selectedWiFi = foundNetworks; // posledný riadok = späť
          if (selectedWiFi > foundNetworks) selectedWiFi = 0;
          drawWiFiScreen();
        }
        if (click) {
          if (selectedWiFi == foundNetworks) { // späť
            currentPage = PAGE_NASTAVENIA;
            drawSettingsScreen();
          } else {
            currentPage = PAGE_WIFI_PASSWORD;
            wifiPassword = "";
            aktualnyZnak = 0;
            drawWiFiPasswordScreen();
          }
        }
        break;

      case PAGE_WIFI_PASSWORD: {
          // Výber znaku hore/dole
          if (move != 0) {
            aktualnyZnak = (aktualnyZnak + move + strlen(abeceda)) % strlen(abeceda);
            drawWiFiPasswordScreen();
          }

          // Pridanie znaku
          if (click) {
            wifiPassword += abeceda[aktualnyZnak];
            drawWiFiPasswordScreen();
          }

          // MAZANIE znaku: dlhé podržanie BTN_OK
          static unsigned long btnDownStart = 0;
          static bool btnDownLast = false;
          bool btnDownNow = digitalRead(BTN_OK) == LOW;
          if (btnDownNow && !btnDownLast) {
            btnDownStart = millis();
          }
          if (!btnDownNow && btnDownLast) {
            btnDownStart = 0;
          }
          if (btnDownNow && btnDownStart && (millis() - btnDownStart > 800)) {
            if (wifiPassword.length() > 0) {
              wifiPassword.remove(wifiPassword.length() - 1);
              drawWiFiPasswordScreen();
              delay(200);
            }
            btnDownStart = millis();
          }
          btnDownLast = btnDownNow;

          // ULOŽENIE HESLA: DLHÝ UP (BTN_UP)
          static unsigned long btnUpStart = 0;
          static bool btnUpLast = false;
          bool btnUpNow = digitalRead(BTN_UP) == LOW;
          if (btnUpNow && !btnUpLast) {
            btnUpStart = millis();
          }
          if (!btnUpNow && btnUpLast) {
            btnUpStart = 0;
          }
          if (btnUpNow && btnUpStart && (millis() - btnUpStart > 1000)) {
            Serial.print("Ukladam heslo pre WiFi: ");
            Serial.println(wifiPassword);
            prefs.begin("wifi", false);
            prefs.putString("ssid", ssidList[selectedWiFi]);
            prefs.putString("pass", wifiPassword);
            prefs.end();

            // WiFi.begin(ssidList[selectedWiFi].c_str(), wifiPassword.c_str());
            currentPage = PAGE_WIFI_SELECT;
            drawWiFiScreen();
            btnUpStart = 0;
            delay(400);  // Debounce
          }
          btnUpLast = btnUpNow;

          break;
        }



      case PAGE_NASTAVENIA:
        if (editMode) {
          if (aktualneNastavenie == SET_NADRZ && move != 0) {
            objemNadrze += move * 0.5;
            if (objemNadrze > 20.0) objemNadrze = 2.0;
            if (objemNadrze < 2.0) objemNadrze = 20.0;
            aktualnyStavNadrze = objemNadrze;
            drawSettingsScreen();
          }
          if (click) {
            editMode = false;
            drawSettingsScreen();
          }
        } else {
          // --- POHYB ---
          if (move != 0) {
            if (aktualneNastavenie < POCET_NASTAVENI) {
              aktualneNastavenie += move;
              if (aktualneNastavenie > POCET_NASTAVENI) aktualneNastavenie = 0;
              if (aktualneNastavenie == POCET_NASTAVENI) {
                // Označiť SPAT
                drawSettingsScreen(); // vykresli SPAT zvýraznené
                return;
              }
            } else {
              // Sme na "SPAT" a ideme ďalej → späť na prvú položku
              aktualneNastavenie = 0;
            }
            drawSettingsScreen();
          }
          // --- POTVRDENIE ---
          if (click) {
            if (aktualneNastavenie == POCET_NASTAVENI) {
              // SPAT stlačené → návrat na index
              currentPage = PAGE_INDEX;
              drawIndexScreen();
              return;
            }
            switch (aktualneNastavenie) {
              case SET_NADRZ:
                editMode = true;
                drawSettingsScreen();
                break;

              case SET_WIFI:
                currentPage = PAGE_WIFI_SELECT;
                wifiScanInProgress = true;
                tft.fillScreen(TFT_BLACK);
                tft.setTextSize(2);
                tft.setCursor(10, 50);
                tft.print("Vyhladavam siete...");
                break;

              case SET_NATANKOVANE:
                aktualnyStavNadrze = objemNadrze;
                posledneDoliatieCas = millis();
                drawSettingsScreen();
                break;
            }
          }
        }
      }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Inicializacia systemu...");
  //  delay(5000);
  // Nastav tvoju kľudovú spotrebu v ampéroch (napr. 0.12 A, odmeraj multimetrom)
  const float PRUD_NAPRAZDNO = 0.002;

  // --- Dynamická kalibrácia offsetu po štarte
  const int samples = 800;
  float sumaNapatie = 0;
  for (int i = 0; i < samples; i++) {
    float napatie = analogRead(ACS712_PIN) * (ADC_REFERENCIA / ADC_ROZLISENIE);
    sumaNapatie += napatie;
    delay(1);
  }
  offsetNapetie = sumaNapatie / samples;
  offsetNapetie += PRUD_NAPRAZDNO * ACS712_SENZITIVITA;  // Pripočítaj idle prúd

  float test = merajPrud();

  Serial.print("Offset napätie (s posunom): ");
  Serial.println(offsetNapetie, 3);


  Wire.begin(22, 21); // explicitne nastaviť piny

  analogReadResolution(12);
  pinMode(ACS712_PIN, INPUT);
  pinMode(PWM_ZHAVENIE, OUTPUT);
  pinMode(PWM_CERPADLO, OUTPUT);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_OK, INPUT_PULLUP);

  ledcSetup(VENT_PWM_CH, 10000, 8);
  ledcAttachPin(PWM_VENTILATOR, VENT_PWM_CH);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(1);

  if (!bme.begin(0x76, &Wire)) {
    Serial.println("BME280 na 0x76 nenájdený, skúšam 0x77...");
    if (!bme.begin(0x77, &Wire)) {
      Serial.println("Chyba inicializacie BME280 senzora!");
      while (1) delay(1000);
    }
  }
  Serial.println("BME280 inicializovany");



  aktualnaFaza = FAZA_NEZAPNUTE;
  kurenieAktivne = false;
  digitalWrite(PWM_ZHAVENIE, LOW);
  digitalWrite(PWM_CERPADLO, LOW);
  ledcWrite(VENT_PWM_CH, 0);
  drawIndexScreen();

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(MIN_VENT_OTACKY, 255);

  prefs.begin("kurenie", false);
  urovenKurenia = prefs.getInt("lastLevel", 1); // Default 1
  prefs.end();

  Serial.println("System inicializovany a pripraveny");

  //inicializáciu SPIFFS: tabulky
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ SPIFFS sa nepodarilo spustiť");
  } else {
    Serial.println("✅ SPIFFS inicializované");
  }
  nacitajLogyZoSPIFFS();
  if (zaznamy.size() > 0) {
    ZaznamLogu& posledny = zaznamy.back();
    pid.SetTunings(posledny.Kp, posledny.Ki, posledny.Kd);
    Serial.printf("PID z logu: Kp=%.4f Ki=%.4f Kd=%.4f\n", posledny.Kp, posledny.Ki, posledny.Kd);
  }

  prefs.begin("wifi", true);
  String ulozeneSSID = prefs.getString("ssid", "");
  String ulozenePASS = prefs.getString("pass", "");
  prefs.end();

  if (ulozeneSSID.length() > 0 && ulozenePASS.length() > 0) {
    Serial.print("Pripajam sa na ulozenu WiFi: ");
    Serial.println(ulozeneSSID);
    WiFi.begin(ulozeneSSID.c_str(), ulozenePASS.c_str());
    // tu môžeš ešte spraviť nejaké timeouty/wait/yield, alebo signalizovať stav na displeji
  }
}



void loop()  {

  static int poslednaFaza = -1;
  static bool posledneKurenieAktivne = false;

  // Vypíš len keď sa niečo zmení
  if (aktualnaFaza != poslednaFaza || kurenieAktivne != posledneKurenieAktivne) {
    Serial.print("Aktualna faza: "); Serial.print(aktualnaFaza);
    Serial.print(" | Kurenie aktivne: "); Serial.println(kurenieAktivne);
    poslednaFaza = aktualnaFaza;
    posledneKurenieAktivne = kurenieAktivne;
  }

  if (tBufik >= EMERGENCY_SHUTDOWN_TEMP) {
    emergencyShutdown();
  }

  if (millis() - lastTeplotaUpdate > 2000) { // Zvýšený interval na 2000 ms
    citajCidla();
    updateSpotrebaStats();

    tNTC = citajNTCTeplotu(13);
    kontrolaPlamena();
    plynulyPrechodVentilatora();

    // Každých 5 sekúnd vypočíta aktuálnu spotrebu
    if (millis() - poslednyResetPulzov > INTERVAL_MERANIA) {
      if (kurenieAktivne) {
        float pulzyZaHodinu = (float)pulzyZaInterval * (3600000.0 / INTERVAL_MERANIA);
        momentalnaSpotrebaLh = pulzyZaHodinu * SPOTREBA_NA_PULZ;
      } else {
        momentalnaSpotrebaLh = 0.0;
      }
      pulzyZaInterval = 0;
      poslednyResetPulzov = millis();
    }

    if (wifiScanInProgress && currentPage == PAGE_WIFI_SELECT) {
      wifiScanInProgress = false;
      scanWiFiNetworks();
      drawWiFiScreen();
    }

    // SPIFFS logovanie a výpočet spotreby každých 30 minút
    if (millis() - poslednyLogCas > LOG_INTERVAL) {
      logujDataDoSPIFFS();
      poslednyLogCas = millis();

      float objemNadrze = 5.0;  // napr. 5 litrová nádrž
      nacitajLogyZoSPIFFS();
      float odhad = vypocitajSpotrebuNa8Hodin(objemNadrze);
    }

    lastTeplotaUpdate = millis();

    if (currentPage == PAGE_INDEX) {
      drawIndexScreen();
    }
    if (currentPage == PAGE_INFO) {
      drawInfoScreen();
    }
  }

  // A1: okamžitá reakcia na prehriatie – pred spracovaním stavov
  if (tBufik >= TEPLOTA_MAX && aktualnaFaza != FAZA_EMERGENCY_CHLADENIE) {
    // okamžite vypni žhavenie a čerpadlo, nechaj bežať ventilátor
    digitalWrite(PWM_ZHAVENIE, LOW);
    digitalWrite(PWM_CERPADLO, LOW);
    ledcWrite(VENT_PWM_CH, MIN_VENT_OTACKY);   // alebo vyššie, podľa tvojho minima
    emergencyStartMs = millis();
    aktualnaFaza = FAZA_EMERGENCY_CHLADENIE;
  }

  switch (aktualnaFaza) {
    case FAZA_NAHRIEVANIE: {
        if (stavZablokovany) {
          // A3: ak je blokované, nič nenaštartuj
          ledcWrite(VENT_PWM_CH, 0);
          digitalWrite(PWM_CERPADLO, LOW);
          digitalWrite(PWM_ZHAVENIE, LOW);
          break;
        }

        if (kurenieAktivne) {
          fazaNahrievania();
          // A2: ak detegujeme plameň, resetni čas a počítadlo
          if (plamenDetekovany) {
            plamenNaposledy = millis();
            restartPokusy = 0; // úspešný nábeh resetuje pokusy
          }
        }
        break;
      }

    case FAZA_KURENIE: {
        if (stavZablokovany) {
          ledcWrite(VENT_PWM_CH, 0);
          digitalWrite(PWM_CERPADLO, LOW);
          digitalWrite(PWM_ZHAVENIE, LOW);
          break;
        }

        if (kurenieAktivne) {
          fazaKurenia();

          // A2: sledovanie plameňa počas kúrenia
          if (plamenDetekovany) {
            plamenNaposledy = millis();
          } else {
            // bez plameňa dlhšie ako PLAMEN_TIMEOUT_MS -> považuj za zhasnutie
            if (millis() - plamenNaposledy > PLAMEN_TIMEOUT_MS) {
              // vypni horák – čerpadlo a žhavenie
              digitalWrite(PWM_CERPADLO, LOW);
              digitalWrite(PWM_ZHAVENIE, LOW);

              if (restartPokusy < MAX_RESTART_POKUSY) {
                restartPokusy++;
                // krátke odvetranie a opäť nábeh
                ledcWrite(VENT_PWM_CH, MIN_VENT_OTACKY);
                delay(1000); // krátky purge; ak máš neblokujúcu verziu, použi ju
                // prechod späť na náhrievanie = auto-reštart
                aktualnaFaza = FAZA_NAHRIEVANIE;
              } else {
                // A3: po neúspešných pokusoch blokuj a signalizuj chybu
                stavZablokovany = true;
                // signalizáciu (buzzer/LED/displej) rieš v samostatnej funkcii, ktorú už voláš v loop()
                kurenieAktivne = false;
                // pre istotu spusti chladenie komory
                ledcWrite(VENT_PWM_CH, MIN_VENT_OTACKY);
                aktualnaFaza = FAZA_CHLADENIE;
              }
            }
          }
        }
        break;
      }

    case FAZA_CHLADENIE: {
        fazaChladenia();
        // voliteľne: ak chceš, ukonči chladenie podľa TEPLOTA_BEZPECNA
        break;
      }

    case FAZA_EMERGENCY_CHLADENIE: {
        // A1: nechaj ventilátor bežať, kým teplota neklesne alebo neuplynie minimálny čas
        ledcWrite(VENT_PWM_CH, MIN_VENT_OTACKY);
        digitalWrite(PWM_CERPADLO, LOW);
        digitalWrite(PWM_ZHAVENIE, LOW);

        bool casUplynul = (millis() - emergencyStartMs) >= DOCHLADENIE_MS;
        bool teplotaOK   = tBufik <= TEPLOTA_BEZPECNA;

        if (casUplynul || teplotaOK) {
          ledcWrite(VENT_PWM_CH, 0);
          // po núdzovom dochladení vráť zariadenie do bezpečného stavu
          kurenieAktivne = false;
          aktualnaFaza = FAZA_NEZAPNUTE;
        }
        break;
      }

    case FAZA_NEZAPNUTE:
    default: {
        ledcWrite(VENT_PWM_CH, 0);
        digitalWrite(PWM_CERPADLO, LOW);
        digitalWrite(PWM_ZHAVENIE, LOW);
        break;
      }
  }
  handleUI();
}
