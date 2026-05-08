#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <math.h>
#include <PID_v1.h>
#include <Update.h>

// =====================
// IDENTITÀ DISPOSITIVO
// =====================
const char* DEVICE_NAME = "ESP32_PT1000";
const char* DEVICE_ID   = "esp32-pt1000-01";
const char* HOSTNAME    = "pippo";

// =====================
// OTA / PROGRAMMAZIONE DA APP
// =====================
// Questo token deve coincidere con quello usato dall'app Android.
// Cambialo prima di consegnare l'app a clienti veri.
const char* FIRMWARE_VERSION = "1.0.0-ota-editor-ready";
const char* OTA_TOKEN = "yzf750";
// Lasciamo decidere alla partizione OTA disponibile: è più sicuro di un limite fisso.
volatile bool otaUpdateInProgress = false;
volatile bool otaUpdateOk = false;
String otaLastError = "";

// =====================
// WIFI / WEB SERVER / STORAGE
// =====================
WebServer server(80);
Preferences prefs;
WiFiManager wifiManager;

// =====================
// PIN ESP32 / CD74HC4067
// =====================
const int SIG_PIN = 34;
const int S0_PIN  = 12;
const int S1_PIN  = 13;
const int S2_PIN  = 14;
const int S3_PIN  = 15;
const int WIFI_RESET_BUTTON = 0;

// =====================
// DIMENSIONI SISTEMA
// =====================
const int NUM_SENSORS = 10;
const int NUM_RELAYS  = 10;
const int NUM_MUX_CHANNELS = 16;

// =====================
// CONFIG PT1000
// =====================
//const float VCC = 3.3f;
const float PT1000_R0 = 1000.0f;
const float PT1000_ALPHA = 0.003935f;
const float PT1000_DIVISOR = PT1000_R0 * PT1000_ALPHA;
const float R_REF = 1000.0f;

uint32_t bootId = 0;

// =====================
// ADC / CALIBRAZIONE
// =====================
const int ADC_SAMPLES = 12;
float CAL_GAIN = 0.98f;
float CAL_OFFSET =0.0f;
float ADC_OFFSET = 0.14f;
float measuredVcc = 3.3f;

// Tipo Sonda 1 PT1000 0 NTC cinese
//const int sonda=0;

const float R_REF_PT1000 = 1000.0f;
const float R_REF_NTC    = 10000.0f;

enum ProbeType {
  PROBE_NTC_10K = 0,
  PROBE_PT1000 = 1
};

const ProbeType SONDA = PROBE_PT1000;
//const ProbeType SONDA = PROBE_NTC_10K;


float getReferenceResistance() {
  return (SONDA == PROBE_PT1000) ? R_REF_PT1000 : R_REF_NTC;
}

// --- NUOVE VARIABILI OFFSET ---
// Offset percentuale applicato all'utente 
float userTempOffsetPercent[NUM_SENSORS] = {
  0.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 0.0f
};//Offset percentuale applicato dal dev 
float DevDisplayOffsetPercent[NUM_SENSORS] = {
  0.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 0.0f
};
// Offset percentuale applicato solo per la visualizzazione (configurazione)
float displayTempOffsetPercent[NUM_SENSORS] = {
  0.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 0.0f
};
// =====================
// TIMING
// =====================
const unsigned long REPORT_INTERVAL_MS  = 5000UL;
const unsigned long MEASURE_INTERVAL_MS = 5000UL;
unsigned long lastReportMs = 0;
unsigned long lastMeasureMs = 0;

// =====================
// CONFIG SENSORI - VETTORI
// =====================
int sensorMuxChannel[NUM_SENSORS] = {
  0,   // sensore 0  -> CH0
  1,   // sensore 1  -> CH1
  2,   // sensore 2  -> CH2
  3,   // sensore 3  -> CH3
  4,   // sensore 4  -> CH4
  5,   // sensore 5  -> CH5
  6,   // sensore 6  -> CH6
  7,   // sensore 7  -> CH7
  8,   // sensore 8  -> CH8
  9,   // sensore 9  -> CH9
  //10,  // sensore 10 -> CH10
  //11   // sensore 11 -> CH11
};


// =====================
// CONFIG RELÈ - VETTORI
// =====================
int relayPin[NUM_RELAYS] = {
  16, 17, 18, 19,
  21, 22, 23, 25,
  26, 27
};

bool relayActiveLow[NUM_RELAYS] = {
  true, true, true, true,
  true, true, true, true,
  true, true
};

// relayEnabled:
// true  = relè usato dal controllo automatico
// false = relè ignorato
bool relayEnabled[NUM_RELAYS] = {
  true, true, true, true,
  true, true, true, true,
  true, true
};

// Ogni relè usa un sensore.
// -1 = nessun sensore assegnato
int relaySensorIndex[NUM_RELAYS] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9
};

// Parametri vettoriali, NON hardcoded nel relay
float relaySetpoint[NUM_RELAYS] = {
  100.0f, 100.0f, 100.0f, 100.0f,
  80.0f, 80.0f, 80.0f, 80.0f,
  80.0f, 80.0f, 
};

float relayHysteresis[NUM_RELAYS] = {
  0.1f, 0.1f, 0.1f, 0.1f,
  0.1f, 0.1f, 0.1f, 0.1f,
  0.1f, 0.1f
};

bool relayState[NUM_RELAYS] = {
  false, false, false, false,
  false, false, false, false,
  false, false
};

// =====================
// DATI MISURATI SENSORI - VETTORI
// =====================
int   sensorAdcRaw[NUM_SENSORS];
float sensorVoltage[NUM_SENSORS];
float sensorResistance[NUM_SENSORS];
float sensorTempRaw[NUM_SENSORS];
float sensorTempCorr[NUM_SENSORS];

// =====================
// DATI MISURATI TUTTI I CANALI MUX
// =====================
int   muxAdcRaw[NUM_MUX_CHANNELS];
float muxVoltage[NUM_MUX_CHANNELS];
float muxResistance[NUM_MUX_CHANNELS];
float muxTempRaw[NUM_MUX_CHANNELS];
float muxTempCorr[NUM_MUX_CHANNELS];
bool  muxActive[NUM_MUX_CHANNELS];

// --- PID ---
// --- PID AUTO-TUNING ---
const unsigned long PID_SAMPLE_TIME_MS = 20000UL; // 20 secondi
double setpointPID[NUM_SENSORS];
double inputPID[NUM_SENSORS];
double outputPID[NUM_SENSORS];

PID pidController[NUM_SENSORS] = {
  PID(&inputPID[0],  &outputPID[0],  &setpointPID[0],  1.0, 0.5, 0.0, DIRECT),
  PID(&inputPID[1],  &outputPID[1],  &setpointPID[1],  1.0, 0.5, 0.0, DIRECT),
  PID(&inputPID[2],  &outputPID[2],  &setpointPID[2],  1.0, 0.5, 0.0, DIRECT),
  PID(&inputPID[3],  &outputPID[3],  &setpointPID[3],  1.0, 0.5, 0.0, DIRECT),
  PID(&inputPID[4],  &outputPID[4],  &setpointPID[4],  1.0, 0.5, 0.0, DIRECT),
  PID(&inputPID[5],  &outputPID[5],  &setpointPID[5],  1.0, 0.5, 0.0, DIRECT),
  PID(&inputPID[6],  &outputPID[6],  &setpointPID[6],  1.0, 0.5, 0.0, DIRECT),
  PID(&inputPID[7],  &outputPID[7],  &setpointPID[7],  1.0, 0.5, 0.0, DIRECT),
  PID(&inputPID[8],  &outputPID[8],  &setpointPID[8],  1.0, 0.5, 0.0, DIRECT),
  PID(&inputPID[9],  &outputPID[9],  &setpointPID[9],  1.0, 0.5, 0.0, DIRECT),
 // PID(&inputPID[10], &outputPID[10], &setpointPID[10], 2.0, 5.0, 1.0, DIRECT),
 // PID(&inputPID[11], &outputPID[11], &setpointPID[11], 2.0, 5.0, 1.0, DIRECT)
};

unsigned long lastPIDUpdate[NUM_SENSORS] = {0};

const unsigned long CONTROL_WINDOW_MS = 20000UL; // come centralina vecchia
const float PID_ENTER_BAND = 4.0f;
const float PID_EXIT_BAND  = 8.0f;
const float HOLD_BAND      = 0.5f;

bool maintainMode[NUM_RELAYS] = {false};
unsigned long pwmWindowStart[NUM_RELAYS] = {0};

float tempHist[NUM_SENSORS][3] = {0};
unsigned long lastTrendUpdate[NUM_SENSORS] = {0};

enum Trend {
  TREND_RISING,
  TREND_FALLING,
  TREND_STABLE
};

// --- Parametri PID iniziali (possono essere modificati via Web UI) ---
double DEFAULT_KP = 1.0;
double DEFAULT_KI = 0.5;
double DEFAULT_KD = 0.0;

struct BufferedSample {
  uint32_t sampleId;
  uint32_t timestampMs;
  uint8_t sensorIndex;
  float tempReal;
  float tempControl;
  float tempDisplay;
  bool relayState;
  float setpoint;
  float pidOutput;
};

const size_t DATA_BUFFER_SIZE = 720;
BufferedSample dataBuffer[DATA_BUFFER_SIZE];
size_t dataBufferHead = 0;
size_t dataBufferCount = 0;
uint32_t nextSampleId = 1;

// --- Variabili per Auto-Tuning ---
struct TuningData {
  double lastTemp = 0;
  double lastError = 0;
  double peakTemp = 0;
  double valleyTemp = 0;
  bool peakFound = false;
  bool valleyFound = false;
  unsigned long lastPeakTime = 0;
  unsigned long lastValleyTime = 0;
  int peakCount = 0;
  int cycleCount = 0;
  bool tuningActive = false;
  unsigned long tuningStartTime = 0;
  double originalKp, originalKi, originalKd;
  bool reachedTarget = false;
  unsigned long targetReachedTime = 0;
};
TuningData tuning[NUM_SENSORS];

// Impostazioni predefinite per il PID (puoi cambiarle via Web UI)
double Kp = 1.0, Ki = 0.5, Kd = 0.0;

// --- Funzione per aggiornare il PID con auto-tuning ---
void updatePIDWithAutoTune(int sensorIndex, float currentTemp, float setpoint) {
  float controlTemp = getControlTemperature(sensorIndex, currentTemp);

  inputPID[sensorIndex] = controlTemp;
  setpointPID[sensorIndex] = setpoint;

  if (millis() - lastPIDUpdate[sensorIndex] >= PID_SAMPLE_TIME_MS) {
    pidController[sensorIndex].Compute();

    Serial.printf(
      "PID_DEBUG: Sensor %d | Input: %.2f | Setpoint: %.2f | Error: %.2f | Kp: %.2f | Ki: %.2f | Kd: %.2f | Output: %.0f\n",
      sensorIndex,
      inputPID[sensorIndex],
      setpointPID[sensorIndex],
      setpointPID[sensorIndex] - inputPID[sensorIndex],
      pidController[sensorIndex].GetKp(),
      pidController[sensorIndex].GetKi(),
      pidController[sensorIndex].GetKd(),
      outputPID[sensorIndex]
    );

    double error = setpoint - controlTemp;

    if (!tuning[sensorIndex].reachedTarget && abs(error) < 2.0) {
      tuning[sensorIndex].reachedTarget = true;
      tuning[sensorIndex].targetReachedTime = millis();
      Serial.printf("Zona %d: Target %.2f°C raggiunto (Control Temp: %.2f°C).\n",
                    sensorIndex, setpoint, controlTemp);
    }

    if (tuning[sensorIndex].reachedTarget) {
      if (controlTemp < (setpoint - 5.0) &&
          (millis() - tuning[sensorIndex].targetReachedTime > 10UL * 60UL * 1000UL)) {
        double kp = pidController[sensorIndex].GetKp();
        double ki = pidController[sensorIndex].GetKi();
        double kd = pidController[sensorIndex].GetKd();

        kp *= 1.2;
        ki *= 1.2;
        kd *= 1.2;

        pidController[sensorIndex].SetTunings(kp, ki, kd);
        Serial.printf("Zona %d: Aumentati guadagni PID. Nuovi valori: P=%.2f I=%.2f D=%.2f\n",
                      sensorIndex, kp, ki, kd);
        tuning[sensorIndex].targetReachedTime = millis();
      }
    }

    if (tuning[sensorIndex].lastTemp != 0) {
      if (controlTemp > tuning[sensorIndex].lastTemp && tuning[sensorIndex].lastError < error) {
        if (tuning[sensorIndex].peakFound && controlTemp > tuning[sensorIndex].peakTemp) {
          tuning[sensorIndex].peakTemp = controlTemp;
        } else if (!tuning[sensorIndex].peakFound) {
          tuning[sensorIndex].peakFound = true;
          tuning[sensorIndex].peakTemp = controlTemp;
          tuning[sensorIndex].lastPeakTime = millis();
        }
      }

      if (controlTemp < tuning[sensorIndex].lastTemp && tuning[sensorIndex].lastError > error) {
        if (tuning[sensorIndex].valleyFound && controlTemp < tuning[sensorIndex].valleyTemp) {
          tuning[sensorIndex].valleyTemp = controlTemp;
        } else if (!tuning[sensorIndex].valleyFound) {
          tuning[sensorIndex].valleyFound = true;
          tuning[sensorIndex].valleyTemp = controlTemp;
          tuning[sensorIndex].lastValleyTime = millis();
        }
      }

      if (tuning[sensorIndex].peakFound && tuning[sensorIndex].valleyFound) {
        double amplitude = abs(tuning[sensorIndex].peakTemp - tuning[sensorIndex].valleyTemp);
        if (amplitude > 5.0) {
          double kp = pidController[sensorIndex].GetKp();
          double ki = pidController[sensorIndex].GetKi();
          double kd = pidController[sensorIndex].GetKd();

          kp *= 0.9;
          ki *= 0.9;
          kd *= 0.9;

          pidController[sensorIndex].SetTunings(kp, ki, kd);
          Serial.printf("Zona %d: Oscillazione grande %.2f°C, ridotti guadagni PID. P=%.2f I=%.2f D=%.2f\n",
                        sensorIndex, amplitude, kp, ki, kd);
        }

        tuning[sensorIndex].peakFound = false;
        tuning[sensorIndex].valleyFound = false;
      }
    }

    tuning[sensorIndex].lastTemp = controlTemp;
    tuning[sensorIndex].lastError = error;
    lastPIDUpdate[sensorIndex] = millis();
  }
}

// =====================
// SALVA/CARICA PARAMETRI
// =====================
void saveRelayParams(int relayIndex) {
  prefs.begin("relay_cfg", false);
  prefs.putBool(("en_" + String(relayIndex)).c_str(), relayEnabled[relayIndex]);
  prefs.putInt(("sens_" + String(relayIndex)).c_str(), relaySensorIndex[relayIndex]);
  prefs.putFloat(("sp_" + String(relayIndex)).c_str(), relaySetpoint[relayIndex]);
  prefs.putFloat(("hy_" + String(relayIndex)).c_str(), relayHysteresis[relayIndex]);
  prefs.end();

  Serial.print("💾 Parametri salvati - Relay ");
  Serial.print(relayIndex);
  Serial.print(" | SP=");
  Serial.print(relaySetpoint[relayIndex], 2);
  Serial.print(" | HY=");
  Serial.println(relayHysteresis[relayIndex], 2);
}

void loadRelayParams() {
  prefs.begin("relay_cfg", true);
  for (int r = 0; r < NUM_RELAYS; r++) {
    bool savedEn = prefs.getBool(("en_" + String(r)).c_str(), relayEnabled[r]);
    int savedSens = prefs.getInt(("sens_" + String(r)).c_str(), relaySensorIndex[r]);
    float savedSP = prefs.getFloat(("sp_" + String(r)).c_str(), relaySetpoint[r]);
    float savedHY = prefs.getFloat(("hy_" + String(r)).c_str(), relayHysteresis[r]);
    
    relayEnabled[r] = savedEn;
    relaySensorIndex[r] = savedSens;
    relaySetpoint[r] = savedSP;
    relayHysteresis[r] = savedHY;
    
    Serial.print("📦 Relay ");
    Serial.print(r);
    Serial.print(" caricato | EN=");
    Serial.print(savedEn ? "1" : "0");
    Serial.print(" | SP=");
    Serial.print(savedSP, 2);
    Serial.print(" | HY=");
    Serial.println(savedHY, 2);
  }
  prefs.end();
}

// =====================
// WIFI / MDNS / CONFIG (MODIFICATO PER WiFiManager)
// =====================
void startMDNS() {
  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("esp32controller", "tcp", 80);
    Serial.print("✅ mDNS attivo: http://");
    Serial.print(HOSTNAME);
    Serial.println(".local");
  } else {
    Serial.println("❌ Errore mDNS");
  }
}

void configModeCallback(WiFiManager *wm) {
  Serial.println("🔧 === PORTALE CONFIGURAZIONE ATTIVO ===");
  Serial.print("📡 AP SSID: ");
  Serial.println(wm->getConfigPortalSSID());
  Serial.print("🌐 AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("📱 Connettiti con il telefono a questa rete");
  Serial.println("🌐 Poi apri browser: http://192.168.4.1");
  Serial.println("⚠️  HOTSPOT MOBILE: usa 2.4GHz + WPA2-Personal!");
  Serial.println("==========================================");
}
void startWiFiSmart() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(HOSTNAME);

  bool forceConfig = false;
  pinMode(WIFI_RESET_BUTTON, INPUT_PULLUP); // Assicurati che il pull-up sia abilitato

  // Reset WiFi se BOOT premuto per 3 secondi
  Serial.println("🔍 Controllo button BOOT...");
  Serial.printf("Stato pin BOOT (GPIO %d) all'avvio: %d\n", WIFI_RESET_BUTTON, digitalRead(WIFI_RESET_BUTTON)); // Debug

  unsigned long pressStart = 0;
  bool pressed = false;

  // Leggi stato iniziale
  if (digitalRead(WIFI_RESET_BUTTON) == LOW) {
    pressStart = millis();
    pressed = true;
    Serial.println("✅ Bottone premuto rilevato all'avvio.");
  }

  // Attendi 3 secondi mantenendo il bottone premuto
  while (pressed && (millis() - pressStart < 3000UL)) {
    if (digitalRead(WIFI_RESET_BUTTON) == HIGH) {
      pressed = false; // Bottone rilasciato prima del tempo
      Serial.println("⚠️ Bottone rilasciato prima di 3 secondi.");
    }
    delay(10); // Piccola pausa per non sovraccaricare il loop
  }

  if (pressed && (millis() - pressStart >= 3000UL)) {
    Serial.println("🔄 BOOT premuto per 3 secondi → reset impostazioni WiFi");
    forceConfig = true;
  } else if (!pressed) {
    Serial.println("🔘 Bottone non premuto o rilasciato presto.");
  }

  // ... resto della funzione come prima ...

  String apName = String("ESP32_SETUP_") + DEVICE_ID;

  if (forceConfig) {
    Serial.println("🔓 Avvio portale configurazione FORZATO...");
    wifiManager.startConfigPortal(apName.c_str());
  } else {
    Serial.println("🔌 Tentativo connessione WiFi salvata...");
    if (!wifiManager.autoConnect(apName.c_str())) {
      Serial.println("❌ WiFi non trovato → avvio portale configurazione");
      wifiManager.startConfigPortal(apName.c_str());
    }
  }

  Serial.println("✅ WiFi connesso!");
  Serial.print("🌐 IP ESP32: ");
  Serial.println(WiFi.localIP());
  Serial.print("📶 RSSI: ");
  Serial.println(WiFi.RSSI());

  startMDNS();
}

void ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.println("⚠️ WiFi disconnesso, provo riconnessione...");
  WiFi.reconnect();
  delay(2000);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✅ WiFi riconnesso!");
    Serial.print("🌐 Nuovo IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("❌ Riconnessione fallita");
  }
}

void printWiFiInfo() {
  Serial.print("📶 Stato WiFi: ");
  Serial.println((int)WiFi.status());
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("🌐 IP ESP32: ");
    Serial.println(WiFi.localIP());
    Serial.print("📶 RSSI: ");
    Serial.println(WiFi.RSSI());
    Serial.print("🔗 mDNS: http://");
    Serial.print(HOSTNAME);
    Serial.println(".local");
  } else {
    Serial.println("❌ WiFi NON connesso");
  }
}

// =====================
// MUX
// =====================
void selectChannel(int channel) {
  digitalWrite(S0_PIN, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(S1_PIN, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(S2_PIN, (channel & 0x04) ? HIGH : LOW);
  digitalWrite(S3_PIN, (channel & 0x08) ? HIGH : LOW);
}


//
const int FILTER_SAMPLES = 24;
const int MAX_DELTA = 3;
const int IIR_ALPHA = 8;

// Stato filtro separato per ogni canale MUX
int32_t filterLastValue[NUM_MUX_CHANNELS];
int32_t filterIirState[NUM_MUX_CHANNELS];
bool filterInitialized[NUM_MUX_CHANNELS];

uint16_t filteredADC(uint8_t pin, int muxChannel) {
  uint32_t sum = 0;

  // 1) media di più campioni
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }

  uint16_t raw = sum / FILTER_SAMPLES;

  // 2) blocco spike
  if (filterInitialized[muxChannel]) {
    int32_t diff = (int32_t)raw - filterLastValue[muxChannel];
    if (diff > MAX_DELTA) {
      raw = filterLastValue[muxChannel] + MAX_DELTA;
    } else if (diff < -MAX_DELTA) {
      raw = filterLastValue[muxChannel] - MAX_DELTA;
    }
  }

  // 3) filtro IIR
  if (!filterInitialized[muxChannel]) {
    filterIirState[muxChannel] = raw;
    filterInitialized[muxChannel] = true;
  } else {
    filterIirState[muxChannel] =
      ((filterIirState[muxChannel] * (IIR_ALPHA - 1)) + raw) / IIR_ALPHA;
  }

  filterLastValue[muxChannel] = raw;

  return (uint16_t)filterIirState[muxChannel];
}
///
// =====================
// >>> LETTURE PT1000 (SOLO QUESTE 3 FUNZIONI MODIFICATE)
// =====================
// 1. LETTURA TENSIONE CON OFFSET CALIBRATO
float readVoltageFromMuxChannel(int muxChannel, int samples) {
  selectChannel(muxChannel);

  // Attesa per assestamento segnale
  delayMicroseconds(300);

  // Lettura fittizia da buttare via (aiuta l'ADC a stabilizzarsi)
  analogRead(SIG_PIN);
  delayMicroseconds(300);

  long sumAdc = 0;
  for (int i = 0; i < samples; i++) {
    sumAdc += filteredADC(SIG_PIN, muxChannel);
    delayMicroseconds(200); // Breve pausa tra letture
  }

  float avgAdc = sumAdc / (float)samples;
  return (avgAdc / 4095.0f) * measuredVcc  + ADC_OFFSET; // Usa il tuo offset
}
// 2. CONVERSIONE TENSIONE → RESISTENZA (formula partitore)
//float voltageToResistance(float v) {
//  if (isnan(v) || v <= 0.01f || v >= (VCC - 0.01f)) return NAN;

//  float rRef = getReferenceResistance();

  // CASO A:
  // 3.3V -> R_REF -> ADC -> SONDA -> GND
//  return rRef * ((VCC / v) - 1.0f);

  // Se con la NTC i valori sono assurdi, prova invece questa formula:
  // return rRef * (v / (VCC - v));
//}
float voltageToResistance(float v) {
  float vSupply = measuredVcc;

  if (isnan(v) || isnan(vSupply) || v <= 0.01f || vSupply <= 0.1f || v >= (vSupply - 0.01f)) {
    return NAN;
  }

  float rRef = getReferenceResistance();

  // Schema: VCC -> R_REF -> ADC -> SONDA -> GND
  return rRef * ((vSupply / v) - 1.0f);
}

// 3. CONVERSIONE RESISTENZA → TEMPERATURA (lineare, dai tuoi dati Excel)
//float resistanceToTempRaw(float resistanceOhm) {
//if (isnan(resistanceOhm) || resistanceOhm <= 0.0f) return NAN;
// return (resistanceOhm / 1000.0f - 1.0f) / 0.00385055f;
float resistanceToTempRaw(float resistanceOhm) {
  if (isnan(resistanceOhm) || resistanceOhm <= 0.0f) return NAN;

  if (SONDA == PROBE_PT1000) {
    return (resistanceOhm - PT1000_R0) / PT1000_DIVISOR;
  } else {
    const float T0 = 298.15f;    // 25°C in Kelvin
    const float R0 = 10000.0f;   // 10kΩ a 25°C
    const float BETA = 3950.0f;

    float invT = (1.0f / T0) + (1.0f / BETA) * log(resistanceOhm / R0);
    float tempK = 1.0f / invT;
    return tempK - 273.15f;
  }
}

float applyCalibration(float tempRaw) {
  if (isnan(tempRaw)) return NAN;
  return (tempRaw * CAL_GAIN) + CAL_OFFSET;
}
float applyPercentOffset(float baseValue, float percent) {
  if (isnan(baseValue)) return baseValue;
  return baseValue + (baseValue * (percent / 100.0f));
}

float getDevTemperature(int sensorIndex, float tRaw) {
  if (isnan(tRaw)) return tRaw;
  return applyPercentOffset(tRaw, DevDisplayOffsetPercent[sensorIndex]);
}

float getControlTemperature(int sensorIndex, float tRaw) {
  if (isnan(tRaw)) return tRaw;

  float tDev = getDevTemperature(sensorIndex, tRaw);
  return applyPercentOffset(tDev, userTempOffsetPercent[sensorIndex]);
}

const float DISPLAY_THRESHOLD = 40.0f;

float getDisplayTemperature(int sensorIndex, float tRaw) {
  if (isnan(tRaw)) return tRaw;

  float tControl = getControlTemperature(sensorIndex, tRaw);

  if (tControl > DISPLAY_THRESHOLD) {
    return applyPercentOffset(tControl, displayTempOffsetPercent[sensorIndex]);
  }

  return tControl;
}

void debugSelectChannel(int channel) {
  Serial.printf(
    "CH%d -> S3S2S1S0 = %d%d%d%d\n",
    channel,
    (channel & 0x08) ? 1 : 0,
    (channel & 0x04) ? 1 : 0,
    (channel & 0x02) ? 1 : 0,
    (channel & 0x01) ? 1 : 0
  );
}

void debugOnlyIndicesOneLine() {
  Serial.print("IDX: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(i);
    Serial.print("->");
    Serial.print(sensorMuxChannel[i]);
    Serial.print(" | ");
  }
  Serial.println();
}

float measure3v3Pin(int samples = 20);
float voltageToResistance(float v);
float resistanceToTempRaw(float resistanceOhm);
float applyCalibration(float tempRaw);



void updateMeasurements() {
  Serial.println("==== START updateMeasurements ====");

  measuredVcc  = measure3v3Pin();
  Serial.print("VCC_MON=");
  Serial.print(measuredVcc , 3);
  Serial.println(" V");

  for (int i = 0; i < NUM_SENSORS; i++) {
    int muxCh = sensorMuxChannel[i];

    Serial.print("IDX=");
    Serial.print(i);
    Serial.print(" | MUX=");
    Serial.println(muxCh);

    selectChannel(muxCh);
    delayMicroseconds(300);
    analogRead(SIG_PIN);
    delayMicroseconds(300);

//    long sumAdc = 0;
//    for (int k = 0; k < ADC_SAMPLES; k++) {
//      sumAdc += analogRead(SIG_PIN);
//      delayMicroseconds(200);
//    }
    //sensorAdcRaw[i] = (int)((sensorVoltage[i] - ADC_OFFSET) / measuredVcc * 4095.0f);
    sensorAdcRaw[i] = filteredADC(SIG_PIN, muxCh);
    //sensorAdcRaw[i] = (int)(sumAdc / ADC_SAMPLES);
    //sensorVoltage[i] = (sensorAdcRaw[i] / 4095.0f) * VCC + ADC_OFFSET;
    
    sensorVoltage[i] = (sensorAdcRaw[i] / 4095.0f) * measuredVcc + ADC_OFFSET;
    sensorResistance[i] = voltageToResistance(sensorVoltage[i]);
    sensorTempRaw[i] = resistanceToTempRaw(sensorResistance[i]);
    sensorTempCorr[i] = applyCalibration(sensorTempRaw[i]);

        float tReal = sensorTempCorr[i];
    float tControl = getControlTemperature(i, tReal);
    float tDisplay = getDisplayTemperature(i, tReal);

    bool rState = false;
    float setpoint = 0.0f;
    float pidOut = outputPID[i];

    for (int r = 0; r < NUM_RELAYS; r++) {
      if (relaySensorIndex[r] == i) {
        rState = relayState[r];
        setpoint = relaySetpoint[r];
        break;
      }
    }

    addSampleToBuffer(
      i,
      tReal,
      tControl,
      tDisplay,
      rState,
      setpoint,
      pidOut
    );

    debugSelectChannel(muxCh);

     Serial.printf(
      "Sensor %d | ADC=%d | Vsig=%.4f V | R=%.2f ohm | Traw=%.2f C | Tcorr=%.2f C\n",
      i,
      sensorAdcRaw[i],
      sensorVoltage[i],
      sensorResistance[i],
      sensorTempRaw[i],
      sensorTempCorr[i]
     );
  }

  Serial.println("==== END updateMeasurements ====");
}

void addSampleToBuffer(
  int sensorIndex,
  float tempReal,
  float tempControl,
  float tempDisplay,
  bool relayStateValue,
  float setpoint,
  float pidOutputValue
) {
  if (sensorIndex < 0 || sensorIndex >= NUM_SENSORS) return;
  if (isnan(tempReal) || isnan(tempControl) || isnan(tempDisplay)) return;
  if (tempReal < -100.0f || tempReal > 1000.0f) return;
  if (tempControl < -100.0f || tempControl > 1000.0f) return;
  if (tempDisplay < -100.0f || tempDisplay > 1000.0f) return;

  BufferedSample &slot = dataBuffer[dataBufferHead];

  slot.sampleId = nextSampleId++;
  slot.timestampMs = millis();
  slot.sensorIndex = (uint8_t)sensorIndex;
  slot.tempReal = tempReal;
  slot.tempControl = tempControl;
  slot.tempDisplay = tempDisplay;
  slot.relayState = relayStateValue;
  slot.setpoint = setpoint;
  slot.pidOutput = pidOutputValue;

  dataBufferHead = (dataBufferHead + 1) % DATA_BUFFER_SIZE;

  if (dataBufferCount < DATA_BUFFER_SIZE) {
    dataBufferCount++;
  }
}

// =====================
// RELÈ
// =====================
void writeRelay(int relayIndex, bool on) {
  relayState[relayIndex] = on;
  if (relayActiveLow[relayIndex]) {
    digitalWrite(relayPin[relayIndex], on ? LOW : HIGH);
  } else {
    digitalWrite(relayPin[relayIndex], on ? HIGH : LOW);
  }
}

const char* relayText(bool state) {
  return state ? "ON" : "OFF";
}

void testAllRelaysStartup() {
  Serial.println();
  Serial.println("========== TEST AVVIO RELE ==========");
  for (int r = 0; r < NUM_RELAYS; r++) {
    Serial.print("Relay ");
    Serial.print(r);
    Serial.print(" -> GPIO ");
    Serial.print(relayPin[r]);
    Serial.println(" | test ON/OFF");
    writeRelay(r, true);
    Serial.print("  ON  -> Relay ");
    Serial.println(r);
    delay(700);
    writeRelay(r, false);
    Serial.print("  OFF -> Relay ");
    Serial.println(r);
    delay(700);
  }
  Serial.println("======= FINE TEST AVVIO RELE =======");
  Serial.println();
}

// =====================
// CONTROLLO AUTOMATICO RELÈ
// =====================
// Nella funzione updateRelayControl o dove stampi il log
//void updateRelayControl() {
//  for (int r = 0; r < NUM_RELAYS; r++) {
//    if (!relayEnabled[r]) {
//      writeRelay(r, false);
//      continue;
//    }
//
//    int s = relaySensorIndex[r];
//    if (s < 0 || s >= NUM_SENSORS) {
//      writeRelay(r, false);
//      continue;
//    }

//    float tReal = sensorTempCorr[s];
//    if (isnan(tReal)) {
//      writeRelay(r, false);
//      continue;
//    }

//    float setpoint = relaySetpoint[r];
//    float hyst = relayHysteresis[r];

//    float tDev = getDevTemperature(s, tReal);
//    float tControl = getControlTemperature(s, tReal);
//    float tDisplay = getDisplayTemperature(s, tReal);

//    updatePIDWithAutoTune(s, tReal, setpoint);

//    bool shouldHeat = relayState[r];

    // Controllo semplice a isteresi
//    if (tControl < (setpoint - hyst)) {
//      shouldHeat = true;
//    } else if (tControl >= setpoint) {
//      shouldHeat = false;
//    }

//    writeRelay(r, shouldHeat);

//    Serial.printf(
//      "Relay %d | Sensor %d | Real=%.2f | Dev=%.2f | Ctrl=%.2f | Disp=%.2f | Set=%.2f | Hyst=%.2f | PID=%.0f | State=%s\n",
//      r, s, tReal, tDev, tControl, tDisplay, setpoint, hyst, outputPID[s], relayText(relayState[r])
//    );
//  }
//}

const float attenuationFactor = 2.0; // Perché hai dimezzato la tensione col partitore

const int VCC_MONITOR_PIN = 35;
const float VCC_DIVIDER_FACTOR = 2.0f; // se hai un partitore 1:1

float measure3v3Pin(int samples) {
  long sum = 0;

  analogRead(VCC_MONITOR_PIN);
  delayMicroseconds(200);

  for (int i = 0; i < samples; i++) {
    sum += analogRead(VCC_MONITOR_PIN);
    delayMicroseconds(200);
  }

  float avgRaw = sum / (float)samples;

  // Conversione ADC -> tensione sul pin ADC
  float voltageAtPin = (avgRaw / 4095.0f) * 3.3f;

  // Ricostruzione tensione reale prima del partitore
  float realVoltage = voltageAtPin * VCC_DIVIDER_FACTOR;

  return realVoltage;
}

int getTrend(int sensorIndex) {
  float a = tempHist[sensorIndex][0];
  float b = tempHist[sensorIndex][1];
  float c = tempHist[sensorIndex][2];

  if (a == 0 || b == 0 || c == 0) return TREND_STABLE;

  if (a < b && b < c) return TREND_RISING;
  if (a > b && b > c) return TREND_FALLING;

  return TREND_STABLE;
}
void updateTrendHistory(int sensorIndex, float temp) {
  unsigned long now = millis();

  if (now - lastTrendUpdate[sensorIndex] >= 5000UL) {
    tempHist[sensorIndex][0] = tempHist[sensorIndex][1];
    tempHist[sensorIndex][1] = tempHist[sensorIndex][2];
    tempHist[sensorIndex][2] = temp;
    lastTrendUpdate[sensorIndex] = now;
  }
}

bool pwmRelayDecision(int relayIndex, float dutyPercent) {
  unsigned long now = millis();

  if (now - pwmWindowStart[relayIndex] >= CONTROL_WINDOW_MS) {
    pwmWindowStart[relayIndex] = now;
  }

  if (dutyPercent <= 0.0f) return false;
  if (dutyPercent >= 100.0f) return true;

  unsigned long onTime = (unsigned long)((dutyPercent / 100.0f) * CONTROL_WINDOW_MS);
  unsigned long elapsed = now - pwmWindowStart[relayIndex];

  return elapsed < onTime;
}

void updateRelayControl() {
  for (int r = 0; r < NUM_RELAYS; r++) {

    if (!relayEnabled[r]) {
      writeRelay(r, false);
      maintainMode[r] = false;
      continue;
    }

    int s = relaySensorIndex[r];

    if (s < 0 || s >= NUM_SENSORS) {
      writeRelay(r, false);
      maintainMode[r] = false;
      continue;
    }

    float tReal = sensorTempCorr[s];

    if (isnan(tReal)) {
      writeRelay(r, false);
      maintainMode[r] = false;
      continue;
    }

    float setpoint = relaySetpoint[r];
    float tControl = getControlTemperature(s, tReal);

    updateTrendHistory(s, tControl);
    int trend = getTrend(s);
    inputPID[s] = tControl;
    setpointPID[s] = setpoint;

    if (!maintainMode[r] && tControl >= setpoint - PID_ENTER_BAND) {
      maintainMode[r] = true;
      pwmWindowStart[r] = millis();
      Serial.printf("Relay %d ENTER MAINTAIN\n", r);
    }

    if (maintainMode[r] && tControl < setpoint - PID_EXIT_BAND) {
      maintainMode[r] = false;
      pwmWindowStart[r] = millis();
      Serial.printf("Relay %d EXIT MAINTAIN\n", r);
    }

    float duty = 0.0f;

    if (!maintainMode[r]) {
      duty = 100.0f;
    } else {
      pidController[s].Compute();
      duty = outputPID[s];

      if (tControl > setpoint + HOLD_BAND && trend == TREND_RISING) {
        duty = 0.0f;
      }

      if (tControl > setpoint && trend == TREND_FALLING) {
        duty = min(duty, 25.0f);
      }

      if (tControl < setpoint - HOLD_BAND && trend == TREND_FALLING) {
        duty = max(duty, 60.0f);
      }

      if (tControl < setpoint && trend == TREND_RISING) {
        duty = min(duty, 50.0f);
      }
    }

    duty = constrain(duty, 0.0f, 100.0f);

    bool shouldHeat = pwmRelayDecision(r, duty);
    writeRelay(r, shouldHeat);

    Serial.printf(
      "Relay %d | Sensor %d | Mode=%s | Temp=%.2f | Set=%.2f | Duty=%.1f%% | Trend=%d | State=%s\n",
      r,
      s,
      maintainMode[r] ? "MAINTAIN" : "RAMP",
      tControl,
      setpoint,
      duty,
      trend,
      relayText(relayState[r])
    );
  }
}

// =====================
// REPORT
// =====================
void printFullReport() {
  Serial.println();
  Serial.println("============== REPORT ==============");
  printWiFiInfo();

  float measuredVcc  = measure3v3Pin();
  Serial.print("⚡ 3V3 monitorata: ");
  Serial.print(measuredVcc , 3);
  Serial.println(" V");

  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" | MUX CH=");
    Serial.print(sensorMuxChannel[i]);
    Serial.print(" | V=");
    Serial.print(sensorVoltage[i], 4);
    Serial.print(" V | R=");
    Serial.print(sensorResistance[i], 2);
    Serial.print(" ohm | Traw=");
    Serial.print(sensorTempRaw[i], 2);
    Serial.print(" C | Tcorr=");
    Serial.print(sensorTempCorr[i], 2);
    Serial.println(" C");
  }
  for (int r = 0; r < NUM_RELAYS; r++) {
    Serial.print("Relay ");
    Serial.print(r);
    Serial.print(" | GPIO=");
    Serial.print(relayPin[r]);
    Serial.print(" | Enabled=");
    Serial.print(relayEnabled[r] ? "true" : "false");
    Serial.print(" | Sensor=");
    Serial.print(relaySensorIndex[r]);
    Serial.print(" | Set=");
    Serial.print(relaySetpoint[r], 2);
    Serial.print(" | Hyst=");
    Serial.print(relayHysteresis[r], 2);
    Serial.print(" | State=");
    Serial.println(relayText(relayState[r]));
  }
  Serial.println("====================================");
  Serial.println();
}

// =====================
// JSON HELPER
// =====================
String numOrNull(float value, int decimals = 2) {
  if (isnan(value)) return "null";
  return String(value, decimals);
}

// =====================
// WEB ROUTES
// =====================
void handleRoot() {
  server.send(200, "text/html", R"rawliteral(
<!DOCTYPE html>
<html lang="it">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Forno ESP32 12 Zone</title>
    <style>
        body { font-family: sans-serif; background-color: #f4f4f9; padding: 20px; }
        h1 { text-align: center; color: #333; }
        .tabs { display: flex; justify-content: center; margin-bottom: 20px; }
        .tab { padding: 10px 20px; cursor: pointer; background: #ddd; border: none; font-size: 16px; }
        .tab.active { background: #007bff; color: white; }
        
        /* TABELLA MONITORING */
        table { width: 100%; border-collapse: collapse; background: white; box-shadow: 0 2px 5px rgba(0,0,0,0.1); margin-bottom: 20px; }
        th, td { padding: 12px; text-align: center; border-bottom: 1px solid #ddd; }
        th { background-color: #333; color: white; }
        
        /* STATI CELLE */
        .status-idle { background-color: #ffffff; }
        .status-yellow { background-color: #ffeb3b; color: #000; font-weight: bold; }
        .status-green { background-color: #4caf50; color: white; font-weight: bold; }

        /* PANNELLI CONFIGURAZIONE */
        .config-panel { display: none; background: white; padding: 20px; max-width: 600px; margin: 0 auto 20px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
        .form-group { margin-bottom: 15px; }
        .form-group label { display: block; margin-bottom: 5px; font-weight: bold; }
        .form-group input, .form-group button { width: 100%; padding: 8px; box-sizing: border-box; }
        .form-group input[type="number"] { width: 80px; display: inline-block; margin-right: 5px; }
        .form-group button { background-color: #28a745; color: white; border: none; padding: 10px 20px; font-size: 16px; cursor: pointer; }
        .form-group button:hover { background-color: #218838; }
        .form-group button.danger { background-color: #dc3545; }
        .form-group button.danger:hover { background-color: #c82333; }

        /* SEZIONE PARTENZA RITARDATA */
        #delayed-start-section { margin-top: 20px; padding: 15px; background-color: #e7f3ff; border-radius: 5px; }
        #ds-controls, #ds-countdown { margin-top: 10px; }
    </style>
</head>
<body>

    <h1>🔥 Forno Controllo 12 Zone</h1>

    <div class="tabs">
        <button class="tab active" onclick="switchTab('monitor')">📊 Monitoraggio</button>
        <button class="tab" onclick="switchTab('config')">⚙️ Configurazione</button>
        <button class="tab" onclick="switchTab('program')">📋 Programma</button>
        <button class="tab" onclick="switchTab('cycles')">🔄 Cicli</button>
    </div>

    <!-- SEZIONE MONITORAGGIO -->
    <div id="monitor-section">
        <table>
            <thead>
                <tr>
                    <th>Zona</th>
                    <th>Stato</th>
                    <th>Temp Attuale</th>
                    <th>Setpoint</th>
                    <th>Comando / Stato Relè</th>
                </tr>
            </thead>
            <tbody id="sensor-table">
                <!-- Generato via JS -->
            </tbody>
        </table>

        <!-- SEZIONE PARTENZA RITARDATA (Visibile solo in Modalità 1) -->
        <div id="delayed-start-section" style="display:none;">
            <h3>⏰ Partenza Ritardata</h3>
            <p id="ds-status-text">Partenza immediata attiva.</p>
            <div id="ds-controls">
                <label for="ds-hours">Imposta Ora (HH:MM):</label>
                <input type="number" id="ds-hours" min="0" max="23" value="0" placeholder="HH">
                :
                <input type="number" id="ds-minutes" min="59" value="0" placeholder="MM">
                <button onclick="setDelayedStart()">Imposta</button>
                <button class="danger" onclick="cancelDelayedStart()">Annulla</button>
            </div>
            <div id="ds-countdown"></div>
        </div>
    </div>

    <!-- SEZIONE CONFIGURAZIONE -->
    <div id="config-section" class="config-panel">
        <h2>🔧 Calibrazione e Parametri</h2>
        <p>Usa questi valori per correggere l'errore del 2% o altri.</p>
        
        <div class="form-group">
            <label>CAL_GAIN (Correzione Guadagno):</label>
            <input type="number" step="0.001" id="cal_gain" value="1.0">
            <small>Es: 0.98 per correggere un +2%</small>
        </div>
        
        <div class="form-group">
            <label>ADC_OFFSET (Correzione Tensione):</label>
            <input type="number" step="0.001" id="adc_offset" value="0.016">
            <small>Offset misurato sul multimetro</small>
        </div>
        
        <div class="form-group">
            <label>PID Kp (Guadagno Proporzionale):</label>
            <input type="number" step="0.01" id="pid_kp" value="2.0">
            <small>Valore iniziale per il guadagno proporzionale.</small>
        </div>
        
        <div class="form-group">
            <label>PID Ki (Guadagno Integrale):</label>
            <input type="number" step="0.01" id="pid_ki" value="5.0">
            <small>Valore iniziale per il guadagno integrale.</small>
        </div>

        <div class="form-group">
            <label>PID Kd (Guadagno Derivativo):</label>
            <input type="number" step="0.01" id="pid_kd" value="1.0">
            <small>Valore iniziale per il guadagno derivativo.</small>
        </div>

        <div class="form-group">
            <label>Default Offset Utente (%):</label>
            <input type="number" step="0.1" id="default_user_offset" value="0.0">
            <small>Offset percentuale predefinito per i canali utente.</small>
        </div>

        <div class="form-group">
            <label>Default Offset Visualizzazione (%):</label>
            <input type="number" step="0.1" id="default_display_offset" value="0.0">
            <small>Offset percentuale predefinito solo per la visualizzazione.</small>
        </div>


        <button class="save" onclick="saveSettings()">💾 Salva Impostazioni</button>
    </div>

    <!-- SEZIONE SCELTA PROGRAMMA -->
    <div id="program-section" class="config-panel">
        <h2>📋 Seleziona Modalità Programma</h2>
        <button onclick="setProgramMode(1)">1. Standard (Canali Indipendenti)</button>
        <button onclick="setProgramMode(2)">2. Accoppiato (Canali Dispari/Pari)</button>
        <button onclick="setProgramMode(3)">3. Cicli Temporizzati</button>
    </div>

    <!-- SEZIONE CONFIGURAZIONE CICLI -->
    <div id="cycles-section" class="config-panel">
        <h2 id="cycles-title">🔄 Configura Cicli</h2>
        <div id="cycle-inputs">
            <!-- Generato via JS -->
        </div>
        <button onclick="saveCycles()">💾 Salva Cicli</button>
        <button onclick="startCycles()">▶️ Avvia Cicli</button>
        <button class="danger" onclick="stopCycles()">⏹️ Ferma Cicli</button>
    </div>

    <script>
        // Variabili per la logica dei colori
        let channelStates = {};
        let yellowTimestamps = {};

        // Variabili per il countdown
        let countdownInterval = null;

        function switchTab(tab) {
            document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
            document.getElementById('monitor-section').style.display = tab === 'monitor' ? 'block' : 'none';
            document.getElementById('config-section').style.display = tab === 'config' ? 'block' : 'none';
            document.getElementById('program-section').style.display = tab === 'program' ? 'block' : 'none';
            document.getElementById('cycles-section').style.display = tab === 'cycles' ? 'block' : 'none';

            if(tab === 'monitor') {
                event.target.classList.add('active');
                updateData(); // Ricarica dati quando torni alla scheda monitor
            } else if(tab === 'config') {
                document.querySelectorAll('.tab.add('active');
            } else if(tab === 'program') {
                document.querySelectorAll('.tab')[2].classList.add('active');
            } else if(tab === 'cycles') {
                document.querySelectorAll('.tab')[3].classList.add('active');
                loadCycles(); // Carica i cicli quando entri nella scheda
            }
        }

        function updateData() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    updateDelayedStartUI(data);

                    const tbody = document.getElementById('sensor-table');
                    tbody.innerHTML = '';

                    if(data.program_mode === 1 || data.program_mode === 2) {
                        let sensorsToShow = data.sensors;
                        if(data.program_mode === 2) {
                            sensorsToShow = data.sensors.filter(sensor => sensor.index % 2 === 0);
                        }

                        sensorsToShow.forEach(sensor => {
                            const idx = sensor.index;
                            const currentRealTemp = parseFloat(sensor.temp_corr);
                            const currentDisplayTemp = parseFloat(sensor.temp_display);
                            const currentControlTemp = parseFloat(sensor.temp_control); // Aggiungi questa
                            const setpoint = data.relays[idx] ? data.relays[idx].setpoint : 20.0;
                            const relayState = data.relays[idx]?.on || false; // Aggiungi questa
                            const relayStateText = (!data.control_active && data.delayed_start_enabled) ? '⏱ In Attesa' : (relayState ? '🔥 SCALDA' : '⏸ RIPOSO');

                            let statusClass = 'status-idle';
                            let statusText = 'Attesa';
                            if (!data.control_active && data.delayed_start_enabled) {
                                statusClass = 'status-idle';
                                statusText = '⏱ In Attesa...';
                            } else {
                                if (channelStates[idx] === undefined) channelStates[idx] = 0;
                                if (currentControlTemp >= setpoint) {
                                    if (channelStates[idx] === 0) {
                                        channelStates[idx] = 1;
                                        yellowTimestamps[idx] = Date.now();
                                        statusClass = 'status-yellow';
                                        statusText = '🟡 Target Raggiunto (Avvio Timer)';
                                    } else if (channelStates[idx] === 1) {
                                        let diff = Date.now() - yellowTimestamps[idx];
                                        if (diff >= 20 * 60 * 1000) {
                                            channelStates[idx] = 2;
                                            statusClass = 'status-green';
                                            statusText = '🟢 Stabilizzato (>20min)';
                                        } else {
                                            statusClass = 'status-yellow';
                                            let minsLeft = 20 - Math.floor(diff / 60000);
                                            statusText = `🟡 Target OK (${minsLeft} min restanti)`;
                                        }
                                    } else if (channelStates[idx] === 2) {
                                        statusClass = 'status-green';
                                        statusText = '🟢 Stabilizzato (>20min)';
                                    }
                                } else {
                                    if (channelStates[idx] === 2) {
                                        statusClass = 'status-green';
                                        statusText = '🟢 Mantenuto';
                                    } else if (channelStates[idx] === 1) {
                                        statusClass = 'status-yellow';
                                        statusText = '🟡 Scesa sotto soglia';
                                    }
                                }
                            }

                           let row = `
                                <tr class="${statusClass}">
                                    <td><strong>Zona ${idx + 1}</strong></td>
                                    <td>${statusText}</td>
                                    <td style="font-size:1.2em">${currentDisplayTemp.toFixed(1)} °C</td> <!-- Usa temp_display -->
                                    <td>${setpoint} °C</td>
                                    <td>
                                        <button onclick="setRelay(${idx}, 1)">ON</button>
                                        <button onclick="setRelay(${idx}, 0)">OFF</button>
                                        <span style="margin-left: 10px;">${relayState ? '🔥 SCALDA' : '⏸ RIPOSO'}</span> <!-- Aggiungi stato relè -->
                                    </td>
                                </tr>
                            `;
                            if(data.program_mode === 2 && (idx + 1) < 12) {
                                const pairedIdx = idx + 1;
                                const pairedSensor = data.sensors.find(s => s.index === pairedIdx);
                                if(pairedSensor) {
                                    const pairedDisplayTemp = parseFloat(pairedSensor.temp_display);
                                    const pairedSetpoint = data.relays[pairedIdx] ? data.relays[pairedIdx].setpoint : 20.0;
                                    let pairedStatusClass = statusClass;
                                    let pairedStatusText = '(Stesso controllo Zona ' + (idx + 1) + ')';
                                    let pairedRelayStateText = relayStateText;

                                    row += `
                                        <tr class="${pairedStatusClass}">
                                            <td><strong>Zona ${pairedIdx + 1} (Accoppiata)</strong></td>
                                            <td>${pairedStatusText}</td>
                                            <td style="font-size:1.2em">${pairedDisplayTemp.toFixed(1)} °C</td>
                                            <td>${pairedSetpoint} °C</td>
                                            <td>${pairedRelayStateText}</td>
                                        </tr>`;
                                }
                            }
                            tbody.innerHTML += row;
                        });
                    } else if(data.program_mode === 3) {
                        let cycleStatusRow = `<tr><td colspan="5"><h3>Stato Cicli: `;
                        if(data.cycles_completed) {
                            cycleStatusRow += `COMPLETATI!</h3><p>Processo terminato. Controllare il dispositivo.</p>`;
                        } else if(data.cycles_running) {
                            cycleStatusRow += `In Esecuzione</h3><p>Ciclo #${data.current_cycle_index + 1} | SP:${data.current_cycle_target_temp}°C | Rampa:${data.current_cycle_ramp_rate}°C/h | Durata:${data.current_cycle_duration_min}min</p>`;
                        } else {
                            cycleStatusRow += ` Fermi</h3><p>Avviare i cicli dalla scheda dedicata.</p>`;
                        }
                        cycleStatusRow += `</td></tr>`;
                        tbody.innerHTML = cycleStatusRow;
                    }
                })
                .catch(err => console.error("Errore lettura dati:", err));
        }

        // --- FUNZIONI PARTENZA RITARDATA ---
        function setDelayedStart() {
            const hours = document.getElementById('ds-hours').value;
            const minutes = document.getElementById('ds-minutes').value;
            if(hours === '' || minutes === '') {
                alert("Inserisci un'ora e minuti validi.");
                return;
            }
            fetch(`/control?set_delayed_start=1&hours=${hours}&minutes=${minutes}`)
                .then(res => res.json())
                .then(data => {
                    if(data.ok) {
                        alert("Partenza ritardata impostata!");
                        updateData();
                    } else {
                        alert("Errore: " + data.message);
                    }
                });
        }

        function cancelDelayedStart() {
            fetch(`/control?cancel_delayed_start=1`)
                .then(res => res.json())
                .then(data => {
                    if(data.ok) {
                        alert("Partenza ritardata annullata!");
                        updateData();
                    } else {
                        alert("Errore: " + data.message);
                    }
                });
        }

        function formatTime(timestamp) {
            const date = new Date(timestamp * 1000);
            const hours = String(date.getHours()).padStart(2, '0');
            const minutes = String(date.getMinutes()).padStart(2, '0');
            return `${hours}:${minutes}`;
        }

        function updateDelayedStartUI(data) {
            const dsSection = document.getElementById('delayed-start-section');
            const dsStatusText = document.getElementById('ds-status-text');
            const dsControls = document.getElementById('ds-controls');
            const dsCountdown = document.getElementById('ds-countdown');

            if(countdownInterval) clearInterval(countdownInterval); // Ferma countdown precedente
            countdownInterval = null;

            if(data.program_mode === 1) {
                dsSection.style.display = 'block';

                if(data.delayed_start_enabled) {
                    dsStatusText.textContent = `In attesa dell'ora impostata: ${formatTime(data.delayed_start_timestamp)}.`;
                    dsControls.style.display = 'none';
                    dsCountdown.style.display = 'block';
                    updateCountdown(data.remaining_seconds);
                    countdownInterval = setInterval(() => {
                        const currentRemaining = parseInt(dsCountdown.getAttribute('data-remaining')) - 1;
                        if(currentRemaining >= 0) {
                            dsCountdown.setAttribute('data-remaining', currentRemaining);
                            updateCountdown(currentRemaining);
                        } else {
                            clearInterval(countdownInterval);
                            countdownInterval = null;
                        }
                    }, 1000);
                } else if(!data.control_active) {
                    dsStatusText.textContent = "Controllo in fase di attivazione...";
                    dsControls.style.display = 'none';
                    dsCountdown.style.display = 'none';
                } else {
                    dsStatusText.textContent = "Partenza immediata attiva.";
                    dsControls.style.display = 'block';
                    dsCountdown.style.display = 'none';
                }
            } else {
                dsSection.style.display = 'none';
            }
        }

        function updateCountdown(seconds) {
            const dsCountdown = document.getElementById('ds-countdown');
            dsCountdown.setAttribute('data-remaining', seconds);
            if(seconds > 0) {
                const hours = Math.floor(seconds / 3600);
                const minutes = Math.floor((seconds % 3600) / 60);
                const secs = seconds % 60;
                dsCountdown.textContent = `Avvio tra: ${hours.toString().padStart(2, '0')}h ${minutes.toString().padStart(2, '0')}m ${secs.toString().padStart(2, '0')}s`;
            } else {
                dsCountdown.textContent = "Avvio imminente...";
            }
        }

        // --- FUNZIONI PROGRAMMA E CICLI ---
        function setProgramMode(mode) {
            fetch(`/control?set_program_mode=1&mode=${mode}`)
                .then(res => res.json())
                .then(data => {
                    if(data.ok) {
                        alert(`Modalità programma cambiata a ${mode}. Ricarico...`);
                        location.reload();
                    } else {
                        alert("Errore: " + data.message);
                    }
                });
        }

        function loadCycles() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    if(data.program_mode === 3) {
                        const container = document.getElementById('cycle-inputs');
                        container.innerHTML = '';
                        for(let i = 0; i < 10; i++) {
                            let cycle = data.cycles_list && data.cycles_list[i] ? data.cycles_list[i] : {target: 20.0, ramp: 0.0, duration: 0.0};
                            let cycleHtml = `
                                <div class="cycle-group" id="cycle-${i}">
                                    <h3>Ciclo ${i+1}</h3>
                                    <div class="form-group">
                                        <label>Temp. Finale (°C):</label>
                                        <input type="number" step="0.1" id="target_${i}" value="${cycle.target}">
                                    </div>
                                    <div class="form-group">
                                        <label>Rampa (°C/h):</label>
                                        <input type="number" step="0.1" id="ramp_${i}" value="${cycle.ramp}">
                                    </div>
                                    <div class="form-group">
                                        <label>Durata (min):</label>
                                        <input type="number" step="1" id="duration_${i}" value="${cycle.duration}">
                                    </div>
                                </div>
                            `;
                            container.innerHTML += cycleHtml;
                        }
                        if(data.cycles_running) {
                            document.getElementById('cycles-title').textContent = `🔄 Cicli - Attivo: #${data.current_cycle_index + 1}`;
                        } else if(data.cycles_completed) {
                            document.getElementById('cycles-title').textContent = `🔄 Cicli - COMPLETATI!`;
                        } else {
                            document.getElementById('cycles-title').textContent = `🔄 Cicli - Fermi`;
                        }
                    } else {
                        document.getElementById('cycles-section').innerHTML = '<h2>Errore: Accedi a questa pagina solo in modalità Cicli</h2>';
                    }
                });
        }

        function saveCycles() {
            let formData = [];
            for(let i = 0; i < 10; i++) {
                const target = document.getElementById(`target_${i}`).value;
                const ramp = document.getElementById(`ramp_${i}`).value;
                const duration = document.getElementById(`duration_${i}`).value;
                if(target !== '' && ramp !== '' && duration !== '') {
                    formData.push(`target_${i}=${target}`);
                    formData.push(`ramp_${i}=${ramp}`);
                    formData.push(`duration_${i}=${duration}`);
                } else {
                    break;
                }
            }
            const queryString = formData.join('&');
            fetch(`/control?set_cycles=1&${queryString}`)
                .then(res => res.json())
                .then(data => {
                    if(data.ok) {
                        alert("Cicli salvati!");
                        loadCycles();
                    } else {
                        alert("Errore: " + data.message);
                    }
                });
        }

        function startCycles() {
            fetch(`/control?start_cycles=1`)
                .then(res => res.json())
                .then(data => {
                    if(data.ok) {
                        alert("Cicli avviati!");
                        loadCycles();
                    } else {
                        alert("Errore: " + data.message);
                    }
                });
        }

        function stopCycles() {
            fetch(`/control?stop_cycles=1`)
                .then(res => res.json())
                .then(data => {
                    if(data.ok) {
                        alert("Cicli fermati!");
                        loadCycles();
                    } else {
                        alert("Errore: " + data.message);
                    }
                });
        }

        // --- FUNZIONI GENERALI ---
        function setRelay(index, state) {
            fetch(`/control?relay=${index}&state=${state}`)
                .then(res => updateData());
        }

      function saveSettings() {
            const gain = document.getElementById('cal_gain').value;
            const offset = document.getElementById('adc_offset').value;
            const kp = document.getElementById('pid_kp').value;
            const ki = document.getElementById('pid_ki').value;
            const kd = document.getElementById('pid_kd').value;
            
            fetch(`/control?save_settings=1&cal_gain=${gain}&adc_offset=${offset}&pid_kp=${kp}&pid_ki=${ki}&pid_kd=${kd}`)
                .then(res => res.json())
                .then(data => {
                    alert("✅ Impostazioni salvate! Ricarico la pagina...");
                    location.reload();
                });
        }

        // Avvio loop aggiornamento (ogni 2 secondi)
        setInterval(updateData, 2000);
        updateData(); // Carica immediatamente
    </script>
</body>
</html>
  )rawliteral");
}

void handleIdentity() {
  String json = "{";
  json += "\"device\":\"" + String(DEVICE_NAME) + "\",";
  json += "\"device_id\":\"" + String(DEVICE_ID) + "\",";
  json += "\"hostname\":\"" + String(HOSTNAME) + "\",";
  json += "\"hostname_local\":\"" + String(HOSTNAME) + ".local\",";
  json += "\"firmware_version\":\"" + String(FIRMWARE_VERSION) + "\",";
  json += "\"ota_ready\":true,";
  json += "\"ota_endpoint\":\"/update\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"wifi_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleData() {
  String json = "{";
  json += "\"device\":\"" + String(DEVICE_NAME) + "\",";
  json += "\"device_id\":\"" + String(DEVICE_ID) + "\",";
  json += "\"hostname\":\"" + String(HOSTNAME) + "\",";
  json += "\"hostname_local\":\"" + String(HOSTNAME) + ".local\",";
  json += "\"firmware_version\":\"" + String(FIRMWARE_VERSION) + "\",";
  json += "\"ota_ready\":true,";
  json += "\"wifi_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"uptime_ms\":" + String(millis()) + ",";
  json += "\"vcc_mon\":" + String(measuredVcc, 3) + ",";
  json += "\"channels_total\":" + String(NUM_MUX_CHANNELS) + ",";
  json += "\"sensor_count\":" + String(NUM_SENSORS) + ",";
  json += "\"relay_count\":" + String(NUM_RELAYS) + ",";
  // Aggiungi dopo json += "\"relay_count\":" + String(NUM_RELAYS) + ",";
json += "\"lastSampleId\":" + String(nextSampleId > 0 ? nextSampleId - 1 : 0) + ",";

  json += "\"sensors\":[";
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i > 0) json += ",";
    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"mux_channel\":" + String(sensorMuxChannel[i]) + ",";
    json += "\"adc_raw\":" + String(sensorAdcRaw[i]) + ",";
    json += "\"voltage\":" + numOrNull(sensorVoltage[i], 4) + ",";
    json += "\"resistance\":" + numOrNull(sensorResistance[i], 2) + ",";
    json += "\"temp_raw\":" + numOrNull(sensorTempRaw[i], 2) + ",";
    json += "\"temp_corr\":" + numOrNull(sensorTempCorr[i], 2) + ","; // Temperatura reale
    json += "\"temp_control\":" + numOrNull(getControlTemperature(i, sensorTempCorr[i]), 2) + ","; // Temperatura cliente
    json += "\"temp_display\":" + numOrNull(getDisplayTemperature(i, sensorTempCorr[i]), 2) + ","; // Temperatura visualizzata
    json += "\"temp_dev\":" + numOrNull(getDevTemperature(i, sensorTempCorr[i]), 2) + ","; // Temperatura sviluppatore
    json += "\"temp_display_dev\":" + numOrNull(getDevTemperature(i, sensorTempCorr[i]), 2) + ",";

    json += "\"user_offset_percent\":" + String(userTempOffsetPercent[i], 2) + ",";
    json += "\"display_offset_percent\":" + String(displayTempOffsetPercent[i], 2) + ",";
    json += "\"dev_display_offset_percent\":" + String(DevDisplayOffsetPercent[i], 2) + ",";
    json += "\"developer_offset_percent\":" + String(DevDisplayOffsetPercent[i], 2); // Nessuna virgola finale qui
    json += "}";
  }
  json += "],";

  // Aggiungi stato relè agli elementi relays
  json += "\"relays\":[";
  for (int r = 0; r < NUM_RELAYS; r++) {
    if (r > 0) json += ",";
    json += "{";
    json += "\"index\":" + String(r) + ",";
    json += "\"enabled\":" + String(relayEnabled[r] ? "true" : "false") + ",";
    json += "\"pin\":" + String(relayPin[r]) + ",";
    json += "\"active_low\":" + String(relayActiveLow[r] ? "true" : "false") + ",";
    json += "\"sensor_index\":" + String(relaySensorIndex[r]) + ",";
    json += "\"setpoint\":" + String(relaySetpoint[r], 2) + ",";
    json += "\"hysteresis\":" + String(relayHysteresis[r], 2) + ",";
    json += "\"state\":\"" + String(relayText(relayState[r])) + "\",";
    json += "\"on\":" + String(relayState[r] ? "true" : "false");
    json += "}";
  }
  json += "]";
  json += "}";
  server.send(200, "application/json", json);
}

void handleDataBuffer() {
  uint32_t afterId = 0;
  int limit = 100;

  if (server.hasArg("after")) {
    afterId = (uint32_t)server.arg("after").toInt();
  }

  if (server.hasArg("limit")) {
    limit = server.arg("limit").toInt();
    if (limit < 1) limit = 1;
    if (limit > 100) limit = 100;   // limite duro per evitare JSON enormi
  }

  String json;
  json.reserve(12000);  // aiuta a ridurre frammentazione
  json = "{";
  json += "\"items\":[";

  bool first = true;
  int emitted = 0;

  size_t startIndex = (dataBufferHead + DATA_BUFFER_SIZE - dataBufferCount) % DATA_BUFFER_SIZE;

  for (size_t n = 0; n < dataBufferCount; n++) {
    size_t idx = (startIndex + n) % DATA_BUFFER_SIZE;
    const BufferedSample &s = dataBuffer[idx];

    if (s.sampleId == 0) continue;
    if (s.sampleId <= afterId) continue;
    if (emitted >= limit) break;

    if (!first) json += ",";
    first = false;

    json += "{";
    json += "\"sampleId\":" + String(s.sampleId);
    json += ",\"timestampMs\":" + String(s.timestampMs);
    json += ",\"sensorIndex\":" + String(s.sensorIndex);
    json += ",\"tempReal\":" + String(s.tempReal, 2);
    json += ",\"tempControl\":" + String(s.tempControl, 2);
    json += ",\"tempDisplay\":" + String(s.tempDisplay, 2);
    json += ",\"relayState\":" + String(s.relayState ? "true" : "false");
    json += ",\"setpoint\":" + String(s.setpoint, 2);
    json += ",\"pidOutput\":" + String(s.pidOutput, 2);
    json += "}";

    emitted++;
  }

  json += "]";
  json += ",\"latestSampleId\":" + String(nextSampleId > 0 ? nextSampleId - 1 : 0);
  json += ",\"bufferCount\":" + String(dataBufferCount);
  json += "}";

  server.send(200, "application/json", json);
}

void handleSensorHistory() {
  int sensor = -1;
  uint32_t afterId = 0;
  int limit = 200;

  if (server.hasArg("sensor")) {
    sensor = server.arg("sensor").toInt();
  }

  if (sensor < 0 || sensor >= NUM_SENSORS) {
    server.send(400, "application/json", "{\"error\":\"invalid sensor\"}");
    return;
  }

  if (server.hasArg("after")) {
    afterId = (uint32_t) server.arg("after").toInt();
  }

  if (server.hasArg("limit")) {
    limit = server.arg("limit").toInt();
    if (limit < 1) limit = 1;
    if (limit > (int)DATA_BUFFER_SIZE) limit = DATA_BUFFER_SIZE;
  }

  String json = "{";
  json += "\"sensor\":" + String(sensor) + ",";
  json += "\"items\":[";

  bool first = true;
  int emitted = 0;

  size_t startIndex = (dataBufferHead + DATA_BUFFER_SIZE - dataBufferCount) % DATA_BUFFER_SIZE;

  for (size_t n = 0; n < dataBufferCount; n++) {
    size_t i = (startIndex + n) % DATA_BUFFER_SIZE;
    const BufferedSample &s = dataBuffer[i];

    if (s.sampleId == 0) continue;
    if (s.sampleId <= afterId) continue;
    if (s.sensorIndex != sensor) continue;
    if (emitted >= limit) break;

    if (!first) json += ",";
    first = false;

    json += "{";
    json += "\"sampleId\":" + String(s.sampleId);
    json += ",\"timestampMs\":" + String(s.timestampMs);
    json += ",\"tempReal\":" + String(s.tempReal, 2);
    json += ",\"tempControl\":" + String(s.tempControl, 2);
    json += ",\"tempDisplay\":" + String(s.tempDisplay, 2);
    json += ",\"relayState\":" + String(s.relayState ? "true" : "false");
    json += ",\"setpoint\":" + String(s.setpoint, 2);
    json += ",\"pidOutput\":" + String(s.pidOutput, 2);
    json += "}";

    emitted++;
  }

  json += "]";
  json += ",\"latestSampleId\":" + String(nextSampleId > 0 ? nextSampleId - 1 : 0);
  json += "}";

  server.send(200, "application/json", json);
}

// >>> GESTIONE COMANDI APP (AGGIUNTO - SETPOINT/ISTERESI/RELÈ)
void handleControl() {
  // Gestione degli offset utente e display (solo per canali specifici)
  if (server.hasArg("set_temp_offset")) {
      int sensorIdx = server.arg("channel").toInt();
      if (sensorIdx >= 0 && sensorIdx < NUM_SENSORS) {
          if(server.hasArg("user_offset_percent")) {
              userTempOffsetPercent[sensorIdx] = server.arg("user_offset_percent").toFloat();
              prefs.begin("offsets", false);
              prefs.putFloat(("user_off_" + String(sensorIdx)).c_str(), userTempOffsetPercent[sensorIdx]);
              prefs.end();
          }
          if(server.hasArg("display_offset_percent")) {
              displayTempOffsetPercent[sensorIdx] = server.arg("display_offset_percent").toFloat();
              prefs.begin("offsets", false);
              prefs.putFloat(("disp_off_" + String(sensorIdx)).c_str(), displayTempOffsetPercent[sensorIdx]);
              prefs.end();
          }
          server.send(200, "application/json", "{\"ok\":true, \"message\":\"Offset impostati\"}");
          return;
      } else {
          server.send(400, "application/json", "{\"ok\":false, \"message\":\"Canale non valido\"}");
          return;
      }
  }

  // Gestione offset sviluppatore (nuovo)
  if (server.hasArg("set_dev_display_offset")) {
      int sensorIdx = server.arg("channel").toInt();
      if (sensorIdx >= 0 && sensorIdx < NUM_SENSORS) {
          if(server.hasArg("dev_display_offset_percent")) {
              DevDisplayOffsetPercent[sensorIdx] = server.arg("dev_display_offset_percent").toFloat();
              prefs.begin("dev_offsets", false);
              prefs.putFloat(("dev_disp_off_" + String(sensorIdx)).c_str(), DevDisplayOffsetPercent[sensorIdx]);
              prefs.end();
          }
          server.send(200, "application/json", "{\"ok\":true, \"message\":\"Offset sviluppatore impostato\"}");
          return;
      } else {
          server.send(400, "application/json", "{\"ok\":false, \"message\":\"Canale non valido\"}");
          return;
      }
  }

  // Gestione impostazioni Generali (Gain, Offset, NumSensors, PID Params)
  if (server.hasArg("save_settings")) {
    if (server.hasArg("cal_gain")) {
      CAL_GAIN = server.arg("cal_gain").toFloat();
      prefs.begin("settings", false);
      prefs.putFloat("gain", CAL_GAIN);
      prefs.end();
    }
    if (server.hasArg("adc_offset")) {
      ADC_OFFSET = server.arg("adc_offset").toFloat();
      prefs.begin("settings", false);
      prefs.putFloat("offset", ADC_OFFSET);
      prefs.end();
    }
    if (server.hasArg("pid_kp")) {
      DEFAULT_KP = server.arg("pid_kp").toFloat();
      for(int i = 0; i < NUM_SENSORS; i++) {
        double ki, kd;
        ki = pidController[i].GetKi();
        kd = pidController[i].GetKd();
        pidController[i].SetTunings(DEFAULT_KP, ki, kd); // Aggiorna solo Kp
      }
    }
    if (server.hasArg("pid_ki")) {
      DEFAULT_KI = server.arg("pid_ki").toFloat();
      for(int i = 0; i < NUM_SENSORS; i++) {
        double kp, kd;
        kp = pidController[i].GetKp();
        kd = pidController[i].GetKd();
        pidController[i].SetTunings(kp, DEFAULT_KI, kd);
      }
    }
    if (server.hasArg("pid_kd")) {
      DEFAULT_KD = server.arg("pid_kd").toFloat();
      for(int i = 0; i < NUM_SENSORS; i++) {
        double kp, ki;
        kp = pidController[i].GetKp();
        ki = pidController[i].GetKi();
        pidController[i].SetTunings(kp, ki, DEFAULT_KD);
      }
    }
    server.send(200, "application/json", "{\"ok\":true, \"message\":\"Impostazioni salvate\"}");
    return;
  }

  // Gestione Relay e Setpoint (come prima)
  if (!server.hasArg("relay") && !server.hasArg("channel")) {
    server.send(400, "text/plain", "Missing index");
    return;
  }

  int r = server.hasArg("channel") ? server.arg("channel").toInt() : server.arg("relay").toInt();
  if (r < 0 || r >= NUM_RELAYS) {
    server.send(400, "text/plain", "Invalid index");
    return;
  }

  bool changedConfig = false;
  bool changedState = false;

  if (server.hasArg("state")) {
    bool state = server.arg("state") == "1";
    writeRelay(r, state);
    changedState = true;
    Serial.print("📱 Comando APP: Relay ");
    Serial.print(r);
    Serial.println(state ? " -> ON" : " -> OFF");
  }

  if (server.hasArg("setpoint")) {
    relaySetpoint[r] = server.arg("setpoint").toFloat();
    changedConfig = true;
  }

  if (server.hasArg("hysteresis")) {
    relayHysteresis[r] = server.arg("hysteresis").toFloat();
    changedConfig = true;
  }

  if (server.hasArg("enabled")) {
    relayEnabled[r] = (server.arg("enabled") == "1");
    changedConfig = true;
  }

  if (server.hasArg("sensor_index")) {
    relaySensorIndex[r] = server.arg("sensor_index").toInt();
    changedConfig = true;
  }

  if (changedConfig) {
    saveRelayParams(r);
  }

  updateRelayControl();

  String json = "{";
  json += "\"ok\":true,";
  json += "\"relay\":" + String(r) + ",";
  json += "\"channel\":" + String(r) + ",";
  json += "\"changed_state\":" + String(changedState ? "true" : "false") + ",";
  json += "\"changed_config\":" + String(changedConfig ? "true" : "false") + ",";
  json += "\"enabled\":" + String(relayEnabled[r] ? "true" : "false") + ",";
  json += "\"sensor_index\":" + String(relaySensorIndex[r]) + ",";
  json += "\"setpoint\":" + String(relaySetpoint[r], 2) + ",";
  json += "\"hysteresis\":" + String(relayHysteresis[r], 2) + ",";
  json += "\"state\":\"" + String(relayText(relayState[r])) + "\",";
  json += "\"vcc_mon\":" + String(measuredVcc, 3) + ",";
  json += "\"on\":" + String(relayState[r] ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleOptions() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(204);
}


// =====================
// OTA / PROGRAMMAZIONE ESP32 DA APP
// =====================
bool isOtaAuthorized() {
  if (server.hasArg("token") && server.arg("token") == OTA_TOKEN) return true;
  if (server.hasHeader("X-OTA-Token") && server.header("X-OTA-Token") == OTA_TOKEN) return true;
  return false;
}

void handleFirmwareInfo() {
  String json = "{";
  json += "\"ok\":true,";
  json += "\"device\":\"" + String(DEVICE_NAME) + "\",";
  json += "\"device_id\":\"" + String(DEVICE_ID) + "\",";
  json += "\"hostname\":\"" + String(HOSTNAME) + "\",";
  json += "\"hostname_local\":\"" + String(HOSTNAME) + ".local\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"firmware_version\":\"" + String(FIRMWARE_VERSION) + "\",";
  json += "\"ota_ready\":true,";
  json += "\"ota_endpoint\":\"/update\",";
  json += "\"compile_flow\":\"app_editor_to_compile_server_to_bin_to_ota\",";
  json += "\"ota_in_progress\":" + String(otaUpdateInProgress ? "true" : "false") + ",";
  json += "\"last_update_ok\":" + String(otaUpdateOk ? "true" : "false") + ",";
  json += "\"last_error\":\"" + otaLastError + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleFirmwareUpdatePost() {
  if (!isOtaAuthorized()) {
    server.send(401, "application/json", "{\"ok\":false,\"error\":\"OTA non autorizzato\"}");
    return;
  }

  if (otaLastError.length() > 0) {
    otaUpdateInProgress = false;
    otaUpdateOk = false;
    String msg = "{\"ok\":false,\"error\":\"" + otaLastError + "\"}";
    server.send(500, "application/json", msg);
    return;
  }

  if (Update.hasError()) {
    otaUpdateInProgress = false;
    otaUpdateOk = false;
    if (otaLastError.length() == 0) otaLastError = "Update.hasError()";
    server.send(500, "application/json", "{\"ok\":false,\"error\":\"Aggiornamento fallito\"}");
    return;
  }

  otaUpdateInProgress = false;
  otaUpdateOk = true;
  otaLastError = "";
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"Firmware ricevuto. Riavvio ESP32...\"}");
  delay(700);
  ESP.restart();
}

void handleFirmwareUpdateUpload() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    otaUpdateInProgress = true;
    otaUpdateOk = false;
    otaLastError = "";

    Serial.println("\n========== OTA UPDATE ==========");
    Serial.printf("File OTA: %s\n", upload.filename.c_str());
    Serial.println("Arresto temporaneo controllo relè durante OTA...");

    // Sicurezza: durante aggiornamento firmware tutti i relè OFF.
    for (int r = 0; r < NUM_RELAYS; r++) {
      writeRelay(r, false);
      relayState[r] = false;
    }

    if (!isOtaAuthorized()) {
      otaLastError = "Token OTA non valido";
      Serial.println("❌ OTA rifiutato: token non valido");
      return;
    }

    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      otaLastError = "Update.begin fallito";
      Update.printError(Serial);
      return;
    }
  }

  else if (upload.status == UPLOAD_FILE_WRITE) {
    if (!otaUpdateInProgress || otaLastError.length() > 0) return;

    size_t written = Update.write(upload.buf, upload.currentSize);
    if (written != upload.currentSize) {
      otaLastError = "Scrittura flash incompleta";
      Update.printError(Serial);
    }
  }

  else if (upload.status == UPLOAD_FILE_END) {
    if (!otaUpdateInProgress || otaLastError.length() > 0) {
      Update.abort();
      Serial.println("❌ OTA abortito");
      return;
    }

    if (Update.end(true)) {
      Serial.printf("✅ OTA completato: %u bytes\n", upload.totalSize);
    } else {
      otaLastError = "Update.end fallito";
      Update.printError(Serial);
    }
    Serial.println("================================\n");
  }

  else if (upload.status == UPLOAD_FILE_ABORTED) {
    otaUpdateInProgress = false;
    otaUpdateOk = false;
    otaLastError = "Upload OTA abortito";
    Update.abort();
    Serial.println("❌ OTA abortito dal client");
  }
}

void handleWifiReset() {
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"Reset WiFi in corso\"}");
  Serial.println("🔄 Reset WiFi richiesto via web...");
  delay(300);
  wifiManager.resetSettings();
  delay(700);
  ESP.restart();
}

void handleNotFound() {
  String msg = "{";
  msg += "\"error\":\"Not Found\",";
  msg += "\"available\":[\"/\",\"/identity\",\"/data\",\"/control\",\"/wifi-reset\"]";
  msg += "}";
  server.send(404, "application/json", msg);
}

// =====================
// SETUP
// =====================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println();
  Serial.println("🚀 BOOT OK");
  Serial.println("ESP32 + CD74HC4067 + PT1000 + RELAY ARRAY");
  Serial.println("==========================================");

  bootId = (uint32_t)esp_random();

  // Init pin MUX
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(S3_PIN, OUTPUT);
  digitalWrite(S0_PIN, LOW);
  digitalWrite(S1_PIN, LOW);
  digitalWrite(S2_PIN, LOW);
  digitalWrite(S3_PIN, LOW);

  // Init pin Relè
  for (int r = 0; r < NUM_RELAYS; r++) {
    pinMode(relayPin[r], OUTPUT);
    writeRelay(r, false);
  }

  // Init ADC
  analogReadResolution(12);
  analogSetPinAttenuation(SIG_PIN, ADC_11db);
  analogSetPinAttenuation(VCC_MONITOR_PIN, ADC_11db);
  pinMode(VCC_MONITOR_PIN, INPUT);

  // Carica parametri salvati in EEPROM
  Serial.println("📦 Caricamento parametri da EEPROM...");
  loadRelayParams();
  Serial.println("✅ Parametri caricati");

  // Carica offset sviluppatore da Preferences (se esistono), altrimenti usa i valori iniziali
  Serial.println("📦 Caricamento offset sviluppatore da Preferences (o usa valori iniziali)...");
  prefs.begin("dev_offsets", true); // true = readonly
  for(int i = 0; i < NUM_SENSORS; i++) {
    String key = "dev_disp_off_" + String(i);
    if(prefs.isKey(key.c_str())) { // Controlla se la chiave esiste
      DevDisplayOffsetPercent[i] = prefs.getFloat(key.c_str(), DevDisplayOffsetPercent[i]); // Usa valore da prefs, altrimenti mantieni valore iniziale
    }
    // Se la chiave non esiste, DevDisplayOffsetPercent[i] rimane quello iniziale
  }
  prefs.end();
  Serial.println("✅ Offset sviluppatore caricati.");

  // Test relè all'avvio
  testAllRelaysStartup();

  // WiFi + mDNS (con WiFiManager per configurazione hotspot)
  Serial.println("🔌 Avvio WiFi Manager...");
  startWiFiSmart();

  // Web Server routes
  const char* otaHeaderKeys[] = {"X-OTA-Token"};
  server.collectHeaders(otaHeaderKeys, 1);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/identity", HTTP_GET, handleIdentity);
  server.on("/firmware", HTTP_GET, handleFirmwareInfo);
  server.on("/update", HTTP_OPTIONS, handleOptions);
  server.on("/update", HTTP_POST, handleFirmwareUpdatePost, handleFirmwareUpdateUpload);
  server.on("/data", HTTP_GET, handleData);
  server.on("/data-buffer", HTTP_GET, handleDataBuffer);
  server.on("/sensor-history", HTTP_GET, handleSensorHistory);
  server.on("/control", HTTP_GET, handleControl);
  server.on("/control", HTTP_POST, handleControl);
  server.on("/control", HTTP_OPTIONS, handleOptions);
  server.on("/wifi-reset", HTTP_GET, handleWifiReset);
  server.on("/wifi-reset", HTTP_POST, handleWifiReset);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("🌐 Web server avviato");

  // Prime misure
  updateMeasurements();
  updateRelayControl();

  // Report iniziale
  printFullReport();

  lastReportMs = millis();
  lastMeasureMs = millis();

    // Inizializza i PID
  for(int i = 0; i < NUM_SENSORS; i++){
    pidController[i].SetMode(AUTOMATIC);
    pidController[i].SetSampleTime(PID_SAMPLE_TIME_MS);
    //pidController[i].SetOutputLimits(0, PID_SAMPLE_TIME_MS);
    pidController[i].SetOutputLimits(0, 100);
    setpointPID[i] = relaySetpoint[i];
    // Inizializza i dati di tuning
    tuning[i].originalKp = DEFAULT_KP;
    tuning[i].originalKi = DEFAULT_KI;
    tuning[i].originalKd = DEFAULT_KD;
    tuning[i].tuningStartTime = millis();
  }
  Serial.println("✅ PID Controllers e Auto-Tuning inizializzati.");
}

// =====================
// LOOP
// =====================
void loop() {
  server.handleClient();

  // Durante upload OTA non facciamo misure/controllo relè:
  // restano OFF fino al riavvio, evitando stati strani mentre la flash viene scritta.
  if (otaUpdateInProgress) {
    for (int r = 0; r < NUM_RELAYS; r++) writeRelay(r, false);
    delay(2);
    return;
  }

  // NOTA: MDNS.update() RIMOSSO - non esiste in ESP32 core 3.3.7
  // Il servizio mDNS funziona automaticamente dopo MDNS.begin()

  // Reset WiFi da serial monitor (comando: resetwifi)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "resetwifi") {
      Serial.println("🔄 Reset WiFi da serial monitor...");
      wifiManager.resetSettings();
      delay(1000);
      ESP.restart();
    }
  }

  // Reset WiFi da button BOOT (3 secondi di pressione)
  static unsigned long btnPressStart = 0;
  if (digitalRead(WIFI_RESET_BUTTON) == LOW) {
    if (btnPressStart == 0) {
      btnPressStart = millis();
    } else if (millis() - btnPressStart > 3000UL) {
      Serial.println("🔄 Reset WiFi da button BOOT...");
      wifiManager.resetSettings();
      delay(1000);
      ESP.restart();
    }
  } else {
    btnPressStart = 0;
  }

  // Controllo connessione WiFi periodico
  static unsigned long lastWifiCheck = 0;
  if (millis() - lastWifiCheck >= 15000UL) {
    lastWifiCheck = millis();
    ensureWiFiConnected();
  }

  // Report periodico su Serial
  if (millis() - lastReportMs >= REPORT_INTERVAL_MS) {
    lastReportMs = millis();
    printFullReport();
  }
  if (millis() - lastMeasureMs >= MEASURE_INTERVAL_MS) {
    lastMeasureMs = millis();
    updateMeasurements();
    //updateMeasurements();  // ✅ CORRETTO
    updateRelayControl(); // Usa la nuova funzione
  }

  yield();
}