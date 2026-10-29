/*
  AQUA-TARA - Node sensor (Heltec WiFi LoRa 32 V3)
  - Baca sensor tekanan (GPIO4) dan turbidity (GPIO5), tampil di OLED
  - Kirim paket LoRa tiap 10 detik ke gateway (Heltec kedua)

  Frekuensi: 921,4 MHz (kanal default AS923-2) -> di dalam pita LPWAN Indonesia 920-923 MHz.
  PASANG ANTENA SEBELUM MENYALAKAN BOARD (mengirim tanpa antena bisa merusak penguat radio).

  Format paket (CSV):  id,seq,turbidity_idx,tekanan_bar,flags
    contoh: 15,203,12.3,1.85,0
    flags (bit): 1 = sensor tekanan rusak/terputus, 2 = sensor turbidity rusak/terputus
*/
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <Wire.h>

// ================= Identitas node =================
#define NODE_ID 15            // isi dengan kolom id alat ini di tabel iot_nodes

// ================= Konfigurasi LoRa =================
#define RF_FREQUENCY                921400000   // Hz (AS923-2 kanal 1, di dalam 920-923 MHz)
#define TX_OUTPUT_POWER             10          // dBm (+ antena ~2 dBi = sekitar 16 mW EIRP)
#define LORA_BANDWIDTH              0           // 0 = 125 kHz
#define LORA_SPREADING_FACTOR       7           // SF7: paket ~60 ms -> 10 dtk sekali = ~0,6% duty cycle
#define LORA_CODINGRATE             1           // 4/5
#define LORA_PREAMBLE_LENGTH        8
#define LORA_FIX_LENGTH_PAYLOAD_ON  false
#define LORA_IQ_INVERSION_ON        false
#define BUFFER_SIZE                 48

const uint32_t LORA_INTERVAL_MS = 10000;   // kirim tiap 10 detik
const uint32_t READ_INTERVAL_MS = 1000;    // baca sensor + OLED tiap 1 detik

// ================= Pin & ADC =================
#define PRESSURE_PIN  4   // lewat divider 10k/15k
#define TURBIDITY_PIN 5   // lewat divider 10k/15k

// 0 = konversi lama (anchor kalibrasi tetap berlaku)
// 1 = analogReadMilliVolts (lebih akurat, TAPI semua anchor tegangan di bawah harus diukur ulang)
#define USE_CALIBRATED_ADC 0

const float DIVIDER_RATIO = 1.6667;   // (10k+15k)/15k
const float ESP32_VREF    = 3.3;
const int   ADC_MAX       = 4095;

// ================= Sensor tekanan (Marnov 1,2 MPa) =================
// Ukur PRESSURE_V_MIN: sensor di udara terbuka (0 bar), lihat "V=" di Serial Monitor.
const float PRESSURE_V_MIN   = 0.5;
const float PRESSURE_V_MAX   = 4.5;
const float PRESSURE_MAX_BAR = 12.0;
const float PRESSURE_FAULT_LOW  = 0.30;   // di bawah ini = kabel putus/korslet
const float PRESSURE_FAULT_HIGH = 4.80;
const float PRESSURE_ZERO_BAR   = 0.05;   // di bawah ini dianggap tidak ada tekanan

// ================= Turbidity (tipe transmisif) =================
// Hasil bench: air bening 3.44 V = 0 NTU | teh 3.39 V = ~35 | jeruk 3.00 V = ~312 | kopi pekat 2.03 V = 1000
// Voltase TURUN = air MAKIN KERUH. Nilai NTU = perkiraan relatif, belum dikalibrasi dengan turbidimeter.
const float TURBIDITY_V_CLEAN = 3.0;
const float TURBIDITY_V_DIRTY = 1.5;
const float TURBIDITY_MAX_NTU = 100.0;
const float TURBIDITY_FAULT_LOW = 0.10;   // terputus / korslet (tisu menutup = 0.24 V masih sah)

const float NTU_WARN = 50.0;
const float NTU_CRIT = 400.0;

// ================= Variabel global =================
static RadioEvents_t RadioEvents;
char txpacket[BUFFER_SIZE];
bool loraIdle = true;
bool txOk = false;
uint32_t seqNum = 0;
uint32_t lastRead = 0, lastTx = 0;
bool haveReading = false, sentOnce = false;

float pressureBar = 0, voltP = 0, turbidityNTU = 0, turbidityIdx = 0, voltT = 0;
bool pressureFault = false, turbidityFault = false;
const char* statusText = "AMAN";

SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// ================= Prototipe =================
float readSensorVolts(int pin, int samples);
float ntuToIndex(float ntu);
void readSensors();
void updateDisplay();
void sendLoRa();
void OnTxDone(void);
void OnTxTimeout(void);

// ================= Fungsi =================

// Tegangan di OUTPUT sensor (sudah dikali rasio divider), rata-rata dari beberapa sampel
float readSensorVolts(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
#if USE_CALIBRATED_ADC
    sum += analogReadMilliVolts(pin);
#else
    sum += analogRead(pin);
#endif
    delay(2);
  }
  float avg = (float)sum / samples;
#if USE_CALIBRATED_ADC
  return (avg / 1000.0) * DIVIDER_RATIO;
#else
  return (avg / ADC_MAX) * ESP32_VREF * DIVIDER_RATIO;
#endif
}

// Skala NTU -> indeks 0-100 untuk server, dibuat selaras dengan ambang di OLED:
//   NTU < 50 -> 0-30 (aman) | 50-400 -> 30-70 (waspada) | >= 400 -> 70-100 (bahaya)
float ntuToIndex(float ntu) {
  if (ntu <= NTU_WARN) return ntu / NTU_WARN * 30.0;
  if (ntu <= NTU_CRIT) return 30.0 + (ntu - NTU_WARN) / (NTU_CRIT - NTU_WARN) * 40.0;
  float idx = 70.0 + (ntu - NTU_CRIT) / (TURBIDITY_MAX_NTU - NTU_CRIT) * 30.0;
  return idx > 100.0 ? 100.0 : idx;
}

void readSensors() {
  // --- Tekanan ---
  voltP = readSensorVolts(PRESSURE_PIN, 20);
  pressureFault = (voltP < PRESSURE_FAULT_LOW || voltP > PRESSURE_FAULT_HIGH);
  if (pressureFault || voltP <= PRESSURE_V_MIN) pressureBar = 0.0;
  else if (voltP >= PRESSURE_V_MAX) pressureBar = PRESSURE_MAX_BAR;
  else pressureBar = ((voltP - PRESSURE_V_MIN) / (PRESSURE_V_MAX - PRESSURE_V_MIN)) * PRESSURE_MAX_BAR;

  // --- Turbidity ---
  voltT = readSensorVolts(TURBIDITY_PIN, 20);
  turbidityFault = (voltT < TURBIDITY_FAULT_LOW);
  turbidityNTU = ((TURBIDITY_V_CLEAN - voltT) / (TURBIDITY_V_CLEAN - TURBIDITY_V_DIRTY)) * TURBIDITY_MAX_NTU;
  if (turbidityNTU < 0) turbidityNTU = 0;
  if (turbidityNTU > TURBIDITY_MAX_NTU) turbidityNTU = TURBIDITY_MAX_NTU;
  turbidityIdx = turbidityFault ? 0.0 : ntuToIndex(turbidityNTU);

  // --- Status (prioritas: rusak > tekanan 0 > kekeruhan) ---
  if (pressureFault || turbidityFault) statusText = "SENSOR ERR";
  else if (pressureBar < PRESSURE_ZERO_BAR) statusText = "TEKANAN 0";
  else if (turbidityNTU >= NTU_CRIT) statusText = "KRITIS";
  else if (turbidityNTU >= NTU_WARN) statusText = "WARNING";
  else statusText = "AMAN";

  haveReading = true;
  Serial.printf(
  "P: %.2f Bar (V=%.3f V) | "
  "Turbidity: %.1f/100 (%.0f NTU, V=%.3f V) | %s\n",
  pressureBar,
  voltP,
  turbidityIdx,
  turbidityNTU,
  voltT,
  statusText
  );
}

void updateDisplay() {
  char buf[32];
  oled.clear();

  // Turbidity index 0-100
  oled.setFont(ArialMT_Plain_10);
  snprintf(buf, sizeof(buf), "Turbidity: %.1f/100", turbidityIdx);
  oled.drawString(0, 0, buf);

  // Water pressure
  oled.setFont(ArialMT_Plain_10);
  snprintf(buf, sizeof(buf), "Pressure: %.1f Bar", pressureBar);
  oled.drawString(0, 16, buf);

  oled.setFont(ArialMT_Plain_10);
  snprintf(buf, sizeof(buf), "AQUA-TARA");
  oled.drawString(0, 32, buf);

  // Status dan nomor transmisi
  oled.setFont(ArialMT_Plain_10);
  snprintf(buf, sizeof(buf), "Transmisi: #%lu",
         (unsigned long)seqNum);
  oled.drawString(0, 48, buf);

  oled.display();
}

void sendLoRa() {
  int flags = (pressureFault ? 1 : 0) | (turbidityFault ? 2 : 0);
  seqNum++;
  snprintf(txpacket, sizeof(txpacket), "%d,%lu,%.1f,%.2f,%d",
           NODE_ID, (unsigned long)seqNum, turbidityIdx, pressureBar, flags);
  Serial.printf("LoRa kirim: \"%s\" (%d byte)\n", txpacket, (int)strlen(txpacket));
  txOk = false;
  Radio.Send((uint8_t *)txpacket, strlen(txpacket));
  loraIdle = false;
  lastTx = millis();
  sentOnce = true;
}

void OnTxDone(void) {
  Serial.println("LoRa TX selesai");
  txOk = true;
  loraIdle = true;
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.println("LoRa TX timeout");
  txOk = false;
  loraIdle = true;
}

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);   // nyalakan daya OLED
  delay(200);

  oled.init();
  oled.clear();
  oled.drawString(0, 0, "AQUA-TARA Monitor");
  oled.drawString(0, 16, "Booting System...");
  oled.display();

  analogReadResolution(12);
  analogSetPinAttenuation(PRESSURE_PIN, ADC_11db);
  analogSetPinAttenuation(TURBIDITY_PIN, ADC_11db);
  pinMode(PRESSURE_PIN, INPUT);
  pinMode(TURBIDITY_PIN, INPUT);

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  delay(1500);
  for (int i = 0; i < 10; i++) {          // buang pembacaan awal (warm-up)
    readSensorVolts(TURBIDITY_PIN, 5);
    delay(20);
  }
}

void loop() {
  uint32_t now = millis();

  if (now - lastRead >= READ_INTERVAL_MS) {
    lastRead = now;
    readSensors();
    updateDisplay();
  }

  if (loraIdle && haveReading && (!sentOnce || now - lastTx >= LORA_INTERVAL_MS)) {
    sendLoRa();
  }

  Radio.IrqProcess();   // wajib dipanggil rutin agar event TX terproses
}
