/*
  AQUA-TARA - Gateway LoRa -> WiFi -> Supabase (Heltec WiFi LoRa 32 V3)

  - Terima paket LoRa dari node pada 921,4 MHz
  - Kirim tiap pembacaan ke tabel iot_measurements lewat HTTPS (Supabase REST)
  - Pengiriman HTTPS berjalan di task terpisah (core 0) supaya penerimaan LoRa tidak terhambat

  Format paket dari node:  id,seq,turbidity_idx,tekanan_bar,flags
    flags: 1 = sensor tekanan rusak, 2 = sensor turbidity rusak  (nilai rusak dikirim sebagai null)

  Wajib: salin secrets.example.h -> secrets.h dan isi. PASANG ANTENA sebelum menyalakan board.
*/
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "secrets.h"

// ===== LoRa (harus SAMA dengan node) =====
#define RF_FREQUENCY                921400000
#define LORA_BANDWIDTH              0
#define LORA_SPREADING_FACTOR       7
#define LORA_CODINGRATE             1
#define LORA_PREAMBLE_LENGTH        8
#define LORA_SYMBOL_TIMEOUT         0
#define LORA_FIX_LENGTH_PAYLOAD_ON  false
#define LORA_IQ_INVERSION_ON        false
#define BUFFER_SIZE                 64

struct Reading {
  int id;
  unsigned long seq;
  float turb;
  float bar;
  int flags;
  int rssi;
  int snr;
};

static RadioEvents_t RadioEvents;
char rxpacket[BUFFER_SIZE];
bool loraIdle = true;
uint32_t rxCount = 0;

QueueHandle_t readingQueue;
volatile uint32_t uploadOk = 0, uploadFail = 0;
volatile int lastHttpCode = 0;

Reading last = {0, 0, 0, 0, 0, 0, 0};
bool haveLast = false;
uint32_t lastDraw = 0;

SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
bool postReading(const Reading &r);
void uploadTask(void *param);
void drawScreen();

// ---------- kirim ke Supabase ----------
bool postReading(const Reading &r) {
  char tbuf[12], pbuf[12], body[160];
  if (r.flags & 2) strcpy(tbuf, "null"); else snprintf(tbuf, sizeof(tbuf), "%.1f", r.turb);
  if (r.flags & 1) strcpy(pbuf, "null"); else snprintf(pbuf, sizeof(pbuf), "%.2f", r.bar);
  snprintf(body, sizeof(body),
           "{\"node_id\":%d,\"turbidity\":%s,\"pressure\":%s,\"is_simulation\":false}",
           r.id, tbuf, pbuf);

  WiFiClientSecure client;
  client.setInsecure();               // DEMO: sertifikat server tidak diperiksa
  HTTPClient http;
  String url = String(SUPABASE_URL) + "/rest/v1/iot_measurements";
  if (!http.begin(client, url)) return false;
  http.setTimeout(8000);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_KEY);
  if (strncmp(SUPABASE_KEY, "eyJ", 3) == 0) {          // key lama (JWT) butuh Authorization juga
    http.addHeader("Authorization", String("Bearer ") + SUPABASE_KEY);
  }
  http.addHeader("Prefer", "return=minimal");

  int code = http.POST((uint8_t *)body, strlen(body));
  lastHttpCode = code;
  if (code < 200 || code >= 300) {
    Serial.printf("Upload gagal HTTP %d: %s\n", code, http.getString().c_str());
  }
  http.end();
  return code >= 200 && code < 300;
}

void uploadTask(void *param) {
  Reading r;
  for (;;) {
    if (xQueueReceive(readingQueue, &r, portMAX_DELAY) == pdTRUE) {
      bool ok = false;
      for (int attempt = 0; attempt < 3 && !ok; attempt++) {
        if (WiFi.status() != WL_CONNECTED) {
          WiFi.reconnect();
          vTaskDelay(pdMS_TO_TICKS(3000));
          continue;
        }
        ok = postReading(r);
        if (!ok) vTaskDelay(pdMS_TO_TICKS(1000));
      }
      if (ok) uploadOk++; else uploadFail++;
    }
  }
}

// ---------- LoRa ----------
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  if (size >= BUFFER_SIZE) size = BUFFER_SIZE - 1;
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();
  rxCount++;

  Reading r;
  r.rssi = rssi;
  r.snr = snr;
  int n = sscanf(rxpacket, "%d,%lu,%f,%f,%d", &r.id, &r.seq, &r.turb, &r.bar, &r.flags);
  if (n == 5) {
    Serial.printf("{\"node_id\":%d,\"seq\":%lu,\"turbidity\":%.1f,\"pressure\":%.2f,\"flags\":%d,\"rssi\":%d,\"snr\":%d}\n",
                  r.id, r.seq, r.turb, r.bar, r.flags, rssi, snr);
    last = r;
    haveLast = true;
    if (xQueueSend(readingQueue, &r, 0) != pdTRUE) {
      Serial.println("Antrean upload penuh, data dibuang");
      uploadFail++;
    }
  } else {
    Serial.printf("Paket tidak dikenal: \"%s\" (rssi %d)\n", rxpacket, rssi);
  }
  loraIdle = true;
}

// ---------- OLED ----------
void drawScreen() {
  char buf[40];
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  if (haveLast) {
    snprintf(buf, sizeof(buf), "N%d #%lu RX:%lu", last.id, last.seq, (unsigned long)rxCount);
    oled.drawString(0, 0, buf);
    oled.setFont(ArialMT_Plain_16);
    snprintf(buf, sizeof(buf), "T idx: %.1f", last.turb);
    oled.drawString(0, 12, buf);
    snprintf(buf, sizeof(buf), "P: %.2f Bar", last.bar);
    oled.drawString(0, 30, buf);
  } else {
    oled.drawString(0, 0, "AQUA-TARA Gateway");
    oled.drawString(0, 16, "921.4 MHz  SF7");
    oled.drawString(0, 30, "Menunggu paket...");
  }
  oled.setFont(ArialMT_Plain_10);
  snprintf(buf, sizeof(buf), "WiFi:%s DB ok:%lu err:%lu",
           WiFi.status() == WL_CONNECTED ? "OK" : "--",
           (unsigned long)uploadOk, (unsigned long)uploadFail);
  oled.drawString(0, 52, buf);
  oled.display();
}

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(200);
  oled.init();
  drawScreen();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  readingQueue = xQueueCreate(10, sizeof(Reading));
  xTaskCreatePinnedToCore(uploadTask, "upload", 10240, NULL, 1, NULL, 0);

  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
}

void loop() {
  if (loraIdle) {
    loraIdle = false;
    Radio.Rx(0);
  }
  Radio.IrqProcess();

  if (millis() - lastDraw >= 1000) {
    lastDraw = millis();
    drawScreen();
  }
}
