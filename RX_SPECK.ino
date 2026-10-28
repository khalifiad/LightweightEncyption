#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <WiFiS3.h>
#include <ArduinoHttpClient.h>
#include "data.h"

// ===================== PIN LORA =====================
#define SS_PIN   10
#define RST_PIN   9
#define DIO0_PIN  2

// ===================== WIFI =====================
const char* ssid       = "-";
const char* password   = "-";
const char* scriptHost = "script.google.com";
const char* scriptURL  = "-";

// ===================== KONFIGURASI =====================
#define SPECK_ROUNDS    26
#define PAKET_PER_SESI  20
#define BLOCK_LEN       16
#define SESI_TIMEOUT_MS 60000UL

// ===================== SPECK =====================
#define ROR32(x, r) (((x) >> (r)) | ((x) << (32 - (r))))
#define ROL32(x, r) (((x) << (r)) | ((x) >> (32 - (r))))

uint32_t speck_key[3] = {0x33221100, 0x77665544, 0xbbaa9988};
uint32_t speck_rk[SPECK_ROUNDS];

void speck_key_schedule() {
  uint32_t l[SPECK_ROUNDS + 1];
  l[0] = speck_key[1];
  l[1] = speck_key[2];
  speck_rk[0] = speck_key[0];
  for (uint32_t i = 0; i < SPECK_ROUNDS - 1; i++) {
    l[i + 2]        = (speck_rk[i] + ROR32(l[i], 8)) ^ i;
    speck_rk[i + 1] = ROL32(speck_rk[i], 3) ^ l[i + 2];
  }
}

void speck_decrypt_block(uint64_t &state) {
  uint32_t x = (uint32_t)(state >> 32);
  uint32_t y = (uint32_t)(state & 0xFFFFFFFF);
  for (int i = SPECK_ROUNDS - 1; i >= 0; i--) { //bedanya disini sama tx
    y = ROR32(y ^ x, 3); // y xor dengan x lalu rotate 3 bit ke kanan
    x = ROL32((x ^ speck_rk[i]) - y, 8); //x xor dengan schedule kurangi y baru lalu di rotate kiri 8 bit
  }
  state = ((uint64_t)x << 32) | y;
}

void decryptBuffer(const uint8_t* cipher, uint8_t* plain) {
  for (int b = 0; b < 2; b++) {
    uint64_t block = 0;
    for (int j = 0; j < 8; j++)
      block = (block << 8) | cipher[b * 8 + j];
    speck_decrypt_block(block);
    for (int j = 0; j < 8; j++)
      plain[b * 8 + j] = (block >> (56 - 8 * j)) & 0xFF;
  }
}

// ===================== BIT ERROR =====================
int hitungBitError(const uint8_t* plain, const uint8_t* ref) {
  int err = 0;
  for (int i = 0; i < 16; i++) {
    uint8_t x = plain[i] ^ ref[i]; //XOR 8 bit
    while (x) { 
      err += x & 1; //cek bit paling kanan di and, 1(ada error) atau 0
      x >>= 1; }  //geser kanan 1 bit
  }
  return err;
}

// ===================== STRUCT PAKET =====================
struct PaketData {
  uint16_t seq;
  float    pv, cur, vol, temp;
  uint32_t lat_us;
  float    ber;
  int      rssi;
  bool     diterima;
  bool     rusak;
};

PaketData sesiBuffer[PAKET_PER_SESI];
int       bufferCount   = 0;
uint16_t  lastSeq       = 0;

bool          sesiAktif     = false;
unsigned long lastRxTime    = 0;
unsigned long latencyGap_ms = 0;
unsigned long sesiStartTime = 0;
unsigned long sesiEndTime   = 0;
unsigned long lastPaketTime = 0;
unsigned int  sesiNumber    = 1;

// ===================== RESET BUFFER =====================
void resetBuffer() {
  for (int i = 0; i < PAKET_PER_SESI; i++) {
    sesiBuffer[i].seq      = 0;
    sesiBuffer[i].pv       = 0.0f;
    sesiBuffer[i].cur      = 0.0f;
    sesiBuffer[i].vol      = 0.0f;
    sesiBuffer[i].temp     = 0.0f;
    sesiBuffer[i].lat_us   = 0;
    sesiBuffer[i].ber      = 0.0f;
    sesiBuffer[i].rssi     = 0;
    sesiBuffer[i].diterima = false;
    sesiBuffer[i].rusak    = false;
  }
  bufferCount = 0;
}

// ===================== FORWARD DECLARATION =====================
void prosesAkhirSesi(bool timeout);
void uploadGoogleSheets(float per, float ber_avg, float throughput, float rssi_avg, int paketOK);

// ===================== PROSES AKHIR SESI =====================
void prosesAkhirSesi(bool timeout) {
  sesiEndTime = millis();

  if (timeout) {
    Serial.println(F("=============================="));
    Serial.print(F("[TIMEOUT] Paket diterima: "));
    Serial.print(bufferCount); Serial.print(F("/")); Serial.println(PAKET_PER_SESI);
    Serial.println(F("[TIMEOUT] Upload paksa..."));
  } else {
    Serial.println(F("=============================="));
    Serial.println(F("[UPLOAD] Paket 20/20 diterima! Hitung metrik & upload..."));
  }

  // PER
  int paketError = 0;
  for (int i = 0; i < PAKET_PER_SESI; i++) {
    if (!sesiBuffer[i].diterima || sesiBuffer[i].rusak) paketError++; //! not
  }
  float per = (float)paketError / (float)PAKET_PER_SESI * 100.0f; //rumusnya PER

  // Throughput
  int paketDiterima = 0;
  for (int i = 0; i < PAKET_PER_SESI; i++) {
    if (sesiBuffer[i].diterima) paketDiterima++;
  }
  uint32_t totalBit   = (uint32_t)paketDiterima * 184UL;
  float    durasi_s   = (float)(sesiEndTime - sesiStartTime) / 1000.0f;
  float    throughput = (durasi_s > 0.0f) ? ((float)totalBit / durasi_s) : 0.0f; //rumusnya, ? kondisi iya, : kondisi tidak

  // Rata-rata
  float berSum = 0.0f, latSum = 0.0f, rssiSum = 0.0f;
  int   hitCount = 0, paketOK = 0; //hitcount yg diterima
  for (int i = 0; i < PAKET_PER_SESI; i++) {
    if (sesiBuffer[i].diterima) {
      berSum  += sesiBuffer[i].ber;
      latSum  += (float)sesiBuffer[i].lat_us;
      rssiSum += (float)sesiBuffer[i].rssi;
      hitCount++;
      if (!sesiBuffer[i].rusak) paketOK++;
    }
  }
  float ber_avg    = (hitCount > 0) ? (berSum  / hitCount) : 0.0f;
  float lat_avg_us = (hitCount > 0) ? (latSum  / hitCount) : 0.0f;
  float rssi_avg   = (hitCount > 0) ? (rssiSum / hitCount) : 0.0f;

  Serial.println(F("──────── RINGKASAN SESI ────────"));
  Serial.print(F("   Paket OK    : ")); Serial.print(paketOK); Serial.print(F("/")); Serial.println(PAKET_PER_SESI);
  Serial.print(F("   PER         : ")); Serial.print(per,         2); Serial.println(F(" %"));
  Serial.print(F("   BER avg     : ")); Serial.print(ber_avg,   3); Serial.println(F(" %"));
  Serial.print(F("   Latensi avg : ")); Serial.print(lat_avg_us,  0); Serial.println(F(" us"));
  Serial.print(F("   Throughput  : ")); Serial.print(throughput,  2); Serial.println(F(" bps"));
  // Serial.print(F("   RSSI avg    : ")); Serial.print(rssi_avg,    2); Serial.println(F(" dBm"));
  Serial.print(F("   Durasi sesi : ")); Serial.print(durasi_s,    1); Serial.println(F(" s"));
  if (timeout) Serial.println(F("   [UPLOAD PAKSA karena TIMEOUT]"));
  Serial.println(F("────────────────────────────────"));

  uploadGoogleSheets(per, ber_avg, throughput, rssi_avg, paketOK);

  resetBuffer();
  sesiAktif     = false;
  lastRxTime    = 0;
  latencyGap_ms = 0;
  lastPaketTime = 0;
  lastSeq       = 0;
  sesiNumber++;

  Serial.println(F("[RX] Siap menerima sesi berikutnya."));
  Serial.println(F("=============================="));
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.print(F("[WiFi] Menghubungkan ke ")); Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); 
    Serial.print(F("."));
  }
  Serial.println();
  Serial.println(F("[WiFi] Terhubung!"));

  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(915E6)) {
    Serial.println(F("[ERROR] LoRa gagal!")); 
    while (1);
  }
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  // LoRa.enableCrc();
  speck_key_schedule();

  resetBuffer();
  lastRxTime    = 0;
  latencyGap_ms = 0;
  lastPaketTime = 0;

  Serial.println(F("=============================="));
  Serial.println(F("  RX SPECK Siap"));
  Serial.println(F("=============================="));
}

// ===================== LOOP =====================
void loop() {
  //paksa berhenti kalau gada yg masuk
  if (sesiAktif && bufferCount > 0) {
    if (millis() - lastPaketTime > SESI_TIMEOUT_MS) {
      prosesAkhirSesi(true);
      return;
    }
  }

  int packetSize = LoRa.parsePacket();

  if (packetSize >= 23) {
    unsigned long currentTime = millis();
    if (lastRxTime != 0) latencyGap_ms = currentTime - lastRxTime;
    else                 latencyGap_ms = 0;
    lastRxTime    = currentTime;
    lastPaketTime = currentTime;

    //baca header
    uint8_t header = LoRa.read();
    if (header != 0x55) {
      while (LoRa.available()) LoRa.read();
      return;
    }

    uint16_t seqNum = 0; //beda juga di tx
    seqNum |= (uint16_t)LoRa.read() << 8;
    seqNum |= (uint16_t)LoRa.read();

    uint32_t encTime_us = 0;
    encTime_us |= (uint32_t)LoRa.read() << 24;
    encTime_us |= (uint32_t)LoRa.read() << 16;
    encTime_us |= (uint32_t)LoRa.read() <<  8;
    encTime_us |= (uint32_t)LoRa.read();

    uint8_t cipher[BLOCK_LEN];
    for (int i = 0; i < BLOCK_LEN; i++)
      cipher[i] = LoRa.available() ? LoRa.read() : 0x00;
    while (LoRa.available()) LoRa.read();

    int rssi = LoRa.packetRssi();

    uint8_t  plain[BLOCK_LEN];
    uint32_t dec_start  = micros();
    decryptBuffer(cipher, plain);
    uint32_t decTime_us = micros() - dec_start;

    //latensi
    uint32_t lat_us = encTime_us + (uint32_t)(latencyGap_ms * 1000UL) + decTime_us;

    float pv, cur, vol, temp;
    memcpy(&pv,   &plain[0],  4);
    memcpy(&cur,  &plain[4],  4);
    memcpy(&vol,  &plain[8],  4);
    memcpy(&temp, &plain[12], 4);

    int paket_ke = ((seqNum - 1) % PAKET_PER_SESI) + 1;
    int dataIdx  = (seqNum - 1) % JUMLAH_DATA;

    if (!sesiAktif) {
      resetBuffer();
      lastRxTime    = currentTime;
      latencyGap_ms = 0;
      sesiAktif     = true;
      lastPaketTime = currentTime;
      lastSeq       = 0;
      sesiStartTime = currentTime;

      Serial.println(F("=============================="));
      Serial.print(F("[SESI] Sesi baru. SeqNum: ")); Serial.println(seqNum);
      Serial.print(F("[SESI] Paket ke-")); Serial.print(paket_ke); Serial.println(F(" pertama diterima"));
    }

    if (sesiAktif && bufferCount == 1 && latencyGap_ms > 0) {
      sesiStartTime = currentTime - ((uint32_t)(paket_ke - 1) * latencyGap_ms);
    }

    if (sesiAktif && lastSeq != 0 && seqNum > lastSeq + 1) {
      for (uint16_t mis = lastSeq + 1; mis < seqNum; mis++) {
        int mis_ke  = ((mis - 1) % PAKET_PER_SESI) + 1;
        int mis_idx = mis_ke - 1;
        if (mis_idx >= 0 && mis_idx < PAKET_PER_SESI && !sesiBuffer[mis_idx].diterima) {
          sesiBuffer[mis_idx].seq      = mis;
          sesiBuffer[mis_idx].diterima = false;
          Serial.print(F("[HILANG] Paket #")); Serial.print(mis);
          Serial.print(F(" (ke-")); Serial.print(mis_ke); Serial.println(F(")"));
        }
      }
    }
    lastSeq = seqNum;

    float ref_pv   = pvVoltage[dataIdx];
    float ref_cur  = current[dataIdx];
    float ref_vol  = voltage[dataIdx];
    float ref_temp = temperature[dataIdx];

    uint8_t ref_bytes[16];
    memcpy(ref_bytes,      &ref_pv,   4);
    memcpy(ref_bytes +  4, &ref_cur,  4);
    memcpy(ref_bytes +  8, &ref_vol,  4);
    memcpy(ref_bytes + 12, &ref_temp, 4);

    //BER
    int   bitError = hitungBitError(plain, ref_bytes);
    float ber      = (float)bitError / 128.0f * 100.0f;
    bool  rusak    = (bitError > 0);

    int bufIdx = paket_ke - 1;
    sesiBuffer[bufIdx].seq      = seqNum;
    sesiBuffer[bufIdx].pv       = pv;
    sesiBuffer[bufIdx].cur      = cur;
    sesiBuffer[bufIdx].vol      = vol;
    sesiBuffer[bufIdx].temp     = temp;
    sesiBuffer[bufIdx].lat_us   = lat_us;
    sesiBuffer[bufIdx].ber      = ber;
    sesiBuffer[bufIdx].rssi     = rssi;
    sesiBuffer[bufIdx].diterima = true;
    sesiBuffer[bufIdx].rusak    = rusak;
    bufferCount++;

    // Serial log
    Serial.println(F("------------------------------"));
    Serial.print(F("[RX #")); Serial.print(seqNum);
    Serial.print(F("] Paket ")); Serial.print(paket_ke);
    Serial.print(F("/")); Serial.println(PAKET_PER_SESI);
    Serial.print(F("   PV     : ")); Serial.print(pv,   2); Serial.println(F(" V"));
    Serial.print(F("   I      : ")); Serial.print(cur,  4); Serial.println(F(" A"));
    Serial.print(F("   V      : ")); Serial.print(vol,  2); Serial.println(F(" V"));
    Serial.print(F("   T      : ")); Serial.print(temp, 2); Serial.println(F(" C"));
    Serial.print(F("   Status : ")); Serial.println(rusak ? F("RUSAK") : F("OK"));
    Serial.print(F("   BitErr : ")); Serial.print(bitError); Serial.print(F("/184  BER: ")); Serial.print(ber, 3); Serial.println(F(" %"));
    Serial.print(F("   Latensi: ")); Serial.print(lat_us); Serial.println(F(" us"));
    // Serial.print(F("   RSSI   : ")); Serial.print(rssi); Serial.println(F(" dBm"));

    if (paket_ke == PAKET_PER_SESI) {
      prosesAkhirSesi(false);
    }
  }

  delay(2);
}

// ===================== UPLOAD GOOGLE SHEETS =====================
void uploadGoogleSheets(float per, float ber_avg, float throughput, float rssi_avg, int paketOK) {
  WiFiSSLClient wifiClient;
  HttpClient    client(wifiClient, scriptHost, 443);

  Serial.println(F("[UPLOAD] Mulai upload per-paket ke Google Sheets..."));

  for (int i = 0; i < PAKET_PER_SESI; i++) {
    String json;

    if (sesiBuffer[i].diterima) {
      // Field "plain" format: "pvV;cur;vol;temp"
      String plainStr = String(sesiBuffer[i].pv,   2) + ";" +
                        String(sesiBuffer[i].cur,  4) + ";" +
                        String(sesiBuffer[i].vol,  2) + ";" +
                        String(sesiBuffer[i].temp, 2);

      json  = "{\"type\":\"packet\",";
      json += "\"seq\":"      + String(sesiBuffer[i].seq)                     + ",";
      json += "\"plain\":\""  + plainStr                                       + "\",";
      json += "\"lat\":"      + String(sesiBuffer[i].lat_us)                  + ",";
      json += "\"ber\":"      + String(sesiBuffer[i].ber, 3)                  + ",";
      json += "\"rssi\":"     + String(sesiBuffer[i].rssi)                    + ",";
      json += "\"status\":\"" + String(sesiBuffer[i].rusak ? "RUSAK" : "OK") + "\"}";
    } else {
      json  = "{\"type\":\"packet\",";
      json += "\"seq\":"      + String(sesiBuffer[i].seq) + ",";
      json += "\"plain\":\"0;0;0;0\",";
      json += "\"lat\":0,\"ber\":0,\"rssi\":0,";
      json += "\"status\":\"HILANG\"}";
    }

    client.post(scriptURL, "application/json", json);
    client.responseStatusCode();
    client.responseBody();
    delay(150);

    Serial.print(F("  [")); Serial.print(i + 1);
    Serial.print(F("/")); Serial.print(PAKET_PER_SESI); Serial.println(F("] diunggah"));
  }

  // Summary sesi
  String jsonSum  = "{\"type\":\"summary\",";
  jsonSum += "\"per\":"        + String(per,        2) + ",";
  jsonSum += "\"ber_avg\":"    + String(ber_avg,    3) + ",";
  jsonSum += "\"throughput\":" + String(throughput, 2) + ",";
  jsonSum += "\"rssi_avg\":"   + String(rssi_avg,   2) + "}";

  client.post(scriptURL, "application/json", jsonSum);
  client.responseStatusCode();
  client.responseBody();

  Serial.println(F("[UPLOAD] Selesai."));
  Serial.print(F("[HASIL] OK:")); Serial.print(paketOK);
  Serial.print(F(" PER:"));      Serial.print(per,      2); Serial.print(F("%"));
  Serial.print(F(" BER:"));      Serial.print(ber_avg,  3); Serial.println(F(" %"));
  Serial.print(F(" TP:"));       Serial.print(throughput,2); Serial.println(F(" bps"));
}
