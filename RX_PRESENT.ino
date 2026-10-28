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
#define ROUNDS          31
#define PAKET_PER_SESI  20
#define BLOCK_LEN       16
#define SESI_TIMEOUT_MS 60000UL

// ===================== KEY =====================
uint8_t key[10] = {
  0xEE, 0xFF, 0xDD, 0xCC, 0xBB,
  0xAA, 0x99, 0x88, 0x77, 0x66
};

// ===================== S-BOX =====================
// S-Box     → lihat POSISI, ambil NILAI
// S-Box inv → cari NILAI di S-Box asli, posisi tempat nilai itu = hasilnya
const uint8_t SBox[16] = { // 0xC itu bukan hex dari 0
  0xC, 0x5, 0x6, 0xB, 0x9, 0x0, 0xA, 0xD, //ini outputnya 0xc blabla
  0x3, 0xE, 0xF, 0x8, 0x4, 0x7, 0x1, 0x2
};

const uint8_t SBox_inv[16] = {
  0x5, 0xE, 0xF, 0x8, 0xC, 0x1, 0x2, 0xD, // 0x5 itu posisi output 0
  0xB, 0x4, 0x6, 0x3, 0x0, 0x7, 0x9, 0xA
};

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

PaketData buffer[PAKET_PER_SESI];
int       bufferCount = 0;
uint16_t  lastSeq     = 0;
bool      sesiAktif   = false;

unsigned long lastReceiveTime = 0;
unsigned long latencyGap      = 0;
unsigned long sesiStartTime   = 0;
unsigned long sesiEndTime     = 0;
unsigned long lastPaketTime   = 0;
unsigned int  sesiNumber      = 1;

// ===================== PRESENT FUNCTIONS =====================
uint64_t sBoxLayer_inv(uint64_t s) {
  uint64_t o = 0;
  for (int i = 0; i < 16; i++)
    o |= (uint64_t)SBox_inv[(s >> (i * 4)) & 0xF] << (i * 4);
  return o;
}

uint64_t pLayer_inv(uint64_t s) {
  uint64_t o = 0;
  for (int i = 0; i < 64; i++) {
    int pos = (16 * i) % 63;
    if (i == 63) pos = 63;
    o |= ((s >> pos) & 1ULL) << i; //1bit digeser kekanan sebanyak pos lalu and paling kanan dengan 1 bit lalu digeser kekiri seanyak i
  }
  return o;
}

void generateRoundKeys(const uint8_t* k, uint64_t rk[32]) {
  uint64_t hi = 0;
  for (int i = 0; i < 8; i++)
    hi = (hi << 8) | k[i];
  uint16_t lo = ((uint16_t)k[8] << 8) | k[9];

  for (uint8_t i = 1; i <= 32; i++) {
    rk[i - 1] = hi;

    uint32_t bot19  = ((uint32_t)(hi & 0x7) << 16) | lo;
    uint64_t new_hi = ((uint64_t)bot19 << 45) | (hi >> 19);
    uint16_t new_lo = (uint16_t)(hi >> 3);
    hi = new_hi;
    lo = new_lo;

    uint8_t top4 = (uint8_t)(hi >> 60) & 0xF;
    hi = (hi & 0x0FFFFFFFFFFFFFFFULL) | ((uint64_t)SBox[top4] << 60);

    hi ^= (uint64_t)(i >> 1);
    lo ^= (uint16_t)((i & 1) << 15);
  }
}

uint64_t present_decrypt_block(uint64_t ct) {
  uint64_t rk[32];
  generateRoundKeys(key, rk);
  uint64_t state = ct;
  state ^= rk[31]; // mulai dari rk terakhir
  for (int8_t r = 30; r >= 0; r--) {
    state = pLayer_inv(state);
    state = sBoxLayer_inv(state);
    state ^= rk[r];
  }
  return state;
}

void decryptBuffer(const uint8_t* cipher, uint8_t* plain) {
  for (int b = 0; b < 2; b++) {
    uint64_t block = 0;
    for (int j = 0; j < 8; j++)
      block = (block << 8) | cipher[b * 8 + j];
    uint64_t pt = present_decrypt_block(block);
    for (int j = 0; j < 8; j++)
      plain[b * 8 + j] = (pt >> (56 - j * 8)) & 0xFF;
  }
}

// ===================== BIT ERROR =====================
int hitungBitError(const uint8_t* plain, const uint8_t* ref) {
  int err = 0;
  for (int i = 0; i < 16; i++) {
    uint8_t x = plain[i] ^ ref[i]; //XOR 8 bit
    while (x) { 
      err += x & 1; //cek bit paling kanan di and, 1(ada error) atau 0
      x >>= 1; } //geser kanan 1 bit
  }
  return err;
}

// ===================== RESET BUFFER =====================
void resetBuffer() {
  for (int i = 0; i < PAKET_PER_SESI; i++) {
    buffer[i].seq      = 0;
    buffer[i].pv       = 0.0f;
    buffer[i].cur      = 0.0f;
    buffer[i].vol      = 0.0f;
    buffer[i].temp     = 0.0f;
    buffer[i].lat_us   = 0;
    buffer[i].ber      = 0.0f;
    buffer[i].rssi     = 0;
    buffer[i].diterima = false;
    buffer[i].rusak    = false;
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
    Serial.println(F("[UPLOAD] Paket 20/20 diterima! Hitung & upload..."));
  }

  // PER
  int paketError = 0;
  for (int i = 0; i < PAKET_PER_SESI; i++) {
    if (!buffer[i].diterima || buffer[i].rusak) paketError++;
  }
  float per = (float)paketError / (float)PAKET_PER_SESI * 100.0f; //rumusnya PER

  // Throughput
  int paketDiterima = 0;
  for (int i = 0; i < PAKET_PER_SESI; i++) {
      if (buffer[i].diterima) paketDiterima++;
  }
  uint32_t totalBit = (uint32_t)paketDiterima * 184UL;
  float durasi_s    = (float)(sesiEndTime - sesiStartTime) / 1000.0f;
  float throughput  = (durasi_s > 0.0f) ? ((float)totalBit / durasi_s) : 0.0f;

  // Rata-rata
  float berSum = 0.0f, latSum = 0.0f, rssiSum = 0.0f;
  int   hitungCount = 0, paketOK = 0;
  for (int i = 0; i < PAKET_PER_SESI; i++) {
    if (buffer[i].diterima) {
      berSum  += buffer[i].ber;
      latSum  += (float)buffer[i].lat_us;
      rssiSum += (float)buffer[i].rssi;
      hitungCount++;
      if (!buffer[i].rusak) paketOK++;
    }
  }
  float ber_avg    = (hitungCount > 0) ? (berSum  / hitungCount) : 0.0f;
  float lat_avg_us = (hitungCount > 0) ? (latSum  / hitungCount) : 0.0f;
  float rssi_avg   = (hitungCount > 0) ? (rssiSum / hitungCount) : 0.0f;

  Serial.println(F("──────── RINGKASAN SESI ────────"));
  Serial.print(F("   Paket OK    : ")); Serial.print(paketOK); Serial.print(F("/")); Serial.println(PAKET_PER_SESI);
  Serial.print(F("   PER         : ")); Serial.print(per,         2); Serial.println(F(" %"));
  Serial.print(F("   BER avg     : ")); Serial.print(ber_avg,   3); Serial.println(F(" %"));
  Serial.print(F("   Latensi avg : ")); Serial.print(lat_avg_us,  0); Serial.println(F(" us"));
  Serial.print(F("   Throughput  : ")); Serial.print(throughput,  2); Serial.println(F(" bps"));
  Serial.print(F("   RSSI avg    : ")); Serial.print(rssi_avg,    2); Serial.println(F(" dBm"));
  Serial.print(F("   Durasi sesi : ")); Serial.print(durasi_s,    1); Serial.println(F(" s"));
  if (timeout) Serial.println(F("   [UPLOAD PAKSA karena TIMEOUT]"));
  Serial.println(F("────────────────────────────────"));

  uploadGoogleSheets(per, ber_avg, throughput, rssi_avg, paketOK);

  resetBuffer();
  sesiAktif       = false;
  lastReceiveTime = 0;
  latencyGap      = 0;
  lastPaketTime   = 0;
  lastSeq         = 0;
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

  resetBuffer();
  lastReceiveTime = 0;
  latencyGap      = 0;
  lastPaketTime   = 0;

  Serial.println(F("=============================="));
  Serial.println(F("  RX PRESENT Siap"));
  Serial.println(F("=============================="));
}

// ===================== LOOP =====================
void loop() {

  if (sesiAktif && bufferCount > 0) {
    if (millis() - lastPaketTime > SESI_TIMEOUT_MS) {
      prosesAkhirSesi(true);
      return;
    }
  }

  int packetSize = LoRa.parsePacket();

  if (packetSize >= 23) {
    unsigned long currentTime = millis();
    if (lastReceiveTime != 0) latencyGap = currentTime - lastReceiveTime;
    lastReceiveTime = currentTime;
    lastPaketTime   = currentTime;

    uint8_t header = LoRa.read();
    if (header != 0x55) { while (LoRa.available()) LoRa.read(); return; }

    uint16_t seqNum = 0;
    seqNum |= (uint16_t)LoRa.read() << 8;
    seqNum |= (uint16_t)LoRa.read();

    if (sesiAktif && seqNum <= lastSeq) {
      Serial.print(F("[REPLAY DITOLAK] SeqNum "));
      Serial.print(seqNum);
      Serial.print(F(" <= lastSeq "));
      Serial.println(lastSeq);
      while (LoRa.available()) LoRa.read();
      return;
    }

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

    uint32_t lat_us = encTime_us + (uint32_t)(latencyGap * 1000UL) + decTime_us;

    float pv, cur, vol, temp;
    memcpy(&pv,   &plain[0],  4);
    memcpy(&cur,  &plain[4],  4);
    memcpy(&vol,  &plain[8],  4);
    memcpy(&temp, &plain[12], 4);

    int paket_ke = ((seqNum - 1) % PAKET_PER_SESI) + 1;
    int dataIdx  = (seqNum - 1) % JUMLAH_DATA;

    if (paket_ke == 1) {
      resetBuffer();
      lastReceiveTime = currentTime;
      latencyGap      = 0;
      sesiAktif       = true;
      sesiStartTime   = currentTime;
      lastPaketTime   = currentTime;
      lastSeq         = 0;
      Serial.println(F("=============================="));
      Serial.print(F("[SESI] Sesi baru. SeqNum: ")); Serial.println(seqNum);
    }

    if (sesiAktif && lastSeq != 0 && seqNum > lastSeq + 1) {
      for (uint16_t mis = lastSeq + 1; mis < seqNum; mis++) {
        int mis_ke  = ((mis - 1) % PAKET_PER_SESI) + 1;
        int mis_idx = mis_ke - 1;
        if (mis_idx >= 0 && mis_idx < PAKET_PER_SESI && !buffer[mis_idx].diterima) {
          buffer[mis_idx].seq      = mis;
          buffer[mis_idx].diterima = false;
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
    float ber      = (float)bitError / 184.0f * 100.0f;
    bool  rusak    = (bitError > 0);

    int idx = paket_ke - 1;
    buffer[idx].seq      = seqNum;
    buffer[idx].pv       = pv;
    buffer[idx].cur      = cur;
    buffer[idx].vol      = vol;
    buffer[idx].temp     = temp;
    buffer[idx].lat_us   = lat_us;
    buffer[idx].ber      = ber;
    buffer[idx].rssi     = rssi;
    buffer[idx].diterima = true;
    buffer[idx].rusak    = rusak;
    bufferCount++;

    // Serial log (tanpa encTime, decTime, latencyGap, SNR)
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
    Serial.print(F("   RSSI   : ")); Serial.print(rssi); Serial.println(F(" dBm"));

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

    if (buffer[i].diterima) {
      // Field "plain" format: "pvV;cur;vol;temp"
      String plainStr = String(buffer[i].pv,   2) + ";" +
                        String(buffer[i].cur,  4) + ";" +
                        String(buffer[i].vol,  2) + ";" +
                        String(buffer[i].temp, 2);

      json  = "{\"type\":\"packet\",";
      json += "\"seq\":"      + String(buffer[i].seq)                     + ",";
      json += "\"plain\":\""  + plainStr                                  + "\",";
      json += "\"lat\":"      + String(buffer[i].lat_us)                  + ",";
      json += "\"ber\":"      + String(buffer[i].ber, 3)                  + ",";
      json += "\"rssi\":"     + String(buffer[i].rssi)                    + ",";
      json += "\"status\":\"" + String(buffer[i].rusak ? "RUSAK" : "OK") + "\"}";
    } else {
      json  = "{\"type\":\"packet\",";
      json += "\"seq\":"      + String(buffer[i].seq) + ",";
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
  Serial.print(F(" BER:"));      Serial.print(ber_avg,  3); Serial.println(F("%"));
  Serial.print(F(" TP:"));       Serial.print(throughput,2); Serial.println(F(" bps"));
}
