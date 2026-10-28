#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "data.h"

// ===================== PIN LORA =====================
#define SS_PIN   10
#define RST_PIN   9
#define DIO0_PIN  2

// ===================== OLED =====================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===================== KONFIGURASI =====================
#define PAKET_PER_SESI   20
#define BLOCK_LEN        16       // 2 blok x 8 byte
#define TX_INTERVAL_MS   2000L
#define WAIT_AFTER_SESI  180000UL // 4 menit

// ===================== SPECK =====================
#define SPECK_ROUNDS 26
#define ROR32(x, r) (((x) >> (r)) | ((x) << (32 - (r)))) //ROR32 = Rotate RIGHT → bit muter ke kanan PELAJARIN LAGI
#define ROL32(x, r) (((x) << (r)) | ((x) >> (32 - (r)))) //ROL32 = Rotate LEFT  → bit muter ke kiri

uint32_t speck_key[3] = {0x33221100, 0x77665544, 0xbbaa9988};
uint32_t speck_rk[SPECK_ROUNDS]; //rk=round key

void speck_key_schedule() {
  uint32_t l[SPECK_ROUNDS + 1];
  l[0] = speck_key[1]; // = 0x77665544 word kedua
  l[1] = speck_key[2]; // = 0xbbaa9988
  speck_rk[0] = speck_key[0]; // = 0x33221100
  for (uint32_t i = 0; i < SPECK_ROUNDS - 1; i++) { //loopnya
    l[i + 2]        = (speck_rk[i] + ROR32(l[i], 8)) ^ i;
    speck_rk[i + 1] = ROL32(speck_rk[i], 3) ^ l[i + 2];
  }
}

void speck_encrypt_block(uint64_t &state) { //mecah bagi 2
  uint32_t x = (uint32_t)(state >> 32); // >> geser kanan, 64 bit digeser 32 bit ke kanan jadi hilang dan sisa 32 bit kiri
  uint32_t y = (uint32_t)(state & 0xFFFFFFFF); // 64 bit di and sama 0ffff jadi sisa 32 bit kanan
  for (int i = 0; i < SPECK_ROUNDS; i++) {
    x = (ROR32(x, 8) + y) ^ speck_rk[i];
    y = ROL32(y, 3) ^ x;
  }
  state = ((uint64_t)x << 32) | y; //digeser kiri 32 bit lalu or dgn y baru
}

// ===================== ubah ciphertext =====================
void encryptBuffer(uint8_t* buf) { //Payload 16 byte dipecah jadi 2 blok 8 byte, karena Speck di sini bekerja per 64 bit
  for (int b = 0; b < 2; b++) {
    uint64_t block = 0; //blocknya 64bit
    for (int j = 0; j < 8; j++)
      block = (block << 8) | buf[b * 8 + j]; //64bit digeser kekiri 8 bit di or sama 8bit, jadi bufnya mengisi kekosongan block
    speck_encrypt_block(block); //manggil encryptblock
    for (int j = 0; j < 8; j++)
      buf[b * 8 + j] = (block >> (56 - 8 * j)) & 0xFF; //geser kanan block sebanyak -bit lalu and dengan 0FF
  } // & 0xFF fungsinya buang semua bit kecuali 8 bit paling bawah
}

// ===================== COUNTER =====================
uint16_t seqNum    = 0;
int      dataIndex = 0;

// ===================== OLED HELPER =====================
void oledShow(const char* l1, const char* l2 = "",
              const char* l3 = "", const char* l4 = "",
              const char* l5 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,  0); display.println(l1);
  display.setCursor(0, 13); display.println(l2);
  display.setCursor(0, 26); display.println(l3);
  display.setCursor(0, 39); display.println(l4);
  display.setCursor(0, 52); display.println(l5);
  display.display();
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(500);

  // -- OLED --
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[ERROR] OLED gagal!");
    while (1);
  }
  oledShow("TX SPECK", "Inisialisasi...");

  // -- LoRa --
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(915E6)) {
    Serial.println("[ERROR] LoRa gagal!");
    oledShow("ERROR", "LoRa gagal!");
    while (1);
  }
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  // LoRa.enableCrc();
  LoRa.setTxPower(20);

  speck_key_schedule();
  oledShow("TX SPECK", "LoRa Ready");
  Serial.println("==============================");
  Serial.println("  TX SPECK Siap");
  Serial.println("==============================");
  delay(2000);
}

// ===================== LOOP =====================
void loop() {
  char ob[22];
  snprintf(ob, sizeof(ob), "Data awal: %d", dataIndex);
  oledShow("=== SESI BARU ===", "Kirim 20 paket", ob);
  delay(1000);

  for (int paket_ke = 1; paket_ke <= PAKET_PER_SESI; paket_ke++) {
    unsigned long t_kirim = millis();

    seqNum++;

    // -- Ambil data dari data.h --
    float pv   = pvVoltage[dataIndex];
    float cur  = current[dataIndex];
    float vol  = voltage[dataIndex];
    float temp = temperature[dataIndex];

    // -- Susun plaintext 16 byte --
    uint8_t plain[BLOCK_LEN];
    memcpy(&plain[0],  &pv,   4);
    memcpy(&plain[4],  &cur,  4);
    memcpy(&plain[8],  &vol,  4);
    memcpy(&plain[12], &temp, 4);

    // -- Log plaintext hex --
    String hexPT = "";
    for (int i = 0; i < BLOCK_LEN; i++) {
      char hb[3]; sprintf(hb, "%02X", plain[i]);
      hexPT += hb;
    }

    // -- Enkripsi + ukur waktu --
    uint32_t enc_start  = micros();
    encryptBuffer(plain);
    uint32_t encTime_us = micros() - enc_start;

    // -- Log ciphertext hex --
    String hexCT = "";
    for (int i = 0; i < BLOCK_LEN; i++) {
      char hb[3]; sprintf(hb, "%02X", plain[i]);
      hexCT += hb;
    }

    // -- Kirim via LoRa: [0x55][seqNum 2B][encTime 4B][encPayload 16B] = 23 byte --
    LoRa.beginPacket();
    LoRa.write(0x55);
    LoRa.write((seqNum     >>  8) & 0xFF);
    LoRa.write( seqNum            & 0xFF);
    LoRa.write((encTime_us >> 24) & 0xFF);
    LoRa.write((encTime_us >> 16) & 0xFF);
    LoRa.write((encTime_us >>  8) & 0xFF);
    LoRa.write( encTime_us        & 0xFF);
    LoRa.write(plain, BLOCK_LEN);
    LoRa.endPacket();

    // -- Serial log --
    Serial.println("------------------------------");
    Serial.print("[TX #"); Serial.print(seqNum);
    Serial.print("] Paket "); Serial.print(paket_ke);
    Serial.print("/"); Serial.println(PAKET_PER_SESI);
    Serial.print("   Data ke-"); Serial.println(dataIndex);
    Serial.print("   PV      = "); Serial.print(pv,   2); Serial.println(" V");
    Serial.print("   I       = "); Serial.print(cur,  4); Serial.println(" A");
    Serial.print("   V       = "); Serial.print(vol,  2); Serial.println(" V");
    Serial.print("   T       = "); Serial.print(temp, 2); Serial.println(" C");
    Serial.print("   PT Hex  = "); Serial.println(hexPT);
    Serial.print("   CT Hex  = "); Serial.println(hexCT);

    // -- OLED update --
    char b1[22], b2[22], b3[22], b4[22];
    snprintf(b1, sizeof(b1), "TX#%u [%d/%d]", seqNum, paket_ke, PAKET_PER_SESI);
    snprintf(b2, sizeof(b2), "Data ke-%d", dataIndex);
    snprintf(b3, sizeof(b3), "PV=%.2fV I=%.3fA", pv, cur);
    snprintf(b4, sizeof(b4), "V=%.1fV T=%.1fC",  vol, temp);
    oledShow(b1, b2, b3, b4);

    dataIndex = (dataIndex + 1) % JUMLAH_DATA;

    // -- Jaga interval --
    long sisa = TX_INTERVAL_MS - (long)(millis() - t_kirim);
    if (sisa > 0) delay((unsigned long)sisa);
  }

  // -- Selesai 1 sesi --
  Serial.println("==============================");
  Serial.println("[SESI] 20 paket terkirim.");
  Serial.print("[TUNGGU] Jeda "); Serial.print(WAIT_AFTER_SESI / 1000);
  Serial.println("s untuk RX upload...");

  // -- Countdown OLED 4 menit --
  unsigned long waitStart = millis();
  while (millis() - waitStart < WAIT_AFTER_SESI) {
    unsigned long sisa_ms = WAIT_AFTER_SESI - (millis() - waitStart);
    unsigned int  sisa_s  = (unsigned int)(sisa_ms / 1000);
    unsigned int  menit   = sisa_s / 60;
    unsigned int  detik   = sisa_s % 60;

    char bw[22];
    snprintf(bw, sizeof(bw), "Tunggu: %02u:%02u", menit, detik);
    oledShow("== 20/20 SELESAI ==", "RX sedang upload", bw, "Jangan matikan!");

    Serial.print("   [TUNGGU] Sisa: "); Serial.print(menit);
    Serial.print("m"); Serial.print(detik); Serial.println("s");
    delay(5000);
  }

  Serial.println("[INFO] Siap memulai sesi berikutnya.");
  oledShow("Siap Sesi Baru", "Mulai dalam 3s...");
  delay(3000);
}
