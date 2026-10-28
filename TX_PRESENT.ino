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
#define ROUNDS          31
#define PAKET_PER_SESI  20
#define BLOCK_LEN       16    // 2 blok x 8 byte
#define TX_INTERVAL_MS  2000L
#define WAIT_AFTER_SESI 180000UL // 4 menit tunggu RX upload

// ===================== KEY =====================
uint8_t key[10] = {
  0xEE, 0xFF, 0xDD, 0xCC, 0xBB,
  0xAA, 0x99, 0x88, 0x77, 0x66
};

// ===================== S-BOX =====================
const uint8_t SBox[16] = {
  0xC, 0x5, 0x6, 0xB, 0x9, 0x0, 0xA, 0xD,
  0x3, 0xE, 0xF, 0x8, 0x4, 0x7, 0x1, 0x2
};

// ===================== COUNTER =====================
uint16_t seqNum    = 0;
int      dataIndex = 0;

// ===================== PRESENT FUNCTIONS =====================
//pembagian 16 nibble dan subtitusinya
uint64_t sBoxLayer(uint64_t s) {
  uint64_t o = 0;
  for (int i = 0; i < 16; i++) //16 nibble, 1 nibble = 4bit, 1 bit = 4 binary
    o |= (uint64_t)SBox[(s >> (i * 4)) & 0xF] << (i * 4); 
    //geser 4 bit sebanyak (i * 4) ke kanan lalu di and 1111 paling kanan lalu di sub sama tabel sbox lalu kembalikan lagi posisi niblenya ke kiri 
  return o;
}

//permutasi
uint64_t pLayer(uint64_t s) {
  uint64_t o = 0;
  for (int i = 0; i < 64; i++) {
    int pos = (16 * i) % 63; //pindah posisi: bit 1 ke posisi 16
    if (i == 63) pos = 63; // bit 0 dan 63 stay
    o |= ((s >> i) & 1ULL) << pos;
    //geser 1 bit ke kanan sebanyak i lalu di and paling kanan dengan 1 bit terus di geser ke kiri sebanyak pos 
  }
  return o;
}

void generateRoundKeys(const uint8_t* k, uint64_t rk[32]) {
  uint64_t hi = 0; //64bit hi bagian atas
  for (int i = 0; i < 8; i++) // tahap 8bit
    hi = (hi << 8) | k[i]; //hi digeser kekiri sebanyak 8 lalu di or dengan key i
  uint16_t lo = ((uint16_t)k[8] << 8) | k[9]; //16bit lo bagian bawah

  //Hi dan lo itu cuma alat bantu untuk generate round key
  for (uint8_t i = 1; i <= 32; i++) {
    rk[i - 1] = hi; //tiap round, hi langsung jadi round key

    // 1. Rotate kiri 61 bit = rotate kanan 19 bit
    uint32_t bot19  = ((uint32_t)(hi & 0x7) << 16) | lo; //hi di and 3 bit 0111 paling kanan lalu geser kiri 16 lalu or dengan lo 16 bit
    uint64_t new_hi = ((uint64_t)bot19 << 45) | (hi >> 19);//bot19 digeser kiri 45 bit di or dgn hi yg uda digeser kanan 19 bit
    uint16_t new_lo = (uint16_t)(hi >> 3); // hi digeser kanan 3 bit
    hi = new_hi;
    lo = new_lo;

    // 2. S-Box di 4 bit teratas
    uint8_t top4 = (uint8_t)(hi >> 60) & 0xF; //hi geser kanan lalu and dengan 4 bit paling kanan 1111
    hi = (hi & 0x0FFFFFFFFFFFFFFFULL) | ((uint64_t)SBox[top4] << 60); //hi di and sama offbanyak jadi sisa yg paling kiri 4 bit
    //0x0FFFFFFFFFFFFFFF = 0000 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111

    // 3. XOR counter
    hi ^= (uint64_t)(i >> 1); //geser kanan 1 bit dari 8 bit lalu di xor sama hi
    lo ^= (uint16_t)((i & 1) << 15);
  }
}

uint64_t present_encrypt_block(uint64_t pt) {
  uint64_t rk[32];
  generateRoundKeys(key, rk); //round key digenerate ulang tiap blok dienkripsi (jadi lambat)
  uint64_t state = pt;
  for (uint8_t r = 0; r < 31; r++) {
    state ^= rk[r]; //XOR round key
    state = sBoxLayer(state); // subs nibble
    state = pLayer(state); // permutasi bit
  }
  state ^= rk[31]; //XOR round key terakhir (tanpa sBox & pLayer)
  return state;
}

// Enkripsi 16 byte (2 blok)
void encryptBuffer(uint8_t* buf) {
  for (int b = 0; b < 2; b++) {
    uint64_t block = 0;
    for (int j = 0; j < 8; j++)
      block = (block << 8) | buf[b * 8 + j];
    uint64_t ct = present_encrypt_block(block);
    for (int j = 0; j < 8; j++)
      buf[b * 8 + j] = (ct >> (56 - j * 8)) & 0xFF;
  }
}

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
  oledShow("TX PRESENT", "Inisialisasi...");

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
  LoRa.setTxPower(20);

  oledShow("TX PRESENT", "LoRa Ready");
  Serial.println("==============================");
  Serial.println("  PRESENT-80 TX Siap");
  Serial.println("==============================");
  delay(2000);
}

// ===================== LOOP =====================
void loop() {
  Serial.println("==============================");
  Serial.print("[SESI] Mulai. SeqNum awal: "); Serial.println(seqNum + 1);
  oledShow("=== SESI BARU ===", "Kirim 20 paket");
  delay(1000);

  for (int paket_ke = 1; paket_ke <= PAKET_PER_SESI; paket_ke++) {
    unsigned long t_kirim = millis();

    seqNum++;

    // -- Ambil data dari data.h --
    float pv   = pvVoltage[dataIndex];
    float cur  = current[dataIndex];
    float vol  = voltage[dataIndex];
    float temp = temperature[dataIndex];
    dataIndex  = (dataIndex + 1) % JUMLAH_DATA;

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
    Serial.print("   DataIdx = "); Serial.println(dataIndex - 1 < 0 ? JUMLAH_DATA - 1 : dataIndex - 1);
    Serial.print("   PV      = "); Serial.print(pv,   2); Serial.println(" V");
    Serial.print("   I       = "); Serial.print(cur,  4); Serial.println(" A");
    Serial.print("   V       = "); Serial.print(vol,  2); Serial.println(" V");
    Serial.print("   T       = "); Serial.print(temp, 2); Serial.println(" C");
    Serial.print("   PT Hex  = "); Serial.println(hexPT);
    Serial.print("   CT Hex  = "); Serial.println(hexCT);

    // -- OLED --
    char b1[22], b2[22], b3[22], b4[22];
    snprintf(b1, sizeof(b1), "TX#%u [%d/%d]",    seqNum, paket_ke, PAKET_PER_SESI);
    snprintf(b2, sizeof(b2), "Data ke-%d", dataIndex);
    snprintf(b3, sizeof(b3), "PV=%.2fV I=%.3fA", pv, cur);
    snprintf(b4, sizeof(b4), "V=%.1fV T=%.1fC",  vol, temp);
    oledShow(b1, b2, b3, b4);

    // -- Jaga interval 2 detik per paket --
    long sisa = TX_INTERVAL_MS - (long)(millis() - t_kirim);
    if (sisa > 0) delay((unsigned long)sisa);
  }

  // -- Selesai 1 sesi --
  Serial.println("==============================");
  Serial.println("[SESI] 20 paket terkirim.");
  Serial.print("[TUNGGU] Jeda "); Serial.print(WAIT_AFTER_SESI / 1000);
  Serial.println("s untuk RX upload...");

  unsigned long waitStart = millis();
  while (millis() - waitStart < WAIT_AFTER_SESI) {
    unsigned long sisa_ms = WAIT_AFTER_SESI - (millis() - waitStart);
    unsigned int  sisa_s  = (unsigned int)(sisa_ms / 1000);
    unsigned int  menit   = sisa_s / 60;
    unsigned int  detik   = sisa_s % 60;

    char bw[22];
    snprintf(bw, sizeof(bw), "Tunggu RX: %02u:%02u", menit, detik);
    oledShow("== 20/20 SELESAI ==", "RX sedang upload", bw, "Jangan matikan!");
    Serial.print("  [TUNGGU] Sisa: "); Serial.print(menit);
    Serial.print("m"); Serial.print(detik); Serial.println("s");
    delay(5000);
  }

  Serial.println("[INFO] Siap memulai sesi berikutnya.");
  oledShow("Siap Sesi Baru", "Mulai dalam 3s...");
  delay(3000);
}
