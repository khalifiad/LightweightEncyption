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
#define OLED_ADDR      0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===================== KONFIGURASI =====================
#define BATCH_SIZE       20
#define TX_INTERVAL_MS   2000L
#define WAIT_AFTER_BATCH 180000UL

// ===================== CHACHA20 =====================
// uint8_t  → 1 byte → 2 digit hex → 0xXX
// uint16_t → 2 byte → 4 digit hex → 0xXXXX
// uint32_t → 4 byte → 8 digit hex → 0xXXXXXXXX
const uint32_t chacha_const[4] = {0x61707865, 0x3320646E, 0x79622D32, 0x6B206574};
uint8_t key[32] = {
  0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
  0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
  0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,
  0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F
};
uint8_t nonce[12] = {0x00,0x00,0x00,0x09,0x00,0x00,0x00,0x4A,0x00,0x00,0x00,0x00};

// ===================== COUNTER & INDEX =====================
uint16_t counter   = 1;
int      dataIndex = 0;

// ===================== BATCH TRACKING =====================
unsigned int  batch_number       = 1;
unsigned int  batch_counter      = 0;
unsigned long batch_packets_sent = 0;
unsigned long total_packets_sent = 0;

// ===================== TIMING STATISTIK (batch) =====================
unsigned long batch_enc_total_us = 0;

// ===================== CHACHA20 FUNCTIONS =====================
#define ROTL32(v,n) (((v) << (n)) | ((v) >> (32-(n))))//v rotate ke kiri sebanyak n lalu di or dengan v di rotate kanan sebanyak 32-n

void quarterRound(uint32_t &a, uint32_t &b, uint32_t &c, uint32_t &d) {//32 bit ada 4 word
  a += b; d ^= a; d = ROTL32(d,16);//baris 1
  c += d; b ^= c; b = ROTL32(b,12);//baris 2
  a += b; d ^= a; d = ROTL32(d, 8);
  c += d; b ^= c; b = ROTL32(b, 7);
}

void chacha20_block(uint32_t out[16], const uint8_t k[32],
                    const uint8_t n[12], uint32_t ctr) {
  uint32_t s[16];//16word tabel
  s[0]=chacha_const[0]; s[1]=chacha_const[1];//konstanta
  s[2]=chacha_const[2]; s[3]=chacha_const[3];

  for(int i=0;i<8;i++) //key
    s[4+i]=((uint32_t)k[4*i])|((uint32_t)k[4*i+1]<<8)|
            ((uint32_t)k[4*i+2]<<16)|((uint32_t)k[4*i+3]<<24);
            
  s[12]=ctr;

  s[13]=((uint32_t)n[0])|((uint32_t)n[1]<<8)|((uint32_t)n[2]<<16)|((uint32_t)n[3]<<24); //nonce
  s[14]=((uint32_t)n[4])|((uint32_t)n[5]<<8)|((uint32_t)n[6]<<16)|((uint32_t)n[7]<<24);
  s[15]=((uint32_t)n[8])|((uint32_t)n[9]<<8)|((uint32_t)n[10]<<16)|((uint32_t)n[11]<<24);
  
  for(int i=0;i<16;i++) out[i]=s[i]; //outnya acak
  for(int r=0;r<10;r++){//10doubleround
    //acak per kolom
    quarterRound(out[0],out[4],out[8],out[12]);
    quarterRound(out[1],out[5],out[9],out[13]);
    quarterRound(out[2],out[6],out[10],out[14]);
    quarterRound(out[3],out[7],out[11],out[15]);
    //acak per diagonal
    quarterRound(out[0],out[5],out[10],out[15]);
    quarterRound(out[1],out[6],out[11],out[12]);
    quarterRound(out[2],out[7],out[8],out[13]);
    quarterRound(out[3],out[4],out[9],out[14]);
  }
  for(int i=0;i<16;i++) out[i]+=s[i]; //hasil out acakan di tambah s awal
}

// ===================== OPTIMASI ENKRIPSI =====================
void encryptBuffer(uint8_t* buf, uint16_t len, uint16_t ctr) { //buf plaintext
  uint32_t blk[16]; //ambil keystream
  chacha20_block(blk, key, nonce, ctr);
  for(uint16_t j=0; j<len; j++){
    uint8_t ks_byte = (blk[j/4] >> ((j%4)*8)) & 0xFF; //ks_byte itu 1 byte keystream yang diambil dari blk untuk di-XOR sama 1 byte plaintext
    buf[j] ^= ks_byte; //XOR plaintext sama keystream
  }
}

// ===================== OLED HELPERS =====================
void oledClear4(const char* l1, const char* l2="",
                const char* l3="", const char* l4="") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,  0); display.println(l1);
  display.setCursor(0, 16); display.println(l2);
  display.setCursor(0, 32); display.println(l3);
  display.setCursor(0, 48); display.println(l4);
  display.display();
}

void oledShowPacket(uint16_t ctr, unsigned int bNum, unsigned int bPos,
                    float pv, float cur, float vol, float tmp,
                    unsigned long enc_us) {
  char l1[22], l2[22], l3[22], l4[22];
  snprintf(l1, sizeof(l1), "TX#%u  B%u(%u/%u)", ctr, bNum, bPos, BATCH_SIZE);
  snprintf(l2, sizeof(l2), "Data ke-%d", dataIndex);
  snprintf(l3, sizeof(l3), "PV:%.2fV I:%.3fA",  pv, cur);
  snprintf(l4, sizeof(l4), "V:%.2fV T:%.2fC",   vol, tmp);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,  0); display.println(l1);
  display.setCursor(0, 16); display.println(l2);
  display.setCursor(0, 32); display.println(l3);
  display.setCursor(0, 48); display.println(l4);
  display.display();
}

void oledShowBatchWait(unsigned int bNum, unsigned int menit, unsigned int detik) {
  char l1[22], l2[22];
  snprintf(l1, sizeof(l1), "Batch%u DONE 20/20",  bNum);
  snprintf(l2, sizeof(l2), "Tunggu: %02u:%02u",  menit, detik);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,  0); display.println(l1);
  display.setCursor(0, 16); display.println(l2);
  display.display();
}

void oledShowDone(unsigned long total) {
  char l2[22];
  snprintf(l2, sizeof(l2), "Total TX: %lu pkt", total);
  oledClear4("=== SELESAI ===", l2, "Semua data", "telah terkirim.");
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(500);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("[ERROR] OLED gagal!"));
    while(1);
  }
  oledClear4("TX ChaCha20", "Inisialisasi...");

  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(915E6)) {
    Serial.println(F("[ERROR] LoRa gagal!"));
    oledClear4("ERROR", "LoRa gagal!");
    while(1);
  }
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  // LoRa.enableCrc();
  LoRa.setTxPower(20);

  oledClear4("TX ChaCha20", "loRa Ready");

  Serial.println(F("=============================="));
  Serial.println(F("  TX ChaCha20 Siap  "));
  Serial.println(F("=============================="));
  delay(1500);
}

// ===================== LOOP =====================
void loop() {
  unsigned long t_kirim = millis();

  // Ambil data
  float pv   = pvVoltage[dataIndex];
  float cur  = current[dataIndex];
  float vol  = voltage[dataIndex];
  float tmp  = temperature[dataIndex];

  // Susun plaintext 16 byte
  uint8_t payload[16];
  memcpy(&payload[0],  &pv,  4);
  memcpy(&payload[4],  &cur, 4);
  memcpy(&payload[8],  &vol, 4);
  memcpy(&payload[12], &tmp, 4);

  // Log plaintext hex
  char hexPT[33]; hexPT[0]='\0';
  for(int i=0;i<16;i++){ 
    char b[3]; 
    sprintf(b,"%02X",payload[i]); 
    strcat(hexPT,b); 
  }

  // Enkripsi + ukur waktu
  unsigned long enc_start  = micros();
  encryptBuffer(payload, 16, counter);
  unsigned long encTime_us = micros() - enc_start;

  // Log ciphertext hex
  char hexCT[33]; hexCT[0]='\0';
  for(int i=0;i<16;i++){ 
    char b[3]; 
    sprintf(b,"%02X",payload[i]); 
    strcat(hexCT,b); 
  }

  // Update statistik batch
  batch_enc_total_us += encTime_us;

  // Kirim paket LoRa (23 byte)
  LoRa.beginPacket();
  LoRa.write(0x55);
  LoRa.write((counter    >> 8) & 0xFF);
  LoRa.write( counter          & 0xFF);
  LoRa.write((encTime_us >> 24) & 0xFF);
  LoRa.write((encTime_us >> 16) & 0xFF);
  LoRa.write((encTime_us >>  8) & 0xFF);
  LoRa.write( encTime_us        & 0xFF);
  LoRa.write(payload, 16);
  LoRa.endPacket();

  // Update OLED
  oledShowPacket(counter, batch_number, batch_counter + 1,
                 pv, cur, vol, tmp, encTime_us);

  // Serial log
  Serial.println(F("------------------------------"));
  Serial.print(F("[TX #")); Serial.print(counter);
  Serial.print(F("] Paket ")); Serial.print(batch_counter+1);
  Serial.print(F("/")); Serial.println(BATCH_SIZE);
  Serial.print(F("   Data ke-")); Serial.println(dataIndex);
  Serial.print(F("   PV       : ")); Serial.print(pv,2);   Serial.println(F(" V"));
  Serial.print(F("   I        : ")); Serial.print(cur,4);  Serial.println(F(" A"));
  Serial.print(F("   V        : ")); Serial.print(vol,2);  Serial.println(F(" V"));
  Serial.print(F("   T        : ")); Serial.print(tmp,2);  Serial.println(F(" C"));
  Serial.print(F("   PT Hex   : ")); Serial.println(hexPT);
  Serial.print(F("   CT Hex   : ")); Serial.println(hexCT);

  total_packets_sent++;
  batch_packets_sent++;
  batch_counter++;
  counter++;
  dataIndex = (dataIndex + 1) % JUMLAH_DATA;

  // Setiap 20 paket: tampil ringkasan & tunggu RX
  if (batch_counter >= BATCH_SIZE) {
    unsigned long avg_enc = batch_enc_total_us / batch_packets_sent;

    Serial.println(F("=============================="));
    Serial.println(F("[BATCH SELESAI - TUNGGU RX]"));
    Serial.print(F("   Batch     : ")); Serial.println(batch_number);
    Serial.print(F("   Sent      : ")); Serial.print(batch_packets_sent); Serial.println(F("/20"));
    Serial.print(F("   Tunggu RX : ")); Serial.print(WAIT_AFTER_BATCH/1000); Serial.println(F(" detik"));

    unsigned long waitStart = millis();
    while (millis() - waitStart < WAIT_AFTER_BATCH) {
      unsigned long sisa_ms = WAIT_AFTER_BATCH - (millis() - waitStart);
      unsigned int  sisa_s  = (unsigned int)(sisa_ms / 1000);
      oledShowBatchWait(batch_number, sisa_s / 60, sisa_s % 60);
      Serial.print(F("   Sisa: "));
      Serial.print(sisa_s/60); Serial.print(F("m "));
      Serial.print(sisa_s%60); Serial.println(F("s"));
      delay(5000);
    }

    // Reset batch
    batch_packets_sent  = 0;
    batch_counter       = 0;
    batch_number++;
    batch_enc_total_us  = 0;

    Serial.println(F("[INFO] Siap batch berikutnya."));
    oledClear4("Siap Batch Baru", "Mulai dalam 3s...");
    delay(3000);
    return;
  }

  long sisa = TX_INTERVAL_MS - (long)(millis() - t_kirim);
  if (sisa > 0) delay((unsigned long)sisa);
}
