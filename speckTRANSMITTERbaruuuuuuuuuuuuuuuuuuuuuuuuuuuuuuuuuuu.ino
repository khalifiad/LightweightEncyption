#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==================
// Pin & OLED Setup
// ==================
#define ss 10
#define rst 9
#define dio0 2

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ==================
// Speck Parameters
// ==================
#define SPECK_ROUNDS 26
uint32_t key[3] = {0x33221100, 0x77665544, 0xbbaa9988}; // 96-bit key
uint32_t round_keys[SPECK_ROUNDS];

// ==================
// Global Variables
// ==================
int counter = 0;

// ===================
// Setup
// ===================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("LCD failed"));
    while (true);
  }

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oledDisplayMessageStatus("Init...", "LoRa Ready");
  delay(2000);
  oled.clearDisplay();

  Serial.println("LoRa Sender");
  LoRa.setPins(ss, rst, dio0);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    oledDisplayMessageStatus("Init Error", "LoRa Failed");
    while (1);
  }

  speck_key_schedule();
}

// ===================
// Main Loop
// ===================
void loop() {
  String msg = "pesan " + String(counter);  // bisa lebih dari 8 karakter
  Serial.print("Plain: ");
  Serial.println(msg);

  // Konversi pesan jadi blok-blok 8 byte
  int length = msg.length();
  int blocks = (length + 7) / 8;  // pembulatan ke atas
  uint8_t ciphertext[blocks][8];  // buffer untuk menyimpan semua blok terenkripsi

  for (int b = 0; b < blocks; b++) {
    uint8_t plaintext[8] = {0};  // kosongkan buffer
    for (int i = 0; i < 8; i++) {
      int idx = b * 8 + i;
      if (idx < length) {
        plaintext[i] = msg[idx];
      } else {
        plaintext[i] = ' ';  // padding jika tidak cukup 8 karakter
      }
    }
    speck_encrypt_block(plaintext, ciphertext[b]);
  }

  // Kirim semua blok terenkripsi
  LoRa.beginPacket();
  for (int b = 0; b < blocks; b++) {
    for (int i = 0; i < 8; i++) {
      LoRa.write(ciphertext[b][i]);
    }
  }
  LoRa.endPacket();

  oledDisplayMessageStatus(msg, "Encrypted Sent");

  counter++;
  delay(2000);
}


// ===================
// OLED Display
// ===================
void oledDisplayMessageStatus(String message, String status) {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);

  oled.setCursor(0, 20);
  oled.println("Msg: " + message);

  oled.setCursor(0, 40);
  oled.println(status);

  oled.display();
}

// ===================
// Speck Encryption
// ===================
#define ROR(x, r) (((x) >> (r)) | ((x) << (32 - (r))))
#define ROL(x, r) (((x) << (r)) | ((x) >> (32 - (r))))

void speck_key_schedule() {
  uint32_t b = key[0];
  uint32_t a[2] = {key[1], key[2]};
  round_keys[0] = b;

  for (int i = 0; i < SPECK_ROUNDS - 1; i++) {
    uint32_t tmp = ROR(a[i % 2], 8);
    a[i % 2] = (tmp + b) ^ i;
    b = ROL(b, 3) ^ a[i % 2];
    round_keys[i + 1] = b;
  }
}

void speck_encrypt_block(uint8_t* input, uint8_t* output) {
  uint32_t x = ((uint32_t*)input)[0];
  uint32_t y = ((uint32_t*)input)[1];

  for (int i = 0; i < SPECK_ROUNDS; i++) {
    x = (ROR(x, 8) + y) ^ round_keys[i];
    y = ROL(y, 3) ^ x;
  }

  ((uint32_t*)output)[0] = x;
  ((uint32_t*)output)[1] = y;
}
