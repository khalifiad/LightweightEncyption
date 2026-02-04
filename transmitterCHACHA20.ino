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

// ===================
// ChaCha20 Constants
// ===================
const uint32_t chacha_const[4] = {
  0x61707865, 0x3320646E, 0x79622D32, 0x6B206574
};

uint8_t key[32] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
  0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
  0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F
};

uint8_t nonce[12] = {
  0x00, 0x00, 0x00, 0x09,
  0x00, 0x00, 0x00, 0x4A,
  0x00, 0x00, 0x00, 0x00
};

// ===================
// Global Variables
// ===================
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
}

// ===================
// Main Loop
// ===================
void loop() {
  String plainMessage = "pesan " + String(counter);
  Serial.print("Plain: ");
  Serial.println(plainMessage);

  uint8_t plaintext[64] = {0};
  uint8_t ciphertext[64] = {0};
  plainMessage.getBytes(plaintext, 64);

  chacha20_encrypt(plaintext, ciphertext, 64, key, nonce, counter);

  LoRa.beginPacket();
  for (int i = 0; i < 64; i++) {
    LoRa.write(ciphertext[i]);
  }
  LoRa.endPacket();

  oledDisplayMessageStatus(plainMessage, "Encrypted Sent");

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
// ChaCha20 Core
// ===================
#define ROTL32(v, n) ((v << n) | (v >> (32 - n)))

void quarterRound(uint32_t& a, uint32_t& b, uint32_t& c, uint32_t& d) {
  a += b; d ^= a; d = ROTL32(d, 16);
  c += d; b ^= c; b = ROTL32(b, 12);
  a += b; d ^= a; d = ROTL32(d, 8);
  c += d; b ^= c; b = ROTL32(b, 7);
}

void chacha20_block(uint32_t output[16], const uint8_t key[32], const uint8_t nonce[12], uint32_t counter) {
  uint32_t state[16];

  state[0]  = chacha_const[0];
  state[1]  = chacha_const[1];
  state[2]  = chacha_const[2];
  state[3]  = chacha_const[3];

  for (int i = 0; i < 8; i++) {
    state[4 + i] = ((uint32_t)key[i * 4]) |
                   ((uint32_t)key[i * 4 + 1] << 8) |
                   ((uint32_t)key[i * 4 + 2] << 16) |
                   ((uint32_t)key[i * 4 + 3] << 24);
  }

  state[12] = counter;
  state[13] = ((uint32_t)nonce[0]) |
              ((uint32_t)nonce[1] << 8) |
              ((uint32_t)nonce[2] << 16) |
              ((uint32_t)nonce[3] << 24);
  state[14] = ((uint32_t)nonce[4]) |
              ((uint32_t)nonce[5] << 8) |
              ((uint32_t)nonce[6] << 16) |
              ((uint32_t)nonce[7] << 24);
  state[15] = ((uint32_t)nonce[8]) |
              ((uint32_t)nonce[9] << 8) |
              ((uint32_t)nonce[10] << 16) |
              ((uint32_t)nonce[11] << 24);

  for (int i = 0; i < 16; i++) output[i] = state[i];

  for (int i = 0; i < 10; i++) {
    quarterRound(output[0], output[4], output[8],  output[12]);
    quarterRound(output[1], output[5], output[9],  output[13]);
    quarterRound(output[2], output[6], output[10], output[14]);
    quarterRound(output[3], output[7], output[11], output[15]);
    quarterRound(output[0], output[5], output[10], output[15]);
    quarterRound(output[1], output[6], output[11], output[12]);
    quarterRound(output[2], output[7], output[8],  output[13]);
    quarterRound(output[3], output[4], output[9],  output[14]);
  }

  for (int i = 0; i < 16; i++) {
    output[i] += state[i];
  }
}

void chacha20_encrypt(uint8_t* input, uint8_t* output, uint16_t length, const uint8_t* key, const uint8_t* nonce, uint32_t counter) {
  uint8_t keystream[64];
  uint32_t block[16];

  for (uint16_t i = 0; i < length; i += 64) {
    chacha20_block(block, key, nonce, counter++);

    for (int j = 0; j < 16; j++) {
      keystream[4*j + 0] = (block[j] >> 0) & 0xFF;
      keystream[4*j + 1] = (block[j] >> 8) & 0xFF;
      keystream[4*j + 2] = (block[j] >> 16) & 0xFF;
      keystream[4*j + 3] = (block[j] >> 24) & 0xFF;
    }

    for (int j = 0; j < 64 && (i + j) < length; j++) {
      output[i + j] = input[i + j] ^ keystream[j];
    }
  }
}