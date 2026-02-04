#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define ss 10
#define rst 9
#define dio0 2

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define ROUNDS 31
uint8_t key[10] = {0xEE, 0xFF, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88, 0x77, 0x66};

const uint8_t SBox[16] = {
  0xC, 0x5, 0x6, 0xB, 0x9, 0x0, 0xA, 0xD,
  0x3, 0xE, 0xF, 0x8, 0x4, 0x7, 0x1, 0x2
};

int counter = 0;

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

  LoRa.setPins(ss, rst, dio0);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    oledDisplayMessageStatus("Init Error", "LoRa Failed");
    while (1);
  }
}

void loop() {
  String plainMessage = "pesan " + String(counter);
  Serial.print("Plain: ");
  Serial.println(plainMessage);

  uint64_t pt1, pt2;
  stringToUint128(plainMessage, pt1, pt2);
  uint64_t ct1 = present_encrypt(pt1, key);
  uint64_t ct2 = present_encrypt(pt2, key);

  LoRa.beginPacket();
  for (int i = 7; i >= 0; i--) LoRa.write((ct1 >> (8 * i)) & 0xFF);
  for (int i = 7; i >= 0; i--) LoRa.write((ct2 >> (8 * i)) & 0xFF);
  LoRa.endPacket();

  oledDisplayMessageStatus(plainMessage, "Encrypted Sent");
  counter++;
  delay(2000);
}

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

uint64_t addRoundKey(uint64_t state, const uint8_t* key, uint8_t round) {
  uint64_t roundKey = ((uint64_t)key[2] << 56) | ((uint64_t)key[3] << 48) |
                      ((uint64_t)key[4] << 40) | ((uint64_t)key[5] << 32) |
                      ((uint64_t)key[6] << 24) | ((uint64_t)key[7] << 16) |
                      ((uint64_t)key[8] << 8)  | ((uint64_t)key[9]);
  return state ^ roundKey;
}

uint64_t sBoxLayer(uint64_t state) {
  uint64_t output = 0;
  for (int i = 0; i < 16; i++) {
    output |= ((uint64_t)SBox[(state >> (i * 4)) & 0xF]) << (i * 4);
  }
  return output;
}

uint64_t pLayer(uint64_t state) {
  uint64_t output = 0;
  for (int i = 0; i < 64; i++) {
    int pos = (16 * i) % 63;
    if (i == 63) pos = 63;
    output |= ((state >> i) & 0x01) << pos;
  }
  return output;
}

void updateKey(uint8_t* key, uint8_t round) {
  uint8_t tmp = key[0];
  for (int i = 0; i < 9; i++) key[i] = key[i + 1];
  key[9] = tmp;

  uint8_t sbox_input = key[9];
  key[9] = (SBox[sbox_input >> 4] << 4) | (sbox_input & 0x0F);

  key[1] ^= round >> 1;
  key[2] ^= round << 7;
}

uint64_t present_encrypt(uint64_t state, uint8_t* key) {
  uint8_t localKey[10];
  memcpy(localKey, key, 10);

  for (uint8_t round = 1; round <= ROUNDS; round++) {
    state = addRoundKey(state, localKey, round);
    state = sBoxLayer(state);
    state = pLayer(state);
    updateKey(localKey, round);
  }
  return addRoundKey(state, localKey, ROUNDS + 1);
}

void stringToUint128(String str, uint64_t &part1, uint64_t &part2) {
  while (str.length() < 16) str += ' ';  
  part1 = 0; part2 = 0;
  for (int i = 0; i < 8; i++) {
    part1 = (part1 << 8) | (uint8_t)str[i];
    part2 = (part2 << 8) | (uint8_t)str[i + 8];
  }
}
