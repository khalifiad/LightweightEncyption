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

const uint8_t SBox_inv[16] = {
  0x5, 0xE, 0xF, 0x8, 0xC, 0x1, 0x2, 0xD,
  0xB, 0x4, 0x6, 0x3, 0x0, 0x7, 0x9, 0xA
};

unsigned long lastReceiveTime = 0;
unsigned long latencyGap = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED gagal"));
    while (true);
  }

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oledDisplayMessageStatus("Init...", "Receiver Ready");
  delay(2000);
  oled.clearDisplay();

  LoRa.setPins(ss, rst, dio0);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);

  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa gagal start!");
    oledDisplayMessageStatus("Init Error", "LoRa Failed");
    while (1);
  }
}

void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize == 16) {
    unsigned long currentTime = millis();
    if (lastReceiveTime != 0) latencyGap = currentTime - lastReceiveTime;
    lastReceiveTime = currentTime;

    uint64_t ct1 = 0, ct2 = 0;
    for (int i = 0; i < 8; i++) ct1 = (ct1 << 8) | LoRa.read();
    for (int i = 0; i < 8; i++) ct2 = (ct2 << 8) | LoRa.read();

    uint64_t pt1 = present_decrypt(ct1, key);
    uint64_t pt2 = present_decrypt(ct2, key);

    String plainMessage = uint128ToString(pt1, pt2);
    int rssi = LoRa.packetRssi();

    Serial.print("Plain: ");
    Serial.println(plainMessage);
    oledDisplayMessageStatus(plainMessage, "RSSI: " + String(rssi));
  }
}

void oledDisplayMessageStatus(String message, String status) {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 20);
  oled.println("Msg: " + message);
  oled.setCursor(0, 40);
  oled.println(status);
  oled.setCursor(0, 55);
  oled.println("Lat: " + String(latencyGap) + " ms");
  oled.display();
}

uint64_t addRoundKey(uint64_t state, const uint8_t* key, uint8_t round) {
  uint64_t roundKey = ((uint64_t)key[2] << 56) | ((uint64_t)key[3] << 48) |
                      ((uint64_t)key[4] << 40) | ((uint64_t)key[5] << 32) |
                      ((uint64_t)key[6] << 24) | ((uint64_t)key[7] << 16) |
                      ((uint64_t)key[8] << 8)  | ((uint64_t)key[9]);
  return state ^ roundKey;
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

void reverseUpdateKey(uint8_t* key, uint8_t round) {
  key[2] ^= round << 7;
  key[1] ^= round >> 1;
  uint8_t sbox_input = key[9];
  key[9] = (SBox_inv[sbox_input >> 4] << 4) | (sbox_input & 0x0F);
  uint8_t tmp = key[9];
  for (int i = 9; i > 0; i--) key[i] = key[i - 1];
  key[0] = tmp;
}

uint64_t sBoxLayer_inv(uint64_t state) {
  uint64_t output = 0;
  for (int i = 0; i < 16; i++) {
    output |= ((uint64_t)SBox_inv[(state >> (i * 4)) & 0xF]) << (i * 4);
  }
  return output;
}

uint64_t pLayer_inv(uint64_t state) {
  uint64_t output = 0;
  for (int i = 0; i < 64; i++) {
    int pos = (16 * i) % 63;
    if (i == 63) pos = 63;
    output |= ((state >> pos) & 1) << i;
  }
  return output;
}

uint64_t present_decrypt(uint64_t state, uint8_t* key) {
  uint8_t localKey[10];
  memcpy(localKey, key, 10);
  for (uint8_t i = 1; i <= ROUNDS; i++) updateKey(localKey, i);
  state = addRoundKey(state, localKey, ROUNDS + 1);
  for (int8_t round = ROUNDS; round >= 1; round--) {
    reverseUpdateKey(localKey, round);
    state = pLayer_inv(state);
    state = sBoxLayer_inv(state);
    state = addRoundKey(state, localKey, round);
  }
  return state;
}

String uint128ToString(uint64_t part1, uint64_t part2) {
  String result = "";
  for (int i = 7; i >= 0; i--) result += (char)((part1 >> (8 * i)) & 0xFF);
  for (int i = 7; i >= 0; i--) result += (char)((part2 >> (8 * i)) & 0xFF);
  result.trim(); // hapus spasi kosong di akhir
  return result;
}
