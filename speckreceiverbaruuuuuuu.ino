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

#define SPECK_ROUNDS 26
uint32_t key[3] = {0x33221100, 0x77665544, 0xbbaa9988}; // harus sama dengan transmitter
uint32_t round_keys[SPECK_ROUNDS];

unsigned long lastReceiveTime = 0;
unsigned long latencyGap = 0;

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

  LoRa.setPins(ss, rst, dio0);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    oledDisplayMessageStatus("Init Error", "LoRa Failed");
    while (1);
  }

  speck_key_schedule();
  oledDisplayMessageStatus("LoRa Receiver", "Started");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0 && packetSize % 8 == 0) {  // pastikan kelipatan 8
    unsigned long currentTime = millis();
    if (lastReceiveTime != 0) latencyGap = currentTime - lastReceiveTime;
    lastReceiveTime = currentTime;

    int blocks = packetSize / 8;
    char fullMessage[packetSize + 1]; // +1 untuk null terminator
    int msgIndex = 0;

    for (int b = 0; b < blocks; b++) {
      uint8_t ciphertext[8];
      for (int i = 0; i < 8; i++) {
        ciphertext[i] = LoRa.read();
      }

      uint8_t decrypted[8];
      speck_decrypt_block(ciphertext, decrypted);

      for (int i = 0; i < 8; i++) {
        fullMessage[msgIndex++] = decrypted[i];
      }
    }

    fullMessage[msgIndex] = '\0'; // null-terminate string
    int rssi = LoRa.packetRssi();

    Serial.print("Plain: ");
    Serial.println(fullMessage);
    oledDisplayMessageStatus(fullMessage, "RSSI: " + String(rssi));
  }
}

void oledDisplayMessageStatus(String label, String message) {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 20);
  oled.println("Msg: " + label);
  oled.setCursor(0, 40);
  oled.println(message);
  oled.setCursor(0, 55);
  oled.println("Lat: " + String(latencyGap) + " ms");
  oled.display();
}

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

void speck_decrypt_block(uint8_t* input, uint8_t* output) {
  uint32_t x = ((uint32_t*)input)[0];
  uint32_t y = ((uint32_t*)input)[1];

  for (int i = SPECK_ROUNDS - 1; i >= 0; i--) {
    y ^= x;
    y = ROR(y, 3);
    x ^= round_keys[i];
    x = (x - y);
    x = ROL(x, 8);
  }

  ((uint32_t*)output)[0] = x;
  ((uint32_t*)output)[1] = y;
}
