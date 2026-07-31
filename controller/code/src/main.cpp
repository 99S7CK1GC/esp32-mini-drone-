#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Replace with your drone's MAC:
uint8_t drone_mac[] = {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};

#pragma pack(push, 1)
struct CtrlPacket {
  uint8_t throttle; // 0–255
  int8_t roll;      // -127..127
  int8_t pitch;     // -127..127
  int8_t yaw;       // -127..127
  uint8_t armed;    // 0 or 1
  uint8_t mode;     // 0=rate 1=stab
  uint32_t seq;     // packet counter
};
#pragma pack(pop)

CtrlPacket pkt;
uint32_t seq = 0;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, drone_mac, 6);
  peer.channel = 0;
  esp_now_add_peer(&peer);
  
  pinMode(4, INPUT_PULLUP);
}

void loop() {
  pkt.throttle = map(analogRead(1), 0,4095, 0,255);
  pkt.roll = map(analogRead(2), 0,4095,-127,127);
  pkt.pitch = map(analogRead(3), 0,4095,-127,127);
  pkt.yaw = map(analogRead(0), 0,4095,-127,127);
  pkt.armed = !digitalRead(4);
  pkt.seq = seq++;
  
  esp_now_send(drone_mac, (uint8_t*)&pkt, sizeof(pkt));
  delay(10); // 100 Hz
}
