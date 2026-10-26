// TwinArm LightGateway 1.0 (ESP32-S3): WiFi <-> UART bridge to the follower Teensy 4.1. LoRa comes later.
// Datagrams from the PC app are text command lines for the Teensy console (motorcontroller.ino) and go to
// the Teensy unchanged; every line the Teensy prints on its UART goes back to the PC as one datagram.
// Lines starting with '#' are for the gateway itself:
//   #hello <t>   makes the sender the PC that gets the telemetry, answered with  G,<t>,<uptime ms>,<rssi dBm>
// Wiring: TXD (IO18) -> Teensy pin 7 (RX2), RXD (IO17) <- Teensy pin 8 (TX2), GND - GND. Both are 3.3 V.
// Board: "ESP32S3 Dev Module" (esp32 core by Espressif), USB CDC On Boot enabled. USB serial prints the
// gateway's IP at boot, then once a second how many datagrams went to the Teensy and lines came back.
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>

const char *AP_SSID = "TwinArm";          // the gateway's own network, used when WIFI_SSID is empty
const char *AP_PASS = "badgerlabs";       // >= 8 characters; change it before a competition
const char *WIFI_SSID = "";               // set these to join an existing network instead
const char *WIFI_PASS = "";
const uint16_t UDP_PORT = 4210;
const int UART_RX = 17, UART_TX = 18;     // the RXD / TXD header
const uint32_t UART_BAUD = 115200;        // must match GATEWAY_BAUD in motorcontroller.ino

WiFiUDP udp;
IPAddress pc;
uint16_t pcPort = 0;                      // 0 until a PC has said hello
char line[160];
size_t len = 0;
uint32_t toTeensy = 0, fromTeensy = 0;  // per second, for the status print

// Signal of the joined network, or of the first PC on our own access point.
int rssi() {
  if (*WIFI_SSID) return WiFi.RSSI();
  wifi_sta_list_t l;
  return esp_wifi_ap_get_sta_list(&l) == ESP_OK && l.num ? l.sta[0].rssi : 0;
}

void setup() {
  Serial.begin(115200);
  Serial1.setRxBufferSize(1024);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
  if (*WIFI_SSID) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) delay(200);
    Serial.printf("joined %s, gateway at %s:%d\n", WIFI_SSID, WiFi.localIP().toString().c_str(), UDP_PORT);
  } else {
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("access point %s, gateway at %s:%d\n", AP_SSID, WiFi.softAPIP().toString().c_str(), UDP_PORT);
  }
  WiFi.setSleep(false);                   // modem sleep adds 100+ ms to every reply
  udp.begin(UDP_PORT);
}

void loop() {
  if (int n = udp.parsePacket()) {
    static char buf[512];
    n = udp.read(buf, sizeof buf - 1);
    if (n > 0) {
      buf[n] = 0;
      pc = udp.remoteIP();
      pcPort = udp.remotePort();
      if (buf[0] == '#') {
        unsigned long t = 0;
        sscanf(buf, "#hello %lu", &t);
        udp.beginPacket(pc, pcPort);
        udp.printf("G,%lu,%lu,%d\n", t, millis(), rssi());
        udp.endPacket();
      } else {
        Serial1.write(buf, n);
        toTeensy++;
      }
    }
  }

  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\r') continue;
    if (c != '\n') {
      line[len++] = c;
      if (len < sizeof line) continue;    // an overlong line goes out in pieces
    }
    if (len) fromTeensy++;
    if (len && pcPort) {
      udp.beginPacket(pc, pcPort);
      udp.write((const uint8_t *)line, len);
      udp.write('\n');
      udp.endPacket();
    }
    len = 0;
  }

  static uint32_t printed = 0;
  if (millis() - printed >= 1000) {     // 0 lines from the Teensy: check TXD/RXD/GND and that it finished homing
    printed = millis();
    Serial.printf("app %s  to Teensy %lu datagrams/s  from Teensy %lu lines/s\n",
                  pcPort ? pc.toString().c_str() : "none yet", (unsigned long)toTeensy, (unsigned long)fromTeensy);
    toTeensy = fromTeensy = 0;
  }
}
