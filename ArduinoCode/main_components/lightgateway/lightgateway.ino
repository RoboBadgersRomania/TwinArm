// TwinArm LightGateway 1.0 (ESP32-S3): WiFi <-> UART bridge to the follower Teensy 4.1. LoRa comes later.
// Datagrams from the PC app hold text command lines for the Teensy console (motorcontroller.ino), which go
// to the Teensy unchanged; every line the Teensy prints on its UART goes back to the PC as one datagram.
// Lines starting with '#' are for the gateway itself:
//   #hello <t>                 makes the sender the PC that gets the telemetry; answer G,<t>,<uptime ms>,<rssi dBm>
//   #save <id> <pKp pKi vKp vKi>  keeps one driver's gains in the gateway's flash; answer B,id,<the four gains>
//   #load                      answer B,id,<gains> for every motor kept, then B,end,<count>
// The gain backup lives in NVS (Preferences): it survives power-off and re-uploading this sketch,
// unless "Erase All Flash Before Sketch Upload" is on.
// Wiring: TXD (IO18) -> Teensy pin 7 (RX2), RXD (IO17) <- Teensy pin 8 (TX2), GND - GND. Both are 3.3 V.
// Board: "ESP32S3 Dev Module" (esp32 core by Espressif), USB CDC On Boot enabled. USB serial prints the
// gateway's IP at boot, then once a second how many lines went to the Teensy and came back.
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <esp_wifi.h>

const char *AP_SSID = "TwinArm";          // the gateway's own network, used when WIFI_SSID is empty
const char *AP_PASS = "badgerlabs";       // >= 8 characters; change it before a competition
const char *WIFI_SSID = "";               // set these to join an existing network instead
const char *WIFI_PASS = "";
const uint16_t UDP_PORT = 4210;
const int UART_RX = 17, UART_TX = 18;     // the RXD / TXD header
const uint32_t UART_BAUD = 115200;        // must match GATEWAY_BAUD in motorcontroller.ino
const int MAX_MOTORS = 8;                 // CAN ids 1..8 can have a gain backup

WiFiUDP udp;
Preferences store;
IPAddress pc;
uint16_t pcPort = 0;                      // 0 until a PC has said hello
char line[160];
size_t len = 0;
uint32_t toTeensy = 0, fromTeensy = 0;  // lines per second, for the status print

// Signal of the joined network, or of the first PC on our own access point.
int rssi() {
  if (*WIFI_SSID) return WiFi.RSSI();
  wifi_sta_list_t l;
  return esp_wifi_ap_get_sta_list(&l) == ESP_OK && l.num ? l.sta[0].rssi : 0;
}

void toPc(const char *s) {
  udp.beginPacket(pc, pcPort);
  udp.print(s);
  udp.print('\n');
  udp.endPacket();
}

// Sends B,id,gains for one stored motor; false if it has no backup.
bool sendBackup(int id) {
  char key[4], out[96];
  float g[4];
  snprintf(key, sizeof key, "g%d", id);
  if (!store.isKey(key) || store.getBytes(key, g, sizeof g) != sizeof g) return false;
  snprintf(out, sizeof out, "B,%d,%.4f,%.4f,%.4f,%.4f", id, g[0], g[1], g[2], g[3]);
  toPc(out);
  return true;
}

void gatewayCommand(const char *s) {
  char out[96];
  unsigned long t;
  int id;
  float g[4];
  if (sscanf(s, "#hello %lu", &t) == 1) {
    snprintf(out, sizeof out, "G,%lu,%lu,%d", t, millis(), rssi());
    toPc(out);
  } else if (sscanf(s, "#save %d %f %f %f %f", &id, &g[0], &g[1], &g[2], &g[3]) == 5 && id >= 1 && id <= MAX_MOTORS) {
    char key[4];
    snprintf(key, sizeof key, "g%d", id);
    store.putBytes(key, g, sizeof g);
    sendBackup(id);                       // read back from flash, so the app sees what was really kept
    Serial.printf("motor %d gains backed up\n", id);
  } else if (strncmp(s, "#load", 5) == 0) {
    int n = 0;
    for (id = 1; id <= MAX_MOTORS; id++) n += sendBackup(id);
    snprintf(out, sizeof out, "B,end,%d", n);
    toPc(out);
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.setRxBufferSize(1024);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
  store.begin("twinarm", false);
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
      for (char *s = strtok(buf, "\n"); s; s = strtok(nullptr, "\n")) {
        if (s[0] == '#') {
          gatewayCommand(s);
        } else {
          Serial1.print(s);
          Serial1.print('\n');
          toTeensy++;
        }
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
    Serial.printf("app %s  to Teensy %lu lines/s  from Teensy %lu lines/s\n",
                  pcPort ? pc.toString().c_str() : "none yet", (unsigned long)toTeensy, (unsigned long)fromTeensy);
    toTeensy = fromTeensy = 0;
  }
}
