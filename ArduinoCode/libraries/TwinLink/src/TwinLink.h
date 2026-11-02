// TwinLink: the encrypted, frequency-hopping LoRa link between the two LightGateway 1.0 boards
// (ESP32-S3 + EBYTE E22-400M22S / SX1268, 433 MHz). The master arm's gateway leads, the follower's answers.
//
//   pairing    both gateways hold the same 16-byte key in flash (NVS), set once over USB:  #key <32 hex digits>
//   handshake  master HELLO(nonce M) -> follower CHALLENGE(nonce F, MAC) -> master CONFIRM(MAC). Each end proves it
//              holds the key with HMAC-SHA256 over both nonces; the key itself never goes on air. The session key is
//              HMAC(key, both nonces), so it is new on every connection
//   hopping    one slot every slotUs: the master's frame, then the follower's reply, both on that slot's channel.
//              Every 7 slots use each of the 7 channels once, in an order shuffled from the session key
//   frames     AES-128-GCM under the session key, the slot number as nonce (never repeats in a session), 8-byte tag.
//              A forged, replayed or garbled frame fails the tag or the slot check and is dropped
//   streams    text lines both ways (app commands out, Teensy telemetry back), cut into chunks spread over the slots:
//              a chunk not acknowledged goes again in the next slot, on the next channel
//   link loss  LINK_TIMEOUT_MS without a good frame: both go back to the handshake, queued lines are dropped
// Used by master_arm.ino and lightgateway.ino. Needs RadioLib; build with this libraries folder on the library path
// (arduino-cli compile --libraries ArduinoCode/libraries, or a copy in the IDE sketchbook's libraries folder).
#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <RadioLib.h>
#include "esp_random.h"
#include "mbedtls/gcm.h"
#include "mbedtls/md.h"

namespace twinlink {
// LightGateway 1.0 wiring of the E22 module, from lightaprs' LoRa_SX1268_TX_RX_Test
const int LORA_VCC = 21, LORA_RXEN = 42, LORA_TXEN = 14, LORA_SCK = 12, LORA_MISO = 13, LORA_MOSI = 11,
          LORA_CS = 10, LORA_RST = 9, LORA_DIO1 = 5, LORA_BUSY = 6;
// MHz, 500 kHz wide each, all inside the 433.05-434.79 MHz ISM band (they overlap: only one end sends at a time)
const float CHANNELS[] = {433.3f, 433.5f, 433.7f, 433.9f, 434.1f, 434.3f, 434.5f};
const int NCH = sizeof CHANNELS / sizeof CHANNELS[0];
const float BW_KHZ = 500;
const uint8_t SF = 5, CR = 5;                     // the fastest LoRa setting: about 5 ms for a short frame
const uint16_t PREAMBLE = 12;                     // symbols: the SX126x needs 12 or more at SF5 / SF6
const float TCXO_V = 1.8f;                        // the E22's 32 MHz TCXO, fed from DIO3 (EBYTE datasheet)
const int8_t POWER_DBM = 10;                      // 10 mW, the EU limit in this band; the module goes to 22
const size_t STATE_M = 11, STATE_F = 2;           // fixed bytes in every master / follower frame
const size_t CHUNK_M = 48, CHUNK_F = 80;          // most stream bytes per master / follower frame
const size_t HDR = 5, TAG = 8;                    // type + slot in the clear (but authenticated), GCM tag
const uint32_t MARGIN_US = 6000, GUARD_US = 4000; // spare time per slot; the follower is on the next channel this early
const uint32_t REPLY_US = 3000;                   // an answer waits this long, so the other end is listening by then
const uint32_t LINK_TIMEOUT_MS = 500, HELLO_MS = 30, DWELL_MS = 250, FIRST_SLOT_US = 10000;
enum : uint8_t { HELLO = 1, CHALLENGE, CONFIRM, DATA_M, DATA_F };
}

inline void put16(uint8_t *b, uint16_t v) { b[0] = v; b[1] = v >> 8; }
inline uint16_t get16(const uint8_t *b) { return b[0] | b[1] << 8; }
inline uint32_t get32(const uint8_t *b) { return b[0] | b[1] << 8 | b[2] << 16 | (uint32_t)b[3] << 24; }

// The master arm's state bytes as the app's line M,<turret>,<shoulder>,<elbow>,<wrist pitch>,<roll pot %>,<gripper>,<ok>.
// Bytes: 4 joints as 14-bit MT6701 angles (0xFFFF = no reading, -1 deg in the line), the wrist-roll pot as 12 bits,
// the gripper (1 = closed). ok: one bit per joint that answered.
inline void masterLine(const uint8_t *s, char *out, size_t n) {
  float d[4];
  int ok = 0;
  for (int j = 0; j < 4; j++) {
    uint16_t v = get16(s + 2 * j);
    ok |= (v != 0xFFFF) << j;
    d[j] = v == 0xFFFF ? -1 : v * 360.0f / 16384;
  }
  snprintf(out, n, "M,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%d", d[0], d[1], d[2], d[3], get16(s + 8) * 100.0f / 4095, s[10], ok);
}

struct Ring {  // byte FIFO; the indices only grow, so used() stays right when they wrap
  uint8_t b[2048];
  uint32_t head = 0, tail = 0;
  size_t used() const { return head - tail; }
  size_t room() const { return sizeof b - used(); }
  void put(uint8_t c) { b[head++ % sizeof b] = c; }
  uint8_t at(size_t i) const { return b[(tail + i) % sizeof b]; }
  void drop(size_t n) { tail += n; }
  void clear() { tail = head; }
};

// The protocol without the radio: key, handshake, frames, hop order, line streams.
class TwinCore {
 public:
  uint8_t state[twinlink::STATE_M] = {};  // master: sent in every frame; follower: the last one received
  uint32_t stateAt = 0;                   // follower: millis() when it arrived

  TwinCore() { mbedtls_gcm_init(&gcm); }
  ~TwinCore() { mbedtls_gcm_free(&gcm); }

  bool write(const char *line) {  // queues one line; false (dropped) when the queue is full
    size_t n = strlen(line);
    if (out.room() < n + 1) return false;
    while (*line) out.put(*line++);
    out.put('\n');
    return true;
  }

  bool read(char *line, size_t n) {  // the next line that came in, cut to n - 1 characters
    size_t u = in.used();
    for (size_t i = 0; i < u; i++) {
      if (in.at(i) != '\n') continue;
      size_t k = min(i, n - 1);
      for (size_t j = 0; j < k; j++) line[j] = in.at(j);
      line[k] = 0;
      in.drop(i + 1);
      return true;
    }
    if (!in.room()) in.clear();  // full without a line end: garbage
    return false;
  }

  // Handshake, a full session's frames with a lost and a tampered one, and a follower with the wrong key.
  static bool selfTest(Print &log) {
    TwinCore *m = new TwinCore, *f = new TwinCore, *x = new TwinCore;
    uint8_t k[16], h[32], c[32], r[32], b[128], b2[128];
    for (int i = 0; i < 16; i++) k[i] = i * 17 + 3;
    m->master = true;
    m->setKey(k);
    f->setKey(k);
    k[5] ^= 1;
    x->setKey(k);
    bool done, ok = true;
    size_t n = m->hello(h), nc = x->handshake(h, n, c, done);
    ok &= nc == 25 && m->handshake(c, nc, r, done) == 0 && !done;  // wrong key: no CONFIRM
    nc = f->handshake(h, n, c, done);
    size_t nr = m->handshake(c, nc, r, done);
    ok &= nr == 17 && done;
    ok &= f->handshake(r, nr, c, done) == 0 && done;
    m->start();
    f->start();
    f->start();  // a second session on the same context must not change the result
    for (uint32_t s = 0; s < 70; s += twinlink::NCH) {  // both ends hop alike, each block of 7 uses all 7
      int seen = 0;
      for (int i = 0; i < twinlink::NCH; i++) {
        ok &= m->hop(s + i) == f->hop(s + i);
        seen |= 1 << m->hop(s + i);
      }
      ok &= seen == (1 << twinlink::NCH) - 1;
    }
    char tl[151], got[200];
    for (int i = 0; i < 150; i++) tl[i] = 'A' + i % 26;  // longer than a follower chunk
    tl[150] = 0;
    ok &= m->write("1 a12.50") && m->write("t -30.0") && f->write(tl);
    m->state[0] = 0x5A;
    for (uint32_t s = 1; s <= 8; s++) {
      m->slot = f->slot = s;
      size_t a = m->seal(b);
      if (s == 3) continue;  // lost on air: the chunk goes again next slot
      b[a - 1] ^= 1;
      ok &= !f->open(b, a);  // a flipped bit fails the tag
      b[a - 1] ^= 1;
      ok &= f->open(b, a);
      size_t z = f->seal(b2);
      ok &= !f->open(b2, z) && m->open(b2, z);  // follower frames only open at the master
    }
    ok &= f->state[0] == 0x5A;
    ok &= f->read(got, sizeof got) && !strcmp(got, "1 a12.50") && f->read(got, sizeof got) && !strcmp(got, "t -30.0");
    ok &= !f->read(got, sizeof got) && m->read(got, sizeof got) && !strcmp(got, tl);
    delete m;
    delete f;
    delete x;
    log.printf("selftest %s\n", ok ? "ok" : "FAILED");
    return ok;
  }

 protected:
  bool master = false, haveKey = false;
  uint8_t psk[16] = {}, nM[8] = {}, nF[8] = {};
  char fp[9] = "";                    // key fingerprint: first 4 bytes of its SHA-256
  mbedtls_gcm_context gcm;
  uint32_t seed = 0, slot = 0;
  Ring out, in;
  uint8_t txSeq = 1, rxSeq = 0;       // stop-and-wait: our chunk's number, the last chunk we took in
  size_t inflight = 0;                // bytes of out in the chunk on air, kept until acknowledged
  float rssi = 0, snr = 0, peerRssi = 0, peerSnr = 0;

  void setKey(const uint8_t *k) {
    uint8_t h[32];
    memcpy(psk, k, sizeof psk);
    mbedtls_md(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), psk, sizeof psk, h);
    snprintf(fp, sizeof fp, "%02x%02x%02x%02x", h[0], h[1], h[2], h[3]);
    haveKey = true;
  }

  void sessionHash(char tag, uint8_t *h) const {  // HMAC(key, tag | nonce M | nonce F)
    uint8_t m[17] = {(uint8_t)tag};
    memcpy(m + 1, nM, 8);
    memcpy(m + 9, nF, 8);
    mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), psk, sizeof psk, m, sizeof m, h);
  }

  static bool same(const uint8_t *a, const uint8_t *b, size_t n) {  // constant time
    uint8_t d = 0;
    while (n--) d |= *a++ ^ *b++;
    return !d;
  }

  size_t hello(uint8_t *b) {
    esp_fill_random(nM, sizeof nM);
    b[0] = twinlink::HELLO;
    memcpy(b + 1, nM, 8);
    return 9;
  }

  // A handshake frame in, the reply's length out (0: none). done: this end checked the other's MAC.
  size_t handshake(const uint8_t *b, size_t n, uint8_t *reply, bool &done) {
    using namespace twinlink;
    uint8_t h[32];
    done = false;
    if (master && n == 25 && b[0] == CHALLENGE && !memcmp(b + 1, nM, 8)) {
      memcpy(nF, b + 9, 8);
      sessionHash('F', h);
      if (!same(b + 17, h, 8)) return 0;  // a follower without our key
      sessionHash('M', h);
      reply[0] = CONFIRM;
      memcpy(reply + 1, nM, 8);
      memcpy(reply + 9, h, 8);
      done = true;
      return 17;
    }
    if (!master && n == 9 && b[0] == HELLO) {
      memcpy(nM, b + 1, 8);
      esp_fill_random(nF, sizeof nF);
      sessionHash('F', h);
      reply[0] = CHALLENGE;
      memcpy(reply + 1, nM, 8);
      memcpy(reply + 9, nF, 8);
      memcpy(reply + 17, h, 8);
      return 25;
    }
    if (!master && n == 17 && b[0] == CONFIRM && !memcmp(b + 1, nM, 8)) {
      sessionHash('M', h);
      done = same(b + 9, h, 8);
    }
    return 0;
  }

  void start() {  // after the handshake: session key, hop order, streams from scratch
    uint8_t h[32];
    sessionHash('K', h);
    mbedtls_gcm_free(&gcm);  // a fresh context: re-keying a used one left the ESP32's GCM on the old hash key
    mbedtls_gcm_init(&gcm);
    mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, h, 128);
    seed = get32(h + 16);
    slot = 0;
    txSeq = 1;
    rxSeq = inflight = 0;
    in.clear();
    out.clear();
  }

  static uint32_t mix(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352d;
    x ^= x >> 15;
    x *= 0x846ca68b;
    return x ^ x >> 16;
  }

  int hop(uint32_t s) const {  // channel of slot s: a fresh shuffle of all channels every NCH slots
    uint8_t order[twinlink::NCH];
    for (int i = 0; i < twinlink::NCH; i++) order[i] = i;
    uint32_t x = mix(seed ^ s / twinlink::NCH);
    for (int i = twinlink::NCH - 1; i > 0; i--) {
      x = mix(x);
      int j = x % (i + 1);
      uint8_t t = order[i];
      order[i] = order[j];
      order[j] = t;
    }
    return order[s % twinlink::NCH];
  }

  // This end's frame for the current slot: [type][slot] then, sealed, [seq][ack][len][state][chunk], then the tag.
  size_t seal(uint8_t *b) {
    using namespace twinlink;
    size_t st = master ? STATE_M : STATE_F;
    uint8_t p[3 + STATE_F + CHUNK_F];
    static_assert(STATE_M + CHUNK_M <= STATE_F + CHUNK_F, "p holds the bigger frame");
    if (!inflight) inflight = min(out.used(), master ? CHUNK_M : CHUNK_F);
    p[0] = txSeq;
    p[1] = rxSeq;
    p[2] = inflight;
    if (master) {
      memcpy(p + 3, state, st);
    } else {  // how the master's frames arrive here
      p[3] = (int8_t)constrain(lroundf(rssi), -128, 127);
      p[4] = (int8_t)constrain(lroundf(snr * 4), -128, 127);
    }
    for (size_t i = 0; i < inflight; i++) p[3 + st + i] = out.at(i);
    size_t n = 3 + st + inflight;
    uint8_t iv[12] = {};
    b[0] = master ? DATA_M : DATA_F;
    for (int i = 0; i < 4; i++) b[1 + i] = slot >> 8 * i;
    memcpy(iv, b, HDR);
    mbedtls_gcm_crypt_and_tag(&gcm, MBEDTLS_GCM_ENCRYPT, n, iv, sizeof iv, b, HDR, p, b + HDR, TAG, b + HDR + n);
    return HDR + n + TAG;
  }

  bool open(const uint8_t *b, size_t n) {  // the other end's frame: true if genuine; takes its chunk, ack and state
    using namespace twinlink;
    size_t st = master ? STATE_F : STATE_M;
    uint8_t p[3 + STATE_F + CHUNK_F], iv[12] = {};
    if (n < HDR + 3 + st + TAG || n - HDR - TAG > sizeof p || b[0] != (master ? DATA_F : DATA_M)) return false;
    size_t pn = n - HDR - TAG;
    memcpy(iv, b, HDR);
    if (mbedtls_gcm_auth_decrypt(&gcm, pn, iv, sizeof iv, b, HDR, b + n - TAG, TAG, b + HDR, p)) return false;
    size_t len = p[2];
    if (3 + st + len != pn) return false;
    if (inflight && p[1] == txSeq) {  // our chunk got there
      out.drop(inflight);
      inflight = 0;
      txSeq = txSeq == 255 ? 1 : txSeq + 1;
    }
    if (len && p[0] != rxSeq && in.room() >= len) {  // a new chunk; no room: not acknowledged, it comes again
      for (size_t i = 0; i < len; i++) in.put(p[3 + st + i]);
      rxSeq = p[0];
    }
    if (master) {
      peerRssi = (int8_t)p[3];
      peerSnr = (int8_t)p[4] / 4.0f;
    } else {
      memcpy(state, p + 3, st);
      stateAt = millis();
    }
    return true;
  }
};

// TwinCore on the SX1268, with the slot timing. Call poll() every loop, as often as possible.
class TwinLink : public TwinCore {
 public:
  TwinLink() : radio(new Module(twinlink::LORA_CS, twinlink::LORA_DIO1, twinlink::LORA_RST, twinlink::LORA_BUSY, spi)) {}

  // Starts the radio; false (and why, on Serial) without a key or a radio. The rest of the sketch keeps working.
  bool begin(bool isMaster) {
    using namespace twinlink;
    master = isMaster;
    prefs.begin("twinlink", false);
    uint8_t k[16];
    if (prefs.getBytes("psk", k, sizeof k) == sizeof k) setKey(k);
    pinMode(LORA_VCC, OUTPUT);
    digitalWrite(LORA_VCC, HIGH);  // the module's supply switch
    delay(500);
    spi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    radio.setRfSwitchPins(LORA_RXEN, LORA_TXEN);
    int st = radio.begin(CHANNELS[0], BW_KHZ, SF, CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, POWER_DBM, PREAMBLE, TCXO_V);
    hasRadio = st == RADIOLIB_ERR_NONE;
    if (!hasRadio) {
      Serial.printf("radio: the SX1268 did not start (RadioLib error %d)\n", st);
      return false;
    }
    radio.setStandbyXOSC(true);  // keep the TCXO running: from RC standby each RX / TX waits 5 ms for it to start
    radio.setDio1Action(onIrq);
    slotUs = radio.getTimeOnAir(HDR + 3 + STATE_M + CHUNK_M + TAG) +
             radio.getTimeOnAir(HDR + 3 + STATE_F + CHUNK_F + TAG) + REPLY_US + MARGIN_US;
    Serial.printf("radio: %s, %d channels %.1f-%.1f MHz, slot %lu us (%.0f Hz), key %s\n",
                  master ? "master" : "follower", NCH, CHANNELS[0], CHANNELS[NCH - 1], (unsigned long)slotUs,
                  1e6f / slotUs, haveKey ? fp : "NOT SET: send #key <32 hex digits> on USB, the same on both gateways");
    drop();
    return haveKey;
  }

  bool up() const { return mode == UP; }
  bool write(const char *line) { return up() && TwinCore::write(line); }  // lines while the link is down are dropped

  void poll() {
    if (!hasRadio || !haveKey) return;
    bool ev = irq;
    uint32_t at = irqAt;
    if (ev) irq = false;
    if (pendN && (int32_t)(micros() - pendAt) >= 0) {  // an answer whose wait is over
      send(pend, pendN);
      pendN = 0;
      ev = false;
    }
    if (mode != UP) search(ev, at);
    else if (master) lead(ev, at);
    else follow(ev, at);
    if (millis() - statAt >= 1000) {
      statAt = millis();
      rate = mode == UP ? slot - slotMark : 0;
      okPct = rate ? min<uint32_t>(100, good * 100 / rate) : 0;
      slotMark = slot;
      good = 0;
    }
  }

  // L,<m|f>,<state>,<slots/s>,<% good>,<rssi>,<snr>,<other end's rssi>,<its snr>,<MHz now>
  // state: -2 no radio, -1 no key, 0 searching, 1 up
  void status(char *out, size_t n) const {
    int st = !hasRadio ? -2 : !haveKey ? -1 : up() ? 1 : 0;
    snprintf(out, n, "L,%c,%d,%lu,%lu,%.0f,%.1f,%.0f,%.1f,%.2f", master ? 'm' : 'f', st, (unsigned long)rate,
             (unsigned long)okPct, rssi, snr, peerRssi, peerSnr, twinlink::CHANNELS[chan]);
  }

  // The radio's own USB commands; true if s was one of them:
  //   #key                 the key's fingerprint, to compare between the gateways
  //   #key <32 hex digits> stores a new key
  //   #radio               frames sent, received, received broken (CRC) or not ours since boot; the last frame's
  //                        signal and how far off frequency it came in
  //   #test                the protocol self-test
  bool command(const char *s, Print &log) {
    if (!strcmp(s, "#radio")) {
      log.printf("radio: sent %lu, received %lu, broken %lu, rejected %lu (old slot %lu, failed check %lu), "
                 "last %.0f dBm SNR %.1f dB off by %.0f Hz; %s slot %lu, last rejected: type %u slot %lu, %u bytes\n",
                 (unsigned long)nTx, (unsigned long)nRx, (unsigned long)nBad, (unsigned long)(nRx - nBad - nOk),
                 (unsigned long)nOld, (unsigned long)nAuth, lastRssi, lastSnr, lastHz, up() ? "up" : "searching",
                 (unsigned long)slot, rejType, (unsigned long)rejSlot, rejLen);
      return true;
    }
    if (!strcmp(s, "#test")) return selfTest(log), true;
    if (strncmp(s, "#key", 4) || (s[4] && s[4] != ' ')) return false;
    const char *h = s + 4;
    while (*h == ' ') h++;
    if (*h) {
      uint8_t k[16];
      if (!unhex(h, k)) {
        log.println("key: needs exactly 32 hex digits (0-9, a-f)");
        return true;
      }
      prefs.putBytes("psk", k, sizeof k);
      setKey(k);
      if (hasRadio) drop();
    }
    log.printf("key: %s\n", haveKey ? fp : "not set. Send #key <32 hex digits>, the same on both gateways");
    return true;
  }

 private:
  SPIClass spi{FSPI};
  SX1268 radio;
  Preferences prefs;
  enum { SEARCH, UP } mode = SEARCH;
  enum { IDLE, TX, RX } op = IDLE;
  bool hasRadio = false, confirming = false;
  int chan = 0;
  uint32_t slotUs = 30000, slotAt = 0, lastSlot = 0, lastGood = 0, helloAt = 0;
  uint32_t statAt = 0, slotMark = 0, rate = 0, okPct = 0, good = 0;
  uint32_t nTx = 0, nRx = 0, nBad = 0, nOk = 0;  // since boot, for #radio
  uint8_t pend[128];                              // an answer waiting REPLY_US before it goes out
  size_t pendN = 0;
  uint32_t pendAt = 0;
  float lastRssi = 0, lastSnr = 0, lastHz = 0;    // the last frame received, broken ones too
  uint32_t nOld = 0, nAuth = 0, rejSlot = 0;      // data frames refused: slot not newer, open() failed
  uint8_t rejType = 0, rejLen = 0;
  static inline volatile bool irq = false;
  static inline volatile uint32_t irqAt = 0;
  static void onIrq() {  // the SX1268's DIO1: a frame sent or received. Not IRAM_ATTR: the linker can't place
                         // a header's inline function there, and Arduino's GPIO interrupts don't need it
    irqAt = micros();
    irq = true;
  }

  static bool unhex(const char *h, uint8_t *k) {
    for (int i = 0; i < 32; i++) {
      char c = tolower(h[i]);
      int v = isdigit(c) ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
      if (v < 0) return false;
      k[i / 2] = i & 1 ? k[i / 2] | v : v << 4;
    }
    return !h[32] || isspace(h[32]);
  }

  void answer(const uint8_t *b, size_t n, uint32_t at) {  // sent REPLY_US after at, the frame it answers
    memcpy(pend, b, n);
    pendN = n;
    pendAt = at + twinlink::REPLY_US;
  }
  void tune(int ch) {
    pendN = 0;
    radio.standby();
    radio.setFrequency(twinlink::CHANNELS[ch], true);  // a few hundred kHz: no new image calibration needed
    chan = ch;
    op = IDLE;
    irq = false;
  }
  void send(const uint8_t *b, size_t n) {
    irq = false;
    op = TX;
    nTx++;
    radio.standby();  // RadioLib only does this before receiving: a frame staged mid-receive goes out garbled
    radio.startTransmit(b, n);
  }
  void listen() {
    irq = false;
    op = RX;
    radio.startReceive();
  }
  size_t received(uint8_t *b, size_t cap) {  // the frame that raised DIO1; 0 if none or broken
    size_t n = radio.getPacketLength();
    nRx++;
    lastRssi = radio.getRSSI();
    lastSnr = radio.getSNR();
    lastHz = radio.getFrequencyError();
    if (!n || n > cap || radio.readData(b, n) != RADIOLIB_ERR_NONE) return nBad++, 0;
    rssi = radio.getRSSI();
    snr = radio.getSNR();
    return n;
  }

  void refuse(const uint8_t *b, size_t n, uint32_t &why) {  // a data frame not taken, for #radio
    why++;
    rejType = b[0];
    rejSlot = get32(b + 1);
    rejLen = n;
  }

  void drop() {  // back to the handshake
    mode = SEARCH;
    confirming = false;
    out.clear();
    inflight = 0;
    helloAt = millis() - twinlink::DWELL_MS;  // start searching right away
    radio.standby();
    op = IDLE;
  }

  void goUp(uint32_t at) {  // at: when CONFIRM finished on air; the master's first slot starts FIRST_SLOT_US later
    start();
    mode = UP;
    confirming = false;
    lastSlot = slotMark = 0;
    lastGood = millis();
    slotAt = at + twinlink::FIRST_SLOT_US - (master ? 0 : slotUs);  // master: slot 1's start; follower: slot 0's
  }

  void search(bool ev, uint32_t at) {
    using namespace twinlink;
    uint8_t b[64], r[64];
    if (pendN) return;  // an answer is about to go out on this channel
    if (master && op != TX && millis() - helloAt >= HELLO_MS) {  // a HELLO on the next channel, then listen there
      helloAt = millis();
      tune((chan + 1) % NCH);
      send(b, hello(b));
    } else if (!master && (op == IDLE || (op == RX && millis() - helloAt >= DWELL_MS))) {  // listen a while on each
      helloAt = millis();
      tune((chan + 1) % NCH);
      listen();
    }
    if (!ev) return;
    if (op == TX) {
      radio.finishTransmit();
      op = IDLE;
      if (confirming) {
        goUp(at);
      } else {
        if (!master) helloAt = millis();  // follower: wait a full dwell for the CONFIRM
        listen();
      }
      return;
    }
    size_t n = received(b, sizeof b);
    bool done;
    size_t m = n ? handshake(b, n, r, done) : 0;
    if (m) {
      nOk++;
      answer(r, m, at);
      confirming = master && done;  // the master is up once its CONFIRM is on air
    } else if (n && done) {
      nOk++;
      goUp(at);
    }
  }

  void lead(bool ev, uint32_t at) {  // master: a frame at every slot start, then listen for the reply
    using namespace twinlink;
    uint8_t b[128];
    uint32_t now = micros();
    if ((int32_t)(now - slotAt) >= 0) {
      if (millis() - lastGood > LINK_TIMEOUT_MS) return drop();
      slotAt = (int32_t)(now - slotAt) >= (int32_t)slotUs ? now + slotUs : slotAt + slotUs;  // after a stall: from now
      slot++;
      tune(hop(slot));
      send(b, seal(b));
      return;
    }
    if (!ev) return;
    if (op == TX) {
      radio.finishTransmit();
      listen();
      return;
    }
    size_t n = received(b, sizeof b);
    if (n < HDR) return;
    if (get32(b + 1) != slot) return refuse(b, n, nOld);
    if (!open(b, n)) return refuse(b, n, nAuth);
    lastGood = millis();
    good++;
    nOk++;
  }

  void follow(bool ev, uint32_t at) {  // follower: on each slot's channel just before it starts, answer at once
    using namespace twinlink;
    uint8_t b[128];
    if (op != TX && !pendN && (int32_t)(micros() - (slotAt + slotUs - GUARD_US)) >= 0) {
      if (millis() - lastGood > LINK_TIMEOUT_MS) return drop();
      slot++;
      slotAt += slotUs;
      tune(hop(slot));
      listen();
      return;
    }
    if (!ev) return;
    if (op == TX) {
      radio.finishTransmit();
      op = IDLE;
      return;
    }
    size_t n = received(b, sizeof b);
    if (n < HDR) return;
    uint32_t s = get32(b + 1);
    if ((int32_t)(s - lastSlot) <= 0) return refuse(b, n, nOld);  // old slots and replays are refused
    if (!open(b, n)) return refuse(b, n, nAuth);
    slot = lastSlot = s;
    slotAt = at - radio.getTimeOnAir(n);  // when the master started it: the clock for the next slots
    lastGood = millis();
    good++;
    nOk++;
    answer(b, seal(b), at);
  }
};
