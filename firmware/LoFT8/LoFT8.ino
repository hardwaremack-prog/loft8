/*
  LoFT8 firmware — turns an SX1262 Meshtastic-style LoRa board into an
  FT8 transmitter. The desktop app does the FT8 encoding and the clock
  timing; this firmware just plays the 79 tones with microsecond timing.

  Serial protocol (115200 baud, one command per line):
    HELLO                      -> LOFT8 <version> <board>
    PWR <dBm>                  -> OK PWR <dBm>          (-9 .. 22)
    TX <f0_hz> <79 tone digits>-> TXON ... TXDONE <ms>  (tone 0 frequency, Hz)
    TUNE <hz> <seconds>        -> TUNEON ... TUNEDONE   (steady carrier, max 30 s)
    LISTEN <hz>                -> LISTENON, then "R <rssi x10>" 50 times a second
                                  (signal detector: the SX1262 can't decode FT8,
                                   but it can show when a signal is on the channel)
    MCW <hz> <tone_hz> <wpm> <text>
                               -> MCWON ... MCWDONE   beep test: 1 s tone, then the text in
                                  Morse, as FM tone modulation an FM handheld can hear
    STOP                       -> STOPPED
    STATUS                     -> IDLE | TX <symbol> | TUNE | LISTEN
  Errors come back as: ERR <reason>

  Transmitting needs an amateur radio license for the frequency you use.
*/
#include <RadioLib.h>
#include <SPI.h>
#include "board.h"

#ifdef HAS_PMU
  #include <Wire.h>
  #include <XPowersLib.h>
#endif

#define FW_VERSION "1.2"

static const double   STEP_HZ      = 32000000.0 / 33554432.0;  // SX126x PLL step, ~0.9537 Hz
static const uint32_t SYMBOL_US    = 160000;                  // FT8 symbol length
static const int      NUM_SYMBOLS  = 79;
static const double   TONE_SPACING = 6.25;
static const uint32_t MAX_TUNE_MS  = 30000;

SX1262 radio = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY, SPI);

enum Mode { MODE_IDLE, MODE_TX, MODE_TUNE, MODE_LISTEN, MODE_MCW };
static Mode     mode = MODE_IDLE;
static uint8_t  tones[NUM_SYMBOLS];
static uint32_t toneFrf[8];
static int      nextSymbol = 0;
static uint32_t txStartUs = 0;
static uint32_t tuneEndMs = 0;
static int8_t   powerDbm = 10;
static double   calFreqHz = 0;    // frequency last used for image calibration
static bool     radioOk = false;
static String   line;
static uint32_t lastRssiMs = 0;
static float    rssiAcc = 0;
static int      rssiN = 0;
static bool     rxConfigured = false;

// beep test (Morse with an FM audio tone)
static const int MAX_ELEMS = 600;
static uint16_t  elemMs[MAX_ELEMS];
static bool      elemOn[MAX_ELEMS];
static int       elemCount = 0, elemIdx = 0;
static uint32_t  elemStartMs = 0;
static uint32_t  mcwCenter = 0, mcwHi = 0, mcwLo = 0;
static uint32_t  halfPeriodUs = 0, lastFlipUs = 0;
static bool      flipHigh = false;

static const char* morseFor(char c) {
  static const char* L[] = {".-","-...","-.-.","-..",".","..-.","--.","....","..",".---","-.-",".-..","--",
                            "-.","---",".--.","--.-",".-.","...","-","..-","...-",".--","-..-","-.--","--.."};
  static const char* D[] = {"-----",".----","..---","...--","....-",".....","-....","--...","---..","----."};
  if (c >= 'A' && c <= 'Z') return L[c - 'A'];
  if (c >= '0' && c <= '9') return D[c - '0'];
  if (c == '/') return "-..-.";
  if (c == '?') return "..--..";
  return nullptr;
}

static void addElem(bool on, uint32_t ms) {
  if (elemCount < MAX_ELEMS) { elemOn[elemCount] = on; elemMs[elemCount] = ms; elemCount++; }
}

static uint32_t frfFor(double hz) { return (uint32_t)llround(hz / STEP_HZ); }

static void led(bool on) {
#ifdef PIN_LED
  digitalWrite(PIN_LED, on ? HIGH : LOW);
#endif
}

#ifdef HAS_PMU
static void powerUpRadio() {
  Wire.begin(PIN_SDA, PIN_SCL);
  static XPowersAXP2101 axp2101;
  static XPowersAXP192  axp192;
  if (axp2101.begin(Wire, AXP2101_SLAVE_ADDRESS, PIN_SDA, PIN_SCL)) {
    axp2101.setALDO2Voltage(3300);   // LoRa radio rail on T-Beam v1.2
    axp2101.enableALDO2();
  } else if (axp192.begin(Wire, AXP192_SLAVE_ADDRESS, PIN_SDA, PIN_SCL)) {
    axp192.setLDO2Voltage(3300);     // LoRa radio rail on T-Beam v1.1
    axp192.enableLDO2();
  }
  delay(50);
}
#endif

static void stopRadio() {
  radio.standby();
  mode = MODE_IDLE;
  led(false);
}

static bool prepareFrequency(double hz) {
  if (hz < 150e6 || hz > 960e6) { Serial.println("ERR frequency out of range"); return false; }
  // re-run image calibration if we moved more than 1 MHz
  if (fabs(hz - calFreqHz) > 1e6) {
    int st = radio.setFrequency((uint32_t)hz);
    if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERR setFrequency %d\n", st); return false; }
    calFreqHz = hz;
  }
  return true;
}

static void startTx(double f0, const char* digits) {
  if (!radioOk) { Serial.println("ERR radio not found"); return; }
  if (strlen(digits) != NUM_SYMBOLS) { Serial.println("ERR need 79 tones"); return; }
  for (int i = 0; i < NUM_SYMBOLS; i++) {
    if (digits[i] < '0' || digits[i] > '7') { Serial.println("ERR bad tone digit"); return; }
    tones[i] = digits[i] - '0';
  }
  if (!prepareFrequency(f0)) return;
  for (int k = 0; k < 8; k++) toneFrf[k] = frfFor(f0 + k * TONE_SPACING);
  mode = MODE_TX;
  nextSymbol = 0;
  txStartUs = micros();
  radio.transmitDirect(toneFrf[tones[0]]);
  nextSymbol = 1;
  led(true);
  Serial.println("TXON");
}

static void startTune(double hz, float seconds) {
  if (!radioOk) { Serial.println("ERR radio not found"); return; }
  if (!prepareFrequency(hz)) return;
  uint32_t ms = (uint32_t)(seconds * 1000);
  if (ms == 0 || ms > MAX_TUNE_MS) ms = MAX_TUNE_MS;
  radio.transmitDirect(frfFor(hz));
  tuneEndMs = millis() + ms;
  mode = MODE_TUNE;
  led(true);
  Serial.println("TUNEON");
}

static void startListen(double hz) {
  if (!radioOk) { Serial.println("ERR radio not found"); return; }
  if (!prepareFrequency(hz)) return;
  radio.setFrequency((uint32_t)hz);
  if (!rxConfigured) {
    // narrowest receive filter the SX1262 has (~4.8 kHz) for the best sensitivity
    radio.setBitRate(600);
    radio.setFrequencyDeviation(600);
    radio.setRxBandwidth(4800);
    radio.setRxBoostedGainMode(true);
    rxConfigured = true;
  }
  int st = radio.startReceive();
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERR startReceive %d\n", st); return; }
  mode = MODE_LISTEN;
  rssiAcc = 0; rssiN = 0; lastRssiMs = millis();
  Serial.println("LISTENON");
}

static void startMcw(double hz, float toneHz, int wpm, const char* text) {
  if (!radioOk) { Serial.println("ERR radio not found"); return; }
  if (!prepareFrequency(hz)) return;
  if (toneHz < 200 || toneHz > 2000) toneHz = 700;
  wpm = constrain(wpm, 5, 30);
  uint32_t dit = 1200 / wpm;
  elemCount = 0;
  addElem(true, 1000);           // steady tone first: easy to hear
  addElem(false, 500);
  uint32_t total = 1500;
  for (const char* p = text; *p && total < 25000; p++) {
    char c = toupper(*p);
    if (c == ' ') { addElem(false, 4 * dit); total += 4 * dit; continue; }
    const char* m = morseFor(c);
    if (!m) continue;
    for (const char* e = m; *e; e++) {
      uint32_t on = (*e == '-') ? 3 * dit : dit;
      addElem(true, on); addElem(false, dit); total += on + dit;
    }
    addElem(false, 2 * dit); total += 2 * dit;
  }
  // FM: swing the carrier +/-2.5 kHz at the tone rate; an FM receiver hears the tone
  mcwCenter = frfFor(hz);
  mcwHi = frfFor(hz + 2500);
  mcwLo = frfFor(hz - 2500);
  halfPeriodUs = (uint32_t)(500000.0f / toneHz);
  elemIdx = 0;
  elemStartMs = millis();
  lastFlipUs = micros();
  mode = MODE_MCW;
  radio.transmitDirect(mcwHi);
  led(true);
  Serial.println("MCWON");
}

static void handleLine(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char buf[160];
  cmd.toCharArray(buf, sizeof(buf));
  char* verb = strtok(buf, " ");
  for (char* p = verb; *p; p++) *p = toupper(*p);

  if (!strcmp(verb, "HELLO")) {
    Serial.printf("LOFT8 %s %s %s\n", FW_VERSION, BOARD_NAME, radioOk ? "SX1262" : "NO-RADIO");
  } else if (!strcmp(verb, "STATUS")) {
    if (mode == MODE_TX) Serial.printf("TX %d\n", nextSymbol);
    else Serial.println(mode == MODE_TUNE ? "TUNE" : (mode == MODE_LISTEN ? "LISTEN" : "IDLE"));
  } else if (!strcmp(verb, "STOP")) {
    stopRadio();
    Serial.println("STOPPED");
  } else if (!strcmp(verb, "PWR")) {
    char* a = strtok(NULL, " ");
    if (!a) { Serial.println("ERR PWR needs dBm"); return; }
    int p = constrain(atoi(a), -9, 22);
    if (radioOk && radio.setOutputPower(p) != RADIOLIB_ERR_NONE) { Serial.println("ERR power"); return; }
    powerDbm = p;
    Serial.printf("OK PWR %d\n", p);
  } else if (!strcmp(verb, "TX")) {
    char* f = strtok(NULL, " ");
    char* t = strtok(NULL, " ");
    if (!f || !t) { Serial.println("ERR TX needs freq and tones"); return; }
    if (mode != MODE_IDLE) stopRadio();
    startTx(atof(f), t);
  } else if (!strcmp(verb, "MCW")) {
    char* f = strtok(NULL, " ");
    char* t = strtok(NULL, " ");
    char* w = strtok(NULL, " ");
    char* text = strtok(NULL, "");
    if (!f || !t || !w) { Serial.println("ERR MCW needs freq tone wpm text"); return; }
    if (mode != MODE_IDLE) stopRadio();
    startMcw(atof(f), atof(t), atoi(w), text ? text : "");
  } else if (!strcmp(verb, "LISTEN")) {
    char* f = strtok(NULL, " ");
    if (!f) { Serial.println("ERR LISTEN needs freq"); return; }
    if (mode != MODE_IDLE) stopRadio();
    startListen(atof(f));
  } else if (!strcmp(verb, "TUNE")) {
    char* f = strtok(NULL, " ");
    char* s = strtok(NULL, " ");
    if (!f) { Serial.println("ERR TUNE needs freq"); return; }
    if (mode != MODE_IDLE) stopRadio();
    startTune(atof(f), s ? atof(s) : 10);
  } else {
    Serial.println("ERR unknown command");
  }
}

void setup() {
  Serial.begin(115200);
#ifdef PIN_LED
  pinMode(PIN_LED, OUTPUT);
  led(false);
#endif
#ifdef HAS_PMU
  powerUpRadio();
#endif
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  radio.tcxoVoltage = RADIO_TCXO;
  ConfigFSK_t cfg;
  cfg.frequency = 432000000UL;
  cfg.power = powerDbm;
  int st = radio.beginFSK(cfg);
  if (st == RADIOLIB_ERR_NONE) {
    radio.setDio2AsRfSwitch(true);
    radioOk = true;
    calFreqHz = 432e6;
  }
  delay(200);
  Serial.printf("LOFT8 %s %s %s\n", FW_VERSION, BOARD_NAME, radioOk ? "SX1262" : "NO-RADIO");
  if (!radioOk) Serial.printf("ERR radio init %d\n", st);
}

void loop() {
  // keep reading serial even while transmitting, so STOP works instantly
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { handleLine(line); line = ""; }
    else if (line.length() < 150) line += c;
  }

  if (mode == MODE_TX) {
    uint32_t elapsed = micros() - txStartUs;
    if (elapsed >= (uint32_t)nextSymbol * SYMBOL_US) {
      if (nextSymbol >= NUM_SYMBOLS) {
        stopRadio();
        Serial.printf("TXDONE %lu\n", (unsigned long)(elapsed / 1000));
      } else {
        radio.transmitDirect(toneFrf[tones[nextSymbol]]);
        nextSymbol++;
        led((nextSymbol / 3) % 2);   // blink while sending
      }
    }
  } else if (mode == MODE_LISTEN) {
    rssiAcc += radio.getRSSI(false);
    rssiN++;
    if (millis() - lastRssiMs >= 20) {
      lastRssiMs = millis();
      Serial.printf("R %d\n", (int)lroundf(rssiAcc / rssiN * 10));
      rssiAcc = 0; rssiN = 0;
    }
    delayMicroseconds(500);
  } else if (mode == MODE_MCW) {
    if (millis() - elemStartMs >= elemMs[elemIdx]) {
      elemIdx++;
      elemStartMs = millis();
      if (elemIdx >= elemCount) {
        stopRadio();
        Serial.println("MCWDONE");
        return;
      }
      if (!elemOn[elemIdx]) radio.transmitDirect(mcwCenter);   // key up: plain carrier = silence
      led(elemOn[elemIdx]);
    }
    if (elemOn[elemIdx] && micros() - lastFlipUs >= halfPeriodUs) {
      lastFlipUs += halfPeriodUs;
      if (micros() - lastFlipUs > 2 * halfPeriodUs) lastFlipUs = micros();   // never fall behind
      flipHigh = !flipHigh;
      radio.transmitDirect(flipHigh ? mcwHi : mcwLo);
    }
  } else if (mode == MODE_TUNE) {
    if ((int32_t)(millis() - tuneEndMs) >= 0) {
      stopRadio();
      Serial.println("TUNEDONE");
    }
  }
}
