#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "display_pins.h"
#include "config.h"

static const uint16_t RUUVI_MANUFACTURER_ID = 0x0499;

// Ruuvi Data Format 5 (RAWv2), payload after the 2-byte company ID.
struct RuuviV5 {
  float temperatureC;   // NAN if invalid
  float humidityPct;    // NAN if invalid
  float pressureHPa;    // NAN if invalid
  float accX, accY, accZ;  // in G, NAN if invalid
  float batteryV;       // NAN if invalid
  int txPowerDbm;       // INT32_MIN if invalid
  int movementCounter;  // -1 if invalid
  int sequence;         // -1 if invalid
  uint8_t mac[6];
};

static int16_t be16s(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }
static uint16_t be16u(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

// p points at the format byte (0x05); len is payload length (expected 24).
static bool decodeV5(const uint8_t *p, int len, RuuviV5 &r) {
  if (len < 24 || p[0] != 0x05) return false;

  int16_t t = be16s(p + 1);
  uint16_t h = be16u(p + 3);
  uint16_t pr = be16u(p + 5);
  int16_t ax = be16s(p + 7), ay = be16s(p + 9), az = be16s(p + 11);
  uint16_t pw = be16u(p + 13);

  r.temperatureC = (t == (int16_t)0x8000) ? NAN : t * 0.005f;
  r.humidityPct = (h == 0xFFFF) ? NAN : h * 0.0025f;
  r.pressureHPa = (pr == 0xFFFF) ? NAN : (pr + 50000) / 100.0f;
  r.accX = (ax == (int16_t)0x8000) ? NAN : ax / 1000.0f;
  r.accY = (ay == (int16_t)0x8000) ? NAN : ay / 1000.0f;
  r.accZ = (az == (int16_t)0x8000) ? NAN : az / 1000.0f;

  uint16_t batt = pw >> 5;     // 11 bits, mV above 1600
  uint8_t tx = pw & 0x1F;      // 5 bits, 2 dBm steps from -40
  r.batteryV = (batt == 0x7FF) ? NAN : (1600 + batt) / 1000.0f;
  r.txPowerDbm = (tx == 0x1F) ? INT32_MIN : -40 + 2 * tx;

  r.movementCounter = (p[15] == 0xFF) ? -1 : p[15];
  uint16_t seq = be16u(p + 16);
  r.sequence = (seq == 0xFFFF) ? -1 : seq;
  memcpy(r.mac, p + 18, 6);
  return true;
}

static void printVal(const char *label, float v, int digits, const char *unit) {
  Serial.print("  ");
  Serial.print(label);
  Serial.print(": ");
  if (isnan(v)) {
    Serial.println("n/a");
  } else {
    Serial.print(v, digits);
    Serial.println(unit);
  }
}

static Adafruit_ILI9341 tft(TFT_PIN_CS, TFT_PIN_DC, TFT_PIN_RST);
static uint32_t lastDraw = 0;
static bool screenInitialised = false;

// ---- Pressure trend ------------------------------------------------------
// One sample every PRESSURE_SAMPLE_INTERVAL_MS, ring buffer of
// PRESSURE_HISTORY_SIZE samples. Trend = newest - oldest sample.
static float pHist[PRESSURE_HISTORY_SIZE];
static int pCount = 0;
static int pHead = 0;  // index of next write
static uint32_t lastPSample = 0;

static void updatePressureHistory(float p) {
  if (isnan(p)) return;
  if (pCount > 0 && millis() - lastPSample < PRESSURE_SAMPLE_INTERVAL_MS) return;
  lastPSample = millis();
  pHist[pHead] = p;
  pHead = (pHead + 1) % PRESSURE_HISTORY_SIZE;
  if (pCount < PRESSURE_HISTORY_SIZE) pCount++;
}

// Returns +1 rising, -1 falling, 0 steady, 2 not enough history yet.
static int pressureTrend(float current) {
  if (pCount < 2 || isnan(current)) return 2;
  int oldest = (pHead - pCount + PRESSURE_HISTORY_SIZE) % PRESSURE_HISTORY_SIZE;
  float d = current - pHist[oldest];
  if (d >= PRESSURE_TREND_THRESHOLD_HPA) return 1;
  if (d <= -PRESSURE_TREND_THRESHOLD_HPA) return -1;
  return 0;
}

// ---- Drawing -------------------------------------------------------------
// Clear from the current cursor to the right edge for a text row of height h.
static void clearToEol(int h) {
  int x = tft.getCursorX();
  int y = tft.getCursorY();
  if (x < tft.width()) tft.fillRect(x, y, tft.width() - x, h, ILI9341_BLACK);
}

// Small cell, 13 characters wide at size 2 (156 px), padded to overwrite old text.
static void drawCell(int x, int y, const char *text) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%-13s", text);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setCursor(x, y);
  tft.print(buf);
}

static void drawReading(const String &name, const RuuviV5 &r, int rssi) {
  char buf[24];

  if (!screenInitialised) {
    screenInitialised = true;
    tft.fillScreen(ILI9341_BLACK);
    tft.drawFastHLine(0, 165, tft.width(), 0x528A);  // grey separator
  }

  // Tag name
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.setCursor(4, 4);
  tft.print(name);
  clearToEol(16);

  // Temperature, big. Degree sign is CP437 0xF8 (needs tft.cp437(true)).
  tft.setCursor(4, 28);
  uint16_t tempColor = ILI9341_WHITE;
  if (!isnan(r.temperatureC)) {
    if (r.temperatureC < TEMP_COLD_BELOW_C) tempColor = TEMP_COLD_COLOR;
    else if (r.temperatureC > TEMP_HOT_ABOVE_C) tempColor = TEMP_HOT_COLOR;
  }
  tft.setTextColor(tempColor, ILI9341_BLACK);
  if (isnan(r.temperatureC)) {
    tft.setTextSize(8);
    tft.print("n/a");
    clearToEol(64);
  } else {
    tft.setTextSize(8);
    tft.print(r.temperatureC, 1);
    int x = tft.getCursorX();
    tft.fillRect(x, 28, tft.width() - x, 64, ILI9341_BLACK);
    tft.setTextSize(4);
    tft.setCursor(x, 28);
    tft.write((uint8_t)0xF8);
    tft.print("C");
  }

  // Pressure, medium, followed by trend arrow (CP437 0x18 up, 0x19 down, 0x1A right).
  tft.setTextSize(3);
  tft.setTextColor(0x867F, ILI9341_BLACK);  // light blue
  tft.setCursor(4, 116);
  if (isnan(r.pressureHPa)) {
    tft.print("n/a");
  } else {
    tft.print(r.pressureHPa, 1);
    tft.print(" hPa ");
    switch (pressureTrend(r.pressureHPa)) {
      case 1:  tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);  tft.write((uint8_t)0x18); break;
      case -1: tft.setTextColor(ILI9341_RED, ILI9341_BLACK);    tft.write((uint8_t)0x19); break;
      case 0:  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);  tft.write((uint8_t)0x1A); break;
      default: tft.setTextColor(0x528A, ILI9341_BLACK);         tft.print("-"); break;
    }
  }
  clearToEol(24);

  // Bottom rows, small
  if (isnan(r.humidityPct)) snprintf(buf, sizeof(buf), "Hum: n/a");
  else snprintf(buf, sizeof(buf), "Hum: %.1f %%", r.humidityPct);
  drawCell(4, 180, buf);

  if (isnan(r.batteryV)) snprintf(buf, sizeof(buf), "Batt: n/a");
  else snprintf(buf, sizeof(buf), "Batt: %.2f V", r.batteryV);
  drawCell(164, 180, buf);

  snprintf(buf, sizeof(buf), "RSSI: %d dBm", rssi);
  drawCell(4, 212, buf);

  if (r.sequence < 0) snprintf(buf, sizeof(buf), "Msg#: n/a");
  else snprintf(buf, sizeof(buf), "Msg#: %d", r.sequence);
  drawCell(164, 212, buf);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  // Backlight on (harmless if the LED pin is wired straight to 3V3).
  pinMode(TFT_PIN_BL, OUTPUT);
  digitalWrite(TFT_PIN_BL, HIGH);

  // The library default is 32 MHz, too fast for jumper wires and for the
  // nRF52840 SPI peripheral on this core. Use 8 MHz.
  tft.begin(8000000);
  // Diagnostics: expected 0x9341 ID, power mode 0x94 and MADCTL 0x48 style values.
  // All 0x00 or 0xFF means MISO/CS/DC wiring or power problem.
  Serial.print("Display ID 0x04: 0x");
  Serial.println(tft.readcommand8(ILI9341_RDID1, 0), HEX);
  Serial.print("Display status 0x09: 0x");
  Serial.println(tft.readcommand8(ILI9341_RDSELFDIAG, 0), HEX);
  Serial.print("Power mode 0x0A: 0x");
  Serial.println(tft.readcommand8(ILI9341_RDMODE), HEX);
  tft.setRotation(1);  // landscape
  tft.cp437(true);     // correct CP437 glyph indices (degree sign, arrows)
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setCursor(4, 4);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Scanning...");

  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (true) {}
  }
  BLE.scan(true);  // allow duplicates so we see every advertisement
  Serial.println("Scanning for Ruuvi tags (0x0499)...");
}

void loop() {
  BLEDevice dev = BLE.available();
  if (!dev) return;

  // manufacturerData() returns the buffer size even when no data exists,
  // so check the real length first.
  if (!dev.hasManufacturerData()) return;
  int len = dev.manufacturerDataLength();
  if (len < 2) return;

  uint8_t data[64] = {0};
  dev.manufacturerData(data, sizeof(data));
  if (len > (int)sizeof(data)) len = sizeof(data);

  uint16_t id = data[0] | (data[1] << 8);  // company ID, little endian
  if (id != RUUVI_MANUFACTURER_ID) return;

  // Only handle the configured tag.
  String addr = dev.address();
  if (!addr.equalsIgnoreCase(RUUVI_TAG_MAC)) return;

  Serial.print(dev.hasLocalName() ? dev.localName() : String("(no name)"));
  Serial.print("  ");
  Serial.print(dev.address());
  Serial.print("  RSSI ");
  Serial.println(dev.rssi());

  RuuviV5 r;
  if (!decodeV5(data + 2, len - 2, r)) {
    Serial.print("  unsupported data format 0x");
    Serial.println(len > 2 ? data[2] : 0, HEX);
    return;
  }

  updatePressureHistory(r.pressureHPa);

  // Redraw at most once per second to avoid flicker and SPI load.
  if (millis() - lastDraw >= 1000) {
    lastDraw = millis();
    drawReading(dev.hasLocalName() ? dev.localName() : String("Ruuvi"), r, dev.rssi());
  }

  printVal("Temperature", r.temperatureC, 2, " C");
  printVal("Humidity", r.humidityPct, 2, " %");
  printVal("Pressure", r.pressureHPa, 2, " hPa");
  printVal("Acc X", r.accX, 3, " G");
  printVal("Acc Y", r.accY, 3, " G");
  printVal("Acc Z", r.accZ, 3, " G");
  printVal("Battery", r.batteryV, 3, " V");
  Serial.print("  TX power: ");
  if (r.txPowerDbm == INT32_MIN) Serial.println("n/a");
  else { Serial.print(r.txPowerDbm); Serial.println(" dBm"); }
  Serial.print("  Movement counter: ");
  Serial.println(r.movementCounter);
  Serial.print("  Sequence: ");
  Serial.println(r.sequence);
  Serial.print("  MAC: ");
  for (int i = 0; i < 6; i++) {
    if (r.mac[i] < 0x10) Serial.print('0');
    Serial.print(r.mac[i], HEX);
    if (i < 5) Serial.print(':');
  }
  Serial.println();
}
