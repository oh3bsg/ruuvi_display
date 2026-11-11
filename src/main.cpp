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

static void drawLine(int row, const char *label, float v, int digits, const char *unit) {
  tft.setCursor(4, 4 + row * 30);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.print(label);
  if (isnan(v)) {
    tft.print("n/a      ");
  } else {
    tft.print(v, digits);
    tft.print(unit);
    tft.print("   ");  // overwrite leftovers from longer previous values
  }
}

static void drawReading(const String &name, const RuuviV5 &r) {
  tft.setTextSize(2);
  tft.setCursor(4, 4);
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.print(name);
  tft.print("        ");
  drawLine(1, "Temp: ", r.temperatureC, 2, " C");
  drawLine(2, "Hum:  ", r.humidityPct, 1, " %");
  drawLine(3, "Pres: ", r.pressureHPa, 1, " hPa");
  drawLine(4, "Batt: ", r.batteryV, 2, " V");
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

  // Redraw at most once per second to avoid flicker and SPI load.
  if (millis() - lastDraw >= 1000) {
    lastDraw = millis();
    drawReading(dev.hasLocalName() ? dev.localName() : String("Ruuvi"), r);
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
