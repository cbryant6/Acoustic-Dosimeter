/**
 * ============================================================
 *  ACOUSTIC DOSIMETER — DISPLAY NODE
 * ============================================================
 *
 *  Hardware:   XIAO ESP32-C3 + SSD1306 128×64 OLED (I2C)
 *              + WS2812B NeoPixel + Tactile Button
 *  Protocol:   BLE (server) — receives exposure data from
 *              the Sensor Node (client)
 *  Function:   Visualizes cumulative sound exposure on an
 *              OLED with a progress bar and numeric readout.
 *              NeoPixel provides at-a-glance status. Physical
 *              button sends a RESET command back to the sensor.
 *
 *  Design decisions:
 *    - The display node acts as BLE server (peripheral) so
 *      the sensor can connect on its own schedule without
 *      requiring user pairing — just power both on.
 *    - The RESET command is bidirectional: button press on
 *      the display writes "RESET" to the shared BLE
 *      characteristic, which the sensor reads on its next
 *      update cycle. This avoids adding a second
 *      characteristic or a notification channel.
 *    - LED color encoding (green/orange/red) follows common
 *      traffic-light conventions for intuitive interpretation
 *      without reading the screen.
 *    - The warning flash at exposure ≥ 100 uses a simple
 *      toggle timer rather than hardware PWM to keep the
 *      NeoPixel driver straightforward on this platform.
 *    - Button debouncing uses a 40 ms window — tested to
 *      reliably reject bounce from the 6×6 mm tact switches
 *      used in this build.
 *
 *  Author:     Coleman Bryant
 *  Course:     TECHIN 512 — Hardware/Software Lab
 *  Program:    MSTI, University of Washington
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

// ── OLED Configuration ─────────────────────────────────────
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C

// ── NeoPixel & Button ──────────────────────────────────────
#define NEOPIXEL_PIN    D8
#define NUMPIXELS       1
#define BUTTON_PIN      D10

// ── BLE UUIDs (must match sensor node) ─────────────────────
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcdefab-1234-1234-1234-abcdefabcdef"

// ── Peripheral Instances ───────────────────────────────────
Adafruit_SSD1306  display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel pixel(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ── BLE State ──────────────────────────────────────────────
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
volatile int receivedValue = 0;

// ── Button Debounce State ──────────────────────────────────
bool          lastButtonReading  = HIGH;
bool          buttonState        = HIGH;
unsigned long lastDebounceTime   = 0;
const unsigned long debounceDelay = 40;

// ── Warning Flash State ────────────────────────────────────
bool          warnFlashState     = false;
unsigned long lastWarnFlashTime  = 0;
const unsigned long warnFlashInterval = 300;  // ms


// ============================================================
//  OLED RENDERING
// ============================================================
// Draws the full display layout:
//   - Header: "Sound Exposure" + connection status
//   - Body:   Numeric value + severity label (or WARN state)
//   - Footer: Progress bar mapped to 0–150 range
//
// The layout adapts between normal and warning modes to
// provide clear visual escalation without requiring the user
// to interpret numbers.

void drawScreen(int value) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  bool warnState = (value >= 100);

  // ── Header ───────────────────────────────────────────────
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Sound Exposure");

  display.setCursor(90, 0);
  display.println(deviceConnected ? "LINK" : "WAIT");

  // ── Body ─────────────────────────────────────────────────
  if (warnState) {
    display.setTextSize(2);
    display.setCursor(0, 16);
    display.println("WARN");
    display.setCursor(72, 16);
    display.println(value);
  } else {
    display.setTextSize(2);
    display.setCursor(0, 18);
    display.print("Val:");
    display.println(value);

    display.setTextSize(1);
    display.setCursor(90, 22);
    display.println(value < 50 ? "LOW" : "MED");
  }

  // ── Progress Bar ─────────────────────────────────────────
  int barX  = 0, barY = 50, barW = 128, barH = 10;
  int fillW = constrain(map(value, 0, 150, 0, barW), 0, barW);

  display.drawRect(barX, barY, barW, barH, SSD1306_WHITE);
  display.fillRect(barX, barY, fillW, barH, SSD1306_WHITE);

  display.display();
}


// ============================================================
//  BLE SERVER CALLBACKS
// ============================================================
// Handles connect/disconnect events. On disconnect, we
// restart advertising so the sensor can reconnect
// automatically after a power cycle or range loss.

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    deviceConnected = true;
    Serial.println("BLE client connected");
    drawScreen(receivedValue);
  }

  void onDisconnect(BLEServer *pServer) override {
    deviceConnected = false;
    Serial.println("BLE client disconnected");
    BLEDevice::startAdvertising();
    Serial.println("Advertising restarted");
    drawScreen(receivedValue);
  }
};


// ============================================================
//  BLE CHARACTERISTIC CALLBACK
// ============================================================
// Called when the sensor node writes a new exposure value.
// The value arrives as an ASCII integer string and is
// parsed to update the display immediately.

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    std::string value = pChar->getValue();
    if (!value.empty()) {
      receivedValue = atoi(value.c_str());
      Serial.printf("Received value: %d\n", receivedValue);
      drawScreen(receivedValue);
    }
  }
};


// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(2000);

  // ── I2C for OLED ─────────────────────────────────────────
  Wire.begin(D4, D5);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // ── OLED Init ────────────────────────────────────────────
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED init failed");
    while (true) delay(10);
  }

  // ── NeoPixel Init ────────────────────────────────────────
  pixel.begin();
  pixel.clear();
  pixel.show();

  drawScreen(receivedValue);

  // ── BLE Server Setup ─────────────────────────────────────
  BLEDevice::init("DosimeterDisplay");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pCharacteristic->setValue("0");
  pService->start();

  // ── Start Advertising ────────────────────────────────────
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("BLE display server ready");
}


// ============================================================
//  MAIN LOOP
// ============================================================
// Continuously handles two responsibilities:
//
//   1. BUTTON INPUT — Debounced read of the physical reset
//      button. On press, writes "RESET" to the BLE
//      characteristic so the sensor node zeroes its exposure
//      index on the next read cycle. Also resets the local
//      display immediately for responsive feedback.
//
//   2. LED STATUS — Color-codes device state at a glance:
//        Red    = BLE disconnected (no sensor link)
//        Green  = connected, exposure below threshold
//        Orange = connected, exposure ≥ 100 (flashing)

void loop() {

  // ── Button Debounce ──────────────────────────────────────
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        Serial.println("Button pressed -> sending RESET");
        pCharacteristic->setValue("RESET");
        receivedValue  = 0;
        warnFlashState = false;
        drawScreen(receivedValue);
      }
    }
  }
  lastButtonReading = reading;

  // ── LED Status Indicator ─────────────────────────────────
  bool warnState = deviceConnected && (receivedValue >= 100);

  if (warnState) {
    // Flashing orange at 300 ms intervals
    if (millis() - lastWarnFlashTime >= warnFlashInterval) {
      lastWarnFlashTime = millis();
      warnFlashState    = !warnFlashState;
      drawScreen(receivedValue);
    }
    pixel.setPixelColor(0, warnFlashState
        ? pixel.Color(50, 20, 0)   // orange
        : pixel.Color(0, 0, 0));   // off
  } else {
    warnFlashState = false;
    pixel.setPixelColor(0, !deviceConnected
        ? pixel.Color(50, 0, 0)    // red — no link
        : pixel.Color(0, 50, 0));  // green — nominal
  }

  pixel.show();
  delay(20);
}
