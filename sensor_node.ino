/**
 * ============================================================
 *  ACOUSTIC DOSIMETER — SENSOR NODE
 * ============================================================
 *
 *  Hardware:   XIAO ESP32-C3 + INMP441 MEMS I2S Microphone
 *  Protocol:   BLE (client) → connects to Display Node (server)
 *  Function:   Captures ambient audio via I2S, computes RMS
 *              energy, applies exponential smoothing, and
 *              transmits a cumulative exposure index (0–400)
 *              to the display node over BLE.
 *
 *  Design decisions:
 *    - Raw audio never leaves this device. Only a single
 *      scalar exposure metric is transmitted, preserving
 *      user privacy by design.
 *    - RMS is smoothed with an IIR filter (α = 0.2) to
 *      reduce transient spikes from impulsive sounds
 *      (doors, coughs) that don't represent sustained exposure.
 *    - The exposure index uses additive accumulation with
 *      multiplicative decay (0.98/update), modeling how
 *      real hearing damage depends on both intensity and
 *      duration — brief loud events contribute less than
 *      sustained moderate noise.
 *    - BLE was chosen over ESP-NOW for bidirectional
 *      communication (RESET commands from display → sensor).
 *
 *  Author:     Coleman Bryant
 *  Course:     TECHIN 512 — Hardware/Software Lab
 *  Program:    MSTI, University of Washington
 * ============================================================
 */

#include <Arduino.h>
#include <driver/i2s.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

// ── BLE Service & Characteristic UUIDs ──────────────────────
// Shared between sensor and display nodes for pairing.
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcdefab-1234-1234-1234-abcdefabcdef"

// ── INMP441 I2S Wiring ─────────────────────────────────────
// The INMP441 outputs a digital I2S audio stream directly —
// no analog-to-digital conversion needed on the MCU side.
#define I2S_WS   D2   // Word Select (L/R clock)
#define I2S_SCK  D3   // Bit Clock
#define I2S_SD   D1   // Serial Data (audio out from mic)

// ── I2S Configuration ──────────────────────────────────────
#define I2S_PORT       I2S_NUM_0
#define SAMPLE_RATE    16000    // 16 kHz — sufficient for occupational noise monitoring
#define BUFFER_SAMPLES 512     // ~32 ms window at 16 kHz

// ── BLE State ──────────────────────────────────────────────
static BLEAddress *serverAddress           = nullptr;
static bool        doConnect               = false;
static bool        connected               = false;
static BLERemoteCharacteristic *pRemoteCharacteristic = nullptr;
static BLEClient  *pClient                 = nullptr;

// ── Audio & Exposure State ─────────────────────────────────
int32_t samples[BUFFER_SAMPLES];
float   smoothRms     = 0.0f;
float   exposureIndex = 0.0f;   // cumulative metric sent to display


// ============================================================
//  BLE SCAN CALLBACK
// ============================================================
// When scanning, we look for a device advertising our service
// UUID. Once found, we stop scanning and flag for connection.

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (advertisedDevice.haveServiceUUID() &&
        advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
      serverAddress = new BLEAddress(advertisedDevice.getAddress());
      advertisedDevice.getScan()->stop();
      doConnect = true;
    }
  }
};


// ============================================================
//  BLE CONNECTION
// ============================================================
// Establishes a client connection to the display node's BLE
// server and obtains a handle to the shared characteristic
// used for bidirectional data exchange.

bool connectToServer() {
  if (serverAddress == nullptr) return false;

  pClient = BLEDevice::createClient();
  if (!pClient->connect(*serverAddress)) {
    return false;
  }

  BLERemoteService *pRemoteService =
      pClient->getService(BLEUUID(SERVICE_UUID));
  if (pRemoteService == nullptr) {
    pClient->disconnect();
    return false;
  }

  pRemoteCharacteristic =
      pRemoteService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
  if (pRemoteCharacteristic == nullptr) {
    pClient->disconnect();
    return false;
  }

  connected = true;
  return true;
}


// ============================================================
//  I2S AUDIO SETUP
// ============================================================
// Configures the ESP32's I2S peripheral to receive 32-bit
// audio frames from the INMP441 at 16 kHz. The mic outputs
// mono on the left channel only.

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode            = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate     = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format  = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = 0,
    .dma_buf_count   = 8,
    .dma_buf_len     = 256,
    .use_apll        = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk      = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_SCK,
    .ws_io_num    = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
}


// ============================================================
//  RMS COMPUTATION
// ============================================================
// Reads a block of I2S samples and computes the root mean
// square (RMS) — a standard measure of signal energy that
// correlates with perceived loudness.
//
// The right-shift by 14 bits scales the raw 32-bit I2S frame
// down to a usable integer range. The INMP441 outputs data
// left-aligned in the 32-bit word, so this extracts the
// meaningful upper bits.

int computeRMS() {
  size_t bytesRead = 0;
  i2s_read(I2S_PORT, samples, sizeof(samples), &bytesRead, portMAX_DELAY);

  int count = bytesRead / 4;
  if (count <= 0) return 0;

  double sumsq = 0.0;
  for (int i = 0; i < count; i++) {
    int32_t s = samples[i] >> 14;
    sumsq += (double)s * (double)s;
  }

  return (int)sqrt(sumsq / count);
}


// ============================================================
//  BLE SCAN LAUNCHER
// ============================================================

void startScan() {
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(0, false);
}


// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(2000);

  setupI2S();
  BLEDevice::init("");
  startScan();
}


// ============================================================
//  MAIN LOOP
// ============================================================
// Every 500 ms:
//   1. Check for RESET command from display (bidirectional BLE)
//   2. Read a block of audio and compute RMS
//   3. Apply IIR smoothing (α = 0.2) to reduce transients
//   4. Update cumulative exposure index:
//      - Additive term proportional to current RMS
//      - Multiplicative decay (×0.98) models recovery during
//        quiet periods
//   5. Transmit the integer exposure index to display node
//
// The exposure model is intentionally simple — it captures
// the core concept that damage = f(intensity × duration)
// without requiring calibrated dB measurements, which would
// need a reference microphone and per-unit calibration.

void loop() {
  static unsigned long lastWrite = 0;

  // Reconnect if BLE dropped
  if (doConnect && !connected) {
    if (!connectToServer()) {
      startScan();
    }
    doConnect = false;
  }

  if (connected && millis() - lastWrite > 500) {
    lastWrite = millis();

    // ── Check for RESET command from display node ──────────
    std::string incoming = pRemoteCharacteristic->readValue();
    if (incoming == "RESET") {
      exposureIndex = 0.0f;
      smoothRms     = 0.0f;
      Serial.println("RESET command received");
    }

    // ── Compute smoothed RMS ───────────────────────────────
    int   rms     = computeRMS();
    smoothRms     = 0.8f * smoothRms + 0.2f * rms;
    int   sendRms = (int)smoothRms;

    // ── Update exposure index ──────────────────────────────
    float contribution = sendRms / 4000.0f;
    exposureIndex += contribution;
    exposureIndex *= 0.98f;                // decay toward zero in quiet
    exposureIndex  = constrain(exposureIndex, 0.0f, 400.0f);

    // ── Transmit to display ────────────────────────────────
    int    sendValue = (int)exposureIndex;
    String out       = String(sendValue);
    pRemoteCharacteristic->writeValue(
        (uint8_t *)out.c_str(), out.length(), false);

    // ── Serial debug output ────────────────────────────────
    Serial.printf("RMS raw: %d | RMS smooth: %d | sent: %d\n",
                  rms, sendRms, sendValue);
  }
}
