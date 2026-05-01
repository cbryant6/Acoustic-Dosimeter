# Acoustic Dosimeter

A two-device, wireless **acoustic dosimeter** that estimates cumulative sound exposure over time and presents it as a slow-moving analog gauge needle. A compact sensing node performs DSP (RMS + FFT band-energy features + time-weighted integration) and transmits a single exposure metric to a tabletop display with a stepper-driven needle, LED status, and a button.

<img width="1472" height="840" alt="image" src="https://github.com/user-attachments/assets/a86f71ff-9956-4151-b6a1-7352dbb66312" />


---

## 1) Overview Slide 

**What it does:** The sensing module continuously samples ambient audio using a MEMS microphone, extracts energy and frequency features (RMS + FFT band energy), and integrates them over time into a single **exposure index**. The display module receives that index wirelessly and visualizes it as a slow-moving gauge needle, with an LED and a single button for reset/calibration.

**Physical features (sketch):** see general sketch above.

---

## 2) Sensor Device (Detailed Sketch + How it Works)

<img width="1472" height="960" alt="image" src="https://github.com/user-attachments/assets/6d364bc0-3ce9-45cc-906d-f06a0046511f" />

<img width="1204" height="996" alt="image" src="https://github.com/user-attachments/assets/acc42f54-17c2-4bea-996f-8193d15734f2" />

<img width="952" height="1269" alt="image" src="https://github.com/user-attachments/assets/83bdb458-92e8-441b-a486-2f5bb4bc17c9" />





**Role:** Captures ambient audio, runs DSP locally, and transmits a single cumulative exposure metric.

**How it works (high level):**
- The microphone provides a digital I2S audio stream.
- The MCU computes short-window RMS and FFT band energies (e.g., 125 Hz-4 kHz bands), then applies time-weighted integration (accumulate during sustained loudness, decay during quiet).
- The sensor node transmits an exposure value (0-100) periodically (e.g., 1-5 Hz) to the display node.

**Key parts (with part numbers):**
- **MCU / Wireless:** ESP32-C3 (RISC-V Wi-Fi + BLE) (datasheet in `datasheets/ESP32-C3_datasheet_en.pdf`) 
- **Microphone:** INMP441 MEMS I2S microphone (datasheet in `datasheets/INMP441_MEMS_microphone_datasheet.pdf`)
- **Power (battery-only):** 1-cell LiPo + MCP73831 Li-ion/LiPo charger IC (datasheet in `datasheets/MCP73831_LiPo_charger_datasheet_Microchip.pdf`) + AP2112 3.3 V LDO regulator (datasheet in `datasheets/AP2112_LDO_datasheet.pdf`)

**Early technical thoughts (wireless, power, processing):** The sensor module concentrates processing (RMS + FFT + integration) on an ESP32-C3 and uses ESP-NOW (or BLE) to transmit a single exposure metric, minimizing bandwidth and improving privacy by not sending raw audio. Power will be managed with duty-cycling (batching FFT windows + deep sleep between updates) to support battery-only operation.

---

## 3) Display Device (Detailed Sketch + How it Works)


<img width="789" height="455" alt="image" src="https://github.com/user-attachments/assets/484eeb7d-372c-400b-b446-1e4922426818" />



**Role:** Receives the exposure index and visualizes it as an analog gauge needle; provides minimal user interaction.

**How it works (high level):**
- The display MCU receives exposure updates wirelessly.
- A stepper motor drives the needle to a mapped angle (e.g., 0-100 -> 0-180 degrees) with smoothing to avoid jitter.
- A single LED provides status (OK / caution / high exposure) and a button triggers reset or calibration.
- (Optional) OLED can show the numeric exposure value and battery state.

**Key parts (with part numbers):**
- **MCU / Wireless:** ESP32-C3 (same family as sensor node)
- **Stepper motor (needle):** 28BYJ-48 5 V stepper motor (datasheet in `datasheets/28BYJ-48_stepper_motor_datasheet.pdf`)
- **Stepper driver:** ULN2003A Darlington array (datasheet in `datasheets/ULN2003A_datasheet_TI.pdf`)
- **LED:** WS2812B addressable RGB LED (datasheet in `datasheets/WS2812B_RGB_LED_datasheet.pdf`) (can be used as a single LED)
- **Optional display:** SSD1306-based 128x64 OLED over I2C (controller datasheet in `datasheets/SSD1306_OLED_controller_datasheet.pdf`)
- **Button:** 6x6 mm tact switch (generic)

**Early technical thoughts (wireless, power, processing):** The display device keeps processing minimal (receive metric + gauge control + UI), prioritizing stable stepper motion and low power draw. Because the stepper dominates energy use, the display will use a thoughtfully sized larger LiPo and will only move the needle when the metric changes meaningfully.

---

## 4) Communication + Detailed HW/SW Diagram (2 Figures)

### Figure A: Device-to-Device Communication

<img width="1472" height="440" alt="image" src="https://github.com/user-attachments/assets/7ff9329c-b44c-4a13-a172-a496187cffa7" />


### Figure B: Detailed System Architecture (HW + SW)

<img width="1472" height="960" alt="image" src="https://github.com/user-attachments/assets/c388b913-5952-45cb-b66d-2c3eb67a037c" />


**How the two devices communicate:** The sensing node transmits an exposure index (0-100) over ESP-NOW (or BLE) at a low update rate; the display node acknowledges receipt (optional) and uses smoothing before commanding the stepper position. This keeps the link reliable and low-power while preserving privacy by keeping raw audio on the sensing device.

***Intro Code***
```

#include <WiFi.h>
#include <esp_now.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <math.h>

// =========================
// USER SETTINGS
// =========================

// Replace with the MAC address of your DISPLAY module ESP32-S3
uint8_t displayMac[] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};

// INMP441 wiring on XIAO ESP32-S3
static const int I2S_WS   = 44; // D7
static const int I2S_SCK  = 7;  // D8
static const int I2S_SD   = 8;  // D9

// Audio settings
static const uint32_t SAMPLE_RATE = 16000;   // 16 kHz
static const uint16_t FFT_SIZE    = 512;     // must be power of 2
static const uint16_t BLOCK_SAMPLES = FFT_SIZE;

// Exposure behavior
static const float INDEX_DECAY_PER_SEC = 0.995f;   // slow decay, near 1.0 = very slow
static const float INDEX_GAIN          = 8.0f;     // raises/lower accumulation speed
static const float SEND_INTERVAL_MS    = 200.0f;   // send updates every 200 ms

// FFT band edges in Hz
static const float BAND1_LOW  = 100.0f;
static const float BAND1_HIGH = 1000.0f;
static const float BAND2_LOW  = 1000.0f;
static const float BAND2_HIGH = 4000.0f;

// =========================
// DATA STRUCTURES
// =========================

enum PacketType : uint8_t {
  PACKET_SENSOR_DATA = 1,
  PACKET_COMMAND     = 2
};

enum CommandType : uint8_t {
  CMD_NONE      = 0,
  CMD_RESET     = 1,
  CMD_CALIBRATE = 2
};

struct SensorPacket {
  uint8_t packetType;
  float exposureIndex;
  float rmsValue;
  float lowBandEnergy;
  float highBandEnergy;
};

struct CommandPacket {
  uint8_t packetType;
  uint8_t command;
};

SensorPacket txPacket;
CommandPacket rxCommand;

// =========================
// FFT BUFFERS
// =========================

double vReal[FFT_SIZE];
double vImag[FFT_SIZE];
arduinoFFT FFT = arduinoFFT(vReal, vImag, FFT_SIZE, SAMPLE_RATE);

// =========================
// STATE
// =========================

float exposureIndex = 0.0f;
float noiseFloorRms = 2000.0f;   // rough starting value; recalibrated in setup
unsigned long lastSendMs = 0;

// =========================
// I2S CONFIG
// =========================

static const i2s_port_t I2S_PORT = I2S_NUM_0;

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = (int)SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // INMP441 single channel
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 128,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
}

// =========================
// ESP-NOW CALLBACKS
// =========================

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // Optional debug:
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send OK" : "Send Fail");
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(CommandPacket)) return;

  memcpy(&rxCommand, data, sizeof(rxCommand));
  if (rxCommand.packetType != PACKET_COMMAND) return;

  if (rxCommand.command == CMD_RESET) {
    exposureIndex = 0.0f;
    Serial.println("Received RESET command.");
  } else if (rxCommand.command == CMD_CALIBRATE) {
    Serial.println("Received CALIBRATE command.");
    // Re-estimate noise floor from current ambient sound
    calibrateNoiseFloor();
  }
}

bool setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed.");
    return false;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, displayMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(displayMac)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add display peer.");
      return false;
    }
  }

  return true;
}

// =========================
// AUDIO PROCESSING
// =========================

bool readAudioBlock(int32_t *buffer, size_t samplesToRead) {
  size_t bytesRead = 0;
  esp_err_t err = i2s_read(I2S_PORT, buffer, samplesToRead * sizeof(int32_t), &bytesRead, portMAX_DELAY);
  if (err != ESP_OK) return false;
  return (bytesRead == samplesToRead * sizeof(int32_t));
}

float computeRMS(const int32_t *samples, size_t n) {
  double sumSq = 0.0;

  for (size_t i = 0; i < n; i++) {
    // INMP441 audio data typically arrives left-aligned in 32-bit frames.
    // Shifting right reduces it to a more manageable signed range.
    int32_t s = samples[i] >> 8;
    sumSq += (double)s * (double)s;
  }

  double meanSq = sumSq / (double)n;
  return sqrt(meanSq);
}

void fillFFTBuffers(const int32_t *samples, size_t n) {
  for (size_t i = 0; i < n; i++) {
    int32_t s = samples[i] >> 8;
    vReal[i] = (double)s;
    vImag[i] = 0.0;
  }
}

float computeBandEnergy(float lowHz, float highHz) {
  double sum = 0.0;

  for (uint16_t i = 1; i < FFT_SIZE / 2; i++) {
    double freq = ((double)i * SAMPLE_RATE) / FFT_SIZE;
    if (freq >= lowHz && freq < highHz) {
      double mag = sqrt(vReal[i] * vReal[i] + vImag[i] * vImag[i]);
      sum += mag * mag;
    }
  }

  return (float)sum;
}

void calibrateNoiseFloor() {
  const int CAL_BLOCKS = 8;
  int32_t sampleBuf[BLOCK_SAMPLES];
  float rmsSum = 0.0f;
  int good = 0;

  Serial.println("Calibrating noise floor... keep room quiet if possible.");

  for (int i = 0; i < CAL_BLOCKS; i++) {
    if (readAudioBlock(sampleBuf, BLOCK_SAMPLES)) {
      rmsSum += computeRMS(sampleBuf, BLOCK_SAMPLES);
      good++;
    }
  }

  if (good > 0) {
    noiseFloorRms = rmsSum / good;
    Serial.print("Noise floor RMS = ");
    Serial.println(noiseFloorRms, 2);
  }
}

// =========================
// EXPOSURE MODEL
// =========================

float updateExposureIndex(float rmsValue, float lowBand, float highBand, float dtSec) {
  // Remove baseline noise floor contribution
  float rmsAboveFloor = max(0.0f, rmsValue - noiseFloorRms);

  // Normalize terms. These constants are tunable.
  float rmsNorm  = rmsAboveFloor / 12000.0f;
  float lowNorm  = lowBand / 1.0e11f;
  float highNorm = highBand / 1.0e11f;

  // Weighted feature score
  float instantScore =
      0.60f * rmsNorm +
      0.20f * lowNorm +
      0.20f * highNorm;

  instantScore = max(0.0f, instantScore);

  // Slow temporal integration
  float decay = powf(INDEX_DECAY_PER_SEC, dtSec);
  exposureIndex = exposureIndex * decay + (instantScore * INDEX_GAIN * dtSec);

  // Clamp to a clean range for display
  if (exposureIndex < 0.0f) exposureIndex = 0.0f;
  if (exposureIndex > 100.0f) exposureIndex = 100.0f;

  return exposureIndex;
}

// =========================
// SETUP / LOOP
// =========================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Sensor module starting...");

  setupI2S();

  if (!setupEspNow()) {
    Serial.println("ESP-NOW setup failed. Halting.");
    while (true) delay(1000);
  }

  calibrateNoiseFloor();
  lastSendMs = millis();
}

void loop() {
  static int32_t sampleBuf[BLOCK_SAMPLES];

  if (!readAudioBlock(sampleBuf, BLOCK_SAMPLES)) {
    Serial.println("I2S read failed.");
    delay(10);
    return;
  }

  float rmsValue = computeRMS(sampleBuf, BLOCK_SAMPLES);

  fillFFTBuffers(sampleBuf, BLOCK_SAMPLES);
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();

  float lowBandEnergy  = computeBandEnergy(BAND1_LOW, BAND1_HIGH);
  float highBandEnergy = computeBandEnergy(BAND2_LOW, BAND2_HIGH);

  unsigned long nowMs = millis();
  float dtSec = (nowMs - lastSendMs) / 1000.0f;
  if (dtSec <= 0.0f) dtSec = SEND_INTERVAL_MS / 1000.0f;

  float index = updateExposureIndex(rmsValue, lowBandEnergy, highBandEnergy, dtSec);

  if ((nowMs - lastSendMs) >= SEND_INTERVAL_MS) {
    txPacket.packetType   = PACKET_SENSOR_DATA;
    txPacket.exposureIndex = index;
    txPacket.rmsValue      = rmsValue;
    txPacket.lowBandEnergy = lowBandEnergy;
    txPacket.highBandEnergy = highBandEnergy;

    esp_err_t result = esp_now_send(displayMac, (uint8_t *)&txPacket, sizeof(txPacket));

    Serial.print("IDX=");
    Serial.print(index, 2);
    Serial.print(" RMS=");
    Serial.print(rmsValue, 2);
    Serial.print(" LOW=");
    Serial.print(lowBandEnergy, 2);
    Serial.print(" HIGH=");
    Serial.print(highBandEnergy, 2);
    Serial.print(" SEND=");
    Serial.println(result == ESP_OK ? "OK" : "ERR");

    lastSendMs = nowMs;
  }
}
```
## Datasheets

All component datasheets are stored in the `datasheets/` folder.
