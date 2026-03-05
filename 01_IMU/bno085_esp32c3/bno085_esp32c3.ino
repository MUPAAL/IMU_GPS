/**
 * BNO085 IMU Data Acquisition via SPI on ESP32-C3
 *
 * Reads multiple sensor reports from BNO085 and outputs JSON over UART at 50Hz.
 * The Adafruit_BNO08x library handles interrupt-driven SPI reads internally.
 *
 * Hardware connections:
 *   MOSI -> GPIO1
 *   MISO -> GPIO6
 *   SCK  -> GPIO7
 *   CS   -> GPIO0
 *   INT  -> GPIO5
 *   RST  -> GPIO2
 *   BOOT -> GPIO9 (built-in, long-press 3s to save calibration)
 *
 * Dependencies:
 *   Adafruit BNO08x Library (install via Arduino Library Manager)
 */

#include <SPI.h>
#include <Adafruit_BNO08x.h>  // also pulls in sh2.h, sh2_SensorValue.h

// ---------- Pin definitions ----------
#define SPI_MOSI  1
#define SPI_MISO  6
#define SPI_SCK   7
#define BNO_CS    0
#define BNO_INT   5
#define BNO_RST   2
#define BOOT_BTN  9   // GPIO9 = BOOT button on ESP32-C3

// ---------- Timing ----------
#define REPORT_INTERVAL_US  10000   // 100 Hz sensor reports
#define OUTPUT_INTERVAL_MS  20      // 50 Hz JSON output

// ---------- Globals ----------
SPIClass mySPI(FSPI);

// Pass BNO_RST to constructor so the library can hardware-reset the chip
Adafruit_BNO08x bno08x(BNO_RST);

// Reusable sensor event container
sh2_SensorValue_t sensorValue;

// Sensor data cache (updated as reports arrive)
struct {
  float rot_qi, rot_qj, rot_qk, rot_qr, rot_acc;
  float game_qi, game_qj, game_qk, game_qr;
  float accel_x, accel_y, accel_z;
  float lin_x, lin_y, lin_z;
  float grav_x, grav_y, grav_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
  uint32_t steps;
  uint8_t cal_status;  // bits [1:0] of sensor report status (0-3)
} sensorData;

uint32_t lastOutputMs   = 0;
uint32_t bootPressStart = 0;
bool     bootPressed    = false;

// ---------- Enable all sensor reports ----------
void enableReports() {
  // Rotation Vector: absolute orientation, fuses accel + gyro + mag
  bno08x.enableReport(SH2_ROTATION_VECTOR,          REPORT_INTERVAL_US);
  // Game Rotation Vector: no magnetometer, stable for indoor use
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR,     REPORT_INTERVAL_US);
  // Raw accelerometer (m/s², includes gravity)
  bno08x.enableReport(SH2_ACCELEROMETER,            REPORT_INTERVAL_US);
  // Linear acceleration (gravity removed)
  bno08x.enableReport(SH2_LINEAR_ACCELERATION,      REPORT_INTERVAL_US);
  // Gravity vector
  bno08x.enableReport(SH2_GRAVITY,                  REPORT_INTERVAL_US);
  // Calibrated gyroscope (rad/s)
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED,     REPORT_INTERVAL_US);
  // Calibrated magnetometer (µT)
  bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, REPORT_INTERVAL_US);
  // Step counter
  bno08x.enableReport(SH2_STEP_COUNTER,             REPORT_INTERVAL_US);
}

// ---------- Setup ----------
void setup() {
  // Increase TX buffer to 1024 bytes to fit full JSON frames (~320 bytes each)
  Serial.setTxBufferSize(1024);
  Serial.begin(921600);
  delay(200);

  Serial.println("# BNO085 ESP32-C3 (Adafruit) starting...");

  pinMode(BOOT_BTN, INPUT_PULLUP);

  // Initialize SPI bus with custom pins.
  // Do NOT pass CS here - let Adafruit_SPIDevice manage CS exclusively
  // to avoid pin state conflicts during begin_SPI().
  mySPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  // Manually drive CS high before handing control to the library
  pinMode(BNO_CS, OUTPUT);
  digitalWrite(BNO_CS, HIGH);
  delay(10);

  Serial.println("# SPI bus initialized. Trying BNO085...");

  // begin_SPI(cs, int_pin, spi_ptr, freq)
  if (!bno08x.begin_SPI(BNO_CS, BNO_INT, &mySPI, 1000000)) {
    Serial.println("# ERROR: BNO085 not detected.");
    Serial.println("# Check: SPI wiring, PS0/PS1 pulled HIGH for SPI mode, power.");
    while (1) { delay(1000); }
  }

  Serial.println("# BNO085 detected. Enabling reports...");
  enableReports();
  Serial.println("# Ready. Outputting JSON at 50Hz.");
}

// ---------- Poll and dispatch one sensor event per loop() call ----------
// Adafruit_BNO08x is designed for one getSensorEvent() call per loop(),
// not a while-loop drain. The loop() runs fast enough to keep up at 100Hz.
void readSensorData() {
  // Re-enable reports after a chip reset
  if (bno08x.wasReset()) {
    Serial.println("# INFO: BNO085 reset, re-enabling reports.");
    enableReports();
  }

  // Process at most one event per loop() call
  if (!bno08x.getSensorEvent(&sensorValue)) return;

  switch (sensorValue.sensorId) {

      case SH2_ROTATION_VECTOR:
        sensorData.rot_qi     = sensorValue.un.rotationVector.i;
        sensorData.rot_qj     = sensorValue.un.rotationVector.j;
        sensorData.rot_qk     = sensorValue.un.rotationVector.k;
        sensorData.rot_qr     = sensorValue.un.rotationVector.real;
        sensorData.rot_acc    = sensorValue.un.rotationVector.accuracy;
        // status bits [1:0]: 0=unreliable, 1=low, 2=medium, 3=high
        sensorData.cal_status = sensorValue.status & 0x03;
        break;

      case SH2_GAME_ROTATION_VECTOR:
        sensorData.game_qi = sensorValue.un.gameRotationVector.i;
        sensorData.game_qj = sensorValue.un.gameRotationVector.j;
        sensorData.game_qk = sensorValue.un.gameRotationVector.k;
        sensorData.game_qr = sensorValue.un.gameRotationVector.real;
        break;

      case SH2_ACCELEROMETER:
        sensorData.accel_x = sensorValue.un.accelerometer.x;
        sensorData.accel_y = sensorValue.un.accelerometer.y;
        sensorData.accel_z = sensorValue.un.accelerometer.z;
        break;

      case SH2_LINEAR_ACCELERATION:
        sensorData.lin_x = sensorValue.un.linearAcceleration.x;
        sensorData.lin_y = sensorValue.un.linearAcceleration.y;
        sensorData.lin_z = sensorValue.un.linearAcceleration.z;
        break;

      case SH2_GRAVITY:
        sensorData.grav_x = sensorValue.un.gravity.x;
        sensorData.grav_y = sensorValue.un.gravity.y;
        sensorData.grav_z = sensorValue.un.gravity.z;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        sensorData.gyro_x = sensorValue.un.gyroscope.x;
        sensorData.gyro_y = sensorValue.un.gyroscope.y;
        sensorData.gyro_z = sensorValue.un.gyroscope.z;
        break;

      case SH2_MAGNETIC_FIELD_CALIBRATED:
        sensorData.mag_x = sensorValue.un.magneticField.x;
        sensorData.mag_y = sensorValue.un.magneticField.y;
        sensorData.mag_z = sensorValue.un.magneticField.z;
        break;

      case SH2_STEP_COUNTER:
        sensorData.steps = sensorValue.un.stepCounter.steps;
        break;

      default:
        break;
    }
}

// ---------- Output one JSON line at 50Hz ----------
void outputJSON() {
  uint32_t nowMs = millis();
  if (nowMs - lastOutputMs < OUTPUT_INTERVAL_MS) return;
  lastOutputMs = nowMs;

  char buf[512];
  snprintf(buf, sizeof(buf),
    "{\"ts\":%lu"
    ",\"rot\":{\"qi\":%.4f,\"qj\":%.4f,\"qk\":%.4f,\"qr\":%.4f,\"acc\":%.4f}"
    ",\"game_rot\":{\"qi\":%.4f,\"qj\":%.4f,\"qk\":%.4f,\"qr\":%.4f}"
    ",\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}"
    ",\"lin_accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}"
    ",\"gravity\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}"
    ",\"gyro\":{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f}"
    ",\"mag\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}"
    ",\"steps\":%lu"
    ",\"cal\":%u"
    "}",
    (unsigned long)nowMs,
    sensorData.rot_qi,  sensorData.rot_qj,  sensorData.rot_qk,
    sensorData.rot_qr,  sensorData.rot_acc,
    sensorData.game_qi, sensorData.game_qj, sensorData.game_qk,
    sensorData.game_qr,
    sensorData.accel_x, sensorData.accel_y, sensorData.accel_z,
    sensorData.lin_x,   sensorData.lin_y,   sensorData.lin_z,
    sensorData.grav_x,  sensorData.grav_y,  sensorData.grav_z,
    sensorData.gyro_x,  sensorData.gyro_y,  sensorData.gyro_z,
    sensorData.mag_x,   sensorData.mag_y,   sensorData.mag_z,
    (unsigned long)sensorData.steps,
    sensorData.cal_status
  );

  Serial.println(buf);
}

// ---------- BOOT button: long-press 3s to save calibration DCD ----------
void handleBootButton() {
  bool pressed = (digitalRead(BOOT_BTN) == LOW);

  if (pressed && !bootPressed) {
    bootPressed    = true;
    bootPressStart = millis();
  } else if (!bootPressed) {
    // nothing
  } else if (!pressed) {
    bootPressed = false;
  } else if (millis() - bootPressStart >= 3000) {
    // Prevent repeated triggering until button is released
    bootPressStart = millis() + 1000000UL;
    Serial.println("# Saving calibration DCD to flash...");
    // sh2_saveDcdNow() is provided by the sh2 library bundled with Adafruit_BNO08x
    int result = sh2_saveDcdNow();
    if (result == SH2_OK) {
      Serial.println("{\"event\":\"calibration_saved\"}");
    } else {
      Serial.print("# ERROR: sh2_saveDcdNow failed, code=");
      Serial.println(result);
    }
  }
}

// ---------- Main loop ----------
void loop() {
  readSensorData();
  outputJSON();
  handleBootButton();
}
