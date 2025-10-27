// test_imu_unit.ino
// Unit test harness for IMU (IMU.h / IMU.cpp).
// - Run on ESP32
// - Serial output at 115200
//
// WARNING: calibrateGyro() requires the device to be perfectly still.
/*
#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include "IMU.h"

// NVS keys (must match IMU.cpp)
static const char *PREFS_NAMESPACE = "imu_cfg_v1";
static const char *KEY_HAS_CAL = "hasCalib";
static const char *KEY_GBX = "gbx";
static const char *KEY_GBY = "gby";
static const char *KEY_GBZ = "gbz";
static const char *KEY_TH_FF = "th_ff";
static const char *KEY_TH_SH = "th_sh";
static const char *KEY_TH_VIB = "th_vib";
static const char *KEY_TH_TILT = "th_tilt";
static const char *KEY_BETA = "madg_bw";
static const char *KEY_LP_HZ = "lp_hz";
static const char *KEY_VIB_WINDOW = "vib_w";
static const char *KEY_TILT_SNAP_MS = "tilt_snap";
static const char *KEY_TILT_CONFIRM = "tilt_cnt";
static const char *KEY_ACCEL_LPF_HZ = "acc_lpf";
static const char *KEY_CFG_VER = "cfg_ver";
static const char *KEY_CFG_CSUM = "cfg_csum";

Preferences prefs;
IMU imu; // default config

// tiny assertion helpers
void pass(const char *msg) { Serial.printf("[PASS] %s\n", msg); }
void fail(const char *msg) { Serial.printf("[FAIL] %s\n", msg); }
void info(const char *msg) { Serial.printf("[INFO] %s\n", msg); }

bool isFiniteFloat(float v) {
  return isfinite(v);
}

void test_begin_and_whoami() {
  Serial.println("\n=== Test 1: begin() and WHO_AM_I ===");
  if (!imu.begin()) {
    fail("IMU.begin() returned false");
    return;
  }
  pass("IMU.begin() returned true (device responded)");
  // WHO_AM_I is printed by IMU.begin(); user can inspect serial logs for expected 0x70 / 0x71 etc
}

void test_dump_registers() {
  Serial.println("\n=== Test 2: register dump (exercise) ===");
  Serial.println("Register dump will be printed below:");
  imu.dumpRegisters();
  pass("dumpRegisters() executed (inspect printed register list)");
}

void test_reset_persistent() {
  Serial.println("\n=== Test 3: resetPersistentConfig() ===");
  imu.resetPersistentConfig();
  prefs.begin(PREFS_NAMESPACE, true);
  uint32_t ver = prefs.getUInt(KEY_CFG_VER, 0);
  prefs.end();
  if (ver == 0) pass("NVS cleared (cfg_ver==0)");
  else fail("NVS not cleared (cfg_ver != 0)");
}

void test_calibrate_and_persist() {
  Serial.println("\n=== Test 4: calibrateGyro() + savePersistentConfig() ===");
  Serial.println("**IMPORTANT**: Keep the device perfectly still for calibration now.");
  bool ok = imu.calibrateGyro();
  if (!ok) {
    fail("calibrateGyro() failed (I2C error?)");
    return;
  }
  // Read stored values from Preferences to confirm saved
  prefs.begin(PREFS_NAMESPACE, true);
  bool has = prefs.getBool(KEY_HAS_CAL, false);
  float gbx = prefs.getFloat(KEY_GBX, NAN);
  float gby = prefs.getFloat(KEY_GBY, NAN);
  float gbz = prefs.getFloat(KEY_GBZ, NAN);
  uint32_t ver = prefs.getUInt(KEY_CFG_VER, 0);
  uint32_t crc = prefs.getUInt(KEY_CFG_CSUM, 0);
  prefs.end();

  if (!has) {
    fail("Preferences: hasCalib not set after calibration");
    return;
  }
  if (!isFiniteFloat(gbx) || !isFiniteFloat(gby) || !isFiniteFloat(gbz)) {
    fail("Preferences: gyro biases not saved correctly (not finite)");
    return;
  }
  Serial.printf("Saved biases (LSB): gbx=%.3f gby=%.3f gbz=%.3f\n", gbx, gby, gbz);
  Serial.printf("Stored cfg_ver=%u cfg_crc=0x%08X\n", ver, crc);
  pass("calibrateGyro() saved biases and version+crc present");
}

void test_persistence_reload() {
  Serial.println("\n=== Test 5: Persistence reload (new IMU instance) ===");
  // Instantiate a new IMU object to simulate reboot-to-verify persistence.
  IMU imu2;
  if (!imu2.begin()) {
    fail("New IMU.begin() failed after calibration – check I2C/wiring");
    return;
  }
  // loadPersistentConfig returns true only when version+crc OK.
  bool loaded = imu2.loadPersistentConfig();
  if (loaded) pass("imu2.loadPersistentConfig() returned true (persistence verified)");
  else fail("imu2.loadPersistentConfig() returned false (persistence failed)");
}

void test_crc_tamper_detection() {
  Serial.println("\n=== Test 6: CRC tamper detection ===");
  prefs.begin(PREFS_NAMESPACE, true);
  uint32_t stored_crc = prefs.getUInt(KEY_CFG_CSUM, 0);
  prefs.end();
  if (stored_crc == 0) {
    fail("No stored CRC available to tamper with – run calibration test first");
    return;
  }
  // Save original
  uint32_t orig_crc = stored_crc;
  // Corrupt CRC
  prefs.begin(PREFS_NAMESPACE, false);
  prefs.putUInt(KEY_CFG_CSUM, stored_crc ^ 0xFFFFFFFFu);
  prefs.end();

  // New IMU instance should not accept config
  IMU imu3;
  bool loaded = imu3.loadPersistentConfig();
  if (!loaded) pass("Tampered CRC detected (loadPersistentConfig returned false)");
  else fail("Tampered CRC not detected (unexpected)");

  // restore CRC
  prefs.begin(PREFS_NAMESPACE, false);
  prefs.putUInt(KEY_CFG_CSUM, orig_crc);
  prefs.end();
  Serial.println("Restored original CRC.");
}

void test_version_mismatch_detection() {
  Serial.println("\n=== Test 7: Version mismatch detection ===");
  // Read stored version
  prefs.begin(PREFS_NAMESPACE, true);
  uint32_t stored_ver = prefs.getUInt(KEY_CFG_VER, 0);
  prefs.end();
  if (stored_ver == 0) {
    fail("No stored version – run calibration test first");
    return;
  }

  // Create a config with bumped cfg_version to simulate new firmware
  IMUConfig cfg_new;
  cfg_new.cfg_version = (uint16_t)(stored_ver + 1); // bump version
  IMU imu_bumped(cfg_new);
  // Note: imu_bumped.loadPersistentConfig() will compare stored_ver vs cfg_version and skip
  bool loaded = imu_bumped.loadPersistentConfig();
  if (!loaded) pass("Version mismatch correctly rejected stored config");
  else fail("Version mismatch NOT detected (unexpected)");
}

void test_update_and_fusion() {
  Serial.println("\n=== Test 8: update() and Madgwick fusion run ===");
  const int N = 50;
  bool all_ok = true;
  for (int i = 0; i < N; ++i) {
    if (!imu.update()) {
      Serial.printf("update() failed at iteration %d\n", i);
      all_ok = false;
      break;
    }
    ImuEuler e = imu.getEuler();
    if (!isFiniteFloat(e.roll) || !isFiniteFloat(e.pitch) || !isFiniteFloat(e.yaw)) {
      Serial.printf("Euler contains non-finite at iter %d: R=%.3f P=%.3f Y=%.3f\n", i, e.roll, e.pitch, e.yaw);
      all_ok = false;
      break;
    }
    delay(5); // small delay
  }
  if (all_ok) pass("update() ran repeatedly and produced finite Euler angles");
  else fail("update() / fusion had issues (see logs)");
}

void test_event_baseline_and_vib() {
  Serial.println("\n=== Test 9: baseline events & vibration RMS read ===");
  // Baseline (device still): expect no freefall; shock/vibration may be false
  ImuEvents ev = imu.getEvents();
  float vib = imu.getVibrationRMS();
  Serial.printf("Events baseline: freefall=%d shock=%d tilt=%d vibration=%d fall=%d\n",
                ev.freefall, ev.shock, ev.tilt, ev.vibration, ev.fall);
  Serial.printf("Vibration RMS: %.6f\n", vib);
  pass("Baseline events retrieved (manual inspection recommended).");
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\n\n==== IMU Unit Test Harness ====");
  info("Make sure the IMU is wired, power is stable, and device is still for calibration.");

  test_begin_and_whoami();
  delay(200);

  test_dump_registers();
  delay(200);

  test_reset_persistent();
  delay(200);

  test_calibrate_and_persist(); // KEEP DEVICE STILL HERE
  delay(200);

  test_persistence_reload();
  delay(200);

  test_crc_tamper_detection();
  delay(200);

  test_version_mismatch_detection();
  delay(200);

  test_update_and_fusion();
  delay(200);

  test_event_baseline_and_vib();
  delay(200);

  Serial.println("\n=== Unit tests complete ===");
  Serial.println("You can re-run specific tests by resetting the board or editing the sketch.");
}

void loop() {
  // Nothing to do; tests are run in setup().
  delay(1000);
}
*/