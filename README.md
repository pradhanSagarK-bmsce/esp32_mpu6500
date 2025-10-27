# IMU Module — Full Documentation (extended)

> This document describes the custom MPU6500/9250 IMU wrapper (IMU.h / IMU.cpp)  built:  
> features: register dump, gyro bias calibration & persistence, raw conversion, Madgwick IMU fusion, IIR filters, vibration RMS, sliding-window tilt detection, fall/freefall/shock detection, persistent config in NVS protected by version + CRC32.

---

# Table of contents

1. Quick start (how to use the API)
    
2. Data types & configuration (IMUConfig, ImuRaw, ImuEuler, ImuEvents)
    
3. Public API: functions and examples
    
4. Persistent storage format, versioning and CRC (exact order & keys)
    
5. Internals — sampling, conversions and filters
    
6. Madgwick IMU — how it works & tuning
    
7. Tilt detection — sliding-window quaternion-diff (math & algorithm)
    
8. Event detection logic (freefall, shock, vibration, fall)
    
9. Vibration RMS and windows
    
10. Calibration procedure (step-by-step)
    
11. Test harness & interpreting unit test output
    
12. Tuning & suggested parameter ranges (for fall vs quake detection)
    
13. Additional signal processing suggestions for earthquake detection
    
14. Troubleshooting & common issues
    
15. Next steps & improvement ideas
    

---

# 1) Quick start — minimal usage example

```cpp
#include "IMU.h"

IMU imu; // default config
void setup() {
  Serial.begin(115200);
  if (!imu.begin()) {
    Serial.println("IMU not detected!");
    while (1) delay(1000);
  }
  imu.dumpRegisters();        // optional
  imu.calibrateGyro();        // do this once while device is perfectly still
}

void loop() {
  if (imu.update()) {
	ImuRaw rd = imu.getRaw();
    ImuEuler e = imu.getEuler();
    ImuEvents ev = imu.getEvents();
    float vib = imu.getVibrationRMS();
    Serial.printf("R%.2f P%.2f Y%.2f vib=%.3f freefall=%d fall=%d\n",
                  e.roll, e.pitch, e.yaw, vib, ev.freefall, ev.fall);
  }
  delay(5);
}
```

To change and persist a tilt parameter at runtime:

```cpp
imu.setTiltDeltaDeg(25.0f);
imu.setTiltConfirmCount(3);
imu.savePersistentConfig(); // stores new thresholds + bias + CRC + version
```

---

# 2) Data types & configuration (what each field means)

### IMUConfig

- `i2c_addr` (uint8_t): I²C bus address, default `0x68`.
    
- `calib_samples` (int): gyro calibration sample count (default 2000).
    
- `beta` (float): Madgwick filter gain. Higher → faster correction (less smoothing). Typical: 0.04–0.12.
    
- `lp_cutoff_hz` (float): low-pass cutoff for gravity estimate (IIR) used to separate gravity from dynamic accel.
    
- `loop_hz` (int): target update rate (used to fallback dt if micros timing fails).
    
- Thresholds:
    
    - `freefall_thresh_g` (float): magnitude below which device is considered in freefall (g).
        
    - `shock_thresh_g` (float): acceleration magnitude above which shock is flagged.
        
    - `tilt_delta_deg` (float): angle (deg) change threshold to register a tilt event.
        
    - `vibration_rms_thresh` (float): RMS threshold for vibration event.
        
- Buffers & windows:
    
    - `vib_window_samples` (int): vibration RMS buffer length.
        
    - `tilt_snap_ms` (int): sliding-window time (ms) used to compare orientation (e.g. default 200 ms).
        
    - `tilt_confirm_count` (int): number of consecutive positive tilt checks to confirm tilt.
        
- `accel_lpf_hz`: lowpass on acceleration magnitude for stable freefall/vibration decisions.
    
- `cfg_version` (uint16_t): stored configuration version. Bump it when changing stored layout.
    

### ImuRaw

- `ax_g, ay_g, az_g`: accelerometer in g (gravity units).
    
- `gx_dps, gy_dps, gz_dps`: gyroscope in degrees/sec.
    
- `temp_c`: sensor temperature.
    

### ImuEuler

- `roll, pitch, yaw` in degrees. Yaw will drift over time unless you use a magnetometer (AK8963); on MPU6500-only boards yaw is unreliable for absolute heading.
    

### ImuEvents

- `freefall`, `shock`, `tilt`, `vibration`, `fall` — booleans capturing event states.
    

---

# 3) Public API — functions, behavior & examples

### Construction / lifecycle

- `IMU(const IMUConfig &cfg = IMUConfig())` — construct with override config.
    
- `~IMU()` — frees internal buffers.
    

### Initialization

- `bool begin()`
    
    - Starts `Wire`, sets clock to 400kHz, wakes sensor (writes `PWR_MGMT_1`), optionally sets default ranges (Accel±2g, Gyro±250 dps), reads `WHO_AM_I`.
        
    - Also calls `loadPersistentConfig()` and reports whether persistent config was loaded and CRC verified.
        
    - Returns `true` if the device responded to I²C commands.
        

### Diagnostics

- `void dumpRegisters()`
    
    - Reads and prints registers `0x00..0x7F` to Serial. Useful to detect variants (WHO_AM_I, AK8963 presence, config bits).
        

### Calibration & persistence

- `bool calibrateGyro()`
    
    - Samples `calib_samples` of raw data while device is still, averages gyroscope LSB values and stores them in `gbx_, gby_, gbz_` (LSB units).
        
    - After success it calls `savePersistentConfig()` to persist bias + thresholds + CRC.
        
- `bool loadPersistentConfig()` — loads saved config only if:
    
    1. `cfg_version` matches expected `cfg_.cfg_version`
        
    2. computed CRC32 over stored fields matches stored CRC  
        Returns `true` if valid saved config is loaded.
        
- `void savePersistentConfig()` — writes bias, thresholds, version, and computed CRC to NVS (Preferences).
    
- `void resetPersistentConfig()` — clears stored keys in NVS for this namespace.
    

### Runtime update & getters

- `bool update()`
    
    - Reads 14 bytes (`ACCEL_XOUT_H` block): accel, temp, gyro.
        
    - Converts values into `ax_g`, `gx_dps` etc using `ACC_LSB_PER_G` and `GYRO_LSB_PER_DPS` constants (adjust these if you change sensor range registers).
        
    - Applies IIR low-pass on gravity estimate (used to compute dynamic acceleration).
        
    - Pushes vibration sample into circular RMS buffer.
        
    - Runs Madgwick IMU-only update (gyro in rad/s, accel normalized).
        
    - Converts quaternion to Euler and calls `processEvents()`.
        
    - Returns `true` on successful I2C read.
        
- `ImuRaw getRaw() const`, `ImuEuler getEuler() const`, `ImuEvents getEvents() const`, `float getVibrationRMS() const` — read-only accessors.
    

### Runtime setters

- `void setBeta(float b)` — adjust Madgwick gain at runtime.
    
- `void setLPcutoffHz(float hz)` — change gravity IIR cutoff.
    
- `void setTiltDeltaDeg(float deg)`, `setTiltConfirmCount(int cnt)`, `setTiltSnapMs(int ms)`, `setAccelLpfHz(float hz)` — adjust tilt & accel smoothing thresholds. Call `savePersistentConfig()` to persist.
    

---

# 4) Persistent storage format, versioning and checksum (exact details)

Storage uses ESP32 `Preferences` (NVS). Namespace: `"imu_cfg_v1"`. Keys used:

- `hasCalib` (bool) — flag set when calib saved (optional human readable).
    
- `gbx`, `gby`, `gbz` (float) — gyro biases (LSB units).
    
- `th_ff` (float) — freefall threshold (g).
    
- `th_sh` (float) — shock threshold (g).
    
- `th_vib` (float) — vibration RMS threshold.
    
- `th_tilt` (float) — tilt_delta_deg.
    
- `madg_bw` (float) — Madgwick beta.
    
- `lp_hz` (float) — accel gravity LP cutoff.
    
- `vib_w` (int) — vib window samples.
    
- `cfg_ver` (uint32) — stored config version.
    
- `cfg_csum` (uint32) — CRC32 checksum.
    

### CRC computation (what is hashed and in what order)

To detect tampering or layout mismatch, we compute a CRC32 across the following fields packed in **this exact order** (binary representation used — 4-byte floats, ints as native size used in the code):

1. `gbx` (float)
    
2. `gby` (float)
    
3. `gbz` (float)
    
4. `freefall_thresh_g` (float)
    
5. `shock_thresh_g` (float)
    
6. `vibration_rms_thresh` (float)
    
7. `tilt_delta_deg` (float)
    
8. `beta` (float)
    
9. `lp_cutoff_hz` (float)
    
10. `vib_window_samples` (int)
    
11. `cfg_version` (uint32)
    

The CRC algorithm: standard CRC-32 (polynomial `0xEDB88320`) implemented in `computeConfigCRC()`. The stored CRC is the bitwise-not of the usual iterated CRC (standard finalization).

### Version handling

- On `loadPersistentConfig()` we first read `cfg_ver` from NVS. If `0` → no config found. If `cfg_ver` ≠ expected `cfg_.cfg_version` → skip load (safe upgrade strategy). This prevents older on-flash layout being misinterpreted by a newer code version.
    

**Important**: When you change the set of fields stored or their order, bump `IMUConfig.cfg_version` (e.g. set to 2), so older stored records are ignored.

---

# 5) Internals — conversions, filters, buffers

### Conversions

- Accel: `ax_g = raw_acc / ACC_LSB_PER_G`. Default `ACC_LSB_PER_G = 16384.0f` (±2g full scale). If you set accel FS to ±4g or ±8g, update this constant accordingly.
    
- Gyro: `gx_dps = (raw_gyro - gbx) / GYRO_LSB_PER_DPS`. `GYRO_LSB_PER_DPS = 131.0f` for ±250 dps. If you change gyro FS adjust constant.
    

### Gravity separation (IIR)

- Gravity estimate (`gx_lp_, gy_lp_, gz_lp_`) computed with a 1-pole IIR:
    
    - `tau = 1/(2π * fc)` where `fc = cfg_.lp_cutoff_hz`
        
    - `alpha = dt / (dt + tau)`
        
    - `g_lp = (1 - alpha)*g_lp + alpha*acc`
        
- This gives a smooth gravity estimate to subtract from accel and isolate dynamic acceleration.
    

### Accel magnitude LPF

- `acc_mag_lpf_` is smoothed by a lowpass (user-configurable `accel_lpf_hz`) before freefall/vibration decisions (reduces momentary spikes that would otherwise trigger false events).
    

### Vibration RMS

- Circular buffer `vib_buf_` of `vib_window_samples`. Each update pushes `fabs(accMag - 1.0f)`, i.e., magnitude deviation from 1g.
    
- RMS computed over available samples: `sqrt(sum(samples^2)/N)`.
    

### Sliding quaternion buffer for tilt detection

- Circular buffers store quaternion components and timestamps at each update. Buffer length equals `vib_window_samples` or another window length appropriate for tilt_snap_ms.
    
- To compare orientation `tilt_snap_ms` ago, the code finds the quaternion sample closest to `now - tilt_snap_ms`. Compute relative rotation angle with dot product (see section 7).
    

---

# 6) Madgwick IMU — what it is & tuning

### Algorithm summary

- Madgwick AHRS fuses gyroscope (integrates rotation) with accelerometer (gravity vector) to correct orientation drift. It’s an efficient gradient-descent algorithm ideal for embedded devices (no magnetometer here → yaw not absolute).
    
- The update expects:
    
    - Gyro in radians/sec,
        
    - Accel in normalized units (unit vector).
        

### Key parameter: `beta`

- `beta` determines how strongly accelerometer corrections are applied:
    
    - Too low → slow correction, better gyro smoothness but slower recovery from drift.
        
    - Too high → aggressive corrections, can produce jitter if accel noisy.
        
- Typical starting values:
    
    - 0.02 — very smooth (slow)
        
    - 0.04 — medium
        
    - 0.08 — faster (used in your defaults)
        
    - 0.12+ — aggressive (for high dynamic environments, but risk of accel noise causing jitter)
        

### Sampling rate / dt effects

- For stable results, call `update()` at a consistent high rate (≥100Hz). You used default `loop_hz = 200` — good for both fall detection and short seismic events.
    
- Madgwick expects dt in seconds accurately. Use `micros()` timers (as implemented) for robust dt.
    

---

# 7) Tilt detection — sliding-window quaternion diff (math & algorithm)

### Why sliding-window?

- A fixed snapshot interval (take Euler now and 200ms earlier) is fine but can be inconsistent if `update()` timing jitters. Comparing current quaternion to one exactly ~200ms earlier gives a deterministic time window unaffected by sampling jitter.
    

### Math — relative rotation angle between quaternions

Given two unit quaternions `q_now` and `q_old`:

- Compute dot product `d = q_now · q_old = q0n*q0o + q1n*q1o + q2n*q2o + q3n*q3o`
    
- Ensure `d` is clamped to [-1,1] (numerical safety).
    
- The relative rotation angle `θ` (radians) equals:
    
    - `θ = 2 * acos( abs(d) )`
        
- Convert to degrees: `θ_deg = θ * (180 / π)`
    
- Reason: relative angle between orientations is related to quaternion inner product. Using absolute value removes sign ambiguity (shortest rotation).
    

### Algorithm in code (high-level)

1. Store quaternion + timestamp each `update()` into circular buffer.
    
2. When processing events:
    
    - Find `q_old` corresponding to `now - cfg_.tilt_snap_ms` (either nearest timestamp or linear interpolation between two samples).
        
    - Compute `θ_deg` as above.
        
    - If `θ_deg > cfg_.tilt_delta_deg`, increment a confirmation counter. If this counter reaches `tilt_confirm_count`, set `tilt=true`. Otherwise, clear counter after a timeout or negative detection.
        

### Why use quaternion instead of Euler diffs?

- Euler differences can suffer gimbal lock and discontinuities near ±180°, leading to false positives. A quaternion-based angle measure is robust, unambiguous and frame-invariant.
    

---

# 8) Event detection logic (freefall, shock, vibration, fall)

### Freefall

- `accMag` computed as `sqrt(ax^2 + ay^2 + az^2)`. After smoothing with accel LPF, a freefall event is `accMag < freefall_thresh_g`.
    
- Suggested start threshold: `~0.3 g` for falling objects; adjust with calibration and trials.
    

### Shock

- Detected either by `accMag > shock_thresh_g` or extremely large angular rates (`|gyro_dps| > 300 dps` in current heuristic).
    
- Suggested shock threshold: `~2.5 g` for impactful events; increase for rough environments.
    

### Vibration

- RMS of `|accMag - 1g|` over `vib_window_samples`. Threshold `vibration_rms_thresh` triggers vibration alert.
    

### Tilt

- From sliding-window quaternion relative angle calculation. Requires `tilt_confirm_count` consecutive detections for robustness.
    

### Fall (simple heuristic)

- Freefall followed by shock within 1 second → fall boolean = true.
    
- This is a pragmatic approach; for production-grade fall detection add more checks (directionality, post-impact motionlessness, heart-rate integration, etc.)
    

---

# 9) Vibration RMS and windows

- The RMS buffer length (`vib_window_samples`) defines the time window for vibration detection:
    
    - At 200Hz: `100` samples ≈ 0.5 s
        
    - Choose `vib_window_samples` to match expected seismic/fall signatures:
        
        - For instant impact detection: short windows (25–50 samples).
            
        - For earthquake detection: longer windows to compute energy over seconds.
            

---

# 10) Calibration procedure (recommended steps)

1. Place device on a stable surface. Eliminate desk vibration, isolate from hand movement.
    
2. Call `imu.calibrateGyro()` in code (or run unit test). Keep device motionless during calibration. Default collects 2000 samples — adjust `calib_samples` for quicker runs.
    
3. After calibration completes, `savePersistentConfig()` is automatically called from `calibrateGyro()`. Reboot and verify by `loadPersistentConfig()`/unit test.
    
4. Optionally, check bias values in preferences via serial logs (printed as LSB). Convert to deg/s by dividing biasLSB by `GYRO_LSB_PER_DPS`.
    
5. If high temperature changes will occur in real deployment, consider re-calibrating at several temperatures or implementing temperature compensation.
    

---

# 11) Unit test harness — what it does & how to interpret results

You already ran the harness — summary of the tests and what they mean:

- **begin() / WHO_AM_I**: confirms I2C and device present. WHO_AM_I=0x70 indicates MPU6500 (no AK8963 magnetometer) or a variant — expected.
    
- **dumpRegisters()**: printed registers 0x00..0x7F. Useful to check config bits and confirm that WHO_AM_I,AK8963 presence etc match your board.
    
- **resetPersistentConfig()**: cleared NVS; test reported `nvs_open failed: NOT_FOUND` initially (normal if NVS namespace didn't exist).
    
- **calibrateGyro() + savePersistentConfig()**: collected samples and saved biases; output showed biases and CRC saved — PASS.
    
- **Persistence reload**: new IMU instance loaded & verified CRC — PASS.
    
- **CRC tamper detection**: intentionally corrupted CRC and `loadPersistentConfig()` correctly rejected the corrupted config — PASS.
    
- **Version mismatch detection**: constructed IMU with bumped `cfg_version` and `loadPersistentConfig()` rejected older data — PASS.
    
- **update() and fusion**: ran `update()` repeatedly (50 iterations) and Euler outputs were finite — PASS.
    
- **baseline events & vib RMS**: baseline captured (`freefall=0, shock=0, tilt=0, vibration=0`) and small RMS value printed — PASS.
    

If you see errors:

- `Preferences begin(): nvs_open failed` can appear when namespace not previously created; the library will still create keys on the first `savePersistentConfig()`.
    

---

# 12) Tuning & recommended parameter ranges

### Sampling frequency

- Fall detection: `>=100 Hz` recommended, `200 Hz` is good.
    
- Seismic/earthquake detection: sensor frequencies of interest are typically 0.1–20 Hz — you can still sample at 200Hz and bandpass down in software.
    

### Madgwick `beta`

- 0.02 → very smooth, slow correction
    
- 0.04 → good compromise
    
- 0.08 → more aggressive correction (used currently)  
    Choose by trial — run while moving device and tune to minimize tilt jitter while keeping good tracking.
    

### Tilt detection

- `tilt_delta_deg`: 10–30° depending on sensitivity desired.
    
- `tilt_snap_ms`: 150–300 ms for human falls; for quakes you might look at longer windows or a multi-scale approach.
    
- `tilt_confirm_count`: 1–3 (higher to avoid false positives).
    

### Shock / freefall

- `freefall_thresh_g`: 0.2–0.5 g
    
- `shock_thresh_g`: 2.0–6.0 g depending on expected impacts
    

### RMS vibration

- `vibration_rms_thresh` starts ~0.3–0.6 (units are g deviation). Tune against expected environment noise.
    

---

# 13) Advanced signal processing for earthquake detection (recommendations)

If your main purpose is to detect earthquakes or structural shaking, consider these additions:

1. **Bandpass filter** (0.1–10 or 0.5–20 Hz) applied to vertical (Z) and horizontal accelerations — most earthquake energy lies under 20Hz.
    
2. **STA/LTA (Short-term average / Long-term average)**:
    
    - Compute STA (e.g., 0.5 s window) and LTA (e.g., 30 s window) of absolute acceleration or energy.
        
    - Detection: STA/LTA ratio > threshold → trigger. This is standard in seismology.
        
3. **Power Spectral Density (PSD)** or short-time Fourier transform (STFT) to determine frequency content — help discriminate between human activity and earthquake (earthquakes show characteristic frequency bands).
    
4. **Multi-axis fusion/Energy**: compute vector magnitude and use bandpass and STA/LTA on that.
    
5. **Cross-sensor correlation**: If you have multiple sensors, correlate to reduce false positive.
    
6. **Event classification**: simple ML classifier (lightweight) on short feature vector (STA/LTA, RMS, dominant frequency) to improve differentiation.
    

---

# 14) Troubleshooting & common issues

### I²C errors / `requestFrom(): i2cWriteReadNonStop returned Error -1`

- Check wiring: SDA to SDA, SCL to SCL, 3.3V supply, ground common.
    
- Ensure pull-ups present on I²C lines (some breakout boards include them; ESP32 internal weak pull-ups may be insufficient).
    
- Confirm device address with I2C scanner — you saw `0x68` and `0x3C` (OLED) — these are correct.
    
- If WHO_AM_I returns unexpected value: check board marking. You earlier saw WHO_AM_I=0x70 → OK (MPU6500/MPU9250 variants vary).
    
- If no device found: check power or try `Wire.begin(SDA, SCL)` with explicit pins (some ESP32 boards need specific pins).
    

### Non-detection of magnetometer (AK8963)

- Many MPU6500-only modules do not include AK8963 magnetometer; WHO_AM_I=0x70 and "No AK8963 detected" is normal.
    
- Yaw will drift if magnetometer missing.
    

### Preferences / NVS errors

- `nvs_open failed: NOT_FOUND` on `prefs.begin(namespace, true)` is expected when no record exists. `savePersistentConfig()` will create the keys.
    

### Quaternion/Euler jitter

- If Euler appears noisy:
    
    - Reduce `beta`.
        
    - Increase accelerometer LPF cutoff or increase smoothing for gravity estimate.
        
    - Re-run gyro calibration.
        
    - Reduce I2C latency and keep consistent update frequency.
        

---

# 15) Next steps & improvement ideas

- Implement optional **Mahony** or **Kalman** filter as a selectable alternative to Madgwick — Mahony has integral term advantages, Kalman can integrate temperature-dependent biases.
    
- Add **temperature compensation** for gyro bias (record bias vs temp at multiple temps and interpolate).
    
- Add **magnetometer** support when present to stabilize yaw (`AK8963` on some MPU9250 boards).
    
- Add **on-device CLI** (serial menu) to set parameters live and store them (I can supply this).
    
- Add **seismic detection pipeline**: bandpass → STA/LTA → classification. I can provide example code for STA/LTA.
    
- Add **OTA** or safe firmware upgrade that automatically bumps stored config version if needed.
    

---

# Example: how the tilt sliding-window routine would be used (pseudocode)

```cpp
// called inside update() after madgwick update
storeQuaternion(q0_, q1_, q2_, q3_, micros());

unsigned long want_ts = micros() - cfg_.tilt_snap_ms * 1000UL;
Quaternion q_old = findQuaternionClosestTo(want_ts); // from circular buffer
if (q_old.valid) {
  float dot = fabs(q0_*q_old.q0 + q1_*q_old.q1 + q2_*q_old.q2 + q3_*q_old.q3);
  dot = constrain(dot, -1.0f, 1.0f);
  float theta = 2.0f * acosf(dot) * (180.0f / M_PI);
  if (theta > cfg_.tilt_delta_deg) {
     tilt_confirm_seen_++;
     if (tilt_confirm_seen_ >= cfg_.tilt_confirm_count)
        event.tilt = true;
  } else {
     tilt_confirm_seen_ = 0;
  }
}
```

---

