# Sensors

The ADE-001 sensor suite covers the three measurements every autonomous
flight depends on: angular rate and specific force (IMU), absolute pressure
(barometer), and global position plus velocity (GNSS). Each sensor lives in
its own directory with a Linux user-space C++ driver, a YAML configuration
file that mirrors the driver defaults, and documentation of the register map
and mounting requirements. The suite-level specifications, mounting
directions, and calibration procedures are collected in
`sensor_documentation.md`.

| Directory    | Device               | Bus              | Driver status           |
| ------------ | -------------------- | ---------------- | ----------------------- |
| `imu/`       | Bosch BMI088         | SPI @ 10 MHz     | reference implementation |
| `gps/`       | u-blox NEO-M9N       | UART, UBX        | reference implementation |
| `barometer/` | Bosch BMP390         | I2C @ 400 kHz    | reference implementation |

All drivers follow the same contract: `open()` verifies the device and
applies the configuration, `selfTest()` checks identity registers,
`readBlocking()` returns a fully converted SI-unit sample with a validity
flag, `latest()` caches the most recent sample without bus traffic, and
`healthy()` reports cumulative bus health for the failsafe chain. No
exceptions, no dynamic allocation after `open()`: the drivers stay usable
from real-time contexts.

## Data flow

The flight controller samples the IMU at 1 kHz and the barometer at 100 Hz;
the GNSS receiver pushes 5 Hz UBX position/velocity solutions. All three
feed the EKF described in `../flight-controller/controller_overview.md`.
Data is timestamped against the host monotonic clock at the moment the
sample register is read, and the drivers tag samples that failed range
sanity checks rather than silently dropping them.

## Bring-up order

1. Bench power the FC card with props removed.
2. `imu` self test and bias calibration (level surface, still air).
3. `gps` fix check outdoors with an unobstructed sky view.
4. `barometer` comparison against a reference gauge.
5. Only then proceed to the controller bench tests per the contribution
   guide.
