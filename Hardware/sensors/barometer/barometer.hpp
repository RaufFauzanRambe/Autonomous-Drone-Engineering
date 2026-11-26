// Copyright 2026 Autonomous-Drone-Engineering Contributors
// SPDX-License-Identifier: MIT
//
// Bosch BMP390 barometric pressure sensor driver (Linux I2C).
//
// Reference implementation for bench bring-up, HIL rigs, and companion
// computers. Reads uncompensated pressure/temperature and the factory
// calibration coefficients, then applies a simplified temperature-stabilized
// compensation model (see the note in compensate()). Altitude follows the
// ISA hypsometric formula against a configurable reference pressure.
// TODO(datasheet-verify): confirm register addresses and coefficient
// scaling against the BMP390 datasheet revision of your part.

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

namespace ade {
namespace sensors {

/// Tunables; mirrors the barometer section of sensor documentation and the
/// defaults used on the ADE bench profile.
struct BaroConfig {
  /// I2C bus device node and sensor 7-bit address (BMP390 SDO low: 0x77,
  /// SDO high: 0x76).
  std::string i2c_device = "/dev/i2c-1";
  uint8_t i2c_address = 0x77;

  /// Oversampling: pressure x8, temperature x1 (dataset's recommended
  /// combination for ~100 Hz with low noise).
  int press_osr = 8;
  int temp_osr = 1;

  /// Reference pressure for the altitude computation [Pa]. Set from the
  /// ground-station QNH before missions; 101325 Pa is the ISA standard.
  double sea_level_pa = 101325.0;

  /// First-order low-pass cutoff on altitude [Hz].
  double altitude_lpf_hz = 10.0;
};

/// One compensated, timestamped barometer sample.
struct BaroSample {
  /// Host monotonic timestamp [us].
  uint64_t t_us = 0;
  /// Compensated pressure [Pa].
  double pressure_pa = 0.0;
  /// Compensated temperature [C].
  double temperature_c = 0.0;
  /// ISA altitude above the sea-level reference [m], low-passed.
  double altitude_m = 0.0;
  /// True when the read and sanity checks passed.
  bool valid = false;
};

/// I2C driver for the BMP390 pressure sensor.
class Barometer {
 public:
  explicit Barometer(BaroConfig config = BaroConfig{});
  ~Barometer();

  Barometer(const Barometer&) = delete;
  Barometer& operator=(const Barometer&) = delete;

  /// Opens the bus, resets the sensor, checks the chip ID, reads the
  /// factory calibration coefficients, and enables pressure+temperature
  /// measurement. Returns false on any failure.
  bool open();

  /// Soft-resets the sensor and closes the bus.
  void close();

  /// Reads the chip ID register and compares it with 0x50 (BMP3).
  bool selfTest();

  /// Blocking read of one pressure+temperature pair, compensated and
  /// converted. Returns an invalid sample on failure.
  BaroSample readBlocking();

  /// Most recent valid sample without bus traffic.
  BaroSample latest() const;

  /// False after any I2C failure until the next successful read.
  bool healthy() const { return healthy_.load(std::memory_order_relaxed); }

  /// Updates the sea-level reference pressure used for altitude [Pa].
  void setSeaLevelPressure(double pa);

 private:
  bool writeReg(uint8_t reg, uint8_t value) const;
  bool readRegs(uint8_t first_reg, uint8_t* out, std::size_t len) const;
  bool readCalibration();
  double compensate(uint32_t adc_p, uint32_t adc_t, double* out_temp_c) const;

  BaroConfig config_;
  int fd_ = -1;

  // Factory calibration coefficients, raw register values.
  uint16_t par_t1_ = 0;
  int16_t par_t2_ = 0;
  int8_t par_t3_ = 0;
  int16_t par_p1_ = 0;
  int16_t par_p2_ = 0;
  int8_t par_p3_ = 0;
  int16_t par_p4_ = 0;
  int16_t par_p5_ = 0;
  int8_t par_p6_ = 0;
  int16_t par_p7_ = 0;
  int16_t par_p8_ = 0;
  int8_t par_p9_ = 0;

  BaroSample latest_{};
  bool lpf_initialized_ = false;
  mutable std::mutex mutex_;
  std::atomic<bool> healthy_{false};
};

}  // namespace sensors
}  // namespace ade
