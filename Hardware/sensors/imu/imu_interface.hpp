// Copyright 2026 Autonomous-Drone-Engineering Contributors
// SPDX-License-Identifier: MIT
//
// BMI088 six-axis inertial measurement unit driver (Linux SPI).
//
// The BMI088 is a dual-die part: the accelerometer and the gyroscope are
// separate chips with separate chip selects, so they appear as two SPI
// device nodes. This driver owns both. It is a reference implementation for
// bench bring-up, HIL rigs, and companion computers: verify the register
// map against the BMI088 datasheet revision of the part on your board
// before flight use.

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

namespace ade {
namespace sensors {

/// Tunables for the IMU pipeline; mirrors `sensors/imu/imu_config.yaml`.
struct ImuConfig {
  /// SPI device nodes. Accelerometer and gyroscope have independent chip
  /// selects (CS_A on spidev0.0, CS_G on spidev0.1 on the reference FC).
  std::string accel_device = "/dev/spidev0.0";
  std::string gyro_device = "/dev/spidev0.1";

  /// SPI clock. The BMI088 tolerates up to 10 MHz; keep traces short.
  uint32_t spi_speed_hz = 10000000;

  /// Full-scale ranges. Accelerometer: 3, 6, 12, or 24 [g].
  /// Gyroscope: 125, 250, 500, 1000, or 2000 [dps].
  int accel_range_g = 24;
  int gyro_range_dps = 2000;

  /// Output data rates in Hz. BMI088 accelerometer: 12.5-1600.
  /// Gyroscope: 100-2000.
  int accel_odr_hz = 400;
  int gyro_odr_hz = 1000;

  /// First-order low-pass cutoffs applied to converted samples [Hz].
  double gyro_lpf_hz = 80.0;
  double accel_lpf_hz = 50.0;

  /// If non-empty, a gyro bias calibration is loaded from this file at
  /// open() (format: three rad/s values, one per line).
  std::string calibration_file;
};

/// One fully converted, timestamped IMU sample.
struct ImuSample {
  /// Host monotonic timestamp captured at the data-register read [us].
  uint64_t t_us = 0;
  /// Specific force in the body frame (X forward, Y right, Z down) [m/s^2].
  std::array<double, 3> accel_mps2{};
  /// Angular rate in the body frame [rad/s].
  std::array<double, 3> gyro_radps{};
  /// Accelerometer die temperature [C].
  double temperature_c = 0.0;
  /// True when the sample passed range sanity checks.
  bool valid = false;
};

/// SPI driver for the BMI088 accelerometer + gyroscope pair.
///
/// Usage:
///   ImuConfig cfg;          // or load from imu_config.yaml
///   ImuInterface imu(cfg);
///   if (!imu.open()) { /* fatal */ }
///   imu.calibrateBias();
///   ImuSample s = imu.readBlocking();
class ImuInterface {
 public:
  explicit ImuInterface(ImuConfig config = ImuConfig{});
  ~ImuInterface();

  ImuInterface(const ImuInterface&) = delete;
  ImuInterface& operator=(const ImuInterface&) = delete;

  /// Opens both SPI devices, resets the chips, applies the configuration,
  /// and verifies both chip IDs. Logs failures to stderr; returns false
  /// without leaving devices half-open.
  bool open();

  /// Soft-resets the chips and closes the file descriptors.
  void close();

  /// Reads both chip-ID registers and compares them with expected values.
  bool selfTest();

  /// Collects `num_samples` samples with the vehicle still and stores the
  /// mean gyro rate as the bias estimate; the bias is applied to every
  /// subsequent gyro sample. Returns false if too few samples were valid.
  bool calibrateBias(int num_samples = 500);

  /// Blocking burst read of accelerometer, gyroscope, and temperature.
  /// Returns an invalid sample on communication failure.
  ImuSample readBlocking();

  /// Most recent valid sample from readBlocking() without bus traffic.
  ImuSample latest() const;

  /// False after any SPI failure until the next successful read.
  bool healthy() const { return healthy_.load(std::memory_order_relaxed); }

  /// Current gyro bias (rad/s), from calibrateBias() or the cal file.
  const std::array<double, 3>& gyroBias() const { return gyro_bias_; }

 private:
  // One full-duplex SPI transaction.
  int spiTransfer(int fd, const uint8_t* tx, uint8_t* rx, std::size_t len) const;
  // Register access; BMI088 accelerometer reads require a dummy byte.
  bool readRegs(int fd, uint8_t first_reg, uint8_t* out, std::size_t len,
                bool dummy_byte) const;
  bool writeReg(int fd, uint8_t reg, uint8_t value, bool dummy_byte) const;
  bool configureAccel();
  bool configureGyro();
  double readTemperatureC();

  ImuConfig config_;
  int accel_fd_ = -1;
  int gyro_fd_ = -1;
  std::array<double, 3> gyro_bias_{};       // rad/s
  std::array<double, 3> accel_lpf_{};
  std::array<double, 3> gyro_lpf_{};
  bool lpf_initialized_ = false;
  ImuSample latest_{};
  mutable std::mutex mutex_;
  std::atomic<bool> healthy_{false};
  double accel_lsb_per_g_ = 682.7;   // recomputed in open() from the range
  double gyro_lsb_per_dps_ = 16.384; // recomputed in open() from the range
};

}  // namespace sensors
}  // namespace ade
