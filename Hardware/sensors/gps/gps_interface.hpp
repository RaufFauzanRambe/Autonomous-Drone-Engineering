// Copyright 2026 Autonomous-Drone-Engineering Contributors
// SPDX-License-Identifier: MIT
//
// u-blox NEO-M9N GNSS driver (Linux UART, UBX protocol).
//
// Receives UBX NAV-PVT solutions at 5 Hz and converts them into the
// navigation-friendly units used by the EKF: degrees, meters, m/s (NED).
// NMEA output is not parsed; configure the receiver in UBX mode (see
// sensor_documentation.md, GNSS section). Reference implementation for
// bench bring-up, HIL rigs, and companion computers.

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

namespace ade {
namespace sensors {

/// Tunables; mirrors `sensors/gps/gps_config.yaml`.
struct GpsConfig {
  /// Serial device node of the receiver (USART1 on the reference FC).
  std::string device = "/dev/ttyAMA1";

  /// Baud rate. The M9N ROM default is 38400; the ADE bench profile sets
  /// 115200 through the u-blox configuration (see sensor_documentation.md).
  int32_t baudrate = 115200;

  /// If true, NAV-PVT messages with fixType < 3 are still published with
  /// the invalid flag set, so consumers can log degraded tracking.
  bool publish_degraded = true;

  /// Drop frames failing the Fletcher-8 checksum, and count them.
  /// Never disable this outside protocol debugging.
  bool verify_checksum = true;
};

/// Fix quality, mirrors the UBX NAV-PVT fixType field.
enum class GpsFixType : uint8_t {
  kNoFix = 0,
  kDeadReckoning = 1,
  kFix2D = 2,
  kFix3D = 3,
  kGnssDeadReckoning = 4,
  kTimeOnly = 5,
};

/// One navigation solution, already in engineering units.
struct GpsFix {
  /// Host monotonic timestamp of the parsed frame [us].
  uint64_t t_us = 0;
  /// UTC time-of-week from the receiver [ms] (debug aid only).
  uint32_t itow_ms = 0;
  /// WGS-84 latitude [deg], positive north.
  double latitude_deg = 0.0;
  /// WGS-84 longitude [deg], positive east.
  double longitude_deg = 0.0;
  /// Height above mean sea level [m].
  double altitude_msl_m = 0.0;
  /// Height above the WGS-84 ellipsoid [m].
  double altitude_ellipsoid_m = 0.0;
  /// Velocity, NED frame [m/s].
  std::array<double, 3> velocity_ned_mps{};
  /// Ground speed [m/s].
  double ground_speed_mps = 0.0;
  /// Course over ground [deg].
  double heading_deg = 0.0;
  /// Reported horizontal accuracy [m].
  double h_accuracy_m = 0.0;
  /// Reported vertical accuracy [m].
  double v_accuracy_m = 0.0;
  /// Reported speed accuracy [m/s].
  double s_accuracy_mps = 0.0;
  /// Position dilution of precision.
  double p_dop = 0.0;
  /// Number of satellites used in the solution.
  uint8_t num_sv = 0;
  GpsFixType fix_type = GpsFixType::kNoFix;
  /// True when the fix is 3D and both flags indicate a valid solution.
  bool valid = false;
};

/// Serial driver for a UBX-mode u-blox receiver.
class GpsInterface {
 public:
  explicit GpsInterface(GpsConfig config = GpsConfig{});
  ~GpsInterface();

  GpsInterface(const GpsInterface&) = delete;
  GpsInterface& operator=(const GpsInterface&) = delete;

  /// Opens the serial port, configures 8N1 raw mode at the configured baud
  /// rate, and drains stale input. Does not change receiver settings.
  bool open();

  /// Closes the serial port.
  void close();

  /// Reads any pending bytes and parses complete UBX frames. Returns the
  /// number of valid NAV-PVT fixes parsed in this call (0 or more).
  int poll();

  /// Most recent NAV-PVT solution.
  GpsFix latest() const;

  /// False after a checksum or framing failure until the next valid frame.
  bool healthy() const { return healthy_.load(std::memory_order_relaxed); }

  /// Total frames accepted / rejected since open(); for link statistics.
  std::pair<uint32_t, uint32_t> frameStats() const {
    return {rx_frames_.load(std::memory_order_relaxed),
            rx_errors_.load(std::memory_order_relaxed)};
  }

 private:
  // Feeds the incremental UBX frame parser; returns 1 when the byte
  // completes a frame that was published, 0 otherwise.
  int feed(uint8_t byte);
  bool handleNavPvt();

  GpsConfig config_;
  int fd_ = -1;

  // Incremental parser state.
  std::array<uint8_t, 128> rx_buf_{};
  uint16_t rx_len_ = 0;        // bytes buffered so far
  uint16_t rx_need_ = 0;       // total frame length once known
  uint8_t ck_a_ = 0;
  uint8_t ck_b_ = 0;

  GpsFix latest_{};
  mutable std::mutex mutex_;
  std::atomic<bool> healthy_{false};
  std::atomic<uint32_t> rx_frames_{0};
  std::atomic<uint32_t> rx_errors_{0};
};

}  // namespace sensors
}  // namespace ade
