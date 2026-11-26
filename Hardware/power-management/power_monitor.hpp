// Copyright 2026 Autonomous-Drone-Engineering Contributors
// SPDX-License-Identifier: MIT
//
// Battery and power-monitoring driver: Texas Instruments INA226 shunt
// monitor over I2C (Linux i2c-dev).
//
// Measures pack voltage, current, and power, and turns them into the
// quantities the failsafe chain needs: load-compensated cell voltage,
// state of charge, and consumed milliamp-hours. Reference implementation
// for bench bring-up, HIL rigs, and companion computers.

#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

namespace ade {
namespace power {

/// Battery pack definition; mirrors `power-management/battery_config.yaml`.
/// The reference aircraft flies a 4S 3300 mAh LiPo.
struct BatteryConfig {
  /// Number of series cells (4S).
  int cells = 4;

  /// Nominal capacity [mAh].
  double capacity_mah = 3300.0;

  /// Total pack internal resistance [ohm] (measured, includes connectors).
  double internal_r_ohm = 0.008;

  /// Cell voltage thresholds, load-compensated [V/cell].
  double full_cell_v = 4.20;
  double warn_cell_v = 3.55;
  double land_cell_v = 3.45;
  double critical_cell_v = 3.30;
  double empty_cell_v = 3.20;

  /// Fraction of nominal capacity considered usable (LiPo longevity).
  double usable_fraction = 0.80;
};

/// Tunables for the monitor itself.
struct MonitorConfig {
  /// I2C bus and sensor address (INA228/INA226 A0..A1 pins: GND-GND=0x40).
  std::string i2c_device = "/dev/i2c-1";
  uint8_t i2c_address = 0x40;

  /// Shunt resistance [ohm] and full-scale current [A] it was sized for.
  /// Reference hardware: 0.5 mOhm / 100 A.
  double shunt_r_ohm = 0.0005;
  double max_current_a = 100.0;
};

/// One measurement of the battery bus.
struct PowerSample {
  /// Host monotonic timestamp [us].
  uint64_t t_us = 0;
  /// Pack (bus) voltage [V].
  double pack_v = 0.0;
  /// Shunt voltage [V].
  double shunt_v = 0.0;
  /// Current flowing out of the battery [A] (positive = discharge).
  double current_a = 0.0;
  /// Instantaneous power [W].
  double power_w = 0.0;
  /// Load-compensated (open-circuit) pack voltage [V].
  double ocv_pack_v = 0.0;
  /// Load-compensated per-cell voltage [V].
  double ocv_cell_v = 0.0;
  /// Coulomb-counted charge consumed since reset [mAh].
  double consumed_mah = 0.0;
  /// Estimated remaining charge fraction [0..1], from OCV + coulomb count.
  double charge_fraction = 0.0;
  /// True when the read passed sanity checks.
  bool valid = false;
};

/// I2C driver for the INA226 battery monitor.
class PowerMonitor {
 public:
  PowerMonitor(BatteryConfig battery = BatteryConfig{},
               MonitorConfig monitor = MonitorConfig{});
  ~PowerMonitor();

  PowerMonitor(const PowerMonitor&) = delete;
  PowerMonitor& operator=(const PowerMonitor&) = delete;

  /// Opens the bus, programs the INA226 configuration (conversion times,
  /// averaging, calibration register), and resets the coulomb counter.
  bool open();

  /// Resets the sensor and closes the bus.
  void close();

  /// Reads the manufacturer/EEPROM-affixed die ID registers.
  bool selfTest();

  /// One blocking measurement, converted and compensated.
  PowerSample readBlocking();

  /// Most recent valid sample without bus traffic.
  PowerSample latest() const;

  /// False after any I2C failure until the next successful read.
  bool healthy() const { return healthy_.load(std::memory_order_relaxed); }

  /// Sets a fresh battery definition (e.g. when swapping packs).
  void setBattery(const BatteryConfig& battery);

  /// Highest failsafe level currently exceeded: 0 none, 1 warn, 2 land,
  /// 3 critical. Based on the last sample's compensated cell voltage.
  int batteryLevel() const;

  /// Resets the coulomb counter (call after charging or pack swap).
  void resetChargeCounter();

 private:
  bool writeReg(uint8_t reg, uint16_t value) const;
  bool readRegs(uint8_t first_reg, uint16_t* out, std::size_t len) const;
  double stateOfChargeFromOcv(double ocv_cell_v) const;

  BatteryConfig battery_;
  MonitorConfig monitor_;
  int fd_ = -1;
  double current_lsb_a_ = 0.0;    // computed from shunt and full scale
  uint16_t cal_reg_ = 0;

  PowerSample latest_{};
  mutable std::mutex mutex_;
  std::atomic<bool> healthy_{false};
};

}  // namespace power
}  // namespace ade
