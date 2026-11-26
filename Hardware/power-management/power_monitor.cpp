// Copyright 2026 Autonomous-Drone-Engineering Contributors
// SPDX-License-Identifier: MIT
//
// INA226 power-monitor implementation (Linux i2c-dev).
//
// Register map per the INA226 datasheet (Texas Instruments):
//   0x00 CONFIGURATION   0x01 SHUNT_VOLTAGE   0x02 BUS_VOLTAGE
//   0x03 POWER           0x04 CURRENT         0x05 CALIBRATION
//   0x06 MASK/ENABLE     0x07 ALERT_LIMIT     0xFE-0xFF die IDs (0x5449)
// Fixed conversions: bus voltage LSB 1.25 mV; shunt LSB 2.5 uV;
// current = shunt_v * calibration / 2048 [A].
// TODO(datasheet-verify): confirm against your part's datasheet revision.

#include "power-management/power_monitor.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

extern "C" {
#include <linux/i2c-dev.h>  // I2C_SLAVE, I2C_RDWR ioctl numbers
#include <linux/i2c.h>      // i2c_msg, i2c_rdwr_ioctl_data, I2C_M_RD
}

namespace ade {
namespace power {
namespace {

constexpr uint8_t kRegConfig = 0x00;
constexpr uint8_t kRegShuntV = 0x01;
constexpr uint8_t kRegBusV = 0x02;
constexpr uint8_t kRegCurrent = 0x04;
constexpr uint8_t kRegCalibration = 0x05;
constexpr uint8_t kRegDieId = 0xFE;
constexpr uint8_t kRegManufacturerId = 0xFF;
constexpr uint16_t kDieIdTi = 0x5449;    // "TI"
constexpr uint16_t kManufacturerIdTi = 0x4954;

constexpr double kBusVLsb = 1.25e-3;   // V per LSB
constexpr double kShuntVLsb = 2.5e-6;  // V per LSB

double nowUs() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) * 1.0e6 + static_cast<double>(ts.tv_nsec) / 1.0e3;
}

uint16_t u16le(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

// Coulomb-counting integration runs in mAh; dt from consecutive timestamps.
constexpr double kUsToHours = 1.0 / 3.6e9;

}  // namespace

PowerMonitor::PowerMonitor(BatteryConfig battery, MonitorConfig monitor)
    : battery_(battery), monitor_(std::move(monitor)) {}

PowerMonitor::~PowerMonitor() { close(); }

bool PowerMonitor::writeReg(uint8_t reg, uint16_t value) const {
  if (fd_ < 0) {
    return false;
  }
  const uint8_t buf[3] = {reg, static_cast<uint8_t>(value >> 8),
                          static_cast<uint8_t>(value & 0xFF)};
  if (::write(fd_, buf, 3) != 3) {
    std::fprintf(stderr, "power: write reg 0x%02X failed: %s\n", reg,
                 std::strerror(errno));
    return false;
  }
  return true;
}

bool PowerMonitor::readRegs(uint8_t first_reg, uint16_t* out,
                            std::size_t len) const {
  if (fd_ < 0 || len == 0 || len > 8) {
    return false;
  }
  // Combined write-pointer + read transaction.
  uint8_t addr = first_reg;
  uint8_t raw[16] = {0};
  i2c_msg msgs[2] = {};
  msgs[0].addr = monitor_.i2c_address;
  msgs[0].flags = 0;
  msgs[0].len = 1;
  msgs[0].buf = &addr;
  msgs[1].addr = monitor_.i2c_address;
  msgs[1].flags = I2C_M_RD;
  msgs[1].len = static_cast<uint16_t>(len * 2);
  msgs[1].buf = raw;
  i2c_rdwr_ioctl_data ioctl_data = {};
  ioctl_data.msgs = msgs;
  ioctl_data.nmsgs = 2;
  if (ioctl(fd_, I2C_RDWR, &ioctl_data) < 0) {
    std::fprintf(stderr, "power: read reg 0x%02X failed: %s\n", first_reg,
                 std::strerror(errno));
    return false;
  }
  for (std::size_t i = 0; i < len; ++i) {
    out[i] = u16le(raw + 2 * i);
  }
  return true;
}

bool PowerMonitor::open() {
  fd_ = ::open(monitor_.i2c_device.c_str(), O_RDWR);
  if (fd_ < 0) {
    std::fprintf(stderr, "power: cannot open %s: %s\n", monitor_.i2c_device.c_str(),
                 std::strerror(errno));
    return false;
  }
  if (ioctl(fd_, I2C_SLAVE, monitor_.i2c_address) < 0) {
    std::fprintf(stderr, "power: cannot select address 0x%02X: %s\n",
                 monitor_.i2c_address, std::strerror(errno));
    close();
    return false;
  }
  if (!selfTest()) {
    close();
    return false;
  }

  // Current resolution: pick the smallest power-of-two LSB that covers the
  // full-scale current: lsb = 2^ceil(log2(max/2^15)).
  const double exponent = std::ceil(std::log2(monitor_.max_current_a / 32768.0));
  current_lsb_a_ = std::pow(2.0, exponent);
  // Calibration register: 0.00512 / (current_lsb * shunt_r).
  const double cal = 0.00512 / (current_lsb_a_ * monitor_.shunt_r_ohm);
  cal_reg_ = static_cast<uint16_t>(cal);

  // CONFIGURATION: averaging 64 samples, 1.1 ms conversion both channels,
  // shunt+bus continuous. Bits: [15:12] avg=0x7 (64), [11:9] bus ct=0x4
  // (1.1 ms), [8:6] shunt ct=0x4, [3:0] mode 0x7 (continuous both).
  const uint16_t config =
      static_cast<uint16_t>((0x7 << 12) | (0x4 << 9) | (0x4 << 6) | 0x7);
  if (!writeReg(kRegConfig, config) || !writeReg(kRegCalibration, cal_reg_)) {
    close();
    return false;
  }

  resetChargeCounter();
  healthy_.store(true);
  return true;
}

void PowerMonitor::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  healthy_.store(false);
}

bool PowerMonitor::selfTest() {
  uint16_t ids[2] = {0, 0};
  if (!readRegs(kRegDieId, ids, 1) || !readRegs(kRegManufacturerId, ids + 1, 1)) {
    return false;
  }
  if (ids[0] != kDieIdTi || ids[1] != kManufacturerIdTi) {
    std::fprintf(stderr, "power: INA226 ids 0x%04X/0x%04X, expected 0x%04X/0x%04X\n",
                 ids[0], ids[1], kDieIdTi, kManufacturerIdTi);
    return false;
  }
  return true;
}

double PowerMonitor::stateOfChargeFromOcv(double ocv_cell_v) const {
  // Piecewise-linear open-circuit-voltage model for a healthy LiPo cell,
  // anchored on the pack definition's known points; linear in between.
  const double table[][2] = {
      {battery_.empty_cell_v, 0.0},
      {3.50, 0.05},
      {3.60, 0.10},
      {3.70, 0.25},
      {3.80, 0.45},
      {3.90, 0.65},
      {4.00, 0.85},
      {battery_.full_cell_v, 1.0},
  };
  if (ocv_cell_v <= table[0][0]) {
    return 0.0;
  }
  for (std::size_t i = 1; i < sizeof(table) / sizeof(table[0]); ++i) {
    if (ocv_cell_v <= table[i][0]) {
      const double span = table[i][0] - table[i - 1][0];
      const double frac = (ocv_cell_v - table[i - 1][0]) / span;
      return table[i - 1][1] + frac * (table[i][1] - table[i - 1][1]);
    }
  }
  return 1.0;
}

PowerSample PowerMonitor::readBlocking() {
  PowerSample sample{};
  const double t_now = nowUs();
  sample.t_us = static_cast<uint64_t>(t_now);
  if (fd_ < 0) {
    return sample;
  }

  uint16_t regs[2] = {0, 0};
  if (!readRegs(kRegBusV, regs, 1) || !readRegs(kRegShuntV, regs + 1, 1)) {
    healthy_.store(false);
    std::lock_guard<std::mutex> lock(mutex_);
    sample = latest_;
    sample.valid = false;
    return sample;
  }

  // INA226 two's-complement readings.
  const double bus_raw = static_cast<int16_t>(regs[0]);
  const double shunt_raw = static_cast<int16_t>(regs[1]);
  sample.pack_v = bus_raw * kBusVLsb;
  sample.shunt_v = shunt_raw * kShuntVLsb;
  // Current from the calibrated shunt reading: shunt_v / (R * 2048) * cal
  // equals shunt_v / R after calibration math; using the direct Ohm's-law
  // form avoids depending on the truncated calibration register.
  sample.current_a = sample.shunt_v / monitor_.shunt_r_ohm;
  sample.power_w = sample.pack_v * sample.current_a;

  // Load compensation: the LiPo OCV is higher than terminal voltage by the
  // internal-resistance drop at the present current.
  sample.ocv_pack_v = sample.pack_v + sample.current_a * battery_.internal_r_ohm;
  sample.ocv_cell_v = sample.ocv_pack_v / battery_.cells;

  // Sanity: 2S..6S-ish pack window, |I| within the shunt full scale.
  if (sample.pack_v < 6.0 || sample.pack_v > 25.2 ||
      std::fabs(sample.current_a) > monitor_.max_current_a * 1.2) {
    healthy_.store(false);
    std::fprintf(stderr, "power: sample out of range (%.2f V, %.1f A)\n",
                 sample.pack_v, sample.current_a);
    std::lock_guard<std::mutex> lock(mutex_);
    sample = latest_;
    sample.valid = false;
    return sample;
  }

  // Coulomb counting + OCV-based charge estimate, blended.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double dt_h = static_cast<double>(sample.t_us - latest_.t_us) * kUsToHours;
    if (latest_.valid && dt_h > 0.0) {
      // Trapezoidal integration of current over the interval.
      sample.consumed_mah = latest_.consumed_mah +
                            0.5 * (sample.current_a + latest_.current_a) * dt_h * 1000.0;
    } else {
      sample.consumed_mah = latest_.consumed_mah;
    }
    const double coulomb_fraction = 1.0 - sample.consumed_mah /
        (battery_.capacity_mah * battery_.usable_fraction);
    const double ocv_fraction = stateOfChargeFromOcv(sample.ocv_cell_v);
    // Blend: OCV anchors long-term drift, coulomb counting is smooth in
    // the short term. Fresh packs track OCV; drifting packs are pulled.
    double charge = 0.6 * coulomb_fraction + 0.4 * ocv_fraction;
    if (charge < 0.0) {
      charge = 0.0;
    }
    if (charge > 1.0) {
      charge = 1.0;
    }
    sample.charge_fraction = charge;
  }

  sample.valid = true;
  healthy_.store(true);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = sample;
  }
  return sample;
}

PowerSample PowerMonitor::latest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

void PowerMonitor::setBattery(const BatteryConfig& battery) {
  std::lock_guard<std::mutex> lock(mutex_);
  battery_ = battery;
}

int PowerMonitor::batteryLevel() const {
  std::lock_guard<std::mutex> lock(mutex_);
  const double v = latest_.ocv_cell_v;
  if (v < battery_.critical_cell_v) {
    return 3;
  }
  if (v < battery_.land_cell_v) {
    return 2;
  }
  if (v < battery_.warn_cell_v) {
    return 1;
  }
  return 0;
}

void PowerMonitor::resetChargeCounter() {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.consumed_mah = 0.0;
}

}  // namespace power
}  // namespace ade
