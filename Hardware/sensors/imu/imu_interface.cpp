// Copyright 2026 Autonomous-Drone-Engineering Contributors
// SPDX-License-Identifier: MIT
//
// BMI088 IMU driver implementation (Linux spidev).
//
// Register map follows the BMI088 datasheet (Bosch Sensortec). The two dies
// have slightly different SPI conventions: accelerometer register reads
// need one dummy byte after the register address, gyroscope reads do not.
// TODO(datasheet-verify): confirm addresses against the datasheet revision
// of the part on your board before flight use.

#include "sensors/imu/imu_interface.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ade {
namespace sensors {
namespace {

// ---- BMI088 register map --------------------------------------------------
constexpr uint8_t kAccChipIdReg = 0x14;    // expected value 0x1E
constexpr uint8_t kAccChipIdVal = 0x1E;
constexpr uint8_t kAccXoutL = 0x12;        // X LSB, then X MSB, Y, Z (6 bytes)
constexpr uint8_t kAccTempL = 0x22;        // 11-bit temperature
constexpr uint8_t kAccConf = 0x40;         // bandwidth / ODR
constexpr uint8_t kAccRange = 0x41;        // full-scale range
constexpr uint8_t kAccPwrConf = 0x7C;      // 0 = active mode
constexpr uint8_t kAccPwrCtrl = 0x7D;      // bit0 = accel on
constexpr uint8_t kAccSoftReset = 0x7E;    // write 0xB6

constexpr uint8_t kGyroChipIdReg = 0x00;   // expected value 0x0F
constexpr uint8_t kGyroChipIdVal = 0x0F;
constexpr uint8_t kGyroRateXoutL = 0x02;   // X LSB, then X MSB, Y, Z (6 bytes)
constexpr uint8_t kGyroRange = 0x0F;       // full-scale range
constexpr uint8_t kGyroBandwidth = 0x10;   // ODR + filter bandwidth
constexpr uint8_t kGyroLpm1 = 0x11;        // power mode (0 = normal)
constexpr uint8_t kGyroSoftReset = 0x14;   // write 0xB6

// Range/ODR lookup tables: [value, register code].
constexpr int kAccRangeCode[][2] = {
    {3, 0x00}, {6, 0x01}, {12, 0x02}, {24, 0x03}};
constexpr int kGyroRangeCode[][2] = {
    {125, 0x00}, {250, 0x01}, {500, 0x02}, {1000, 0x03}, {2000, 0x04}};

// Accelerometer CONF register ODR encodings (value = bits 3..0).
constexpr int kAccOdrCode[][2] = {
    {12, 0x05}, {25, 0x06}, {50, 0x07}, {100, 0x08},
    {200, 0x09}, {400, 0x0A}, {800, 0x0B}, {1600, 0x0C}};

// Gyroscope BANDWIDTH register: (ODR code << 5) | filter bandwidth.
constexpr int kGyroOdrCode[][2] = {
    {100, 0x00}, {200, 0x01}, {400, 0x02}, {1000, 0x03}, {2000, 0x04}};

constexpr uint8_t kSpiMode = SPI_MODE_3;   // BMI088: CPOL=1, CPHA=1
constexpr uint8_t kSpiBits = 8;
constexpr uint16_t kSpiDelayUs = 0;

constexpr double kGravity = 9.80665;       // m/s^2
constexpr double kDegToRad = M_PI / 180.0;

double nowUs() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) * 1.0e6 + static_cast<double>(ts.tv_nsec) / 1.0e3;
}

}  // namespace

ImuInterface::ImuInterface(ImuConfig config) : config_(std::move(config)) {}

ImuInterface::~ImuInterface() { close(); }

int ImuInterface::spiTransfer(int fd, const uint8_t* tx, uint8_t* rx,
                              std::size_t len) const {
  spi_ioc_transfer xfer{};
  xfer.tx_buf = reinterpret_cast<uint64_t>(tx);
  xfer.rx_buf = reinterpret_cast<uint64_t>(rx);
  xfer.len = static_cast<uint32_t>(len);
  xfer.speed_hz = config_.spi_speed_hz;
  xfer.bits_per_word = kSpiBits;
  xfer.delay_usecs = kSpiDelayUs;
  xfer.cs_change = 0;
  return ioctl(fd, SPI_IOC_MESSAGE(1), &xfer) < 0 ? -1 : 0;
}

bool ImuInterface::readRegs(int fd, uint8_t first_reg, uint8_t* out,
                            std::size_t len, bool dummy_byte) const {
  if (fd < 0 || len == 0 || len > 16) {
    return false;
  }
  std::size_t tx_len = len + 1 + (dummy_byte ? 1 : 0);
  uint8_t tx[18] = {0};
  uint8_t rx[18] = {0};
  tx[0] = first_reg | 0x80;  // read flag, MSB set
  if (spiTransfer(fd, tx, rx, tx_len) != 0) {
    std::fprintf(stderr, "imu: SPI read of reg 0x%02X failed: %s\n", first_reg,
                 std::strerror(errno));
    return false;
  }
  std::memcpy(out, rx + 1 + (dummy_byte ? 1 : 0), len);
  return true;
}

bool ImuInterface::writeReg(int fd, uint8_t reg, uint8_t value,
                            bool dummy_byte) const {
  if (fd < 0) {
    return false;
  }
  // Writes carry the register address with the read flag cleared; the
  // accelerometer also expects a dummy byte on write transactions.
  std::size_t len = dummy_byte ? 3 : 2;
  uint8_t tx[3] = {static_cast<uint8_t>(reg & 0x7F), value, 0x00};
  uint8_t rx[3] = {0};
  if (spiTransfer(fd, tx, rx, len) != 0) {
    std::fprintf(stderr, "imu: SPI write to reg 0x%02X failed: %s\n", reg,
                 std::strerror(errno));
    return false;
  }
  return true;
}

bool ImuInterface::open() {
  accel_fd_ = ::open(config_.accel_device.c_str(), O_RDWR);
  gyro_fd_ = ::open(config_.gyro_device.c_str(), O_RDWR);
  if (accel_fd_ < 0 || gyro_fd_ < 0) {
    std::fprintf(stderr, "imu: cannot open %s or %s: %s\n",
                 config_.accel_device.c_str(), config_.gyro_device.c_str(),
                 std::strerror(errno));
    close();
    return false;
  }
  const uint8_t mode = kSpiMode;
  const uint8_t bits = kSpiBits;
  uint32_t speed = config_.spi_speed_hz;
  if (ioctl(accel_fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
      ioctl(accel_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
      ioctl(accel_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0 ||
      ioctl(gyro_fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
      ioctl(gyro_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
      ioctl(gyro_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
    std::fprintf(stderr, "imu: SPI mode setup failed: %s\n", std::strerror(errno));
    close();
    return false;
  }

  // Soft reset both dies, then wait for boot (accelerometer needs 1 ms,
  // gyroscope 30 ms, after its reset the gyro enters normal mode itself).
  writeReg(accel_fd_, kAccSoftReset, 0xB6, false);
  writeReg(gyro_fd_, kGyroSoftReset, 0xB6, false);
  usleep(50000);

  // Power up the accelerometer: active mode, then accel-on.
  if (!writeReg(accel_fd_, kAccPwrConf, 0x00, false) ||
      !writeReg(accel_fd_, kAccPwrCtrl, 0x04, false)) {
    close();
    return false;
  }
  usleep(50000);

  if (!selfTest()) {
    close();
    return false;
  }
  if (!configureAccel() || !configureGyro()) {
    close();
    return false;
  }

  // Scale factors: signed 16-bit full scale symmetric about zero.
  accel_lsb_per_g_ = 32768.0 / (2 * config_.accel_range_g);
  gyro_lsb_per_dps_ = 32768.0 / (2 * config_.gyro_range_dps);
  lpf_initialized_ = false;

  if (!config_.calibration_file.empty()) {
    FILE* f = std::fopen(config_.calibration_file.c_str(), "r");
    if (f != nullptr) {
      double bx = 0.0, by = 0.0, bz = 0.0;
      if (std::fscanf(f, "%lf %lf %lf", &bx, &by, &bz) == 3) {
        gyro_bias_ = {bx, by, bz};
      } else {
        std::fprintf(stderr, "imu: bad calibration file %s\n",
                     config_.calibration_file.c_str());
      }
      std::fclose(f);
    } else {
      std::fprintf(stderr, "imu: cannot read calibration file %s: %s\n",
                   config_.calibration_file.c_str(), std::strerror(errno));
    }
  }

  healthy_.store(true);
  return true;
}

void ImuInterface::close() {
  if (accel_fd_ >= 0) {
    ::close(accel_fd_);
    accel_fd_ = -1;
  }
  if (gyro_fd_ >= 0) {
    ::close(gyro_fd_);
    gyro_fd_ = -1;
  }
  healthy_.store(false);
}

bool ImuInterface::selfTest() {
  uint8_t acc_id = 0;
  uint8_t gyro_id = 0;
  const bool ok_acc =
      readRegs(accel_fd_, kAccChipIdReg, &acc_id, 1, true) && acc_id == kAccChipIdVal;
  const bool ok_gyro =
      readRegs(gyro_fd_, kGyroChipIdReg, &gyro_id, 1, false) && gyro_id == kGyroChipIdVal;
  if (!ok_acc) {
    std::fprintf(stderr, "imu: accelerometer chip id 0x%02X, expected 0x%02X\n",
                 acc_id, kAccChipIdVal);
  }
  if (!ok_gyro) {
    std::fprintf(stderr, "imu: gyroscope chip id 0x%02X, expected 0x%02X\n", gyro_id,
                 kGyroChipIdVal);
  }
  return ok_acc && ok_gyro;
}

bool ImuInterface::configureAccel() {
  uint8_t range_code = 0x03;  // default to +-24 g
  for (const auto& entry : kAccRangeCode) {
    if (entry[0] == config_.accel_range_g) {
      range_code = static_cast<uint8_t>(entry[1]);
      break;
    }
  }
  uint8_t odr_code = 0x0A;  // default 400 Hz
  for (const auto& entry : kAccOdrCode) {
    if (entry[0] == config_.accel_odr_hz) {
      odr_code = static_cast<uint8_t>(entry[1]);
      break;
    }
  }
  // CONF: ODR in the low nibble, bandwidth bits 4-5 (normal mode).
  const uint8_t conf = static_cast<uint8_t>((0x01 << 4) | (odr_code & 0x0F));
  return writeReg(accel_fd_, kAccConf, conf, false) &&
         writeReg(accel_fd_, kAccRange, range_code, false);
}

bool ImuInterface::configureGyro() {
  uint8_t range_code = 0x04;  // default +-2000 dps
  for (const auto& entry : kGyroRangeCode) {
    if (entry[0] == config_.gyro_range_dps) {
      range_code = static_cast<uint8_t>(entry[1]);
      break;
    }
  }
  int odr_code = 0x03;  // default 1000 Hz ODR
  for (const auto& entry : kGyroOdrCode) {
    if (entry[0] == config_.gyro_odr_hz) {
      odr_code = entry[1];
      break;
    }
  }
  // BANDWIDTH: ODR bits 4-7, filter bandwidth 0x00 (ODR/2, fastest settling).
  const uint8_t bw =
      static_cast<uint8_t>(((odr_code & 0x07) << 4) | 0x00);
  return writeReg(gyro_fd_, kGyroLpm1, 0x00, false) &&
         writeReg(gyro_fd_, kGyroRange, range_code, false) &&
         writeReg(gyro_fd_, kGyroBandwidth, bw, false);
}

double ImuInterface::readTemperatureC() {
  // 11-bit two's-complement, 0.125 K per LSB, offset per datasheet.
  uint8_t raw[2] = {0};
  if (!readRegs(accel_fd_, kAccTempL, raw, 2, true)) {
    return 0.0;
  }
  const uint16_t word = static_cast<uint16_t>(raw[0] << 3) | (raw[1] >> 5);
  int16_t value = static_cast<int16_t>(word << 4) >> 4;  // sign-extend 11 bits
  return static_cast<double>(value) * 0.125 + 23.0;
}

bool ImuInterface::calibrateBias(int num_samples) {
  if (accel_fd_ < 0 || gyro_fd_ < 0) {
    return false;
  }
  std::array<double, 3> acc{};  // accumulated gyro rates, rad/s
  int valid = 0;
  std::fprintf(stderr, "imu: calibrating gyro bias, keep still...\n");
  for (int i = 0; i < num_samples; ++i) {
    ImuSample s = readBlocking();
    if (s.valid) {
      acc[0] += s.gyro_radps[0];
      acc[1] += s.gyro_radps[1];
      acc[2] += s.gyro_radps[2];
      ++valid;
    }
    usleep(2000);
  }
  if (valid < num_samples / 2) {
    std::fprintf(stderr, "imu: only %d/%d valid samples, calibration failed\n",
                 valid, num_samples);
    return false;
  }
  const double n = static_cast<double>(valid);
  for (int k = 0; k < 3; ++k) {
    gyro_bias_[k] = acc[k] / n;
  }
  std::fprintf(stderr, "imu: gyro bias [rad/s] %.6f %.6f %.6f\n",
               gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
  return true;
}

ImuSample ImuInterface::readBlocking() {
  ImuSample sample{};
  sample.t_us = static_cast<uint64_t>(nowUs());
  if (accel_fd_ < 0 || gyro_fd_ < 0) {
    return sample;
  }

  uint8_t acc_raw[6] = {0};
  uint8_t gyro_raw[6] = {0};
  const bool acc_ok = readRegs(accel_fd_, kAccXoutL, acc_raw, 6, true);
  const bool gyro_ok = readRegs(gyro_fd_, kGyroRateXoutL, gyro_raw, 6, false);
  if (!acc_ok || !gyro_ok) {
    healthy_.store(false);
    std::lock_guard<std::mutex> lock(mutex_);
    sample = latest_;
    sample.valid = false;
    return sample;
  }

  auto word16 = [](const uint8_t* p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                (static_cast<uint16_t>(p[1]) << 8));
  };
  for (int k = 0; k < 3; ++k) {
    const double a_counts = static_cast<double>(word16(acc_raw + 2 * k));
    const double g_counts = static_cast<double>(word16(gyro_raw + 2 * k));
    sample.accel_mps2[k] = a_counts / accel_lsb_per_g_ * kGravity;
    sample.gyro_radps[k] =
        (g_counts / gyro_lsb_per_dps_) * kDegToRad - gyro_bias_[k];
  }
  sample.temperature_c = readTemperatureC();

  // Sanity: specific force within +-16 g of zero, rates within +-2500 dps.
  for (int k = 0; k < 3; ++k) {
    if (std::fabs(sample.accel_mps2[k]) > 16.0 * kGravity ||
        std::fabs(sample.gyro_radps[k]) > 2500.0 * kDegToRad) {
      sample.valid = false;
      healthy_.store(false);
      return sample;
    }
  }

  // First-order low-pass on converted samples (per axis).
  if (!lpf_initialized_) {
    accel_lpf_ = sample.accel_mps2;
    gyro_lpf_ = sample.gyro_radps;
    lpf_initialized_ = true;
  } else {
    const double a_alpha =
        1.0 - std::exp(-2.0 * M_PI * config_.accel_lpf_hz / config_.accel_odr_hz);
    const double g_alpha =
        1.0 - std::exp(-2.0 * M_PI * config_.gyro_lpf_hz / config_.gyro_odr_hz);
    for (int k = 0; k < 3; ++k) {
      accel_lpf_[k] += a_alpha * (sample.accel_mps2[k] - accel_lpf_[k]);
      gyro_lpf_[k] += g_alpha * (sample.gyro_radps[k] - gyro_lpf_[k]);
      sample.accel_mps2[k] = accel_lpf_[k];
      sample.gyro_radps[k] = gyro_lpf_[k];
    }
  }

  sample.valid = true;
  healthy_.store(true);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = sample;
  }
  return sample;
}

ImuSample ImuInterface::latest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

}  // namespace sensors
}  // namespace ade
