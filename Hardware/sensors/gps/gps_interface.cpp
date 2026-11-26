// Copyright 2026 Autonomous-Drone-Engineering Contributors
// SPDX-License-Identifier: MIT
//
// u-blox GNSS driver implementation (Linux termios, UBX framing).
//
// UBX protocol summary: every frame is
//   B5 62 <class> <id> <len:u16 LE> <payload[len]> <ck_a> <ck_b>
// where ck_a/ck_b are Fletcher-8 checksums over class, id, length, and
// payload. NAV-PVT is class 0x01, id 0x07, 92-byte payload.
// Field offsets below follow the u-blox M8/M9 receiver description;
// TODO(datasheet-verify): confirm against your receiver firmware version.

#include "sensors/gps/gps_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <utility>

namespace ade {
namespace sensors {
namespace {

constexpr uint8_t kUbxSync1 = 0xB5;
constexpr uint8_t kUbxSync2 = 0x62;
constexpr uint8_t kNavPvtClass = 0x01;
constexpr uint8_t kNavPvtId = 0x07;
constexpr uint16_t kNavPvtLen = 92;

double nowUs() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) * 1.0e6 + static_cast<double>(ts.tv_nsec) / 1.0e3;
}

speed_t baudConstant(int32_t baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return 0;  // unsupported
  }
}

// Little-endian field readers over the payload buffer.
uint8_t u1(const uint8_t* p) { return p[0]; }
uint16_t u2(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}
uint32_t u4(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
int32_t s4(const uint8_t* p) { return static_cast<int32_t>(u4(p)); }

}  // namespace

GpsInterface::GpsInterface(GpsConfig config) : config_(std::move(config)) {}

GpsInterface::~GpsInterface() { close(); }

bool GpsInterface::open() {
  fd_ = ::open(config_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    std::fprintf(stderr, "gps: cannot open %s: %s\n", config_.device.c_str(),
                 std::strerror(errno));
    return false;
  }
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    std::fprintf(stderr, "gps: tcgetattr failed: %s\n", std::strerror(errno));
    close();
    return false;
  }
  const speed_t speed = baudConstant(config_.baudrate);
  if (speed == 0) {
    std::fprintf(stderr, "gps: unsupported baud rate %d\n", config_.baudrate);
    close();
    return false;
  }
  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);   // ignore modem lines, enable receiver
  tio.c_cflag &= ~CSTOPB;            // 1 stop bit
  tio.c_cflag &= ~CRTSCTS;           // no hardware flow control
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;               // fully non-blocking reads
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    std::fprintf(stderr, "gps: serial setup failed: %s\n", std::strerror(errno));
    close();
    return false;
  }
  tcflush(fd_, TCIFLUSH);  // drop stale bytes

  rx_len_ = 0;
  rx_need_ = 0;
  healthy_.store(true);
  return true;
}

void GpsInterface::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  healthy_.store(false);
}

int GpsInterface::poll() {
  if (fd_ < 0) {
    return 0;
  }
  int fixes = 0;
  uint8_t chunk[256];
  for (;;) {
    const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
    if (n <= 0) {
      if (n < 0 && errno != EAGAIN && errno != EINTR) {
        healthy_.store(false);
        std::fprintf(stderr, "gps: read error: %s\n", std::strerror(errno));
      }
      break;
    }
    for (ssize_t i = 0; i < n; ++i) {
      fixes += feed(static_cast<uint8_t>(chunk[i]));
    }
  }
  return fixes;
}

int GpsInterface::feed(uint8_t byte) {
  // The buffer always holds the frame in progress, starting with B5 62.
  if (rx_len_ == 0 && byte != kUbxSync1) {
    return 0;  // idle until sync word 1
  }
  if (rx_len_ == 1 && byte != kUbxSync2) {
    rx_len_ = 0;
    return 0;
  }
  if (rx_len_ >= rx_buf_.size()) {
    rx_len_ = 0;  // runaway frame; resync
    return 0;
  }
  rx_buf_[rx_len_++] = byte;

  // Header complete: class, id, length known -> expected total frame size.
  if (rx_need_ == 0 && rx_len_ == 6) {
    const uint16_t payload_len = u2(rx_buf_.data() + 4);
    if (payload_len > rx_buf_.size() - 8) {
      rx_len_ = 0;  // too long for this parser; resync
      return 0;
    }
    rx_need_ = static_cast<uint16_t>(8 + payload_len);
  }
  if (rx_need_ == 0 || rx_len_ < rx_need_) {
    return 0;  // header incomplete or frame in progress
  }

  // Frame complete: verify Fletcher-8 over bytes [2, len+6).
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  for (uint16_t i = 2; i < 6 + (rx_need_ - 8); ++i) {
    ck_a += rx_buf_[i];
    ck_b += ck_a;
  }
  const uint8_t wire_a = rx_buf_[rx_need_ - 2];
  const uint8_t wire_b = rx_buf_[rx_need_ - 1];
  const bool crc_ok = (ck_a == wire_a) && (ck_b == wire_b);
  if (!crc_ok) {
    if (config_.verify_checksum) {
      rx_errors_.fetch_add(1, std::memory_order_relaxed);
      healthy_.store(false);
      std::fprintf(stderr, "gps: UBX checksum error (class 0x%02X id 0x%02X)\n",
                   rx_buf_[2], rx_buf_[3]);
    }
    rx_len_ = 0;
    rx_need_ = 0;
    return 0;
  }

  rx_frames_.fetch_add(1, std::memory_order_relaxed);
  const uint8_t msg_class = rx_buf_[2];
  const uint8_t msg_id = rx_buf_[3];
  int published = 0;
  if (msg_class == kNavPvtClass && msg_id == kNavPvtId &&
      u2(rx_buf_.data() + 4) == kNavPvtLen) {
    published = handleNavPvt() ? 1 : 0;
  }
  // Other frame types are validated and dropped; extend here as needed.

  rx_len_ = 0;
  rx_need_ = 0;
  return published;
}

bool GpsInterface::handleNavPvt() {
  const uint8_t* p = rx_buf_.data() + 6;  // payload base

  GpsFix fix{};
  fix.t_us = static_cast<uint64_t>(nowUs());
  fix.itow_ms = u4(p + 0);
  fix.longitude_deg = static_cast<double>(s4(p + 16)) * 1.0e-7;
  fix.latitude_deg = static_cast<double>(s4(p + 20)) * 1.0e-7;
  fix.altitude_ellipsoid_m = static_cast<double>(s4(p + 24)) / 1000.0;
  fix.altitude_msl_m = static_cast<double>(s4(p + 28)) / 1000.0;
  fix.h_accuracy_m = static_cast<double>(u4(p + 32)) / 1000.0;
  fix.v_accuracy_m = static_cast<double>(u4(p + 36)) / 1000.0;
  fix.velocity_ned_mps = {
      static_cast<double>(s4(p + 40)) / 1000.0,
      static_cast<double>(s4(p + 44)) / 1000.0,
      static_cast<double>(s4(p + 48)) / 1000.0};
  fix.ground_speed_mps = static_cast<double>(s4(p + 52)) / 1000.0;
  fix.heading_deg = static_cast<double>(s4(p + 56)) * 1.0e-5;
  fix.s_accuracy_mps = static_cast<double>(u4(p + 60)) / 1000.0;
  fix.p_dop = static_cast<double>(u2(p + 68)) / 100.0;
  // Tail fields: fixType, flags, numSV. TODO(datasheet-verify): these
  // offsets are stable across M8/M9 firmware we validated against; check
  // your receiver description if num_sv reads implausibly.
  fix.fix_type = static_cast<GpsFixType>(u1(p + 70));
  const uint8_t flags = u1(p + 71);
  fix.num_sv = u1(p + 72);

  fix.valid = (fix.fix_type == GpsFixType::kFix3D ||
               fix.fix_type == GpsFixType::kGnssDeadReckoning) &&
              (flags & 0x01) != 0;  // bit0: fix is valid

  const bool publish = fix.valid || config_.publish_degraded;
  if (!publish) {
    return false;
  }
  if (fix.valid) {
    healthy_.store(true);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = fix;
  }
  return true;
}

GpsFix GpsInterface::latest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

}  // namespace sensors
}  // namespace ade
