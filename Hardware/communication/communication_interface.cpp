// Copyright 2026 Autonomous-Drone-Engineering Contributors
// SPDX-License-Identifier: MIT
//
// Telemetry link implementation (Linux termios, MAVLink v2 framing).
//
// MAVLink v2 frame layout (little-endian):
//   0xFD len incompat compat seq sysid compid msgid(3) payload crc_a crc_b
// - len is the payload length (0-255).
// - incompat bit0: 13-byte signature follows the checksum.
// - The checksum is CRC-16/X.25 (init 0xFFFF, poly 0x1021 reflected,
//   final XOR 0xFFFF), seeded with the message's CRC_EXTRA byte, computed
//   from 'len' through the payload.
// Known CRC_EXTRA values are in the table below; extend it as more
// message types are handled. RADIO_STATUS rssi scaling follows the SiK
// radio documentation: dBm = rssi/1.9 - 127 (remote likewise).

#include "communication/communication_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace ade {
namespace comm {
namespace {

constexpr uint8_t kMavV2Stx = 0xFD;
constexpr uint8_t kMavV1Stx = 0xFE;   // detected, dropped (v1 unsupported)

constexpr uint8_t kMsgHeartbeat = 0;
constexpr uint8_t kMsgRadioStatus = 109;
constexpr uint8_t kCrcHeartbeat = 50;    // HEARTBEAT crc_extra
constexpr uint8_t kCrcRadioStatus = 166; // RADIO_STATUS crc_extra
// TODO(protocol-verify): crc_extra values are from the MAVLink v2
// message definitions; cross-check when adding message types.

// Heartbeat payload: custom_mode(4) type(1) autopilot(1) base_mode(1)
// system_status(1) mavlink_version(1) = 9 bytes.
constexpr uint16_t kHeartbeatLen = 9;

// CRC-16/X.25 over a byte stream, starting from 'seed_state'.
uint16_t crcX25(const uint8_t* data, std::size_t len, uint16_t seed_state) {
  uint16_t crc = seed_state;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408)
                      : static_cast<uint16_t>(crc >> 1);
    }
  }
  return static_cast<uint16_t>(crc ^ 0xFFFF);
}

speed_t baudConstant(int32_t baud) {
  switch (baud) {
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return 0;
  }
}

}  // namespace

TelemetryLink::TelemetryLink(TelemetryConfig config)
    : config_(std::move(config)) {}

TelemetryLink::~TelemetryLink() { close(); }

bool TelemetryLink::open() {
  fd_ = ::open(config_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    std::fprintf(stderr, "telemetry: cannot open %s: %s\n", config_.device.c_str(),
                 std::strerror(errno));
    return false;
  }
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    std::fprintf(stderr, "telemetry: tcgetattr failed: %s\n", std::strerror(errno));
    close();
    return false;
  }
  const speed_t speed = baudConstant(config_.baudrate);
  if (speed == 0) {
    std::fprintf(stderr, "telemetry: unsupported baud rate %d\n", config_.baudrate);
    close();
    return false;
  }
  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    std::fprintf(stderr, "telemetry: serial setup failed: %s\n", std::strerror(errno));
    close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);

  rx_len_ = 0;
  rx_need_ = 0;
  tx_sequence_ = 0;
  healthy_.store(true);
  return true;
}

void TelemetryLink::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  healthy_.store(false);
}

void TelemetryLink::setMessageHandler(MessageHandler handler) {
  std::lock_guard<std::mutex> lock(mutex_);
  handler_ = std::move(handler);
}

int TelemetryLink::poll() {
  if (fd_ < 0) {
    return 0;
  }
  int frames = 0;
  uint8_t chunk[512];
  for (;;) {
    const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
    if (n <= 0) {
      if (n < 0 && errno != EAGAIN && errno != EINTR) {
        healthy_.store(false);
        std::fprintf(stderr, "telemetry: read error: %s\n", std::strerror(errno));
      }
      break;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.rx_bytes += static_cast<uint64_t>(n);
    }
    for (ssize_t i = 0; i < n; ++i) {
      frames += feed(static_cast<uint8_t>(chunk[i]));
    }
  }
  return frames;
}

// Returns 1 when this byte completed a dispatched frame, else 0.
int TelemetryLink::feed(uint8_t byte) {
  // MAVLink v1 markers are dropped so v1 noise cannot jam the parser.
  if (rx_len_ == 0 && (byte == kMavV1Stx || byte != kMavV2Stx)) {
    return 0;
  }
  if (rx_len_ >= sizeof(rx_buf_)) {
    rx_len_ = 0;
    return 0;
  }
  rx_buf_[rx_len_++] = byte;

  // Full header: compute the expected frame size. Header is 10 bytes,
  // payload 'len', checksum 2, plus 13 more if the signature flag is set.
  if (rx_len_ == 10) {
    const uint8_t len = rx_buf_[1];
    const uint8_t incompat = rx_buf_[2];
    const uint16_t total = static_cast<uint16_t>(10 + len + 2 +
                                                ((incompat & 0x01) ? 13 : 0));
    if (total > sizeof(rx_buf_)) {
      rx_len_ = 0;  // malformed; resync
      return 0;
    }
    rx_need_ = total;
  }
  if (rx_need_ == 0 || rx_len_ < rx_need_) {
    return 0;
  }

  int dispatched = 0;
  if ((rx_buf_[2] & 0x01) && config_.reject_signed_frames) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.crc_errors;  // counted as dropped frames
  } else {
    dispatched = handleFrame() ? 1 : 0;
  }

  rx_len_ = 0;
  rx_need_ = 0;
  return dispatched;
}

// Verifies and dispatches one complete frame. Returns true if the frame
// was forwarded to the handler.
bool TelemetryLink::handleFrame() {
  const uint8_t len = rx_buf_[1];
  const uint8_t seq = rx_buf_[4];
  const uint8_t sysid = rx_buf_[5];
  const uint8_t compid = rx_buf_[6];
  const uint32_t msgid = static_cast<uint32_t>(rx_buf_[7]) |
                         (static_cast<uint32_t>(rx_buf_[8]) << 8) |
                         (static_cast<uint32_t>(rx_buf_[9]) << 16);
  const uint8_t* payload = rx_buf_ + 10;
  const uint8_t wire_a = rx_buf_[10 + len];
  const uint8_t wire_b = rx_buf_[10 + len + 1];

  // CRC verification needs the message's crc_extra seed. We only know the
  // seeds of the messages this driver handles; unknown message types are
  // forwarded with crc_ok=false (unverifiable, not corrupt).
  uint8_t crc_extra = 0;
  bool crc_extra_known = false;
  if (msgid == kMsgHeartbeat) {
    crc_extra = kCrcHeartbeat;
    crc_extra_known = true;
  } else if (msgid == kMsgRadioStatus) {
    crc_extra = kCrcRadioStatus;
    crc_extra_known = true;
  }

  bool crc_ok = false;
  if (crc_extra_known) {
    // CRC over bytes [1, 10+len) continued over the crc_extra seed byte.
    const uint16_t crc = crcX25(rx_buf_ + 1, 9 + len, 0xFFFF);
    const uint16_t seeded =
        crcX25(&crc_extra, 1, static_cast<uint16_t>(crc ^ 0xFFFF));
    crc_ok = (static_cast<uint8_t>(seeded & 0xFF) == wire_a) &&
             (static_cast<uint8_t>(seeded >> 8) == wire_b);
    if (config_.verify_checksum && !crc_ok) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.crc_errors;
      healthy_.store(false);
      return false;  // corrupt frame: dropped and counted
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.rx_frames;
  }
  healthy_.store(true);

  if (msgid == kMsgRadioStatus && len >= 5) {
    handleRadioStatus(payload);
  }

  MessageHandler local_handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_handler = handler_;
  }
  if (local_handler) {
    MavlinkFrame frame;
    frame.system_id = sysid;
    frame.component_id = compid;
    frame.message_id = msgid;
    frame.sequence = seq;
    frame.payload_len = len;
    frame.payload.assign(reinterpret_cast<const char*>(payload), len);
    frame.crc_ok = crc_ok;
    local_handler(frame);
  }
  return true;
}

void TelemetryLink::handleRadioStatus(const uint8_t* payload) {
  // RADIO_STATUS: rssi, remrssi, txbuf, noise, remnoise (all u1).
  const double rssi = static_cast<double>(payload[0]) / 1.9 - 127.0;
  const double remrssi = static_cast<double>(payload[1]) / 1.9 - 127.0;
  const uint8_t rxbuf = payload[2];
  std::lock_guard<std::mutex> lock(mutex_);
  stats_.rssi_dbm = rssi;
  stats_.remote_rssi_dbm = remrssi;
  stats_.remote_rxbuf_pct = rxbuf;
}

bool TelemetryLink::sendMessage(uint32_t message_id, const uint8_t* payload,
                                uint16_t payload_len, uint8_t crc_extra) {
  if (fd_ < 0 || payload_len > 255) {
    return false;
  }
  uint8_t frame[280] = {};
  uint16_t idx = 0;
  const uint8_t seq = tx_sequence_++;
  frame[idx++] = kMavV2Stx;
  frame[idx++] = static_cast<uint8_t>(payload_len);
  frame[idx++] = 0x00;  // incompat: no flags (unsigned)
  frame[idx++] = 0x00;  // compat
  frame[idx++] = seq;
  frame[idx++] = config_.system_id;
  frame[idx++] = config_.component_id;
  frame[idx++] = static_cast<uint8_t>(message_id & 0xFF);
  frame[idx++] = static_cast<uint8_t>((message_id >> 8) & 0xFF);
  frame[idx++] = static_cast<uint8_t>((message_id >> 16) & 0xFF);
  if (payload_len > 0 && payload != nullptr) {
    std::memcpy(frame + idx, payload, payload_len);
  }
  idx = static_cast<uint16_t>(idx + payload_len);

  // CRC over bytes [1, 10+len) then the crc_extra seed byte.
  const uint16_t crc = crcX25(frame + 1, 9 + payload_len, 0xFFFF);
  const uint16_t seeded = crcX25(&crc_extra, 1, static_cast<uint16_t>(crc ^ 0xFFFF));
  frame[idx++] = static_cast<uint8_t>(seeded & 0xFF);
  frame[idx++] = static_cast<uint8_t>(seeded >> 8);

  const ssize_t written = ::write(fd_, frame, idx);
  if (written != idx) {
    healthy_.store(false);
    std::fprintf(stderr, "telemetry: short write (%zd of %u): %s\n", written,
                 idx, std::strerror(errno));
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.tx_frames;
  }
  return true;
}

bool TelemetryLink::sendHeartbeat(uint8_t base_mode, uint8_t system_status,
                                  uint32_t custom_mode) {
  uint8_t payload[kHeartbeatLen] = {};
  payload[0] = static_cast<uint8_t>(custom_mode & 0xFF);
  payload[1] = static_cast<uint8_t>((custom_mode >> 8) & 0xFF);
  payload[2] = static_cast<uint8_t>((custom_mode >> 16) & 0xFF);
  payload[3] = static_cast<uint8_t>((custom_mode >> 24) & 0xFF);
  payload[4] = 2;   // type: MAV_TYPE_QUADROTOR
  payload[5] = 0;   // autopilot: MAV_AUTOPILOT_GENERIC
  payload[6] = base_mode;
  payload[7] = system_status;
  payload[8] = 3;   // mavlink_version: 3 (v2 wire)
  return sendMessage(kMsgHeartbeat, payload, kHeartbeatLen, kCrcHeartbeat);
}

LinkStats TelemetryLink::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

}  // namespace comm
}  // namespace ade
