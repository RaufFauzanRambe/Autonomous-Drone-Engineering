// Copyright 2026 Autonomous-Drone-Engineering Contributors
// SPDX-License-Identifier: MIT
//
// Telemetry link driver: MAVLink v2 framing over a SiK/RFD900-class
// 900 MHz serial radio (Linux UART).
//
// The driver owns the serial link: it transmits framed MAVLink v2 messages
// with sequence numbering, parses incoming frames with checksum
// verification, and maintains link statistics (RSSI from RADIO_STATUS,
// frame counters, CRC errors). Reference implementation for ground
// stations, HIL rigs, and companion computers.

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

namespace ade {
namespace comm {

/// Tunables; mirrors `communication/communication_config.yaml`.
struct TelemetryConfig {
  /// Serial device node of the airside radio (USART3 on the reference FC).
  std::string device = "/dev/ttyAMA3";

  /// Serial baud rate between host and radio module. The radio's RF
  /// link settings (frequency, air rate, net ID) are programmed
  /// separately through the radio's AT command interface.
  int32_t baudrate = 57600;

  /// MAVLink system and component IDs of this vehicle.
  uint8_t system_id = 1;
  uint8_t component_id = 1;

  /// Drop incoming frames that fail the CRC; count them either way.
  bool verify_checksum = true;

  /// Signature handling: v2 signed frames are outside this driver's scope;
  /// they are dropped and counted. Enable signing at the protocol layer.
  bool reject_signed_frames = true;
};

/// One received MAVLink message, reassembled.
struct MavlinkFrame {
  uint8_t system_id = 0;
  uint8_t component_id = 0;
  uint32_t message_id = 0;
  uint8_t sequence = 0;
  uint16_t payload_len = 0;
  /// Payload bytes (up to 255 for v2).
  std::string payload;
  /// True when the CRC was verified (or checksum verification is off).
  bool crc_ok = true;
};

/// Link health and counters for the ground station and the failsafe chain.
struct LinkStats {
  uint64_t tx_frames = 0;
  uint64_t rx_frames = 0;
  uint64_t crc_errors = 0;
  uint64_t rx_bytes = 0;
  /// Last RADIO_STATUS local RSSI [dBm], valid after the first report.
  double rssi_dbm = -127.0;
  /// Last RADIO_STATUS remote (ground) RSSI [dBm].
  double remote_rssi_dbm = -127.0;
  /// RADIO_STATUS rxbuf percentage of the remote buffer, 0-100.
  uint8_t remote_rxbuf_pct = 0;
};

/// Serial driver for the MAVLink v2 telemetry link.
class TelemetryLink {
 public:
  using MessageHandler = std::function<void(const MavlinkFrame&)>;

  explicit TelemetryLink(TelemetryConfig config = TelemetryConfig{});
  ~TelemetryLink();

  TelemetryLink(const TelemetryLink&) = delete;
  TelemetryLink& operator=(const TelemetryLink&) = delete;

  /// Opens the serial port in 8N1 raw mode and drains stale input.
  bool open();

  /// Closes the serial port.
  void close();

  /// Registers the callback invoked for every CRC-valid incoming message.
  void setMessageHandler(MessageHandler handler);

  /// Reads pending bytes, parses frames, dispatches to the handler, and
  /// updates statistics. Returns the number of valid frames parsed.
  int poll();

  /// Sends one MAVLink v2 message. `crc_extra` is the per-message CRC seed
  /// from the MAVLink message definitions (the caller knows the message).
  /// Returns false if the frame could not be written.
  bool sendMessage(uint32_t message_id, const uint8_t* payload,
                   uint16_t payload_len, uint8_t crc_extra);

  /// Sends a HEARTBEAT (message 0, crc_extra 50) announcing this vehicle.
  bool sendHeartbeat(uint8_t base_mode, uint8_t system_status,
                     uint32_t custom_mode);

  /// Copy of the current link statistics.
  LinkStats stats() const;

  /// False after a serial error or a CRC failure, until the next good frame.
  bool healthy() const { return healthy_.load(std::memory_order_relaxed); }

 private:
  // Feeds the incremental frame parser; returns 1 when the byte completes
  // a dispatched frame, 0 otherwise.
  int feed(uint8_t byte);
  // Verifies and dispatches one complete frame.
  bool handleFrame();
  void handleRadioStatus(const uint8_t* payload);

  TelemetryConfig config_;
  int fd_ = -1;
  MessageHandler handler_;

  // Incremental parser state (max v2 frame: 12 hdr + 255 payload + 2 + 13).
  uint8_t rx_buf_[280] = {};
  uint16_t rx_len_ = 0;
  uint16_t rx_need_ = 0;
  uint8_t tx_sequence_ = 0;

  LinkStats stats_{};
  mutable std::mutex mutex_;
  std::atomic<bool> healthy_{false};
};

}  // namespace comm
}  // namespace ade
