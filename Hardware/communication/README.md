# Communication

The ADE-001 telemetry link pairs a SiK/RFD900-class 900 MHz serial radio on
the aircraft with a matching radio at the ground station, carrying MAVLink
v2 between the vehicle and the ground control software. This directory
contains the host-side link driver, its configuration, and the interface
conventions the rest of the stack relies on.

## What the driver does

- Owns the serial link to the airside radio (`/dev/ttyAMA3` on the
  reference FC, 57600 baud 8N1).
- Transmits MAVLink v2 frames with correct sequence numbering and CRC-16/X.25
  checksums, including the per-message CRC_EXTRA seed.
- Parses incoming frames incrementally, verifies checksums for the message
  types it knows (HEARTBEAT, RADIO_STATUS), and forwards every message to a
  registered callback.
- Maintains link statistics: transmitted and received frames, CRC errors,
  byte counts, and RSSI (local and remote) from the radio's RADIO_STATUS
  reports, converted to dBm with the SiK scaling (`rssi/1.9 - 127`).

## What it deliberately does not do

The driver does not reconfigure the radio (frequency, net ID, air rate are
programmed over the radio's AT command interface before installation), it
does not implement MAVLink signing (v2 signed frames are dropped and
counted until signing lands at the protocol layer), and it does not know
the full MAVLink message set: messages whose CRC_EXTRA seed is not in the
driver's table are forwarded with `crc_ok = false` so upper layers can
decide how much to trust them. Extending the known-message table is a
small, well-contained change; see the `TODO(protocol-verify)` hook in the
implementation.

## Link budget expectation

With the reference radio profile (900 MHz, 2 dBi antennas both ends,
64 kbps air rate, ECC on) the expected ground-range for a 30 m AGL
multirotor is 3-8 km depending on terrain. Plan missions around the
conservative number: the failsafe chain in
`../flight-controller/controller_config.yaml` treats loss of ground-station
contact (not RC loss) as a warning-level event in AUTO mode, and loss of
both links escalates to RTL.

## Files

| File                          | Contents                                  |
| ----------------------------- | ----------------------------------------- |
| `README.md`                   | this overview                             |
| `communication_interface.hpp` | driver API (config, stats, callbacks)     |
| `communication_interface.cpp` | implementation, protocol details          |
| `communication_config.yaml`   | machine-readable link configuration       |

## Bench check

Loopback: connect the airside radio to a ground-side radio on the bench,
send heartbeats at 1 Hz, and watch the ground station see them and the
air-side stats see RADIO_STATUS arrive. A CRC error rate above zero on a
bench loopback means a wiring or configuration problem: fix it before
flight, because in the air it becomes a link-loss failsafe.
