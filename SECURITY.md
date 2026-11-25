# Security Policy

Autonomous-Drone-Engineering is flight software and flight hardware for an
uncrewed aircraft. Security defects here are not just data-loss bugs: a
malformed telemetry frame that overflows a parser buffer, or a geofence check
that can be bypassed, can end with an uncontrolled aircraft in the air. We
treat such reports with priority.

## Supported versions

| Version | Supported          |
| ------- | ------------------ |
| 0.1.x   | yes (current line) |
| < 0.1   | no                 |

## Reporting a vulnerability

Please report vulnerabilities privately through the repository hosting
platform's private security advisory feature (the "Report a vulnerability"
action in the Security tab), which routes the report directly to the
maintainers. If that channel is unavailable to you, contact a maintainer
directly and agree on a private channel before sharing details.

Include in the report:

- The affected file(s) and a description of the defect.
- A reproducer (malformed UBX/MAVLink byte stream, config file, command
  sequence).
- Your assessment of the flight-safety impact.

Do **not** open a public issue for anything security related, and do not test
exploits against aircraft in flight. We aim to acknowledge reports within 72
hours and to coordinate a fix and disclosure timeline with you, typically
within 30 days for flight-safety issues.

## Scope

Of particular interest for this repository:

- `hardware/sensors/` UBX frame parser: length fields, checksum handling,
  buffer sizes.
- `hardware/communication/` MAVLink v2 framing: sequence handling, payload
  length validation, unknown-message handling, RSSI/RADIO_STATUS parsing.
- Geofence, failsafe, and battery-protection logic defaults in
  `hardware/flight-controller/controller_config.yaml` and
  `hardware/power-management/battery_config.yaml`: anything that could allow
  an unsafe state to look safe.
- Firmware configuration and update documentation that could lead someone to
  flash mismatched firmware.

Out of scope: denial of service against the ground station UI, and issues in
dependencies that already have their own CVE process.

## Flight-safety considerations

When preparing a fix, maintainers will check whether the defect can lead to
loss of control, flyaway, battery damage, or geofence breach. Fixes with
flight impact are released with an explicit callout in `CHANGELOG.md` and, if
in-flight configurations are affected, a pinned upgrade notice in the
README's safety section.
