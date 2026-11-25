# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2026-11-25

### Added

- Initial repository structure for the ADE-001 reference quadrotor.
- Flight-controller package: control architecture overview, cascaded PID
  configuration (`controller_config.yaml`), block diagram and schematic
  (draw.io + PDF).
- Sensor drivers (Linux user-space reference implementations): BMI088 IMU
  over SPI, u-blox GNSS over UART/UBX, BMP390 barometer over I2C, each with a
  matching YAML configuration and documentation.
- Communication package: SiK/RFD900-class telemetry link driver with MAVLink
  v2 framing, heartbeat transmission, and link statistics.
- Power management package: INA226 power-monitor driver, 4S battery
  configuration, power budget and flight-time analysis.
- Frame design package: mechanical design document, weight budget, CAD
  exports of the 350 mm quad-X reference frame (STEP + STL), and a
  dimensioned drawing (PDF).
- Electrical schematics package: system block diagram, wiring diagram
  (draw.io), and electronics design summary (PDF).
- Repository infrastructure: MIT license, contributing guide, code of
  conduct, security policy, Python tooling configuration, and a CMake build
  for the hardware driver library.
