# Hardware Package

This directory contains the complete hardware engineering package for the
ADE-001 reference quadrotor: flight-controller electronics and configuration,
sensor drivers, the telemetry link, power management, and the mechanical
frame design. The reference platform is a 350 mm wheelbase quad-X of 1.20 kg
all-up weight, powered by a 4S 3300 mAh LiPo (see the root `README.md` for
the full specification table).

## Subsystems

| Directory             | Contents                                                                 |
| --------------------- | ------------------------------------------------------------------------ |
| `flight-controller/` | FC card description, control architecture document, controller config   |
| `sensors/`           | C++ drivers + configs + documentation for IMU, GNSS, and barometer      |
| `communication/`     | Telemetry link driver (MAVLink v2 over SiK-class radio) + config        |
| `power-management/`  | INA226 power monitor driver, battery config, power budget analysis      |
| `frame-design/`      | Mechanical design doc, CAD exports (STEP/STL), dimensioned drawing      |
| `schematics/`        | System block diagram, wiring diagram, electronics design summary        |

## How the pieces fit together

The ADE-FC-001 flight controller card (STM32H743ZIT6) runs the hard
real-time flight stack; its tunables live in
`flight-controller/controller_config.yaml`. The C++ drivers under `sensors/`,
`communication/`, and `power-management/` are Linux user-space reference
implementations of the same device interfaces, used on the bench, in
hardware-in-the-loop rigs, and on companion computers. Keeping one shared
source of truth per device means a register map or conversion factor is
fixed once, in a file that both the firmware and the host tools read.

## Conventions

- **Units are SI** everywhere in configs and code (volts, amperes, pascals,
  rad/s, m/s). Degrees appear only in explicitly suffixed keys such as
  `max_tilt_deg`.
- **Datasheet provenance**: every register constant, scale factor, and
  protocol offset in the drivers carries a comment naming its source. Items
  pending verification against a physical part are tagged
  `TODO(datasheet-verify)`.
- **Revisions**: electrical documents carry rev letters (Rev A initial
  release). A schematic change bumps the letter and gets an entry in the
  document's own revision table and in the root `CHANGELOG.md`.
- **Config mirrors code**: each YAML file lists the same defaults as the
  corresponding C++ `Config` struct; a change in one place requires a change
  in the other within the same pull request.

## Build

From the repository root:

```bash
cmake -S . -B build && cmake --build build
```

## Safety

Read the root `README.md` safety notice and `CONTRIBUTING.md` (hardware and
flight-test safety protocol) before powering anything. Props come off for
bench tests.
