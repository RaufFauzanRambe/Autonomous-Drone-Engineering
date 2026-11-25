# Autonomous-Drone-Engineering

Open-source hardware and software reference stack for a mid-size autonomous
quadrotor. The project gathers in one place everything needed to understand,
build, and iterate on the physical side of an autonomous drone: flight-control
electronics and configuration, sensor drivers, telemetry communication, power
management, and airframe design files.

The C++ drivers in `hardware/` are written against the standard Linux
user-space device APIs (`/dev/spidev*`, `/dev/i2c-*`, `/dev/tty*`). They run
unmodified on embedded Linux flight computers (Raspberry Pi Compute Module 4,
NVIDIA Jetson class) and on bench bring-up rigs, and they port cleanly to
bare-metal firmware. The flight controller card itself runs an RTOS firmware
stack whose tunables are captured in
`hardware/flight-controller/controller_config.yaml`.

## Key features

- Cascaded flight-control architecture: rate, attitude, velocity, and position
  loops with a quad-X motor mixer, fully parameterized in YAML.
- Reference C++ drivers for the sensor suite: BMI088 IMU (SPI), u-blox M9N
  GNSS (UART/UBX), and BMP390 barometer (I2C), with self-tests, bias
  calibration, and health reporting.
- Telemetry link driver with MAVLink v2 framing, sequence numbering, TX
  queueing, and link-quality statistics (RSSI, packet loss).
- Battery and power monitoring via an INA226 shunt monitor, with a documented
  4S power budget and flight-time analysis.
- Mechanical design package: frame geometry rationale, weight budget,
  dimensioned drawing (PDF), and CAD exports (STEP + STL) of the reference
  350 mm quad-X frame.
- Full electrical system diagrams (draw.io block diagram and wiring diagram,
  plus a PDF electronics design summary).

## Reference platform

| Item            | Selection                                                              |
| --------------- | ---------------------------------------------------------------------- |
| Airframe        | ADE-FR-001: 350 mm wheelbase quad-X, carbon-fiber plate and arms        |
| AUW             | 1.20 kg including 4S battery                                           |
| Flight computer | ADE-FC-001 based on STM32H743ZIT6 (Cortex-M7, 480 MHz, 2 MB flash)      |
| IMU             | BMI088 6-axis, SPI @ 10 MHz, 1 kHz gyro sampling                        |
| Barometer       | BMP390, I2C @ 400 kHz, 100 Hz                                          |
| GNSS            | u-blox NEO-M9N, UART, UBX protocol, 5 Hz                               |
| Telemetry       | SiK/RFD900-class 900 MHz radio, MAVLink v2                             |
| Power           | 4S 3300 mAh LiPo, INA226 battery monitor, 5 V/3 A BEC                  |
| Propulsion      | 2216-class 900 Kv motors, 8x4.5 in props, 4-in-1 ESC (DShot600)        |
| Endurance       | approx. 9 min hover to 20 % state of charge; plan missions at 7.5 min  |

## Repository layout

```text
Autonomous-Drone-Engineering/
├── README.md                        this file
├── LICENSE                          MIT license
├── .gitignore
├── CONTRIBUTING.md
├── CODE_OF_CONDUCT.md
├── SECURITY.md
├── CHANGELOG.md
├── requirements.txt                 Python-side tooling dependencies
├── pyproject.toml                   Python project + tool configuration
├── CMakeLists.txt                   builds the hardware driver library
└── hardware/
    ├── README.md                    hardware package overview
    ├── flight-controller/           FC card, control architecture, config
    ├── sensors/                     IMU / GPS / barometer drivers + docs
    ├── communication/               telemetry link driver + config
    ├── power-management/            power monitor driver, battery, budget
    ├── frame-design/                mechanical design, CAD (STEP/STL), drawing
    └── schematics/                  system block diagram, wiring, electronics
```

## Getting started

### Build the C++ hardware library

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
```

The build produces the `drone_hardware` static library containing the sensor,
telemetry, and power-monitoring drivers. The drivers have no dependencies
beyond libc, pthreads, and the Linux kernel userspace headers, so this works
on any Linux host and on an embedded Linux flight computer.

### Python-side tooling

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

The Python dependencies (pymavlink, pyserial, PyYAML, numpy, matplotlib) are
used for telemetry monitoring, configuration validation, and flight-log
analysis while bringing up hardware.

## Documentation index

| Document                                            | Contents                                     |
| --------------------------------------------------- | -------------------------------------------- |
| `hardware/README.md`                                | hardware package overview and conventions    |
| `hardware/flight-controller/controller_overview.md` | control architecture, loops, mixer, failsafes|
| `hardware/sensors/sensor_documentation.md`          | sensor specs, mounting, calibration          |
| `hardware/power-management/power_analysis.md`       | power budget and flight-time analysis        |
| `hardware/frame-design/mechanical_design.md`        | frame geometry, materials, weight budget     |
| `hardware/schematics/electronics_design.pdf`        | system block diagram and power distribution  |

## Safety notice

An autonomous drone is a flying robot capable of causing serious injury and
property damage. Read the safety sections of `CONTRIBUTING.md` before
bench-testing or flying. Remove props for all bench tests, verify failsafes
before every autonomous flight, and fly only within the regulations that apply
at your location.

## Contributing and license

Contributions are welcome: see `CONTRIBUTING.md` (process and code style) and
`CODE_OF_CONDUCT.md`. Security issues are handled per `SECURITY.md`.

This project is licensed under the MIT License; see `LICENSE`.
