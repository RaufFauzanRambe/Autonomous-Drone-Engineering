# Flight Controller

The ADE-FC-001 flight controller card is the hard real-time brain of the
ADE-001 reference quad. It samples the inertial sensors, runs the state
estimator and the cascaded control loops, mixes commands onto the four
motors, and owns every failsafe. This directory documents the card and its
control architecture, and carries the machine-readable configuration the
firmware and the HIL rigs share.

## Reference hardware

| Item            | Selection                                                          |
| --------------- | ------------------------------------------------------------------ |
| MCU             | STM32H743ZIT6, Cortex-M7 @ 480 MHz, 2 MB flash, 1 MB RAM            |
| IMU             | BMI088 (accel + gyro dies), SPI @ 10 MHz                           |
| Barometer       | BMP390, I2C @ 400 kHz                                              |
| GNSS            | u-blox NEO-M9N on USART1 (see `../sensors/`)                       |
| Motor outputs   | 4x DShot600 on timer channels, opt.` telemetry from 4-in-1 ESC      |
| Telemetry       | SiK/RFD900-class radio on USART3 (see `../communication/`)         |
| RC link         | UART, CRSF/SBUS                                                    |
| Logging         | microSD over SDMMC                                                |
| Power input     | 5 V from the BEC, with 3.3 V LDO and brown-out protection          |
| Board           | 90 x 55 mm, 6-layer, 18 g including vibration mount                |

The board is designed for a soft-mount (silicone grommets) with the IMU as
close to the vehicle center of mass as the stack allows; see
`../sensors/sensor_documentation.md` for mounting details.

## Files in this directory

| File                                       | Contents                                     |
| ------------------------------------------ | -------------------------------------------- |
| `README.md`                                | this card description                        |
| `controller_overview.md`                   | control architecture, loops, mixer, failsafes|
| `controller_config.yaml`                   | machine-readable control configuration       |
| `schematics/controller_block_diagram.drawio` | control-data-flow block diagram (draw.io)  |
| `schematics/controller_schematic.pdf`      | card interconnect schematic (PDF, Rev A)     |

## Control architecture at a glance

Cascaded loops with strictly nested bandwidths: a 1 kHz rate loop inside a
500 Hz attitude loop inside a 250 Hz velocity loop, with a 50 Hz position
loop on top and a 250 Hz EKF producing the state estimate. The mixer maps
body torque and collective thrust commands to four motor DShot commands
using the quad-X matrix. `controller_overview.md` is the authoritative
description, and `controller_config.yaml` carries the tunable parameters.

## Firmware

The card runs an RTOS flight stack. The parameters in
`controller_config.yaml` are the build-time defaults; mission tuning happens
through the ground station parameter interface and must be checked back into
the YAML file when a value proves itself (see the contribution guide in the
root of the repository).

## Revision history

| Rev | Date       | Change                                             |
| --- | ---------- | -------------------------------------------------- |
| A   | 2026-11-25 | Initial release, matched to frame ADE-FR-001 Rev A |
