# Sensor Documentation

Suite-level reference for the ADE-001 sensor set: what each sensor measures,
where it mounts, how it is wired, and how it is calibrated. Device-level
register detail lives in the driver comments (each driver cites the
datasheet it was written against); this document is the integration view the
bench engineer needs before the first flight.

## Sensor suite summary

| Sensor  | Part        | Bus                  | Rate  | Purpose                          |
| ------- | ----------- | -------------------- | ----- | -------------------------------- |
| IMU     | BMI088      | SPI1 @ 10 MHz        | 1 kHz | Body rates and specific force    |
| Baro    | BMP390      | I2C1 @ 400 kHz       | 50 Hz | Absolute pressure, altitude      |
| GNSS    | u-blox M9N  | USART1 @ 115200      | 5 Hz  | Position, velocity, UTC time     |

## IMU (BMI088)

### Orientation and mounting

The IMU is mounted at the electrical center of the flight controller, which
is bolted to the frame center plate stack on silicone grommets. The board
silkscreen marks the +X (forward) and +Y (right) axes; the aircraft nose
must align with +X within 0.5 degrees. The mechanical drawing in
`../frame-design/drawings/frame_dimensions.pdf` shows the mounting-hole
pattern for the FC stack. Keep the IMU at least 25 mm from motor wires; the
BMI088's susceptibility to vibration is handled by the soft mount plus the
notch filter at the blade-pass frequency (see
`../flight-controller/controller_config.yaml`, `filters`).

### Calibration

Gyro bias is estimated by the driver's `calibrateBias()` with the aircraft
level, still, and warm (let the FC run 60 s first). A bias above
0.02 rad/s on any axis after calibration indicates a mounting or hardware
problem; do not fly. Accelerometer scale and misalignment are calibrated in
the firmware (six-position calibration, nose-up/down, roll left/right,
level inverted/normal); the driver's converted output is only used for
bench checks. Temperature drift is compensated by the EKF bias states.

### SPI wiring

| FC pin | BMI088 pin | Note                     |
| ------ | ---------- | ------------------------ |
| SPI1_SCK   | SCL/SCK (both dies) | shared bus      |
| SPI1_MISO  | SDO (both dies)     | shared bus      |
| SPI1_MOSI  | SDI (both dies)     | shared bus      |
| GPIO CE0   | ACC CSB1            | /dev/spidev0.0  |
| GPIO CE1   | GYRO CSB2           | /dev/spidev0.1  |
| EXTI0      | ACC INT1            | data-ready DRDY |
| EXTI1      | GYRO INT3           | data-ready DRDY |

Mode 3 (CPOL=1, CPHA=1), 10 MHz. Accelerometer reads use a dummy byte; the
driver handles this, other tools must too.

## Barometer (BMP390)

### Mounting and airflow

The barometer sits on the FC with a small hole in the case above it. The
case opening must see static pressure only: never inside the prop wash, and
never in a sealed cavity (a sealed cavity turns altitude into a slow
oscillator). On the reference frame, the FC stack is centered under the top
plate with vent slots on the sides. Illumination airflow, propwash, and
battery heat all bias the reading; the EKF gates the baro against GNSS and
innovations, so a slowly drifting sensor degrades altitude hold gracefully
instead of failing hard.

### Reference pressure

`setSeaLevelPressure()` takes QNH from the ground station at mission start
(default ISA 101325 Pa). Flying with the default QNH on a high- or
low-pressure day yields altitude errors of tens of meters, which is fine for
control (the EKF fuses GNSS altitude too) but wrong for absolute altitude
reporting: set QNH before logging data you care about.

## GNSS (u-blox NEO-M9N)

### Antenna placement

The patch antenna mounts on a 2 mm carbon or FR4 ground plane at the frame
top center, at least 20 mm above the battery and away from the telemetry
radio (90-degree separation: radio on a side rail, antenna top-center).
Motor ESCs are below the battery shield; this arrangement keeps the antenna
view of the sky clean and the RF noise floor low. Expect 1.5 m CEP class
accuracy with a clear sky view, degrading near structures.

### Receiver configuration

The bench profile stored in the receiver:

- Protocol: UBX in/out, NMEA disabled.
- Baud rate: 115200 (ROM default is 38400; set through u-center or a
  CFG-VALSET message: key CFG-UART1-BAUDRATE, layer RAM).
- Messages: NAV-PVT enabled at 5 Hz.
- Constellations: GPS + Galileo + GLONASS + SBAS, jamming detection on.

The driver does not reconfigure the receiver; it reads whatever profile is
stored. If `poll()` never returns a fix and the wiring is correct, check
that the receiver actually speaks UBX at the configured baud rate with a
logic analyzer or u-center.

### Fix quality gates

The EKF accepts NAV-PVT only when `fix.valid` is true and reported
horizontal accuracy is better than `ekf.gps_gate_m`. During bench testing,
`num_sv` below 8 or `p_dop` above 2.5 usually means antenna placement
problems, not receiver failure.

## Timestamping

All drivers timestamp with the host `CLOCK_MONOTONIC` at the instant the
data registers are read, not when the sample was physically taken. For the
1 kHz IMU this introduces up to one sample period of latency; the flight
controller compensates with the data-ready interrupt. For the 5 Hz GNSS fix
the receiver's own time-of-week is authoritative for logging; the host
timestamp is only for correlation.

## Bench validation checklist

Run before every FC board revision and after any sensor replacement:

1. `imu` self test: both chip IDs read correctly.
2. IMU bias: 3 consecutive calibrations within 10 percent of each other.
3. Rotation check: rotate nose-up 90 degrees, converted specific force
   follows gravity within 5 percent per axis.
4. Baro: compare pressure against a reference gauge within 1.5 hPa after a
   10-minute soak.
5. GNSS: 3D fix outdoors, `num_sv >= 8`, `h_accuracy_m <= 1.5`.
6. Vibration: run motors at hover throttle (props on, aircraft tied down
   per the safety protocol) and confirm the gyro spectrum peak stays at the
   blade-pass frequency with the notch filter active.
