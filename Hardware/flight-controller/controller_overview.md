# Controller Overview

This document is the authoritative description of the ADE-001 flight-control
architecture: what runs at which rate, how commands flow from the mode logic
down to the motors, and which failsafes guard each layer. The tunable
parameters referenced here live in `controller_config.yaml`; when a value in
that file and this document disagree, the YAML wins and this document has a
bug (fix both in the same pull request).

## Architecture

The controller is a strictly cascaded pipeline. Each loop's bandwidth is
roughly 2-3x the loop above it so the inner loops track their setpoints fast
enough that the outer loops can treat them as ideal actuators:

```text
  mode logic (position/velocity/altitude setpoints, 50-250 Hz)
        |
        v
  position loop (50 Hz) -> velocity setpoints (m/s, NED)
        v
  velocity loop (250 Hz) -> acceleration -> attitude + collective thrust
        v
  attitude loop (500 Hz) -> body-rate setpoints (rad/s)
        v
  rate loop (1 kHz) -> body torque commands (normalized)
        v
  quad-X mixer -> 4 DShot600 motor commands
```

All loops consume the state estimate produced by the 250 Hz EKF (see below).
The rate loop additionally consumes the raw 1 kHz gyro samples through a
feedforward path so a disturbance is countered before it appears in the
estimate.

## Loop schedule

| Loop            | Rate  | Sensor source            | Output                        | Worst-case budget |
| --------------- | ----- | ------------------------ | ----------------------------- | ----------------- |
| Gyro sampling   | 1 kHz | BMI088 gyro (SPI)        | filtered rate + FF path       | 100 us            |
| Rate loop       | 1 kHz | EKF rate + gyro FF       | normalized torque cmd         | 250 us            |
| Attitude loop   | 500 Hz| EKF attitude quaternion  | rate setpoints                | 250 us            |
| Velocity loop   | 250 Hz| EKF velocity (GNSS-fused)| attitude + thrust setpoints   | 400 us            |
| Position loop   | 50 Hz | EKF position             | velocity setpoints            | 2 ms              |
| EKF update      | 250 Hz| IMU + baro + GNSS        | full state + covariance       | 1.5 ms            |
| Mixer + DShot   | 1 kHz | rate loop output         | 4 motor commands              | 100 us            |

Measured CPU load of the full pipeline on the STM32H743 at 480 MHz is
budgeted below 25 percent, leaving headroom for logging and the
companion-computer link. The scheduler is fixed-priority preemptive; all
control loops are above the telemetry task in priority.

## State estimation

A 24-state EKF runs at 250 Hz: attitude quaternion, velocity, position, gyro
bias, accelerometer bias, and earth-frame magnetic field. Propagation uses
the 1 kHz inertial measurements decimated to the update rate; corrections
come from the barometer (altitude), the NEO-M9N (position and velocity,
5 Hz), and optionally a magnetometer on the companion bus. Innovation
gating rejects GNSS samples with reported accuracy worse than the gate
(`ekf.gps_gate_m` in the config). If innovations stay inconsistent for more
than 1 s the estimator declares divergence, which escalates through the
failsafe chain (see Failsafes).

## Rate and attitude loops

The rate loop is a parallel-form PID per axis with:

- Derivative on measurement (not on error) to avoid setpoint kick.
- Derivative low-pass (`pid.rate.*.d_lpf_hz`, 100 Hz).
- Anti-windup by clamping the integral (`i_term_max`), with the clamp on the
  integrator state, not just the output.
- Motor-output feedforward from the rate setpoint, sized by the mixer's
  authority model.

The attitude loop outputs rate setpoints from a quaternion error controller.
Tracking is bounded by `limits.max_tilt_deg` in position-controlled modes and
by the pilot stick in Stabilize.

## Velocity and position loops

The velocity loop maps horizontal acceleration commands to an attitude
setpoint (tilt) and a collective thrust command. Thrust is normalized against
a hover-thrust estimate learned in flight, so the vertical loop's gain stays
constant as the battery sags. The position loop is a simple P controller
producing velocity setpoints, rate-limited by `limits.max_velocity_mps`.

## Mixer

Quad-X geometry with the motor numbering and signs of the standard X frame.
Given normalized roll/pitch/yaw torque commands and collective thrust:

```text
M1 (rear-right)  = thrust + roll - pitch - yaw
M2 (rear-left)   = thrust - roll - pitch + yaw
M3 (front-right) = thrust + roll + pitch + yaw
M4 (front-left)  = thrust - roll + pitch - yaw
```

Outputs are clipped at [0, 1] with desaturation biased toward preserving yaw
authority, then emitted as DShot600 frames at 1 kHz.

## Flight modes

| Mode       | Description                                                        |
| ---------- | ------------------------------------------------------------------ |
| MANUAL     | Direct rate control, no attitude stabilization (setup/emergency)   |
| STABILIZE  | Attitude stabilized, pilot controls thrust directly                |
| ALTCTL     | Stabilize + altitude hold from the barometer                        |
| POSCTL     | Altitude + position hold from the EKF, stick moves set position    |
| AUTO       | Waypoint missions from the companion computer, geofence enforced   |

Mode changes are latency-bounded: every mode transition re-initializes only
the loops that change dimension (for example switching POSCTL to ALTCTL
keeps the vertical loop states).

## Failsafes

All failsafes escalate through the same ladder: hold -> return-to-launch
(RTL) -> land. The table gives the trigger and the direct entry point into
the ladder; each step down is announced on telemetry and logged.

| Failsafe        | Trigger                                   | Action ladder                 |
| --------------- | ----------------------------------------- | ----------------------------- |
| RC loss         | no valid CRSF frames for 0.5 s            | 3 s hold, then RTL            |
| Telemetry loss  | no GCS heartbeat for 10 s in AUTO         | continue mission, warn        |
| Battery warning | any cell under 3.55 V (load-compensated)  | warn, RTL advised             |
| Battery land    | any cell under 3.45 V (load-compensated)  | RTL, land at destination      |
| Battery critical| any cell under 3.30 V (load-compensated)  | immediate land                |
| Geofence        | beyond 500 m radius or 120 m AGL          | stop at fence, RTL            |
| EKF divergence  | innovation gates failing > 1 s            | ALTCTL if stable, else land   |
| Motor/ESC fault | ESC telemetry: overcurrent or desync      | land immediately              |

Battery thresholds use the load-compensated cell voltage from the INA226
monitor (see `../power-management/`), not the raw pack voltage, so a
high-current climb does not trigger a false warning. Geofence defaults are
conservative; check the regulations that apply at your flying site and
adjust before flight.

## Tuning guide

Tune in this order, on the bench with props off for the first two steps:

1. Rate loop: verify gyro sign conventions and filter delay with step
   responses on the bench; then fly Stabilize and adjust `kp` until fast
   without oscillation, adding `kd` only if needed.
2. Attitude loop: adjust `kp` in POSCTL-stick moves; the aircraft should
   stop crisply without overshoot.
3. Vertical: tune the thrust normalization in ALTCTL first, then the
   vertical velocity PI.
4. Horizontal: velocity loop gains, then position P.

Record every flight's gains into `controller_config.yaml` with the flight
log reference in the commit message (see the contribution guide).
