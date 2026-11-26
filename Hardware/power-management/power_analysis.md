# Power Analysis

Power budget, endurance model, and sizing rationale for the ADE-001
reference quad. Every number here is either measured on the bench, computed
from datasheets, or derived with the formula shown next to it; the
machine-readable counterparts live in `battery_config.yaml`. When you
change the aircraft, re-do this analysis first: the failsafe thresholds and
the mission planner's endurance figure both trace back to it.

## All-up weight

| Component                        | Mass [g] | Count | Total [g] |
| -------------------------------- | -------- | ----- | --------- |
| Frame (plate + arms + hardware)  | 265      | 1     | 265       |
| Motor 2216-class 900 Kv          | 62       | 4     | 248       |
| Propeller 8x4.5 in carbon        | 12       | 4     | 48        |
| ESC 4-in-1 40 A, DShot600        | 42       | 1     | 42        |
| Flight controller + mount        | 18       | 1     | 18        |
| GNSS module + mast               | 15       | 1     | 15        |
| Telemetry radio + antenna        | 16       | 1     | 16        |
| Battery 4S 3300 mAh              | 385      | 1     | 385       |
| PDB, wiring, connectors          | 70       | 1     | 70        |
| Damping, straps, landing gear    | 90       | 1     | 90        |
| **All-up weight (AUW)**          |          |       | **1197**  |

Rounded to 1.20 kg for all derived figures. Thrust-to-weight at full
throttle: 4 x 850 g / 1197 g = **2.84**; hover at a comfortable 36 percent
of maximum thrust.

## Current draw

Motor data for the 2216 900 Kv with 8x4.5 in props on 4S (bench, static):

| Operating point        | Per motor [A] | Total [A] | Notes                |
| ---------------------- | ------------- | --------- | -------------------- |
| Hover (300 g thrust)   | 4.2           | 16.8      | 14.0 V nominal       |
| Cruise (1.3x hover)    | 5.5           | 22.0      |                      |
| Full throttle          | 14.0          | 56.0      | burst only           |

Avionics loads (5 V rail via BEC):

| Load                | Current @ 5 V [A] | Power [W] |
| ------------------- | ----------------- | --------- |
| FC + sensors        | 0.18              | 0.90      |
| GNSS receiver       | 0.05              | 0.25      |
| Telemetry radio     | 0.20 (avg)        | 1.00      |
| **Total avionics**  | **0.43**          | **2.15**  |

Drawn from the battery through the 3 A BEC at ~87 percent efficiency:
0.43 A x 5 V / 14.8 V / 0.87 = 0.17 A, under 1 percent of hover current.
BEC margin: 3 A rated / 0.43 A used = 7x.

## Endurance

Usable capacity: 3300 mAh x 0.80 = **2640 mAh**.

- Hover: 2.64 Ah / 16.8 A = 0.157 h = **9.4 min** to the 20 percent
  state-of-charge floor.
- Mission planning (cruise at 1.3x hover current, 20 percent landing
  reserve): 2.64 Ah / 22.0 A x 0.8 = 0.096 h = **5.8 min**. **Plan
  missions at 7.5 min only if the profile is mostly hover**; the planner
  default is 6 min.

The failsafe ladder (warn 3.55, land 3.45, critical 3.30 V/cell,
load-compensated) brackets these numbers: warn fires a few minutes into a
hover-heavy flight, land lands with reserve intact.

## Sizing rationale

- **Battery**. 3300 mAh at 1.2 kg AUW gives 9.4 min hover - the classic
  multirotor knee of the endurance curve, where doubling battery mass buys
  minutes but costs hover current (and the curve flattens). The 75 C rating
  (247 A) vastly exceeds the 56 A burst: the rating is chosen for low
  internal resistance and cool running, not for headroom to abuse.
- **Shunt**. 0.5 mOhm at 100 A full scale: hover dissipation
  I^2 R = 16.8^2 x 0.0005 = 0.14 W, burst 1.6 W for seconds - the shunt
  barely warms, so its tempco does not bias the compensation.
- **BEC**. 3 A covers the avionics rail with 7x margin so telemetry TX
  spikes and a future payload sensor cannot brown out the flight
  controller; the FC card's own 3.3 V LDO has a further 2x on its loads.
- **Wire**. ESC feed 10 AWG (56 A burst with headroom), battery-to-PDB
  12 AWG (XT60 is the limiting element at ~60 A continuous), avionics 20
  AWG. See `../schematics/wiring_diagram.drawio` for the colors and
  connector pinout.

## Flight-time verification log

| Date       | Battery | Hover current [A] | Time to warn [min] | Notes              |
| ---------- | ------- | ----------------- | ------------------ | ------------------ |
| 2026-11-25 | (new)   | (pending first flight log) | --       | fill on first hover test |

The first hovering test flight should add the measured numbers here and
to `battery_config.yaml` if they disagree with the model by more than 10
percent.
