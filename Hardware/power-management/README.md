# Power Management

The power system moves energy from a 4S LiPo to every subsystem while
keeping the vehicle honest about what is left in the pack. It has three
parts: the battery pack, the power distribution board (PDB) with its INA226
shunt monitor, and the software that turns measurements into failsafe
decisions. This directory carries the monitor driver, the machine-readable
battery definition, and the power budget that sizes all of it.

## Architecture

```text
4S 3300 mAh LiPo (XT60)
  |
  +-- PDB + INA226 (0.5 mOhm shunt, I2C to FC)
  |     |
  |     +-- 4-in-1 ESC (battery voltage, 10 AWG) --> 4x 2216 motors
  |     |
  |     +-- 5V/3A BEC (20 AWG) --> FC, GNSS, telemetry radio
  |             |
  |             +-- 3.3V LDO on the FC --> sensors, MCU
```

The ESC takes raw battery voltage; everything else is regulated. The INA226
sits on the main feed so motor and avionics current are both counted. See
`../schematics/electronics_design.pdf` for the full picture and wire
gauges.

## What the driver provides

- Pack voltage, shunt voltage, current, and power (INA226, I2C).
- **Load-compensated cell voltage**: terminal voltage plus internal-
  resistance drop, divided by cell count. This is what the failsafe
  thresholds consume, which is why a full-throttle climb does not cry
  wolf.
- **Charge estimate**: a blend of coulomb counting (smooth short term) and
  an open-circuit-voltage LiPo model (drift-free long term), reported as
  `charge_fraction` in [0, 1].
- **Failsafe level** (`batteryLevel()`): 0 none, 1 warn, 2 land, 3
  critical, straight from the compensated cell voltage. The flight
  controller maps these to the ladder in `controller_config.yaml`.

After charging or swapping packs, call `resetChargeCounter()` so the
coulomb counter starts from the new pack's state.

## Files

| File                    | Contents                                       |
| ----------------------- | ---------------------------------------------- |
| `README.md`             | this overview                                  |
| `power_monitor.hpp`     | driver API (battery model, stats, levels)      |
| `power_monitor.cpp`     | implementation, INA226 register detail         |
| `battery_config.yaml`   | pack + monitor configuration                   |
| `power_analysis.md`     | power budget, endurance, sizing rationale      |

## Bench check

1. Self test passes (INA226 die IDs read back as "TI").
2. Zero-current reading within +/- 0.15 A (shunt offset).
3. Known-load check: a 12 V/35 W lamp on the pack should read within 5
   percent on both current and power.
4. Compensation check: at a 20 A load, compensated cell voltage should sit
   about 20-30 mV above terminal cell voltage (IR drop / cells).

## Safety

The thresholds in `battery_config.yaml` must match
`../flight-controller/controller_config.yaml` exactly; a mismatch means
the monitor and the failsafe chain disagree about when to land. Changing
either file is a two-reviewer pull request (see the contribution guide).
Never fly a pack that is physically damaged, puffed, or above 12 mOhm
internal resistance.
