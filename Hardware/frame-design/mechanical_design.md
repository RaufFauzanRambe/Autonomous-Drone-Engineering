# Mechanical Design

Geometry rationale, material choices, and structural analysis for the
ADE-FR-001 quad-X frame. The drawing (`drawings/frame_dimensions.pdf`) is
the authority for dimensions; this document explains why they are what
they are.

## Geometry

### Layout

The X configuration puts the four motors on the diagonals at 175 mm radius
from the vehicle center: a 350 mm motor-to-motor wheelbase. X (rather
than +) gives symmetric pitch and roll authority, leaves the nose free of
motor booms for a camera or sensor mast, and matches the mixer table in
`../flight-controller/controller_overview.md`.

### Center plate

The 120 x 120 x 4 mm plate carries the avionics stack: PDB on standoffs
below, flight controller on silicone grommets above, GNSS mast at the
center-rear. 120 mm was chosen to fit a standard 30.5 x 30.5 mm hole
pattern FC plus wiring clearance, while staying under half the arm span so
the plate does not constrain prop diameter. Four M3 holes on 24 x 24 mm
corners clamp the arms; four more on the plate corners anchor the landing
gear.

### Arms

Four 135 x 22 x 4 mm arms run along the diagonals, clamped between the
center plate and 40 x 40 mm motor mounts. The arm inner edge starts at
50 mm radius (inside the plate clamping zone) and ends at 185 mm, just
past the motor center, so the load path from motor to plate is a straight,
double-shear bolted joint instead of a cantilever.

### Motor mounts

40 x 40 x 4 mm plates with the standard 19 x 19 mm M3 pattern that
2216-class motors use. The mount bolts through the arm end with two M3
fasteners and a 6 mm nylon spacer column, which electrically isolates the
motor case from the carbon (carbon is conductive; an un-isolated motor
short pulse-throughs the winding case into the frame - a classic crash
cause on CF frames).

## Materials

- **Plate, arms, mounts**: 3K carbon fiber, 2x2 twill, quasi-isotropic
  layup, 4 mm nominal. Stiffness-to-weight beats aluminum 6061 by ~3x at
  equal mass; the 2x2 twill gives balanced bending in the two in-plane
  directions, which matters for arms loaded at 45 degrees to the fiber.
- **Fasteners**: M3, A2-70 stainless with nylon-insert lock nuts on
  anything that vibrates (all motor and arm bolts).
- **Standoffs**: aluminum for the PDB (rigid, thermal path for the BEC),
  silicone grommets for the FC (isolation of IMU vibration, see
  `../sensors/sensor_documentation.md`).
- **Landing gear**: 90 g of 6 mm aluminum tube bolted to the plate
  corners; sacrificial, replaceable after the landings that dent it.

## Structural analysis

### Motor thrust loads

Worst-case steady load is a single motor at full thrust pulling the arm
up with 850 g (power analysis burst case, 8.5 N) plus the transient of a
landing impact: design load 3x static = 25 N at the motor center, 90 mm
outboard of the arm root.

- Arm bending at root: M = 25 N x 0.090 m = 2.25 N m. Section modulus of
  a 22 x 4 mm rectangle: Z = b h^2 / 6 = 22 x 16 / 6 = 58.7 mm^3.
  Bending stress = 2250 N mm / 58.7 mm^3 = 38 MPa. CF quasi-isotropic
  flexural strength is ~500-600 MPa: safety factor > 12.
- Arm deflection at motor: F L^3 / (3 E I). E = 40 GPa effective,
  I = b h^3 / 12 = 22 x 64 / 12 = 117.3 mm^4 = 1.173e-10 m^4.
  Deflection = 25 x 0.090^3 / (3 x 40e9 x 1.173e-10) = 1.4 mm at design
  load; at hover (8.5/3 N) it is 0.5 mm. Stiff enough that the control
  system sees the frame as rigid at 1 kHz.

### Crash case

The design crash is a 5 m/s vertical arrival on one arm tip: energy
0.5 x 1.2 x 5^2 = 15 J, absorbed by arm fracture near the root (a
deliberate weak section - the arm is one flat CF bar, cheap and
replaceable) rather than by the center plate. Repair cost after the
expected crash: one arm and two fasteners.

### Vibration

The first bending mode of a clamped arm is well above the blade-pass
frequency of the props at hover (f = 1.875^2 / 2pi x sqrt(EI/(rho A L^4))
computes to ~180 Hz with prop mass added - the silicone FC mount and the
firmware notch filter at 190 Hz handle the residual; see the filter
settings in `../flight-controller/controller_config.yaml`).

## Mass budget

The frame weighs 265 g assembled: 92 g plate + 76 g arms (4 x 19 g) +
41 g mounts (4 x 10.2 g) + 56 g fasteners, spacers, and grommets. CF
density taken at 1.6 g/cm^3 for the quasi-isotropic layup. This is 22
percent of the 1.20 kg AUW - the normal range for CF frames of this class
(15-25 percent).

## Limitations and future revisions

- 9 in propellers fit geometrically (19.5 mm tip clearance) but the
  thrust/current model in the power analysis would need re-measuring;
  treat 9 in as a bench-validated-only option.
- The square-section arm trades some torsional stiffness for simplicity.
  If ESC telemetry ever shows yaw-axis resonance, the B-revision arm is a
  box section.
- No camera mount is modeled; the nose space between the front arms
  (roughly 60 x 40 mm above the plate) is reserved for it.
