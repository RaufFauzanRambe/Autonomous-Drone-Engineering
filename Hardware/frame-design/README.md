# Frame Design

The ADE-FR-001 airframe is a 350 mm wheelbase quad-X built from carbon-
fiber plate and square-section arms. It is designed around the ADE-001
reference electronics: a 1.20 kg all-up aircraft with 8x4.5 in propellers,
whose power budget lives in `../power-management/power_analysis.md`. This
directory carries the design rationale, the dimensioned drawing, and CAD
exports of the reference geometry.

## Design summary

| Property             | Value                              |
| -------------------- | ---------------------------------- |
| Layout               | quad-X, motors at 45 deg diagonal  |
| Wheelbase (diag)     | 350 mm, motor center to center     |
| Center plate         | 120 x 120 x 4 mm                   |
| Arm                  | 135 x 22 x 4 mm, four arms         |
| Motor mount          | 40 x 40 x 4 mm, M3 19x19 pattern   |
| Material             | carbon fiber 3K, 2x2 twill, 4 mm   |
| Frame mass           | 265 g with fasteners               |
| Stack height         | 32 mm (PDB + FC + damping)         |

## Files

| File                              | Contents                                |
| --------------------------------- | --------------------------------------- |
| `README.md`                       | this summary                            |
| `mechanical_design.md`            | geometry rationale, materials, analysis |
| `cad/frame_design.step`           | B-rep export of the center plate (mm)   |
| `cad/frame_design.stl`            | mesh export of the full frame (mm)      |
| `drawings/frame_dimensions.pdf`   | dimensioned drawing, top view + notes   |

## CAD export notes

The STEP file carries the center plate as a parametric-ready B-rep; the
STL carries the full frame (plate, arms, motor mounts) as a mesh for
visualization, 3D-printed mockups, and fit checks. Both are exports of the
same reference geometry described in `mechanical_design.md`; treat the
drawing PDF as the authoritative dimensions for manual layout and
inspection, and regenerate the CAD pair whenever the drawing changes.
Units are millimeters throughout.

## Propeller clearance

Adjacent motor centers are 350 / sqrt(2) = 247.5 mm apart. Props of 8 in
(203 mm diameter) leave 247.5 - 203 = 44.5 mm tip clearance between
adjacent prop discs; the reference prop choice is limited by this number:
9 in props fit with 19.5 mm clearance but demand a thrust re-analysis
(see `mechanical_design.md`, "Limitations").

## Revision history

| Rev | Date       | Change                                    |
| --- | ---------- | ----------------------------------------- |
| A   | 2026-11-25 | Initial release, matched to FC ADE-FC-001 |
