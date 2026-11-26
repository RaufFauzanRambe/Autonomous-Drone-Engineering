# Schematics

System-level electrical documentation for the ADE-001 reference quad:
the block diagram that shows what talks to what, the wiring diagram that
shows how it is physically interconnected (gauges, colors, connectors),
and the electronics design summary that collects the power distribution
and design rules in one printable document.

## Files

| File                              | Contents                                     |
| --------------------------------- | -------------------------------------------- |
| `README.md`                       | this overview                                |
| `system_block_diagram.drawio`     | system block diagram (draw.io, editable)     |
| `wiring_diagram.drawio`           | physical wiring diagram (draw.io, editable)  |
| `electronics_design.pdf`          | electronics summary: blocks + power tree     |

## Document conventions

- **draw.io files** are the editable sources: open them at
  app.diagrams.net or in the drawio desktop app. Each file has one page
  per diagram; cells are named after the component they represent.
- **The PDF** is the printable snapshot for the workshop wall. When a
  draw.io source changes, re-export the PDF in the same pull request so
  the two never diverge (see the contribution guide).
- **Wire colors** follow the reference build: red = battery +, black =
  battery -, orange = 5 V regulated, white = signal, yellow = telemetry
  TX, green = telemetry RX, shield/gray = CAN or spare.
- **Revisions**: the title block of every document carries a rev letter
  and date; a change bumps the letter, and the change is logged in the
  root `CHANGELOG.md`.

## Related subsystem documents

- Flight controller card and interconnect: `../flight-controller/`
  (includes its own schematic for the FC card itself).
- Sensor interfaces and pinouts: `../sensors/sensor_documentation.md`.
- Power distribution rationale: `../power-management/power_analysis.md`.

## Revision history

| Rev | Date       | Change                                    |
| --- | ---------- | ----------------------------------------- |
| A   | 2026-11-25 | Initial release, matched to FC Rev A      |
