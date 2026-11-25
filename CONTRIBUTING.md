# Contributing to Autonomous-Drone-Engineering

Thank you for considering a contribution. This project covers flight
hardware, drivers, and design documentation for an autonomous aircraft, so
contributions are held to a higher standard of review than a typical software
repository: an error that ships in a config default or a driver can turn into
a flyaway or a fire. Please read this guide before opening a pull request.

## Ways to contribute

- Report bugs and hardware issues (see the issue templates below).
- Improve drivers and documentation; documentation patches are as welcome as
  code.
- Add or improve sensor/telemetry drivers following the existing structure.
- Review schematics and mechanical drawings; flag anything ambiguous.
- Share validation data (bench logs, thrust-stand results, flight logs) that
  backs a change.

## Development environment

```bash
# C++ driver library
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build

# Python tooling
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

Linux is required to build the drivers: they talk to real devices through
`/dev/spidev*`, `/dev/i2c-*`, and `/dev/tty*`. On machines without those
nodes the code still compiles; unit-level review is possible without hardware.

## Code style

### C++ (drivers under `hardware/`)

- C++17. Headers use `#pragma once` and live next to their implementation.
- All project code lives in `namespace ade::<subsystem>` (for example
  `ade::sensors`).
- No exceptions and no RTTI in driver code: it must stay usable from
  real-time contexts. Report failures through return values and log to stderr.
- Prefer fixed-width types (`uint8_t`, `int32_t`) in protocol code.
- Every register constant, unit conversion, and protocol offset gets a comment
  naming its datasheet source. If you are unsure of a constant, say so in a
  `// TODO(datasheet-verify):` comment rather than guessing silently.
- Format with clang-format; the style follows Google with a 100-column limit.

### Python (tooling)

- Format with `black`, line length 100.
- Lint with `ruff` (config in `pyproject.toml`).
- Type hints for all public functions.

### YAML (all configuration files)

- Two-space indent, comments above every tunable, and units in the comment or
  the key name. Defaults must match the values used in the documentation.

### Documentation and diagrams

- Markdown files wrap at roughly 80 columns.
- draw.io files must keep a page per diagram and meaningful cell names; export
  to PDF when a schematic changes so the PDF stays authoritative.

## Commit messages

Use [Conventional Commits](https://www.conventionalcommits.org/):

```text
feat(sensors): add UBX NAV-POSLLH fallback parsing
fix(power): correct INA226 shunt scaling at high currents
docs(frame): clarify motor mount bolt pattern in drawing
```

## Pull request process

1. Fork, create a topic branch (`feat/imu-temperature-comp`).
2. Make the change; keep commits atomic and titled per the convention above.
3. Run the build and lint steps, and any hardware validation you can.
4. Update the affected documentation and `CHANGELOG.md` (Unreleased section).
5. Open the pull request and complete the checklist below.

**Pull request checklist**

- [ ] Build passes: `cmake --build build` with no new warnings.
- [ ] Config defaults match documentation values.
- [ ] New/changed constants cite their datasheet source.
- [ ] Documentation and CHANGELOG updated.
- [ ] No safety-critical default changed without validation data.

## Hardware and flight-test safety

Any change that can affect flight behavior follows the staged protocol:

1. **Bench, props off.** Driver and loop changes are validated with props
   removed and the battery current-limited.
2. **Tethered.** New control gains get a tethered or hand-held (by an
   experienced operator) test.
3. **Supervised flight.** First flights happen at an approved field with a
   flight log recorded, then the log is attached to the PR.

Never disable a failsafe to make a test pass. Failsafe changes need two
reviewers.

## Reporting bugs

Open an issue with: hardware revision (schematic rev letter), component
batch/date code if relevant, software version from `CHANGELOG.md`, steps to
reproduce, and full logs. For anything that could create a hazardous
condition, follow `SECURITY.md` instead of a public issue.
