# AE3200 DSE Project 11

[![Bending Tests](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/bending-tests.yml/badge.svg)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/bending-tests.yml)
[![C++ Build](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/build-bending-cpp.yml/badge.svg)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/build-bending-cpp.yml)
[![CodeQL](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/codeql.yml/badge.svg)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/codeql.yml)
[![codecov](https://codecov.io/gh/Halfblood-Prince/AE3200-DSE-Project-11/graph/badge.svg)](https://codecov.io/gh/Halfblood-Prince/AE3200-DSE-Project-11)

Structural sizing tools for the AE3200 design project. The Python model
calculates arm bending response and sizes a two-leg landing system against
stress, tip-deflection, axial-deformation, and Euler-buckling constraints.
Equivalent C++ implementations provide portable command-line binaries with
Python wrappers for applications that need the compiled models.

## Models

- `Arm` models a hollow circular CFRP beam under propeller thrust and
  distributed self-weight.
- `Leg` models a solid circular Aluminium 6061-T6 landing leg.
- `Leg.minimise_mass()` searches leg angle and radius for the lightest
  feasible design.
- `leg_optimizer_gui.py` provides an interactive Tk GUI for evaluating and
  optimizing leg designs.
- `cpp/arm.cpp` and `cpp/leg.cpp` contain separate compiled implementations.
- `arm_wrapper.py` and `leg_wrapper.py` invoke the native binary and return
  its JSON result as a Python dictionary.

All structural dimensions use SI units internally. The default vehicle mass
is `4.5 kg`, the safety factor is `1.5`, and the vehicle load is shared across
two legs.

## Installation

Create a virtual environment and install the runtime dependencies:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

Run the command-line leg optimization:

```powershell
python bending.py
```

Run the GUI:

```powershell
python leg_optimizer_gui.py
```

## Compiled Models

The C++ executables use SI units and expose `calculate` and `optimize`
commands. The binary workflow builds both models for:

- Linux AMD64 and ARM64
- macOS AMD64 and ARM64
- Windows AMD64 and ARM64

After every matching branch push, the workflow removes the previous contents
of `bin/`, downloads all matrix outputs, and commits these files:

```text
arm-linux-amd64
arm-linux-arm64
arm-macos-amd64
arm-macos-arm64
arm-windows-amd64.exe
arm-windows-arm64.exe
leg-linux-amd64
leg-linux-arm64
leg-macos-amd64
leg-macos-arm64
leg-windows-amd64.exe
leg-windows-arm64.exe
```

Use the native binary from Python through the wrappers:

```python
from arm_wrapper import Arm
from leg_wrapper import Leg

arm_result = Arm().calculate()
leg_result = Leg(vehicle_mass=4.5).calculate()
best_leg = Leg(vehicle_mass=4.5).minimise_mass()
```

The wrappers select the current operating system and CPU architecture
automatically. A custom executable can be supplied with `binary_path` for
local builds or testing.

## Testing

Install the development dependencies:

```powershell
python -m pip install -r requirements-dev.txt
```

The test suite is split by engineering scope:

- `tests/unit`: isolated cross-section, load, stress, deflection, mass, and
  buckling formulas.
- `tests/subsystem`: connected arm and leg analysis pipelines.
- `tests/system`: default end-to-end optimization and application entry point.
- `tests/edge`: zero-load, infeasible-design, overridden-limit, and buckling
  boundary cases.

Run all tests:

```powershell
python -m pytest
```

Run one test layer:

```powershell
python -m pytest tests/unit
python -m pytest tests/subsystem
python -m pytest tests/system
python -m pytest tests/edge
```

Check branch coverage for `bending.py`:

```powershell
python -m pytest --cov=bending --cov-branch --cov-report=term-missing
```

CI requires at least 90% branch coverage. The workflow also uploads
`coverage.xml` as a downloadable artifact and reports coverage to Codecov.

## Continuous Integration

The `Bending Tests` workflow runs every test layer independently and then runs
the complete suite with coverage. The `Build C++ Binaries` workflow performs
the six-platform matrix build and publishes `bin/`. CodeQL analyzes GitHub
Actions, Python, and C++ on pushes to `main`, `master`, and
`structures_bending`.

## License

See [LICENSE](LICENSE).
