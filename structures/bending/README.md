# Bending Structural Models

[![Unit Tests](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/unit-tests.yml/badge.svg?branch=Iterative_tool)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/unit-tests.yml)
[![Subsystem Tests](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/subsystem-tests.yml/badge.svg?branch=Iterative_tool)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/subsystem-tests.yml)
[![System Tests](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/system-tests.yml/badge.svg?branch=Iterative_tool)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/system-tests.yml)
[![Edge Case Tests](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/edge-tests.yml/badge.svg?branch=Iterative_tool)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/edge-tests.yml)
[![Coverage Check](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/coverage-check.yml/badge.svg?branch=Iterative_tool)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/coverage-check.yml)
[![C++ Build](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/build-bending-cpp.yml/badge.svg?branch=Iterative_tool)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/build-bending-cpp.yml)
[![CodeQL](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/codeql.yml/badge.svg?branch=Iterative_tool)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/codeql.yml)

This folder contains the arm-bending and landing-leg sizing tools for the
AE3200 design project. It includes reference Python calculations, interactive
leg optimization, standalone C++ models, native Python wrappers, tests, and
precompiled binaries.

All calculations use SI units internally.

## Folder Contents

- `bending.py`: reference Python `Arm` and `Leg` calculations.
- `leg_optimizer_gui.py`: Tk GUI for evaluating and optimizing a leg.
- `materials.py`: material properties, gravity, safety factors, and deformation
  limits.
- `arm_wrapper.py`: Python interface to the compiled arm model.
- `leg_wrapper.py`: Python interface to the compiled leg model.
- `binary_bridge.py`: native-platform detection and binary execution.
- `cpp/arm.cpp`: compiled arm calculation and mass optimization.
- `cpp/leg.cpp`: compiled leg calculation and mass optimization.
- `bin/`: Linux, macOS, and Windows binaries for AMD64 and ARM64.
- `tests/`: unit, subsystem, system, and edge-case tests.

## Installation

From `structures/bending/`, create an environment and install dependencies:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

Install development dependencies when running tests:

```powershell
python -m pip install -r requirements-dev.txt
```

## Material Configuration

Default properties and design limits are defined in `materials.py`.

The `arm` class provides:

- Density and Young's modulus
- Failure stress
- Safety factor
- Maximum allowable tip deflection

The `leg` class provides:

- Density and Young's modulus
- Yield strength
- Safety factor
- Maximum allowable tip deflection
- Maximum allowable axial compression

The `constants` class provides gravitational acceleration.

Both compiled-model wrappers accept another file with the same structure:

```python
from arm_wrapper import Arm
from leg_wrapper import Leg

arm = Arm(materials_file="materials.py")
leg = Leg(materials_file="materials.py")
```

Explicit wrapper arguments override values loaded from the material file:

```python
arm = Arm(materials_file="materials.py", safety_factor=2.0)
leg = Leg(materials_file="materials.py", density=2800)
```

## Optional Override Reference

Values passed directly to a wrapper take precedence over values loaded from
`materials_file`. Parameters left as `None` use the relevant value from the
material file.

### Arm Constructor Overrides

| Parameter | Unit | Default | Purpose |
| --- | --- | --- | --- |
| `thickness` | m | `0.002` | Tube wall thickness used by `calculate()` |
| `radius` | m | `0.01` | Tube outer radius used by `calculate()` |
| `length` | m | `0.25` | Arm length |
| `thrust` | N | `10.0` | Point force applied at the arm tip |
| `safety_factor` | - | `materials.arm.safety_factor` | Bending-stress safety factor |
| `materials_file` | path | Local `materials.py` | Alternative material configuration |
| `density` | kg/m3 | `materials.arm.density` | Arm material density |
| `youngs_modulus` | Pa | `materials.arm.youngs_modulus` | Arm elastic modulus |
| `failure_stress` | Pa | `materials.arm.failure_stress` | Allowable material failure stress |
| `gravity` | m/s2 | `materials.constants.g` | Gravitational acceleration |
| `max_tip_deflection` | m | `materials.arm.max_tip_deflection` | Tip-deflection constraint |
| `binary_path` | path | Auto-detected | Explicit compiled arm executable |

Example with every constructor override:

```python
arm = Arm(
    thickness=0.0015,
    radius=0.012,
    length=0.30,
    thrust=12.0,
    safety_factor=2.0,
    materials_file="materials.py",
    density=1650,
    youngs_modulus=135e9,
    failure_stress=1.2e9,
    gravity=9.81,
    max_tip_deflection=0.004e-3,
    binary_path="bin/arm-windows-amd64.exe",
)
```

### Arm Method Overrides

`Arm.calculate()` accepts:

| Parameter | Unit | Default | Purpose |
| --- | --- | --- | --- |
| `L` | m | `arm.length` | Length override for this call |
| `T` | N | `arm.thrust` | Tip-force override for this call |

`Arm.minimise_mass()` accepts:

| Parameter | Unit | Default | Purpose |
| --- | --- | --- | --- |
| `radius_min` | m | `0.002` | Minimum searched outer radius |
| `radius_max` | m | `0.05` | Maximum searched outer radius |
| `radius_step` | m | `0.0001` | Outer-radius search increment |
| `thickness_min` | m | `0.0005` | Minimum searched wall thickness |
| `thickness_step` | m | `0.0001` | Wall-thickness search increment |
| `L` | m | `arm.length` | Length override for this optimization |
| `T` | N | `arm.thrust` | Tip-force override for this optimization |

### Leg Constructor Overrides

| Parameter | Unit | Default | Purpose |
| --- | --- | --- | --- |
| `vehicle_mass` | kg | `4.5` | Total supported vehicle mass |
| `radius` | m | `0.005` | Leg radius used by `calculate()` |
| `angle_deg` | deg | `30.0` | Leg angle used by `calculate()` |
| `length` | m | `0.1` | Leg length |
| `safety_factor` | - | `materials.leg.safety_factor` | Load and mass safety factor |
| `number_of_legs` | - | `2` | Number of legs sharing the vehicle load |
| `materials_file` | path | Local `materials.py` | Alternative material configuration |
| `density` | kg/m3 | `materials.leg.density` | Leg material density |
| `youngs_modulus` | Pa | `materials.leg.youngs_modulus` | Leg elastic modulus |
| `yield_strength` | Pa | `materials.leg.yield_strength` | Yield-stress constraint |
| `gravity` | m/s2 | `materials.constants.g` | Gravitational acceleration |
| `effective_length_factor` | - | `1.0` | Euler-buckling effective-length factor |
| `max_tip_deflection` | m | `materials.leg.max_tip_deflection` | Lateral tip-deflection constraint |
| `max_compressive_deformation` | m | `materials.leg.max_compressive_deformation` | Axial-compression constraint |
| `binary_path` | path | Auto-detected | Explicit compiled leg executable |

Example with every constructor override:

```python
leg = Leg(
    vehicle_mass=5.0,
    radius=0.006,
    angle_deg=25.0,
    length=0.12,
    safety_factor=2.0,
    number_of_legs=2,
    materials_file="materials.py",
    density=2750,
    youngs_modulus=70e9,
    yield_strength=280e6,
    gravity=9.81,
    effective_length_factor=2.0,
    max_tip_deflection=0.004e-3,
    max_compressive_deformation=0.008e-3,
    binary_path="bin/leg-windows-amd64.exe",
)
```

### Leg Method Overrides

`Leg.calculate()` uses the constructor configuration and has no method-level
override parameters.

`Leg.minimise_mass()` accepts:

| Parameter | Unit | Default | Purpose |
| --- | --- | --- | --- |
| `angle_min` | deg | `5.0` | Minimum searched leg angle |
| `angle_max` | deg | `60.0` | Maximum searched leg angle |
| `angle_step` | deg | `1.0` | Leg-angle search increment |
| `radius_min` | m | `0.001` | Minimum searched leg radius |
| `radius_max` | m | `0.05` | Maximum searched leg radius |
| `radius_step` | m | `0.0001` | Leg-radius search increment |

## Compiled Arm Wrapper

Create an arm using the local material configuration:

```python
from arm_wrapper import Arm

arm = Arm()
```

### Calculate One Arm

```python
arm_result = arm.calculate()
```

`calculate()` evaluates the configured outer radius, wall thickness, length,
and tip point force. It returns a dictionary containing:

- Cross-sectional area and second moment of area
- Distributed self-weight
- Maximum shear force and its position
- Maximum bending moment and its position
- Maximum bending stress
- Tip deflection
- Arm mass
- Overall feasibility

Length `L` and tip point force `T` can be overridden for one call:

```python
arm_result = arm.calculate(L=0.30, T=12.0)
```

`L` is in metres and `T` is in newtons. The override does not modify the
stored `arm.length` or `arm.thrust`.

### Minimize Arm Mass

```python
best_arm = arm.minimise_mass()
```

The optimizer searches outer radius and wall thickness and selects the
lowest-mass design satisfying:

- Safety-factor-adjusted bending-stress limit
- Maximum tip-deflection limit

The returned dictionary contains `found`, `checked_designs`, `radius`,
`thickness`, and the complete calculated `design`. If no candidate is
feasible, `found` is `False`.

Search bounds and operating loads are configurable:

```python
best_arm = arm.minimise_mass(
    radius_min=0.002,
    radius_max=0.05,
    radius_step=0.0001,
    thickness_min=0.0005,
    thickness_step=0.0001,
    L=0.30,
    T=12.0,
)
```

## Compiled Leg Wrapper

Create and evaluate a leg:

```python
from leg_wrapper import Leg

leg = Leg(vehicle_mass=4.5)
leg_result = leg.calculate()
```

`calculate()` returns geometry, bending and axial forces, stresses,
deflections, mass, Euler buckling load, buckling margin, buckling safety, and
overall feasibility.

The vehicle load is shared across `number_of_legs`, which defaults to two.

### Minimize Leg Mass

```python
best_leg = Leg(vehicle_mass=4.5).minimise_mass()
```

The optimizer searches leg angle and radius for the lowest-mass design
satisfying:

- Tip-deflection limit
- Axial-compression limit
- Yield-strength limit
- Euler-buckling requirement

Search bounds can be changed:

```python
best_leg = leg.minimise_mass(
    angle_min=5.0,
    angle_max=60.0,
    angle_step=1.0,
    radius_min=0.001,
    radius_max=0.05,
    radius_step=0.0001,
)
```

The result contains `found`, `checked_designs`, and the complete optimized
`design` when a feasible candidate exists.

## Native Binaries

`binary_bridge.py` automatically selects the binary matching the current
operating system and CPU architecture:

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

A custom local binary can be supplied for development:

```python
arm = Arm(binary_path="path/to/arm.exe")
leg = Leg(binary_path="path/to/leg.exe")
```

The executables expose `calculate` and `optimize` commands and return JSON.

## Python Reference Model

Run the reference leg optimizer:

```powershell
python bending.py
```

The reference model uses the same material limits in `materials.py`. It is
also used by the Python test suite to validate formulas and optimization
behavior.

## Leg Optimizer GUI

Run the interactive GUI:

```powershell
python leg_optimizer_gui.py
```

The GUI allows the user to change leg radius, angle, vehicle mass, leg length,
safety factor, Euler effective-length factor, and deformation limits. It
reports forces, stresses, deflections, mass, buckling performance, and overall
feasibility.

## Testing

Run the complete test suite from `structures/bending/`:

```powershell
python -m pytest
```

Run an individual test layer:

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

CI enforces at least 90% branch coverage.

## Continuous Integration

The workflows on the `Iterative_tool` branch:

- Run unit, subsystem, system, and edge-case tests independently.
- Enforce the `bending.py` coverage threshold.
- Build arm and leg binaries for Linux, macOS, and Windows on AMD64 and ARM64.
- Replace and commit the generated files under `structures/bending/bin/`.
- Analyze GitHub Actions, Python, and C++ with CodeQL.
- Run repository hygiene checks.

## License

See [LICENSE](LICENSE).
