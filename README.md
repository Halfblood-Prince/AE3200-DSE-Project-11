# AE3200 DSE Project 11

[![CI](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/ci.yml/badge.svg?branch=Iterative_tool)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/ci.yml)
[![Unit Tests](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/unit-tests.yml/badge.svg?branch=Iterative_tool)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/unit-tests.yml)
[![Subsystem Tests](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/subsystem-tests.yml/badge.svg?branch=Iterative_tool)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/subsystem-tests.yml)
[![System Tests](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/system-tests.yml/badge.svg?branch=Iterative_tool)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/system-tests.yml)
[![Edge Case Tests](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/edge-tests.yml/badge.svg?branch=Iterative_tool)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/edge-tests.yml)
[![Coverage Check](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/coverage-check.yml/badge.svg?branch=Iterative_tool)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/coverage-check.yml)
[![C++ Build](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/build-bending-cpp.yml/badge.svg?branch=Iterative_tool)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/build-bending-cpp.yml)
[![CodeQL](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/codeql.yml/badge.svg?branch=Iterative_tool)](https://github.com/Halfblood-Prince/AE3200-DSE-Project-11/actions/workflows/codeql.yml)

Official repository of Group 11 for the AE3200 DSE 2025-26 project.

This repository contains the current UAV sizing workflow together with the
propulsion, EPS, and structural models used to estimate vehicle mass and
check feasibility. The main integrated sizing loop is implemented in
`Sizing_tool.py` and couples:

- propeller selection and motor/ESC estimates from `propulsion/`
- battery and power calculations from `EPS/`
- arm and landing-gear structural sizing from `structures/`

All calculations use SI units unless a file explicitly states otherwise.

## Repository Layout

- `Sizing_tool.py`: top-level iterative sizing loop.
- `EPS/`: electrical power and battery sizing utilities.
- `propulsion/`: propeller data, propeller selection, and CSV generation tools.
- `structures/`: structural mass estimation, shear sizing, and bending models.
- `structures/bending/`: detailed arm and landing-leg wrappers, binaries, and
  tests.
- `tests/`: repository-wide tests and the custom test runner.
- `aerodynamics/`: aerodynamic analysis files.
- `stability_and_control/`: flight-dynamics and control work.
- `tools/`: supporting project utilities.

## Branches

- [Main branch](../../tree/main)
- [ROS](../../tree/ros_simulation): complete UAV simulation in ROS2
- [Iterative Tool](../../tree/Iterative_tool): active development branch for the
  integrated sizing workflow, including `Sizing_tool.py`, test automation, and
  validation of the coupled propulsion, EPS, and structures models
- [Sensitivity Analysis](../../tree/sensitivity_analysis): Monte Carlo
  sensitivity-analysis work

## Running the Sizing Tool

Run the integrated sizing loop from the repository root:

```powershell
python Sizing_tool.py
```

The script currently evaluates the coaxial baseline case in its `__main__`
entrypoint. For custom studies, import `run_sizing_tool(...)` and provide your
own mission and configuration inputs.

## Testing

Install the repository test and coverage dependencies from the repository root:

```powershell
python -m pip install -r requirements-dev.txt
```

Run the repository test suite from the repository root:

```powershell
python tests/test_main.py
```

The custom runner:

- prints the test script being executed
- prints per-file totals for passed, failed, and skipped tests
- prints a full end-of-run summary
- stores temporary pytest files outside the repository and cleans them up after
  the run

`tests/test_main.py` keeps `FAST = False` by default, so the full suite runs,
including `test_SIZE_ST_04`. If you want a quicker local run, you can
temporarily switch `FAST` to `True` to skip that expensive system test.

Run one test file directly with `pytest`:

```powershell
python -m pytest "tests/System tests/System_tests.py"
python -m pytest tests/propulsion/test_Iter_function.py
```

Run only the repository test runner output without changing the default suite:

```powershell
python tests/test_main.py -q
```

Run repository-wide coverage and print the coverage distribution per module:

```powershell
python tests/run_coverage.py
```

Useful coverage options:

- `python tests/run_coverage.py --html`: create `htmlcov/index.html`
- `python tests/run_coverage.py --xml`: create `coverage.xml`
- `python tests/run_coverage.py --fast`: skip `test_SIZE_ST_04` for quicker local checks

The coverage report tracks the main sizing code in `Sizing_tool.py`, `EPS/`,
`propulsion/`, and `structures/`, and the terminal output includes both a
file-by-file report and a grouped module summary. The CI coverage workflow
currently enforces a 90% repository-wide coverage threshold for that sizing
code.

## Structural Bending Submodule

The detailed bending and landing-leg model has its own documentation in
[structures/bending/README.md](structures/bending/README.md). That README
covers:

- wrapper usage
- material-file configuration
- native binaries
- bending-specific test layers
- coverage and CI details

## Continuous Integration

The workflows referenced above currently check repository hygiene, run the
Python test layers, enforce repository-wide coverage for the sizing code, build native bending
binaries, and run CodeQL analysis.

## License

See [LICENSE](LICENSE).
