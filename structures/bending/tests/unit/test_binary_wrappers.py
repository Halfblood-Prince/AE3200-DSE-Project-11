import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import arm_wrapper
import binary_bridge
import leg_wrapper


pytestmark = pytest.mark.unit


def test_native_target_normalizes_operating_system_and_architecture(
    monkeypatch,
):
    monkeypatch.setattr(binary_bridge.platform, "system", lambda: "Darwin")
    monkeypatch.setattr(binary_bridge.platform, "machine", lambda: "arm64")

    assert binary_bridge.native_target() == ("macos", "arm64")


def test_binary_bridge_invokes_binary_and_parses_json(
    monkeypatch,
):
    binary = Path("leg-test").resolve()
    calls = []

    def fake_run(arguments, **kwargs):
        calls.append((arguments, kwargs))
        return SimpleNamespace(stdout=json.dumps({"feasible": True}))

    monkeypatch.setattr(binary_bridge.subprocess, "run", fake_run)
    monkeypatch.setattr(
        binary_bridge,
        "resolve_binary",
        lambda component, binary_path: binary,
    )

    result = binary_bridge.run_binary(
        "leg",
        "calculate",
        {"mass": 4.5},
        binary,
    )

    assert result == {"feasible": True}
    assert calls[0][0] == [str(binary.resolve()), "calculate", "--mass", "4.5"]
    assert calls[0][1]["check"] is True


def test_arm_wrapper_passes_configuration_to_arm_binary(monkeypatch):
    call = {}

    def fake_run(component, command, options, binary_path):
        call.update(
            component=component,
            command=command,
            options=options,
            binary_path=binary_path,
        )
        return {"mass": 1.0}

    monkeypatch.setattr(arm_wrapper, "run_binary", fake_run)

    result = arm_wrapper.Arm(binary_path="arm-test").calculate()

    assert result == {"mass": 1.0}
    assert call["component"] == "arm"
    assert call["command"] == "calculate"
    assert call["options"]["thickness"] == pytest.approx(0.002)
    assert call["options"]["gravity"] == pytest.approx(9.81)
    assert call["binary_path"] == "arm-test"


def test_arm_wrapper_accepts_optional_length_and_tip_force(monkeypatch):
    calls = []

    def fake_run(component, command, options, binary_path):
        calls.append((command, options))
        return {}

    monkeypatch.setattr(arm_wrapper, "run_binary", fake_run)
    arm = arm_wrapper.Arm(
        length=0.25,
        thrust=10.0,
        binary_path="arm-test",
    )

    arm.calculate(L=0.4, T=18.0)
    arm.minimise_mass(L=0.35, T=15.0)

    assert calls[0][0] == "calculate"
    assert calls[0][1]["length"] == pytest.approx(0.4)
    assert calls[0][1]["thrust"] == pytest.approx(18.0)
    assert calls[1][0] == "optimize"
    assert calls[1][1]["length"] == pytest.approx(0.35)
    assert calls[1][1]["thrust"] == pytest.approx(15.0)
    assert arm.length == pytest.approx(0.25)
    assert arm.thrust == pytest.approx(10.0)


def test_leg_wrapper_passes_search_range_to_leg_binary(monkeypatch):
    call = {}

    def fake_run(component, command, options, binary_path):
        call.update(
            component=component,
            command=command,
            options=options,
            binary_path=binary_path,
        )
        return {"found": True}

    monkeypatch.setattr(leg_wrapper, "run_binary", fake_run)

    result = leg_wrapper.Leg(binary_path="leg-test").minimise_mass(
        radius_max=0.02
    )

    assert result == {"found": True}
    assert call["component"] == "leg"
    assert call["command"] == "optimize"
    assert call["options"]["number-of-legs"] == 2
    assert call["options"]["gravity"] == pytest.approx(9.81)
    assert call["options"]["radius-max"] == pytest.approx(0.02)
    assert call["binary_path"] == "leg-test"


def test_wrappers_load_material_properties_from_supplied_file(monkeypatch):
    calls = []
    materials_file = (
        Path(__file__).parents[1] / "data" / "custom_materials.py"
    )

    def fake_run(component, command, options, binary_path):
        calls.append((component, options))
        return {}

    monkeypatch.setattr(arm_wrapper, "run_binary", fake_run)
    monkeypatch.setattr(leg_wrapper, "run_binary", fake_run)

    arm_wrapper.Arm(materials_file=materials_file).calculate()
    leg_wrapper.Leg(materials_file=materials_file).calculate()

    arm_options = calls[0][1]
    leg_options = calls[1][1]
    assert arm_options["density"] == pytest.approx(1234.0)
    assert arm_options["youngs-modulus"] == pytest.approx(88e9)
    assert arm_options["failure-stress"] == pytest.approx(700e6)
    assert arm_options["safety-factor"] == pytest.approx(1.8)
    assert arm_options["max-tip-deflection"] == pytest.approx(0.002e-3)
    assert arm_options["gravity"] == pytest.approx(3.71)
    assert leg_options["density"] == pytest.approx(4321.0)
    assert leg_options["youngs-modulus"] == pytest.approx(55e9)
    assert leg_options["yield-strength"] == pytest.approx(210e6)
    assert leg_options["safety-factor"] == pytest.approx(2.1)
    assert leg_options["max-tip-deflection"] == pytest.approx(0.003e-3)
    assert leg_options["max-compressive-deformation"] == pytest.approx(
        0.004e-3
    )
    assert leg_options["gravity"] == pytest.approx(3.71)


def test_explicit_material_value_overrides_supplied_file(monkeypatch):
    call = {}
    materials_file = (
        Path(__file__).parents[1] / "data" / "custom_materials.py"
    )

    def fake_run(component, command, options, binary_path):
        call.update(options)
        return {}

    monkeypatch.setattr(arm_wrapper, "run_binary", fake_run)

    arm_wrapper.Arm(
        materials_file=materials_file,
        density=999.0,
        safety_factor=1.2,
    ).calculate()

    assert call["density"] == pytest.approx(999.0)
    assert call["youngs-modulus"] == pytest.approx(88e9)
    assert call["safety-factor"] == pytest.approx(1.2)


def test_missing_binary_has_an_actionable_error():
    missing = Path(__file__).parent / "definitely-missing"

    with pytest.raises(FileNotFoundError, match="Compiled arm binary"):
        binary_bridge.resolve_binary("arm", missing)


def test_missing_materials_file_has_an_actionable_error():
    missing = Path(__file__).parent / "missing-materials.py"

    with pytest.raises(FileNotFoundError, match="Materials file not found"):
        arm_wrapper.Arm(materials_file=missing)
