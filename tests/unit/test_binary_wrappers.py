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
    assert call["binary_path"] == "arm-test"


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
    assert call["options"]["radius-max"] == pytest.approx(0.02)
    assert call["binary_path"] == "leg-test"


def test_missing_binary_has_an_actionable_error():
    missing = Path(__file__).parent / "definitely-missing"

    with pytest.raises(FileNotFoundError, match="Compiled arm binary"):
        binary_bridge.resolve_binary("arm", missing)
