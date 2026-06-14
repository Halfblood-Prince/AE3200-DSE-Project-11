import pathlib
import sys

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

import Sizing_tool as sizing_tool


def _get_sizing_inputs(**overrides):
    inputs = {
        "coaxial": False,
        "N_prop": 4,
        "flight_time": 0.25,
        "P_payload": 80.0,
        "P_avionics": 15.0,
        "Lipo_spec_energy": 220.0,
        "M_pay": 0.8,
    }
    inputs.update(overrides)
    return inputs


def _unexpected_call(*_args, **_kwargs):
    raise AssertionError("This dependency should not be called in this test.")


def test_SIZE_TL_MOD_UT_01(monkeypatch, capsys):
    load_calls = []

    def fake_load_propeller_dict(path):
        load_calls.append(path)
        return {}

    monkeypatch.setattr(sizing_tool, "load_propeller_dict", fake_load_propeller_dict)

    result = sizing_tool.run_sizing_tool(0.0, **_get_sizing_inputs())

    assert result is False
    assert "Initial MTOM guess must be positive." in capsys.readouterr().out
    assert load_calls == []


def test_SIZE_TL_MOD_UT_02(monkeypatch):
    monkeypatch.setattr(sizing_tool, "load_propeller_dict", lambda _path: {"mock": object()})
    monkeypatch.setattr(
        sizing_tool,
        "find_prop",
        lambda MTOW, N_prop, props, coaxial: ("", None, {}),
    )
    monkeypatch.setattr(sizing_tool, "motor_mass", _unexpected_call)
    monkeypatch.setattr(sizing_tool, "ESC_mass", _unexpected_call)
    monkeypatch.setattr(sizing_tool, "power_after_efficiencies", _unexpected_call)
    monkeypatch.setattr(sizing_tool, "battery_sizing", _unexpected_call)
    monkeypatch.setattr(sizing_tool, "struct_size", _unexpected_call)

    result = sizing_tool.run_sizing_tool(1.5, **_get_sizing_inputs(), test=True)

    assert result is False


def test_SIZE_TL_MOD_UT_03(monkeypatch, capsys):
    monkeypatch.setattr(sizing_tool, "load_propeller_dict", lambda _path: {"mock": object()})
    monkeypatch.setattr(
        sizing_tool,
        "find_prop",
        lambda MTOW, N_prop, props, coaxial: (
            "mock_prop",
            {
                "Power_required": 120.0,
                "OEI_condition": (6000, 9.0),
                "data": object(),
            },
            {"mock_prop": object()},
        ),
    )
    monkeypatch.setattr(sizing_tool, "motor_mass", lambda _info: (0.0, 0.0))
    monkeypatch.setattr(sizing_tool, "ESC_mass", lambda _current: 0.0)
    monkeypatch.setattr(sizing_tool, "power_after_efficiencies", lambda *_args: 100.0)
    monkeypatch.setattr(sizing_tool, "battery_sizing", lambda *_args: 12.0)
    monkeypatch.setattr(sizing_tool, "struct_size", lambda *_args, **_kwargs: 0.0)

    result = sizing_tool.run_sizing_tool(1.0, **_get_sizing_inputs(), test=True)

    assert result is False
    assert "MTOM diverging, check for errors." in capsys.readouterr().out


def test_SIZE_TL_MOD_UT_04(monkeypatch):
    calls = {}

    def fake_load_propeller_dict(path):
        calls["path"] = path
        return {"mock": object()}

    def fake_find_prop(MTOW, N_prop, props, coaxial):
        calls["find_prop"] = {
            "MTOW": MTOW,
            "N_prop": N_prop,
            "props": props,
            "coaxial": coaxial,
        }
        return "", None, {}

    monkeypatch.setattr(sizing_tool, "load_propeller_dict", fake_load_propeller_dict)
    monkeypatch.setattr(sizing_tool, "find_prop", fake_find_prop)
    monkeypatch.setattr(sizing_tool, "motor_mass", _unexpected_call)
    monkeypatch.setattr(sizing_tool, "ESC_mass", _unexpected_call)
    monkeypatch.setattr(sizing_tool, "power_after_efficiencies", _unexpected_call)
    monkeypatch.setattr(sizing_tool, "battery_sizing", _unexpected_call)
    monkeypatch.setattr(sizing_tool, "struct_size", _unexpected_call)

    result = sizing_tool.run_sizing_tool(
        1.0,
        **_get_sizing_inputs(coaxial=True, N_prop=8),
        test=True,
    )

    assert result is False
    assert calls["path"] == "propulsion/6.0_4pitch_E_1000.csv"
    assert calls["find_prop"]["coaxial"] is True
    assert calls["find_prop"]["N_prop"] == 8


def test_SIZE_TL_MOD_MT_01(monkeypatch):
    load_paths = []
    mtow_calls = []
    power_calls = []
    battery_calls = []
    structure_calls = []

    def fake_load_propeller_dict(path):
        load_paths.append(path)
        return {"mock": object()}

    def fake_find_prop(MTOW, N_prop, props, coaxial):
        mtow_calls.append((MTOW, N_prop, props, coaxial))
        return (
            "mock_prop",
            {
                "Power_required": 120.0,
                "OEI_condition": (6000, 9.0),
                "data": object(),
            },
            {"mock_prop": object()},
        )

    def fake_power_after_efficiencies(P_payload, P_avionics, P_prop):
        power_calls.append((P_payload, P_avionics, P_prop))
        return 240.0

    def fake_battery_sizing(P_bat, flight_time, Lipo_spec_energy):
        battery_calls.append((P_bat, flight_time, Lipo_spec_energy))
        return 1.0

    def fake_struct_size(MTOM, thrust, length, height, n, coaxial):
        structure_calls.append((MTOM, thrust, length, height, n, coaxial))
        return 0.6

    monkeypatch.setattr(sizing_tool, "load_propeller_dict", fake_load_propeller_dict)
    monkeypatch.setattr(sizing_tool, "find_prop", fake_find_prop)
    monkeypatch.setattr(sizing_tool, "motor_mass", lambda _info: (0.1, 10.0))
    monkeypatch.setattr(sizing_tool, "ESC_mass", lambda current: 0.05 if current == 10.0 else None)
    monkeypatch.setattr(sizing_tool, "power_after_efficiencies", fake_power_after_efficiencies)
    monkeypatch.setattr(sizing_tool, "battery_sizing", fake_battery_sizing)
    monkeypatch.setattr(sizing_tool, "struct_size", fake_struct_size)

    result = sizing_tool.run_sizing_tool(2.0, **_get_sizing_inputs(), test=True)

    (
        mtom_list,
        p_prop_list,
        m_battery_list,
        m_structures_list,
        residual_list,
        final_mtom,
    ) = result

    assert load_paths == ["propulsion/4.0_E_3000.csv"]
    assert len(mtow_calls) == 2
    assert mtow_calls[0][0] == pytest.approx(2.0 * 9.81)
    assert mtow_calls[1][0] == pytest.approx(3.0 * 9.81)
    assert all(call[1] == 4 for call in mtow_calls)
    assert all(call[3] is False for call in mtow_calls)

    assert mtom_list == pytest.approx([2.0, 3.0])
    assert p_prop_list == [120.0, 120.0]
    assert m_battery_list == [1.0, 1.0]
    assert m_structures_list == [0.6, 0.6]
    assert residual_list == pytest.approx([1.0, 0.0])
    assert final_mtom == pytest.approx(3.0)

    assert power_calls == [(80.0, 15.0, 120.0), (80.0, 15.0, 120.0)]
    assert battery_calls == [(240.0, 0.25, 220.0), (240.0, 0.25, 220.0)]
    assert structure_calls[0] == (2.0, 9.0, 0.1, 0.3, 4, False)
    assert structure_calls[1][0] == pytest.approx(3.0)
    assert structure_calls[1][1:] == (9.0, 0.1, 0.3, 4, False)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
