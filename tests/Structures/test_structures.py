import pathlib
import sys

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

import structures.iter_function as structures_module


def test_SIZE_STR_MOD_UT_01(monkeypatch):
    calls = {}

    class FakeArm:
        def __init__(self, materials_file, length, thrust):
            calls["arm_init"] = (materials_file, length, thrust)
            self.thickness = 0.002
            self.failure_shear_stress = 90.0
            self.safety_factor = 1.5

        def calculate(self):
            calls["calculate_called"] = True
            return {"max_shear_force": -18.0}

        def minimise_mass(self, L, T, radius_min):
            calls["arm_minimise"] = (L, T, radius_min)
            return {
                "found": True,
                "radius": 0.01,
                "thickness": 0.002,
                "design": {"mass": 0.4},
            }

    class FakeLeg:
        def __init__(self, materials_file, height, vehicle_mass, max_tip_deflection):
            calls["leg_init"] = (materials_file, height, vehicle_mass, max_tip_deflection)
            self.number_of_legs = 4

        def minimise_mass(self):
            calls["leg_minimise_called"] = True
            return {
                "found": True,
                "design": {
                    "radius": 0.02,
                    "angle_deg": 25.0,
                    "height": 0.35,
                    "length": 0.3,
                    "mass": 0.15,
                },
            }

    def fake_shear_arm(V, t, tau_allow):
        calls["shear_call"] = (V, t, tau_allow)
        return 0.05

    monkeypatch.setattr(structures_module, "Arm", FakeArm)
    monkeypatch.setattr(structures_module, "Leg", FakeLeg)
    monkeypatch.setattr(structures_module, "shear_arm", fake_shear_arm)

    result = structures_module.struct_size(
        MTOM=5.0,
        thrust=12.0,
        length=0.6,
        height=0.35,
        n=8,
        coaxial=True,
    )

    assert result == pytest.approx(4 * 0.4 + 4 * 0.15 + 0.3)
    assert calls["calculate_called"] is True
    assert calls["leg_minimise_called"] is True
    assert calls["shear_call"] == (18.0, 0.002, 60.0)
    assert calls["arm_minimise"] == (0.6, 12.0, 0.05)
    assert calls["arm_init"][1:] == (0.6, 12.0)
    assert calls["leg_init"][1:] == (0.35, 5.0, 1)
    assert calls["arm_init"][0].name == "materials.py"
    assert calls["arm_init"][0].parent.name == "bending"
    assert calls["leg_init"][0] == calls["arm_init"][0]


def test_SIZE_STR_MOD_UT_02(monkeypatch):
    class FakeArm:
        def __init__(self, materials_file, length, thrust):
            self.thickness = 0.001
            self.failure_shear_stress = 75.0
            self.safety_factor = 1.5

        def calculate(self):
            return {"max_shear_force": 10.0}

        def minimise_mass(self, L, T, radius_min):
            return {
                "found": True,
                "radius": 0.01,
                "thickness": 0.001,
                "design": {"mass": 0.2},
            }

    class FakeLeg:
        def __init__(self, materials_file, height, vehicle_mass, max_tip_deflection):
            self.number_of_legs = 3

        def minimise_mass(self):
            return {
                "found": True,
                "design": {
                    "radius": 0.02,
                    "angle_deg": 20.0,
                    "height": 0.3,
                    "length": 0.25,
                    "mass": 0.1,
                },
            }

    monkeypatch.setattr(structures_module, "Arm", FakeArm)
    monkeypatch.setattr(structures_module, "Leg", FakeLeg)
    monkeypatch.setattr(structures_module, "shear_arm", lambda V, t, tau_allow: 0.03)

    result = structures_module.struct_size(
        MTOM=4.0,
        thrust=9.0,
        length=0.4,
        height=0.3,
        n=6,
        coaxial=False,
    )

    assert result == pytest.approx(6 * 0.2 + 3 * 0.1 + 0.3)


def test_SIZE_STR_MOD_UT_03(monkeypatch):
    class FakeArm:
        def __init__(self, materials_file, length, thrust):
            self.thickness = 0.001
            self.failure_shear_stress = 60.0
            self.safety_factor = 1.5

        def calculate(self):
            return {"max_shear_force": 8.0}

        def minimise_mass(self, L, T, radius_min):
            return {"found": False}

    class FakeLeg:
        def __init__(self, materials_file, height, vehicle_mass, max_tip_deflection):
            self.number_of_legs = 4

        def minimise_mass(self):
            return {
                "found": True,
                "design": {
                    "radius": 0.02,
                    "angle_deg": 20.0,
                    "height": 0.3,
                    "length": 0.25,
                    "mass": 0.1,
                },
            }

    monkeypatch.setattr(structures_module, "Arm", FakeArm)
    monkeypatch.setattr(structures_module, "Leg", FakeLeg)
    monkeypatch.setattr(structures_module, "shear_arm", lambda V, t, tau_allow: 0.02)

    with pytest.raises(
        RuntimeError,
        match="No feasible arm design found in the configured search range.",
    ):
        structures_module.struct_size(4.0, 9.0, 0.4)


def test_SIZE_STR_MOD_UT_04(monkeypatch):
    class FakeArm:
        def __init__(self, materials_file, length, thrust):
            self.thickness = 0.001
            self.failure_shear_stress = 60.0
            self.safety_factor = 1.5

        def calculate(self):
            return {"max_shear_force": 8.0}

        def minimise_mass(self, L, T, radius_min):
            return {
                "found": True,
                "radius": 0.01,
                "thickness": 0.001,
                "design": {"mass": 0.2},
            }

    class FakeLeg:
        def __init__(self, materials_file, height, vehicle_mass, max_tip_deflection):
            self.number_of_legs = 4

        def minimise_mass(self):
            return {"found": False}

    monkeypatch.setattr(structures_module, "Arm", FakeArm)
    monkeypatch.setattr(structures_module, "Leg", FakeLeg)
    monkeypatch.setattr(structures_module, "shear_arm", lambda V, t, tau_allow: 0.02)

    with pytest.raises(
        RuntimeError,
        match="No feasible leg design found in the configured search range.",
    ):
        structures_module.struct_size(4.0, 9.0, 0.4)


def test_SIZE_STR_MOD_UT_05(monkeypatch, capsys):
    class FakeArm:
        def __init__(self, materials_file, length, thrust):
            self.thickness = 0.002
            self.failure_shear_stress = 90.0
            self.safety_factor = 1.5

        def calculate(self):
            return {"max_shear_force": 14.0}

        def minimise_mass(self, L, T, radius_min):
            return {
                "found": True,
                "radius": 0.012,
                "thickness": 0.002,
                "design": {"mass": 0.35},
            }

    class FakeLeg:
        def __init__(self, materials_file, height, vehicle_mass, max_tip_deflection):
            self.number_of_legs = 4

        def minimise_mass(self):
            return {
                "found": True,
                "design": {
                    "radius": 0.018,
                    "angle_deg": 22.0,
                    "height": 0.31,
                    "length": 0.28,
                    "mass": 0.12,
                },
            }

    monkeypatch.setattr(structures_module, "Arm", FakeArm)
    monkeypatch.setattr(structures_module, "Leg", FakeLeg)
    monkeypatch.setattr(structures_module, "shear_arm", lambda V, t, tau_allow: 0.025)

    structures_module.struct_size(4.5, 11.0, 0.45, verbose=True)

    output = capsys.readouterr().out
    assert "Optimal arm parameters:" in output
    assert "Optimal leg parameters:" in output
    assert "Mass per arm:" in output
    assert "Mass per leg:" in output


def test_SIZE_STR_MOD_MT_01(monkeypatch):
    call_sequence = []

    class FakeArm:
        def __init__(self, materials_file, length, thrust):
            call_sequence.append(("arm_init", materials_file.name, length, thrust))
            self.thickness = 0.003
            self.failure_shear_stress = 150.0
            self.safety_factor = 3.0

        def calculate(self):
            call_sequence.append(("arm_calculate",))
            return {"max_shear_force": -21.0}

        def minimise_mass(self, L, T, radius_min):
            call_sequence.append(("arm_minimise", L, T, radius_min))
            return {
                "found": True,
                "radius": 0.014,
                "thickness": 0.003,
                "design": {"mass": 0.25},
            }

    class FakeLeg:
        def __init__(self, materials_file, height, vehicle_mass, max_tip_deflection):
            call_sequence.append(
                ("leg_init", materials_file.name, height, vehicle_mass, max_tip_deflection)
            )
            self.number_of_legs = 4

        def minimise_mass(self):
            call_sequence.append(("leg_minimise",))
            return {
                "found": True,
                "design": {
                    "radius": 0.02,
                    "angle_deg": 18.0,
                    "height": 0.32,
                    "length": 0.27,
                    "mass": 0.11,
                },
            }

    def fake_shear_arm(V, t, tau_allow):
        call_sequence.append(("shear_arm", V, t, tau_allow))
        return 0.04

    monkeypatch.setattr(structures_module, "Arm", FakeArm)
    monkeypatch.setattr(structures_module, "Leg", FakeLeg)
    monkeypatch.setattr(structures_module, "shear_arm", fake_shear_arm)

    result = structures_module.struct_size(
        MTOM=6.0,
        thrust=13.0,
        length=0.5,
        height=0.32,
        n=6,
        coaxial=False,
    )

    assert result == pytest.approx(6 * 0.25 + 4 * 0.11 + 0.3)
    assert call_sequence == [
        ("arm_init", "materials.py", 0.5, 13.0),
        ("leg_init", "materials.py", 0.32, 6.0, 1),
        ("arm_calculate",),
        ("shear_arm", 21.0, 0.003, 50.0),
        ("arm_minimise", 0.5, 13.0, 0.04),
        ("leg_minimise",),
    ]


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
