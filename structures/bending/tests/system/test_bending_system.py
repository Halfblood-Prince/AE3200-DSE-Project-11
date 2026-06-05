import pytest

import bending
from bending import Leg


pytestmark = pytest.mark.system


def test_default_leg_optimization_finds_a_feasible_minimum(capsys):
    leg = Leg(4.5)

    best = leg.minimise_mass()

    output = capsys.readouterr().out
    assert best is not None
    assert leg.checked_designs == 27_496
    assert leg.angle_deg == pytest.approx(5.0)
    assert leg.R == pytest.approx(0.0078)
    assert leg.tip_deflection <= leg.max_tip_deflection
    assert leg.longitudinal_deflection <= leg.max_compressive_deformation
    assert leg.maximum_normal_stress <= leg.yield_strength
    assert leg.buckling_safe
    assert leg.mass == pytest.approx(best["mass"])
    assert "Optimal leg mass:" in output


def test_main_runs_the_default_vehicle_analysis(monkeypatch):
    calls = []

    def record_optimization(self):
        calls.append(self.m)
        return {}

    monkeypatch.setattr(Leg, "minimise_mass", record_optimization)

    bending.main()

    assert calls == [4.5]
