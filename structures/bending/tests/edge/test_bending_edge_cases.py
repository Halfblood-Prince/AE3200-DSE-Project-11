from math import isinf

import pytest

from bending import Leg


pytestmark = pytest.mark.edge


def test_zero_vehicle_mass_has_no_leg_load_and_infinite_buckling_margin():
    leg = Leg(0.0)

    leg.calculate_max_bending_stress()

    assert leg.bending_force == pytest.approx(0.0)
    assert leg.axial_force == pytest.approx(0.0)
    assert leg.sigma_z_max == pytest.approx(0.0)
    assert isinf(leg.buckling_margin)
    assert leg.buckling_safe


def test_optimizer_reports_when_no_candidate_is_feasible(capsys):
    leg = Leg(4.5)

    result = leg.minimise_mass(
        angle_min=60.0,
        angle_max=60.0,
        R_min=0.001,
        R_max=0.001,
        max_tip_deflection=0.0,
    )

    assert result is None
    assert leg.checked_designs == 1
    assert "No feasible leg design found" in capsys.readouterr().out


def test_optimizer_accepts_an_explicit_tip_deflection_override():
    leg = Leg(4.5)
    leg.max_compressive_deformation = 1.0
    leg.yield_strength = float("inf")

    result = leg.minimise_mass(
        angle_min=5.0,
        angle_max=5.0,
        R_min=0.001,
        R_max=0.001,
        max_tip_deflection=1.0,
    )

    assert result is not None
    assert result["tip_deflection"] > leg.max_tip_deflection


def test_extreme_effective_length_can_make_a_leg_buckle():
    leg = Leg(4.5)
    leg.effective_length_factor = 1000.0

    leg.calculate_euler_buckling()

    assert not leg.buckling_safe
    assert leg.buckling_margin < 1.0
