from math import cos, pi, sin

import pytest

from bending import Leg


pytestmark = pytest.mark.unit


def test_leg_full_calculation_matches_closed_form_equations():
    leg = Leg(4.5)

    leg.calculate_max_bending_stress()

    expected_area = pi * leg.R**2
    expected_inertia = pi * leg.R**4 / 4
    expected_bending_force = (
        leg.m * leg.g * sin(leg.angle) * leg.SF / leg.number_of_legs
    )
    expected_axial_force = (
        leg.m * leg.g * cos(leg.angle) * leg.SF / leg.number_of_legs
    )

    assert leg.area == pytest.approx(expected_area)
    assert leg.I == pytest.approx(expected_inertia)
    assert leg.bending_force == pytest.approx(expected_bending_force)
    assert leg.axial_force == pytest.approx(expected_axial_force)
    assert leg.bending_moment == pytest.approx(expected_bending_force * leg.L)
    assert leg.mass == pytest.approx(
        expected_area * leg.L * leg.rho * leg.SF
    )
    assert leg.maximum_normal_stress == pytest.approx(
        max(abs(leg.sigma_z_max), leg.longitudinal_compressive_stress)
    )


def test_uniaxial_calculation_initializes_area_when_needed():
    leg = Leg(4.5)

    stress, deflection = leg.calculate_uniaxial_compression_and_deflection()

    assert leg.area == pytest.approx(pi * leg.R**2)
    assert stress == pytest.approx(abs(leg.axial_force) / leg.area)
    assert deflection == pytest.approx(
        abs(leg.axial_force) * leg.L / (leg.E * leg.area)
    )


def test_effective_length_factor_scales_euler_load_by_inverse_square():
    leg = Leg(4.5)

    pinned_load = leg.calculate_euler_buckling(effective_length_factor=1.0)
    cantilever_load = leg.calculate_euler_buckling(
        effective_length_factor=2.0
    )

    assert cantilever_load == pytest.approx(pinned_load / 4)
    assert leg.effective_length_factor == pytest.approx(2.0)
