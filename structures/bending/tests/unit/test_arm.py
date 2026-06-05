from math import pi

import numpy as np
import pytest

import bending
from bending import Arm


pytestmark = pytest.mark.unit


def make_arm():
    arm = Arm.__new__(Arm)
    arm.t = 0.002
    arm.R = 0.01
    arm.L = 0.25
    arm.T = 10.0
    arm.SF = 1.5
    arm.rho = 1600.0
    arm.resolution = 0.001
    arm.Y = 140e9
    arm.area = None
    arm.inertia = None
    arm.w = None
    arm.z = None
    arm.V = None
    arm.Mx = None
    arm.max_bending_moment = 0.0
    arm.z_at_max_bending_moment = 0.0
    arm.sigma_z_max = 0.0
    return arm


def test_arm_cross_section_properties_and_distributed_load():
    arm = make_arm()
    inner_radius = arm.R - arm.t

    arm.calculate_cross_section_area()
    arm.calculate_area_moment_of_inertia()
    arm.calculate_w()

    assert arm.area == pytest.approx(pi * (arm.R**2 - inner_radius**2))
    assert arm.inertia == pytest.approx(
        (pi / 4) * (arm.R**4 - inner_radius**4)
    )
    assert arm.w == pytest.approx(arm.rho * 9.81 * arm.area)


def test_arm_bending_stress_and_deflection_formulas():
    arm = make_arm()
    arm.calculate_cross_section_area()
    arm.calculate_area_moment_of_inertia()
    arm.calculate_w()

    arm.calculate_maximum_longitudinal_bending_moment()
    arm.calculate_maximum_bending_stress()

    expected_root_moment = arm.T * arm.L - 0.5 * arm.w * arm.L**2
    expected_deflection = (
        arm.w * arm.L**4 / (8 * arm.Y * arm.inertia)
        + arm.T * arm.L**3 / (3 * arm.Y * arm.inertia)
    )

    assert arm.z_at_max_bending_moment == pytest.approx(0.0)
    assert arm.max_bending_moment == pytest.approx(expected_root_moment)
    assert arm.sigma_z_max == pytest.approx(
        abs(expected_root_moment) * arm.R / arm.inertia
    )
    assert arm.calculate_deflection() == pytest.approx(expected_deflection)


def test_arm_minimise_mass_can_scan_a_small_candidate_set(monkeypatch, capsys):
    arm = make_arm()
    arm.sigma_z_max = arm.Y - 1.0
    real_arange = np.arange

    def small_arange(start, stop, step):
        if start == 0.002:
            return real_arange(0.002, 0.003, 0.001)
        return np.array([0.0005, 0.001])

    monkeypatch.setattr(bending.np, "arange", small_arange)

    arm.minimise_mass()

    output = capsys.readouterr().out
    assert "2" in output
    assert arm.R == pytest.approx(0.002)
    assert arm.t == pytest.approx(0.001)
