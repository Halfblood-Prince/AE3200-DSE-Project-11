import matplotlib.pyplot as plt
import pytest

import materials
from bending import Arm, Leg


pytestmark = pytest.mark.subsystem


def test_arm_initialization_runs_analysis_reporting_and_plotting(
    monkeypatch, capsys
):
    monkeypatch.setattr(plt, "show", lambda: None)

    arm = Arm()
    arm.__main__()

    output = capsys.readouterr().out
    figure = plt.gcf()

    assert "Maximum bending moment:" in output
    assert "Design bending stress with SF:" in output
    assert len(figure.axes) == 2
    assert arm.z.size == arm.V.size == arm.Mx.size
    assert arm.sigma_z_max > 0
    assert arm.SF == pytest.approx(materials.arm.safety_factor)
    assert arm.max_tip_deflection == pytest.approx(
        materials.arm.max_tip_deflection
    )

    plt.close("all")


def test_leg_analysis_pipeline_produces_consistent_buckling_state():
    leg = Leg(4.5)

    leg.calculate_max_bending_stress()

    assert leg.euler_buckling_load > 0
    assert leg.buckling_margin == pytest.approx(
        leg.euler_buckling_load / abs(leg.axial_force)
    )
    assert leg.buckling_safe is (
        leg.euler_buckling_load >= abs(leg.axial_force)
    )
    assert leg.tip_deflection > 0
    assert leg.longitudinal_deflection > 0
