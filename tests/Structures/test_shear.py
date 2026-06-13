import pathlib
import sys

import numpy as np
import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from structures.shear import shear_arm


def _tau_max(V, Ro, t):
    Ri = Ro - t
    return 4 * V * (Ro**2 + Ro * Ri + Ri**2) / (3 * np.pi * (Ro**4 - Ri**4))


def test_SIZE_STR_SHEAR_UT_01():
    with pytest.raises(ValueError, match="V must be positive."):
        shear_arm(0, 0.002, 1.0e6)


def test_SIZE_STR_SHEAR_UT_02():
    with pytest.raises(ValueError, match="t must be positive."):
        shear_arm(100, 0.0, 1.0e6)


def test_SIZE_STR_SHEAR_UT_03():
    with pytest.raises(ValueError, match="tau_allow must be positive."):
        shear_arm(100, 0.002, 0.0)


def test_SIZE_STR_SHEAR_UT_04():
    V = 120.0
    t = 0.004
    tau_solid = 4 * V / (3 * np.pi * t**2)

    result = shear_arm(V, t, tau_allow=tau_solid * 1.2)

    assert result == pytest.approx(t)


def test_SIZE_STR_SHEAR_UT_05():
    V = 250.0
    t = 0.003
    tau_allow = 4.0e6

    result = shear_arm(V, t, tau_allow=tau_allow)
    tau_result = _tau_max(V, result, t)

    assert result > t
    assert tau_result == pytest.approx(tau_allow, rel=1e-9)


def test_SIZE_STR_SHEAR_UT_06():
    t = 0.003
    tau_allow = 5.0e6

    low_load_radius = shear_arm(150.0, t, tau_allow=tau_allow)
    high_load_radius = shear_arm(300.0, t, tau_allow=tau_allow)

    assert high_load_radius > low_load_radius


def test_SIZE_STR_SHEAR_MT_01():
    cases = [
        (80.0, 0.0025, 6.0e6),
        (200.0, 0.0030, 4.5e6),
        (350.0, 0.0040, 3.5e6),
    ]

    for V, t, tau_allow in cases:
        result = shear_arm(V, t, tau_allow=tau_allow)
        tau_result = _tau_max(V, result, t)

        assert result >= t
        assert tau_result <= tau_allow * (1 + 1e-9)
        if result > t:
            assert tau_result == pytest.approx(tau_allow, rel=1e-9)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
