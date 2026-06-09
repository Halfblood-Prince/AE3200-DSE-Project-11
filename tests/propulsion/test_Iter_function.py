import pathlib
import sys

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

import propulsion.Iter_function as iter_function


class MockPropeller:
    def __init__(self, thrust_map, power_map, rpm_max):
        self.Thrust = dict(thrust_map)
        self.Power = dict(power_map)
        self.RPMmax = rpm_max


def test_SIZE_PROP_ITF_UT_01():
    # Test Required_thrust with a custom thrust-to-weight ratio
    assert iter_function.Required_thrust(4.0, thrust_to_weight_ratio=2.0) == 8.0


def test_SIZE_PROP_ITF_UT_02():
    # Test Required_thrust with the default thrust-to-weight ratio
    assert iter_function.Required_thrust(4.0) == 6.0


def test_SIZE_PROP_ITF_UT_03():
    # Test OEI_performance_check for a non-coaxial configuration
    propeller = MockPropeller(
        thrust_map={10000: 5.0, 12000: 8.0, 14000: 10.0},
        power_map={10000: 1.0, 12000: 2.0, 14000: 3.0},
        rpm_max=16000,
    )

    result = iter_function.OEI_performance_check(4, 15.0, propeller, coaxial=False)

    assert result == (True, (12000, 8.0))


def test_SIZE_PROP_ITF_UT_04():
    # Test OEI_performance_check for a coaxial configuration
    propeller = MockPropeller(
        thrust_map={10000: 6.0, 12000: 9.0, 14000: 12.0},
        power_map={10000: 1.0, 12000: 2.0, 14000: 3.0},
        rpm_max=16000,
    )

    result = iter_function.OEI_performance_check(6, 24.0, propeller, coaxial=True)

    assert result == (True, (12000, 9.0))


def test_SIZE_PROP_ITF_UT_05():
    # Test OEI_performance_check returns False when only RPMmax meets the thrust requirement
    propeller = MockPropeller(
        thrust_map={10000: 5.0, 14000: 9.0},
        power_map={10000: 1.0, 14000: 2.0},
        rpm_max=14000,
    )

    result = iter_function.OEI_performance_check(4, 18.0, propeller, coaxial=False)

    assert result == (False, None)


def test_SIZE_PROP_ITF_UT_06():
    # Test get_power_required for a non-coaxial configuration
    propeller = MockPropeller(
        thrust_map={10000: 5.0, 12000: 7.0, 14000: 9.0},
        power_map={10000: 1.5, 12000: 2.0, 14000: 2.5},
        rpm_max=16000,
    )

    result = iter_function.get_power_required(4, 24.0, propeller, coaxial=False)

    assert result == 8.0


def test_SIZE_PROP_ITF_UT_07():
    # Test get_power_required for a coaxial configuration with the interference factor
    propeller = MockPropeller(
        thrust_map={10000: 6.0, 12000: 7.0, 14000: 8.0},
        power_map={10000: 1.5, 12000: 2.0, 14000: 2.5},
        rpm_max=16000,
    )

    result = iter_function.get_power_required(4, 20.0, propeller, coaxial=True)

    assert result == 8.0


def test_SIZE_PROP_ITF_UT_08():
    # Test get_power_required returns None when no RPM point satisfies the thrust target
    propeller = MockPropeller(
        thrust_map={10000: 4.0, 12000: 5.0},
        power_map={10000: 1.5, 12000: 2.0},
        rpm_max=14000,
    )

    result = iter_function.get_power_required(4, 24.0, propeller, coaxial=False)

    assert result is None


def test_SIZE_PROP_ITF_UT_09():
    # Test find_prop selects the option with the lowest required total power
    propeller_a = MockPropeller(
        thrust_map={10000: 8.0, 12000: 9.0},
        power_map={10000: 2.0, 12000: 2.5},
        rpm_max=14000,
    )
    propeller_b = MockPropeller(
        thrust_map={10000: 8.0, 12000: 9.0},
        power_map={10000: 1.0, 12000: 1.5},
        rpm_max=14000,
    )

    best_name, best_info, options = iter_function.find_prop(
        8.0,
        4,
        {"prop_a": propeller_a, "prop_b": propeller_b},
        coaxial=False,
    )

    assert best_name == "prop_b"
    assert best_info["OEI_condition"] == (10000, 8.0)
    assert best_info["Power_required"] == 4.0
    assert set(options) == {"prop_a", "prop_b"}


def test_SIZE_PROP_ITF_UT_10():
    # Test motor_mass returns the fixed motor mass and the expected max current
    propeller = MockPropeller(
        thrust_map={10000: 8.0, 14000: 10.0},
        power_map={10000: 80.0, 14000: 111.0},
        rpm_max=14000,
    )
    save_info = {"OEI_condition": (10000, 8.0), "Power_required": 320.0, "data": propeller}

    motor_mass, max_current = iter_function.motor_mass(save_info)

    assert motor_mass == 0.1
    assert max_current == pytest.approx(5.0)


def test_SIZE_PROP_ITF_UT_11():
    # Test ESC_mass returns the current fixed ESC mass estimate
    assert iter_function.ESC_mass(12.5) == 0.04


def test_SIZE_PROP_ITF_MT_01():
    # Test the full propeller-selection and component-sizing workflow
    propeller_a = MockPropeller(
        thrust_map={10000: 8.0, 12000: 9.0, 14000: 10.0},
        power_map={10000: 2.0, 12000: 2.5, 14000: 111.0},
        rpm_max=14000,
    )
    propeller_b = MockPropeller(
        thrust_map={10000: 8.0, 12000: 9.5, 14000: 11.0},
        power_map={10000: 1.5, 12000: 2.0, 14000: 88.8},
        rpm_max=14000,
    )

    best_name, best_info, options = iter_function.find_prop(
        8.0,
        4,
        {"prop_a": propeller_a, "prop_b": propeller_b},
        coaxial=False,
    )
    motor_mass, max_current = iter_function.motor_mass(best_info)
    esc_mass = iter_function.ESC_mass(max_current)

    assert set(options) == {"prop_a", "prop_b"}
    assert best_name == "prop_b"
    assert best_info["Power_required"] == 6.0
    assert motor_mass == 0.1
    assert max_current == pytest.approx(4.0)
    assert esc_mass == 0.04


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
