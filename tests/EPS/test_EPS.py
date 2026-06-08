import pytest 
import pathlib
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from EPS.Iter_fuctions import power_after_efficiencies, battery_sizing

def test_SIZE_POW_UT_01():
    # Test power_after_efficiencies function with known values
    P_payload = 150   # W
    P_avionics = 0    # W
    P_prop = 80       # W
    expected_P_bat = 80 / (0.75 * 0.82) + (150 + 0) / 0.94
    assert power_after_efficiencies(P_payload, P_avionics, P_prop) == expected_P_bat

def test_SIZE_POW_UT_02():
    # Test battery_sizing function with known values
    P_bat = 200       # W
    flight_time = 7 / 60  # hours
    bat_spec_energy = 275 # Wh/kg
    bat_percentage_used = 0.8
    E_req = P_bat * flight_time
    E_bat = E_req / bat_percentage_used
    expected_Bat_mass = E_bat / bat_spec_energy
    assert battery_sizing(P_bat, flight_time, bat_spec_energy, bat_percentage_used) == expected_Bat_mass

def test_SIZE_POW_MT_01():
    # Test battery_sizing and power_after_efficiencies together with a realistic scenario
    P_payload = 120      # W
    P_avionics = 35      # W
    P_prop = 250         # W
    flight_time = 45 / 60  # hours
    bat_spec_energy = 275  # Wh/kg
    bat_percentage_used = 0.8

    P_bat = power_after_efficiencies(P_payload, P_avionics, P_prop)
    bat_mass = battery_sizing(P_bat, flight_time, bat_spec_energy, bat_percentage_used)

    expected_P_bat = 250 / (0.75 * 0.82) + (120 + 35) / 0.94
    expected_E_req = expected_P_bat * flight_time
    expected_E_bat = expected_E_req / bat_percentage_used
    expected_bat_mass = expected_E_bat / bat_spec_energy

    assert P_bat == expected_P_bat
    assert bat_mass == expected_bat_mass
    
if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))

