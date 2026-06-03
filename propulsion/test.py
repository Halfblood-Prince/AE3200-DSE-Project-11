import sys
from pathlib import Path

# Add parent directory to path so we can import EPS module
sys.path.insert(0, str(Path(__file__).parent.parent))

from propClass import load_propeller_dict
from Iter_function import find_prop, motor_mass, ESC_mass
from EPS.Iter_fuctions import power_after_efficiencies, battery_sizing

# for actual iteration loop test for both 6 and 8 propellers 
N_prop = 8
flight_time = 10 /60 #hours
props = load_propeller_dict("propulsion/6.0_E.csv")  #change to 6.0_E  for 8 propellers and 8.0_E for 6 propellers (yes it is confusing, sorry)
MTOW =3.2 * 9.81 # mass * g
P_payload = 250   #W
P_avionics = 50   #W
Lipo_spec_energy = 250 #Wh/kg
(best, best_info), options = find_prop(MTOW, N_prop, props)
print((best, best_info))
print(options)
P_propellers = best_info['Power_required']
P_bat = power_after_efficiencies(P_payload,P_avionics, P_propellers)
Mass_battery = battery_sizing(P_bat, flight_time, Lipo_spec_energy)
print(Mass_battery)
m_motor, I_max = motor_mass(best_info)
m_ESC = ESC_mass(I_max)
