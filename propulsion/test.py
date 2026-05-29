import sys
from pathlib import Path

# Add parent directory to path so we can import EPS module
sys.path.insert(0, str(Path(__file__).parent.parent))

from propClass import load_propeller_dict
from Iter_function import find_prop
from EPS.Iter_fuctions import power_after_efficiencies, battery_sizing
N_prop = 8
flight_time = 10 /60 #hours
props = load_propeller_dict("propulsion/6.0_E.csv")  #change to 6.0_E  for 8 propellers and 8.0_E for 6 propellers (yes it is confusing, sorry)
MTOW =3.2 * 9.81 # mass * g
P_payload = 200   #W
P_avionics = 50   #W
Lipo_spec_energy = 250 #Wh/kg
best, options = find_prop(MTOW, N_prop, props)
print(best)
print(options)
P_propellers = best[1]['Power_required']
P_bat = power_after_efficiencies(P_payload,P_avionics, P_propellers)
Mass_battery = battery_sizing(P_bat, flight_time, Lipo_spec_energy)
print(Mass_battery)

