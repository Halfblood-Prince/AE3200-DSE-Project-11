import sys
from pathlib import Path
import numpy as np

# Add parent directory to path so we can import EPS, propulsion and structures modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from propulsion.propClass import load_propeller_dict
from propulsion.Iter_function import find_prop, motor_mass, ESC_mass
from EPS.Iter_fuctions import power_after_efficiencies, battery_sizing

coaxial = True
N_prop = 8
flight_time = 7 /60 #hours
props = load_propeller_dict("propulsion/6.5_E_1000.csv")  #change to correct diameter range
# MTOW =3.2 * 9.81 # mass * g    #change to initial guess
MTOM = 3.2
P_payload = 250   #W
P_avionics = 50   #W
Lipo_spec_energy = 275 #Wh/kg
M_pay = 1.2
M_avionics = 0
M_structures = 0.328



while True:
    '''Propulsion'''
    MTOW = MTOM * 9.81
    #find all possible propellers that meet constrains and best options (lowest power consumption)
    best, best_info, options = find_prop(MTOW, N_prop, props,coaxial)
    # print(f'P_required:{best_info["Power_required"]}')
    #print(options)
    P_propellers = best_info['Power_required']                              #W
    m_motor, I_max = motor_mass(best_info)                                  #kg
    m_motor_tot = m_motor * N_prop                                          #kg
    # print(f'm_motors: {m_motor_tot}')
    m_ESC_tot = ESC_mass(I_max) * N_prop                                    #kg
    # print(f'm_ESCs: {m_ESC_tot}')
    T_OEI_prop = best_info["OEI_condition"][1]                              #N

    '''EPS'''
    P_bat = power_after_efficiencies(P_payload, P_avionics, P_propellers)   #W
    m_battery = battery_sizing(P_bat, flight_time, Lipo_spec_energy)        #kg


    '''Structures'''

    M_structures = .328


    '''MTOM Update'''
    MTOM_new = m_motor_tot + m_ESC_tot + m_battery + M_avionics + M_pay + M_structures
  
    
    if np.abs(MTOM_new-MTOM) <= 0.01:
        MTOM = MTOM_new
        break
    else:
        MTOM = MTOM_new
print(f'MTOM_new: {MTOM_new}[kg]')
print(best)
print(best_info["Power_required"]/8)
print(best_info["data"].Power)
print(f'm_battery: {m_battery}')