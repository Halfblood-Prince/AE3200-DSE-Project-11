import sys
from pathlib import Path
import numpy as np

# Add parent directory to path so we can import EPS, propulsion and structures modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from propulsion.propClass import load_propeller_dict
from propulsion.Iter_function import find_prop, motor_mass, ESC_mass
from EPS.Iter_fuctions import power_after_efficiencies, battery_sizing
from structures.iter_function import struct_size

coaxial = True
N_prop = 8
flight_time = 7 /60 #hours
props = load_propeller_dict("propulsion/6.0_4pitch_E_1000.csv")  #change to correct diameter range
MTOM = 4
P_payload = 150   #W
P_avionics = 0   #W
Lipo_spec_energy = 275 #Wh/kg
M_pay = 1.4
M_avionics = 0
M_structures = 0.5

iterations = 0
while True:
    '''Propulsion'''
    MTOW = MTOM * 9.81
    best, best_info, options = find_prop(MTOW, N_prop, props,coaxial)
    P_propellers = best_info['Power_required']                              #W
    m_motor, I_max = motor_mass(best_info)                                  #kg
    m_motor_tot = m_motor * N_prop                                          #kg
    m_ESC_tot = ESC_mass(I_max) * N_prop                                    #kg
    T_OEI_prop = best_info["OEI_condition"][1]                              #N

    '''EPS'''
    P_bat = power_after_efficiencies(P_payload, P_avionics, P_propellers)   #W
    m_battery = battery_sizing(P_bat, flight_time, Lipo_spec_energy)        #kg


    '''Structures'''

    M_structures = struct_size(MTOM, T_OEI_prop, length=0.1, height=0.3, n=N_prop, coaxial=coaxial) #kg


    '''MTOM Update'''
    MTOM_new = m_motor_tot + m_ESC_tot + m_battery + M_avionics + M_pay + M_structures
    
    
    if np.abs(MTOM_new-MTOM) <= 0.001:
        MTOM = MTOM_new
        print(f'MTOM_new: {MTOM_new}[kg]')
        print(f'Propeller:{best}')
        print(f'Power required: {best_info["Power_required"]/8} [W]')
        print(f'Propeller data: {best_info["data"].Power} [W]')
        print(f'm_battery: {m_battery}')
        print(f' iterations: {iterations}')
        break
    elif np.abs(MTOM_new-MTOM)/MTOM > 10:
        print("MTOM diverging, check for errors.")
        break
    else:
        MTOM = MTOM_new
        iterations += 1
