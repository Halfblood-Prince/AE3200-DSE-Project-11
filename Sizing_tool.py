import sys
from pathlib import Path
import numpy as np

# Add parent directory to path so we can import EPS, propulsion and structures modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from propulsion.propClass import load_propeller_dict
from propulsion.Iter_function import find_prop, motor_mass, ESC_mass
from EPS.Iter_fuctions import power_after_efficiencies, battery_sizing
from structures.iter_function import struct_size



N_prop = 8
flight_time = 7 /60 #hours
# props = load_propeller_dict("propulsion/6.0_4pitch_E_1000.csv")  #change to correct diameter range
P_payload = 120   #W
P_avionics = 0   #W
Lipo_spec_energy = 230 #Wh/kg
M_pay = 1.418


def _build_test_trace_result(
    mtom_list,
    p_prop_list,
    m_battery_list,
    m_structures_list,
    residual_list,
    final_mtom,
    is_valid,
    failure_iteration=None,
    failure_mtom=None,
):
    return {
        "mtom_list": list(mtom_list),
        "p_prop_list": list(p_prop_list),
        "m_battery_list": list(m_battery_list),
        "m_structures_list": list(m_structures_list),
        "residual_list": list(residual_list),
        "final_mtom": final_mtom,
        "is_valid": is_valid,
        "failure_iteration": failure_iteration,
        "failure_mtom": failure_mtom,
    }



def run_sizing_tool(
    MTOM_guess: float,
    coaxial: bool,
    N_prop: int,
    flight_time: float,
    P_payload: float,
    P_avionics: float,
    Lipo_spec_energy: float,
    M_pay: float,
    test: bool = False,
    return_trace_metadata: bool = False,
) -> bool:
    if MTOM_guess <= 0:
        if test:
            if return_trace_metadata:
                return _build_test_trace_result(
                    [MTOM_guess],
                    [],
                    [],
                    [],
                    [],
                    np.nan,
                    False,
                    failure_iteration=1,
                    failure_mtom=MTOM_guess,
                )
            return False
        print("Initial MTOM guess must be positive.")
        return False

    if coaxial:
        props = load_propeller_dict("propulsion/6.0_4pitch_E_1000.csv")
    else:
        props = load_propeller_dict("propulsion/4.0_E_1000.csv")
        
    iterations = 0
    MTOM = MTOM_guess
    mtom_list = []
    p_prop_list = []
    m_battery_list = []
    m_structures_list = []
    residual_list = []
    while True:
        '''Propulsion'''
        MTOW = MTOM * 9.81
        best, best_info, options = find_prop(MTOW, N_prop, props,coaxial)
        if best_info is None:
            if test:
                if return_trace_metadata:
                    return _build_test_trace_result(
                        mtom_list,
                        p_prop_list,
                        m_battery_list,
                        m_structures_list,
                        residual_list,
                        np.nan,
                        False,
                        failure_iteration=len(mtom_list) + 1,
                        failure_mtom=MTOM,
                    )
                return False
            print("No valid propeller could be selected for this sizing run.")
            return False

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
        MTOM_new = m_motor_tot + m_ESC_tot + m_battery + M_pay + M_structures
        residual = np.abs(MTOM_new - MTOM)

        if test:
            mtom_list.append(MTOM)
            p_prop_list.append(P_propellers)
            m_battery_list.append(m_battery)
            m_structures_list.append(M_structures)
            residual_list.append(residual)
        
        
        if residual <= 0.001:
            if not test:
                MTOM = MTOM_new
                print(f'MTOM_new: {MTOM_new}[kg]')
                print(f'Propeller:{best}')
                print(f'Power required: {best_info["Power_required"]/8} [W]')
                print(f'Propeller data: {best_info["data"].Power} [W]')
                print(f'Propeller thurst data: {best_info["data"].Thrust} [N]')
                print(f'm_battery: {m_battery}')
                print(f' iterations: {iterations}')
            else: 
                if return_trace_metadata:
                    return _build_test_trace_result(
                        mtom_list,
                        p_prop_list,
                        m_battery_list,
                        m_structures_list,
                        residual_list,
                        MTOM_new,
                        True,
                    )
                return (
                    mtom_list,
                    p_prop_list,
                    m_battery_list,
                    m_structures_list,
                    residual_list,
                    MTOM_new,
                )
            break
        elif residual/MTOM > 10:
            print("MTOM diverging, check for errors.")
            if test:
                if return_trace_metadata:
                    return _build_test_trace_result(
                        mtom_list,
                        p_prop_list,
                        m_battery_list,
                        m_structures_list,
                        residual_list,
                        np.nan,
                        False,
                        failure_iteration=len(mtom_list) + 1,
                        failure_mtom=MTOM_new,
                    )
                return False
            break
        else:
            MTOM = MTOM_new
            iterations += 1
    return True
if __name__ == "__main__":
    MTOM_guess = 4
    coaxial = True
    print("coaxial")
    run_sizing_tool(MTOM_guess, coaxial=coaxial, N_prop=N_prop, flight_time=flight_time, P_payload=P_payload, P_avionics=P_avionics, Lipo_spec_energy=Lipo_spec_energy, M_pay=M_pay)
    # coaxial = False
    # print("non-coaxial")
    # run_sizing_tool(MTOM_guess, coaxial=coaxial, N_prop=N_prop, flight_time=flight_time, P_payload=P_payload, P_avionics=P_avionics, Lipo_spec_energy=Lipo_spec_energy, M_pay=M_pay)
