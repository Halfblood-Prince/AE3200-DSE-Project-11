import numpy as np

def Required_thrust(MTOW, thrust_to_weight_ratio=1.5):
    T_req = thrust_to_weight_ratio * MTOW
    return T_req

def OEI_performance_check(N_prop, T_req, prop_type):
    T_prop = T_req / (N_prop-2)
    for rpm, thrust in prop_type.Thrust.items():
        if thrust >= T_prop and rpm < prop_type.RPMmax:
            return True, (rpm, thrust)
        else:
            pass
    
    return False, None

def get_power_required(N_prop, T_req, prop_type):
    T_prop = T_req / N_prop
    for rpm, thrust in prop_type.Thrust.items():
        if thrust >= T_prop and rpm < prop_type.RPMmax:
            #find required power for this thrust for all propellers
            P_req = prop_type.Power[rpm] * N_prop
            return P_req
        else:
            pass      
    return None

def find_prop(MTOW,N_prop, prop_list):
    options = dict()
    T_req = Required_thrust(MTOW)
    for name, prop in prop_list.items():
        check, cond = OEI_performance_check(N_prop, T_req, prop)
        if check:
            P_req = get_power_required(N_prop, T_req, prop)
            options[name]= {"OEI_condition": cond, "Power_required": P_req, "data":prop}
        else:
            pass
    best = 100000000000000000
    save = None
    save_name = ''
    for opt in options:
        if options[opt]["Power_required"] < best:
            best = options[opt]["Power_required"]
            save_info = options[opt]
            save_name = opt
        else:
            pass
    return (save_name,save_info), options

def motor_mass(save_info):
    RPM_max = save_info["data"].RPMmax
    #Battery max voltage 22.2 (6S batteries)
    kv = RPM_max / 22.2
    P_max = save_info["data"].Power[RPM_max]
    I_max = P_max * 22.2
    L_motor = 4.8910 * I_max**0.1751 * P_max**0.2476
    D_motor = 41.45 * kv **(-0.1919)*P**0.1935
    m_motor = 0.0109 * kv**0.5122 * P_max**(-0.1902) * np.log10(L_motor) ** 2.5582 * np.log10(D_motor)**12.8502
    return m_motor, I_max

def ESC_mass(I_max):
    return 0.8013*I_max**0.9727




        
