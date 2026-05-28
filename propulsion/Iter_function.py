import numpy as np

def Required_thrust(MTOW, thrust_to_weight_ratio=1.5):
    T_req = thrust_to_weight_ratio * MTOW
    return T_req

def OEI_performance_check(N_prop, T_req, prop_type):
    T_prop = T_req / (N_prop-2)
    for rpm, thrust in prop_type.thrust.items():
        if thrust >= T_prop and rpm < prop_type.maxRPM:
            return True, (rpm, thrust)
        else:
            pass
    
    return False, None

def get_power_required(N_prop, T_req, prop_type):
    T_prop = T_req / N_prop
    for rpm, thrust in prop_type.thrust.items():
        if thrust >= T_prop and rpm < prop_type.maxRPM:
            #find required power for this thrust for all propellers
            P_req = prop_type.power[rpm] * N_prop
            return P_req
        else:
            pass      
    return None

def find_prop(MTOW,N_prop, prop_list):
    options = dict()
    T_req = Required_thrust(MTOW)
    for prop in prop_list:
        check, cond = OEI_performance_check(N_prop, T_req, prop)
        if check:
            P_req = get_power_required(N_prop, T_req, prop)
            options[prop.name]= {"OEI_condition": cond, "Power_required": P_req}
        else:
            pass
    best = 100000000000000000
    save = None
    for opt in options:
        if options[opt]["Power_required"] < best:
            best = options[opt]["Power_required"]
            save = options[opt]
        else:
            pass
    return save, options
        
