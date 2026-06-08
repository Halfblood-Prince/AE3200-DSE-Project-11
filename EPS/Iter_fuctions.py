def power_after_efficiencies(P_payload: float, P_avionics: float, P_prop: float):
    # Motor efficiency: 0.75 https://doi.org/10.48550/arXiv.2109.04741 
    # DC-DC converter: 0.94 https://doi.org/10.1109/ITNG.2011.135 
    # ESC efficiency: 0.82 https://search.informit.org/doi/10.3316/informit.267157986125668
    P_prop_eff = P_prop / (0.75 * 0.82)
    P_other_eff = (P_payload + P_avionics) / 0.94
    P_bat = P_prop_eff + P_other_eff
    return P_bat

def battery_sizing(P_bat: float, flight_time: float, bat_spec_energy: float, bat_percentage_used: float = 0.8):
    E_req = P_bat * flight_time
    E_bat = E_req/bat_percentage_used
    Bat_mass = E_bat/bat_spec_energy
    return Bat_mass
