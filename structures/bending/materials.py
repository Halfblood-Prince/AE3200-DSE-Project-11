class constants:
    g = 9.81        # m/s^2


class arm:
    # CFRP
    name = "CFRP"
    density = 1600                       # kg/m^3
    youngs_modulus = 140e9               # Pa
    safety_factor = 1.5
    max_tip_deflection = 0.005e-3         # m
    poisson_ratio = 0.30
    failure_strain = 0.01
    failure_stress = failure_strain * youngs_modulus
    ultimate_tensile_strength = failure_stress
    shear_modulus = youngs_modulus / (2 * (1 + poisson_ratio))
    specific_stiffness = youngs_modulus / density
    specific_strength = failure_stress / density

class leg:
    # Temporarily Aluminium 6061-T6
    name = "Aluminium 6061-T6"
    density = 2700                       # kg/m^3
    youngs_modulus = 69e9                # Pa
    safety_factor = 1.5
    max_tip_deflection = 0.005e-3         # m
    max_compressive_deformation = 0.01e-3 # m
    poisson_ratio = 0.33
    yield_strength = 276e6               # Pa
    ultimate_tensile_strength = 310e6    # Pa
    failure_stress = yield_strength
    failure_strain = failure_stress / youngs_modulus
    shear_modulus = youngs_modulus / (2 * (1 + poisson_ratio))
    specific_stiffness = youngs_modulus / density
    specific_strength = failure_stress / density
