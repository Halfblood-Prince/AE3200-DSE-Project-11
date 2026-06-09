class constants:
    g = 9.81        # m/s^2


class arm:
    # CFRP
    name = "CFRP"                       # T700S Toray SM
    density = 1800                       # kg/m^3
    youngs_modulus = 230e9               # Pa
    thickness = 7e-6                   # m
    safety_factor = 1.5
    max_tip_deflection = 0.005e-3         # m
    poisson_ratio = 0.30
    failure_strain = 0.01
    failure_stress = failure_strain * youngs_modulus
    ultimate_tensile_strength = failure_stress
    shear_modulus = youngs_modulus / (2 * (1 + poisson_ratio))
    specific_stiffness = youngs_modulus / density
    specific_strength = failure_stress / density
    failure_shear_stress = 50e6 # Pa

class leg:
    #  Aluminium 6082-T6
    name = "Aluminium 6082-T6"
    density = 2700                       # kg/m^3
    youngs_modulus = 70e9                # Pa
    safety_factor = 1.5
    max_tip_deflection = 0.005e-3         # m
    max_compressive_deformation = 0.01e-3 # m
    poisson_ratio = 0.33
    yield_strength = 250e6               # Pa
    ultimate_tensile_strength = 330e6    # Pa
    failure_stress = yield_strength
    failure_strain = failure_stress / youngs_modulus
    shear_modulus = youngs_modulus / (2 * (1 + poisson_ratio))
    specific_stiffness = youngs_modulus / density
    specific_strength = failure_stress / density
    failure_shear_stress = 200e6 # Pa
