class constants:
    g = 3.71


class arm:
    density = 1234.0
    youngs_modulus = 88e9
    failure_stress = 700e6
    safety_factor = 1.8
    max_tip_deflection = 0.002e-3


class leg:
    density = 4321.0
    youngs_modulus = 55e9
    yield_strength = 210e6
    safety_factor = 2.1
    max_tip_deflection = 0.003e-3
    max_compressive_deformation = 0.004e-3
