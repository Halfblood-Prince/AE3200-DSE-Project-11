from pathlib import Path
import sys

if __package__:
    from .bending.arm_wrapper import Arm
    from .bending.leg_wrapper import Leg
    from .shear import shear_arm
else:
    structures_dir = Path(__file__).resolve().parent
    if str(structures_dir) not in sys.path:
        sys.path.insert(0, str(structures_dir))

    from bending.arm_wrapper import Arm
    from bending.leg_wrapper import Leg
    from shear import shear_arm

import numpy as np



def struct_size(MTOM: float, thrust: float, length: float, height: float = 0.3, n: int = 8, coaxial: bool = True) -> float:

    if coaxial: arms = n // 2
    else: arms = n
    
    materials_file = Path(__file__).resolve().parent / "bending" / "materials.py"

    arm = Arm(materials_file=materials_file, length=length, thrust=thrust)
    leg = Leg(materials_file=materials_file, height=height, vehicle_mass=MTOM, max_tip_deflection=1)

    arm_results = arm.calculate()
    max_shear_force = arm_results["max_shear_force"]
    max_allowed_shear = arm.failure_shear_stress / arm.safety_factor

    rmin = shear_arm(V=np.abs(max_shear_force), t=arm.thickness, tau_allow=max_allowed_shear)

    best_arm = arm.minimise_mass(L=length, T=thrust, radius_min=rmin)
    best_leg = leg.minimise_mass()

    if not best_arm.get("found"):
        raise RuntimeError("No feasible arm design found in the configured search range.")
    if not best_leg.get("found"):
        raise RuntimeError("No feasible leg design found in the configured search range.")

    arm_design = best_arm["design"]
    leg_design = best_leg["design"]

    print("Optimal arm parameters:")
    print(f"  Radius: {best_arm['radius'] * 1000:.3f} mm")
    print(f"  Wall thickness: {best_arm['thickness'] * 1000:.3f} mm")
    print(f"  Length: {length:.3f} m")
    print(f"  Mass per arm: {arm_design['mass']:.6f} kg")

    print("Optimal leg parameters:")
    print(f"  Radius: {leg_design['radius'] * 1000:.3f} mm")
    print(f"  Angle: {leg_design['angle_deg']:.1f} deg")
    print(f"  Height: {leg_design['height']:.3f} m")
    print(f"  Length: {leg_design['length']:.3f} m")
    print(f"  Mass per leg: {leg_design['mass']:.6f} kg")

    best_arm_mass = arm_design["mass"]
    best_leg_mass = leg_design["mass"]

    return arms * best_arm_mass + leg.number_of_legs * best_leg_mass + 0.3

if __name__ == "__main__":
    length = 0.1
    thrust = 10
    height = 0.3
    MTOM = 4
    mass = struct_size(MTOM, thrust, length, height)
    print(f"Estimated structure mass: {mass:.3f} kg")
