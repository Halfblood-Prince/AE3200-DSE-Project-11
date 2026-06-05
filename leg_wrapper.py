from dataclasses import dataclass

import materials
from binary_bridge import run_binary


@dataclass
class Leg:
    vehicle_mass: float = 4.5
    radius: float = 0.005
    angle_deg: float = 30.0
    length: float = 0.1
    safety_factor: float = 1.5
    number_of_legs: int = 2
    density: float = materials.leg.density
    youngs_modulus: float = materials.leg.youngs_modulus
    yield_strength: float = materials.leg.yield_strength
    effective_length_factor: float = 1.0
    max_tip_deflection: float = 0.005e-3
    max_compressive_deformation: float = 0.01e-3
    binary_path: str | None = None

    def _configuration(self):
        return {
            "mass": self.vehicle_mass,
            "radius": self.radius,
            "angle-deg": self.angle_deg,
            "length": self.length,
            "safety-factor": self.safety_factor,
            "number-of-legs": self.number_of_legs,
            "density": self.density,
            "youngs-modulus": self.youngs_modulus,
            "yield-strength": self.yield_strength,
            "effective-length-factor": self.effective_length_factor,
            "max-tip-deflection": self.max_tip_deflection,
            "max-compressive-deformation":
                self.max_compressive_deformation,
        }

    def calculate(self):
        return run_binary(
            "leg",
            "calculate",
            self._configuration(),
            self.binary_path,
        )

    def minimise_mass(
        self,
        angle_min=5.0,
        angle_max=60.0,
        angle_step=1.0,
        radius_min=0.001,
        radius_max=0.05,
        radius_step=0.0001,
    ):
        options = self._configuration()
        options.update(
            {
                "angle-min": angle_min,
                "angle-max": angle_max,
                "angle-step": angle_step,
                "radius-min": radius_min,
                "radius-max": radius_max,
                "radius-step": radius_step,
            }
        )
        return run_binary(
            "leg",
            "optimize",
            options,
            self.binary_path,
        )
