from dataclasses import dataclass

import materials
from binary_bridge import run_binary


@dataclass
class Arm:
    thickness: float = 0.002
    radius: float = 0.01
    length: float = 0.25
    thrust: float = 10.0
    safety_factor: float = 1.5
    density: float = materials.arm.density
    youngs_modulus: float = materials.arm.youngs_modulus
    failure_stress: float = materials.arm.failure_stress
    max_tip_deflection: float = 0.005e-3
    binary_path: str | None = None

    def _configuration(self):
        return {
            "thickness": self.thickness,
            "radius": self.radius,
            "length": self.length,
            "thrust": self.thrust,
            "safety-factor": self.safety_factor,
            "density": self.density,
            "youngs-modulus": self.youngs_modulus,
            "failure-stress": self.failure_stress,
            "max-tip-deflection": self.max_tip_deflection,
        }

    def calculate(self):
        return run_binary(
            "arm",
            "calculate",
            self._configuration(),
            self.binary_path,
        )

    def minimise_mass(
        self,
        radius_min=0.002,
        radius_max=0.05,
        radius_step=0.0001,
        thickness_min=0.0005,
        thickness_step=0.0001,
    ):
        options = self._configuration()
        options.update(
            {
                "radius-min": radius_min,
                "radius-max": radius_max,
                "radius-step": radius_step,
                "thickness-min": thickness_min,
                "thickness-step": thickness_step,
            }
        )
        return run_binary(
            "arm",
            "optimize",
            options,
            self.binary_path,
        )
