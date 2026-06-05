from dataclasses import dataclass
from pathlib import Path

from binary_bridge import load_materials_file, run_binary


@dataclass
class Arm:
    thickness: float = 0.002
    radius: float = 0.01
    length: float = 0.25
    thrust: float = 10.0
    safety_factor: float | None = None
    materials_file: str | Path = Path(__file__).with_name("materials.py")
    density: float | None = None
    youngs_modulus: float | None = None
    failure_stress: float | None = None
    gravity: float | None = None
    max_tip_deflection: float | None = None
    binary_path: str | None = None

    def __post_init__(self):
        material_data = load_materials_file(
            self.materials_file,
            required_sections=("constants", "arm"),
        )
        if self.density is None:
            self.density = material_data.arm.density
        if self.youngs_modulus is None:
            self.youngs_modulus = material_data.arm.youngs_modulus
        if self.failure_stress is None:
            self.failure_stress = material_data.arm.failure_stress
        if self.safety_factor is None:
            self.safety_factor = material_data.arm.safety_factor
        if self.max_tip_deflection is None:
            self.max_tip_deflection = material_data.arm.max_tip_deflection
        if self.gravity is None:
            self.gravity = material_data.constants.g

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
            "gravity": self.gravity,
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
