from dataclasses import dataclass
from pathlib import Path

try:
    from .binary_bridge import load_materials_file, run_binary
except ImportError:
    from binary_bridge import load_materials_file, run_binary


@dataclass
class Leg:
    vehicle_mass: float = 4.5
    radius: float = 0.005
    angle_deg: float = 30.0
    height: float = 0.1
    safety_factor: float | None = None
    number_of_legs: int = 2
    materials_file: str | Path = Path(__file__).with_name("materials.py")
    density: float | None = None
    youngs_modulus: float | None = None
    yield_strength: float | None = None
    gravity: float | None = None
    effective_length_factor: float = 1.0
    max_tip_deflection: float | None = None
    max_compressive_deformation: float | None = None
    binary_path: str | None = None

    def __post_init__(self):
        material_data = load_materials_file(
            self.materials_file,
            required_sections=("constants", "leg"),
        )
        if self.density is None:
            self.density = material_data.leg.density
        if self.youngs_modulus is None:
            self.youngs_modulus = material_data.leg.youngs_modulus
        if self.yield_strength is None:
            self.yield_strength = material_data.leg.yield_strength
        if self.safety_factor is None:
            self.safety_factor = material_data.leg.safety_factor
        if self.max_tip_deflection is None:
            self.max_tip_deflection = material_data.leg.max_tip_deflection
        if self.max_compressive_deformation is None:
            self.max_compressive_deformation = (
                material_data.leg.max_compressive_deformation
            )
        if self.gravity is None:
            self.gravity = material_data.constants.g

    def _configuration(self):
        return {
            "mass": self.vehicle_mass,
            "radius": self.radius,
            "angle-deg": self.angle_deg,
            "height": self.height,
            "safety-factor": self.safety_factor,
            "number-of-legs": self.number_of_legs,
            "density": self.density,
            "youngs-modulus": self.youngs_modulus,
            "yield-strength": self.yield_strength,
            "gravity": self.gravity,
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
