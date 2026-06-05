from math import pi, sin, cos, radians
import numpy as np
import matplotlib.pyplot as plt


import materials


class Arm:
    def __init__(self):
        self.t = 0.002          # wall thickness [m]
        self.R = 0.01           # outer radius [m]
        self.L = 0.25           # beam/arm length [m]
        self.T = 10              # propeller thrust [N]
        self.SF = 1.5        # Safety factor
        self.rho = materials.arm.density     # material density [kg/m^3]
        self.resolution = 0.001         # calculation step size [m]
        self.Y = materials.arm.youngs_modulus
        self.max_tip_deflection = 0.005e-3  # [m]
        self.area = None
        self.inertia = None
        self.w = None
        self.z = None
        self.V = None
        self.Mx = None
        self.max_bending_moment = 0.0
        self.z_at_max_bending_moment = 0.0
        self.sigma_z_max = 0.0

        self.calculate_cross_section_area()
        self.calculate_w()
        self.calculate_area_moment_of_inertia()
        self.calculate_maximum_longitudinal_bending_moment()
        self.calculate_maximum_bending_stress()
        self.print_results()
        self.plot_shear_and_bending()

    def __main__(self):
        self.calculate_cross_section_area()
        self.calculate_w()
        self.calculate_area_moment_of_inertia()
        self.calculate_maximum_longitudinal_bending_moment()
        self.calculate_maximum_bending_stress()


    def calculate_cross_section_area(self):
        R_inner = self.R - self.t
        self.area = pi * (self.R**2 - R_inner**2)

    def calculate_w(self):
        self.w = self.rho * 9.81 * self.area

    def calculate_area_moment_of_inertia(self):
        R_inner = self.R - self.t
        self.inertia = (pi / 4) * (self.R**4 - R_inner**4)

    def calculate_maximum_longitudinal_bending_moment(self):
        self.z = np.arange(0.0, self.L + 1e-7, self.resolution)
        self.V = -self.T + self.w * (self.L - self.z)
        self.Mx = (self.T * (self.L - self.z) - 0.5 * self.w * (self.L - self.z)**2)
        idx = np.argmax(np.abs(self.Mx))
        self.max_bending_moment = self.Mx[idx]
        self.z_at_max_bending_moment = self.z[idx]

    def calculate_maximum_bending_stress(self):
        self.sigma_z_max = abs(self.max_bending_moment) * self.R / self.inertia

    def print_results(self):
        print("Cross-sectional area:", self.area, "m^2")
        print("Distributed load w:", self.w, "N/m")
        print("Second moment of area I:", self.inertia, "m^4")
        print(
            "Maximum bending moment:",
            self.max_bending_moment,
            "Nm at z =",
            self.z_at_max_bending_moment,
            "m"
        )
        print("Maximum bending stress:", self.sigma_z_max, "Pa")
        print("Maximum bending stress:", self.sigma_z_max / 1e6, "MPa")
        print("Design bending stress with SF:", self.SF * self.sigma_z_max / 1e6, "MPa")

    def plot_shear_and_bending(self):
        fig, (ax_shear, ax_bending) = plt.subplots(1, 2, figsize=(12, 5), sharex=True)

        ax_shear.plot(self.z, self.V, color="green", label="Shear Force")
        ax_shear.axhline(0, color="black", linewidth=0.8)
        ax_shear.set_xlabel("z [m]")
        ax_shear.set_ylabel("Shear force [N]")
        ax_shear.set_title("Shear Force Diagram")
        ax_shear.legend()
        ax_shear.grid(True)

        ax_bending.plot(self.z, self.Mx, color="orange", label="Bending Moment")
        ax_bending.axhline(0, color="black", linewidth=0.8)
        ax_bending.set_xlabel("z [m]")
        ax_bending.set_ylabel("Bending moment [Nm]")
        ax_bending.set_title("Bending Moment Diagram")
        ax_bending.legend()
        ax_bending.grid(True)

        fig.tight_layout()
        plt.show()

    def calculate_deflection(self):
        return (self.w*self.L**4)/(8*self.Y*self.inertia) + (self.T*self.L**3)/(3*self.Y*self.inertia)

    def minimise_mass(self):
        r_optimal, t_optimal = 0.0, 0.0
        c = 0
        temp = 1e10
        for r in np.arange(0.002, 10.0, 0.001):
            self.R = r
            for t in np.arange(0.0005, r, 0.005):
                self.t = t
                error = self.Y - self.sigma_z_max
                if error < temp:
                    temp = error
                    r_optimal = r
                    t_optimal = t
                c+=1
                print(c)

        print(r, t)


class Leg:
    def __init__(self, m):
        self.E = materials.leg.youngs_modulus
        self.rho = materials.leg.density
        self.yield_strength = materials.leg.yield_strength
        self.m = m
        self.area = 0.0
        self.I = 0
        self.g = materials.constants.g
        self.number_of_legs = 2
        self.angle_deg = 30
        self.angle = radians(self.angle_deg)
        self.R = 0.005
        self.SF = 1.5
        self.L = 0.1
        self.effective_length_factor = 1.0
        self.max_tip_deflection = 0.005e-3  # [m]
        self.max_compressive_deformation = 0.01e-3
        self.bending_force = 0.0
        self.bending_moment = 0.0
        self.axial_force = 0.0
        self.longitudinal_compressive_stress = 0.0
        self.longitudinal_deflection = 0.0
        self.maximum_normal_stress = 0.0
        self.euler_buckling_load = 0.0
        self.buckling_margin = 0.0
        self.buckling_safe = False
        self.tip_deflection = 0.0
        self.sigma_z_max = 0.0
        self.mass = 0.0
        self.checked_designs = 0

    def calculate_max_bending_stress(self):
        self.area = pi * self.R**2
        self.I = (pi*self.R**4)/4
        self.bending_force = (
            self.m * self.g * sin(self.angle) * self.SF / self.number_of_legs
        )
        self.bending_moment = self.bending_force * self.L
        self.tip_deflection = (self.bending_force * self.L**3)/(3*self.E*self.I)
        self.sigma_z_max = (self.bending_moment/self.I)*self.R
        self.mass = self.area * self.L * self.rho * self.SF
        self.calculate_uniaxial_compression_and_deflection()
        self.maximum_normal_stress = max(
            abs(self.sigma_z_max),
            self.longitudinal_compressive_stress,
        )
        self.calculate_euler_buckling()

    def calculate_uniaxial_compression_and_deflection(self):
        if self.area == 0:
            self.area = pi * self.R**2

        self.axial_force = (
            self.m * self.g * cos(self.angle) * self.SF / self.number_of_legs
        )
        self.longitudinal_compressive_stress = abs(self.axial_force) / self.area
        self.longitudinal_deflection = abs(self.axial_force) * self.L / (self.E * self.area)
        self.maximum_normal_stress = max(
            abs(self.sigma_z_max),
            self.longitudinal_compressive_stress,
        )
        return self.longitudinal_compressive_stress, self.longitudinal_deflection

    def calculate_euler_buckling(self, effective_length_factor=None):
        if effective_length_factor is not None:
            self.effective_length_factor = effective_length_factor

        if self.I == 0:
            self.area = pi * self.R**2
            self.I = (pi*self.R**4)/4

        effective_length = self.effective_length_factor * self.L
        self.calculate_uniaxial_compression_and_deflection()
        self.euler_buckling_load = (pi**2 * self.E * self.I)/(effective_length**2)

        if self.axial_force == 0:
            self.buckling_margin = float("inf")
        else:
            self.buckling_margin = self.euler_buckling_load / abs(self.axial_force)

        self.buckling_safe = self.euler_buckling_load >= abs(self.axial_force)
        return self.euler_buckling_load

    def minimise_mass(
        self,
        angle_min=5.0,
        angle_max=60.0,
        angle_step=1.0,
        R_min=0.001,
        R_max=0.05,
        R_step=0.0001,
        max_tip_deflection=None,
    ):
        if max_tip_deflection is None:
            max_tip_deflection = self.max_tip_deflection

        best = None
        self.checked_designs = 0

        for angle_deg in np.arange(angle_min, angle_max + 0.5 * angle_step, angle_step):
            angle = radians(angle_deg)
            bending_force = (
                self.m * self.g * sin(angle) * self.SF / self.number_of_legs
            )
            bending_moment = bending_force * self.L
            axial_force = (
                self.m * self.g * cos(angle) * self.SF / self.number_of_legs
            )

            for R in np.arange(R_min, R_max + 0.5 * R_step, R_step):
                area = pi * R**2
                I = (pi * R**4)/4
                tip_deflection = (bending_force * self.L**3)/(3*self.E*I)
                sigma_z_max = (bending_moment/I)*R
                longitudinal_compressive_stress = abs(axial_force) / area
                longitudinal_deflection = abs(axial_force) * self.L / (self.E * area)
                maximum_normal_stress = max(
                    abs(sigma_z_max),
                    longitudinal_compressive_stress,
                )
                effective_length = self.effective_length_factor * self.L
                euler_buckling_load = (pi**2 * self.E * I)/(effective_length**2)
                buckling_margin = (
                    float("inf")
                    if axial_force == 0
                    else euler_buckling_load / abs(axial_force)
                )
                buckling_safe = euler_buckling_load >= abs(axial_force)
                mass = area * self.L * self.rho * self.SF
                self.checked_designs += 1

                if (
                    tip_deflection <= max_tip_deflection and
                    longitudinal_deflection <= self.max_compressive_deformation and
                    maximum_normal_stress <= self.yield_strength and
                    buckling_safe
                ):
                    if best is None or mass < best["mass"]:
                        best = {
                            "angle_deg": angle_deg,
                            "angle": angle,
                            "R": R,
                            "area": area,
                            "I": I,
                            "bending_force": bending_force,
                            "bending_moment": bending_moment,
                            "axial_force": axial_force,
                            "tip_deflection": tip_deflection,
                            "sigma_z_max": sigma_z_max,
                            "longitudinal_compressive_stress": longitudinal_compressive_stress,
                            "longitudinal_deflection": longitudinal_deflection,
                            "maximum_normal_stress": maximum_normal_stress,
                            "euler_buckling_load": euler_buckling_load,
                            "buckling_margin": buckling_margin,
                            "buckling_safe": buckling_safe,
                            "mass": mass,
                        }

        if best is None:
            print("No feasible leg design found for the configured search range.")
            return None

        self.angle_deg = best["angle_deg"]
        self.angle = best["angle"]
        self.R = best["R"]
        self.area = best["area"]
        self.I = best["I"]
        self.bending_force = best["bending_force"]
        self.bending_moment = best["bending_moment"]
        self.axial_force = best["axial_force"]
        self.tip_deflection = best["tip_deflection"]
        self.sigma_z_max = best["sigma_z_max"]
        self.longitudinal_compressive_stress = best["longitudinal_compressive_stress"]
        self.longitudinal_deflection = best["longitudinal_deflection"]
        self.maximum_normal_stress = best["maximum_normal_stress"]
        self.euler_buckling_load = best["euler_buckling_load"]
        self.buckling_margin = best["buckling_margin"]
        self.buckling_safe = best["buckling_safe"]
        self.mass = best["mass"]

        print("Checked leg designs:", self.checked_designs)
        print("Optimal leg angle:", self.angle_deg, "deg")
        print("Optimal leg radius:", self.R*1000, "mm")
        print("Optimal leg mass:", self.mass*1000, "g")
        print("Tip deflection:", self.tip_deflection*1000, "mm")
        print("Maximum compressive deformation limit:", self.max_compressive_deformation * 1000, "mm")
        print("Maximum bending stress:", self.sigma_z_max / 1e6, "MPa")
        print("Governing normal stress:", self.maximum_normal_stress / 1e6, "MPa")
        print("Yield stress:", self.yield_strength / 1e6, "MPa")
        print("Euler buckling load:", self.euler_buckling_load, "N")
        print("Axial compressive load:", self.axial_force, "N")
        print("Longitudinal compressive stress:", self.longitudinal_compressive_stress / 1e6, "MPa")
        print("Longitudinal deflection:", self.longitudinal_deflection * 1000, "mm")
        print("Buckling margin:", self.buckling_margin)

        return best


def main():
    leg = Leg(4.5)
    leg.minimise_mass()


if __name__ == "__main__":
    main()
