from math import pi
import numpy as np
import matplotlib.pyplot as plt

import material


class Bending:
    def __init__(self):
        self.t = 0.002          # wall thickness [m]
        self.R = 0.01           # outer radius [m]
        self.L = 0.25           # beam/arm length [m]
        self.T = 0.1              # propeller thrust [N]
        self.SF = 1.5        # Safety factor
        self.rho = material.density     # material density [kg/m^3]
        self.resolution = 0.001         # calculation step size [m]
        self.Y = material.youngs_modulus
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


bend = Bending()
bend.minimise_mass()