from math import cos, pi, radians, sin
import tkinter as tk
from tkinter import ttk

import materials


class LegOptimizerWindow:
    def __init__(self, root):
        self.root = root
        self.root.title("Leg Optimizer")
        self.root.minsize(760, 520)

        self.material = materials.leg
        self.number_of_legs = 2
        self.pending_update = None

        self.radius_var = tk.DoubleVar(value=5.0)      # mm
        self.angle_var = tk.DoubleVar(value=30.0)      # deg
        self.radius_text = tk.StringVar(value="5.000")
        self.angle_text = tk.StringVar(value="30.000")

        self.settings = {
            "mass": tk.StringVar(value="4.5"),                # kg
            "length": tk.StringVar(value="50.0"),             # mm
            "safety_factor": tk.StringVar(value="1.5"),
            "effective_length_factor": tk.StringVar(value="1.0"),
            "max_tip_deflection": tk.StringVar(value="0.01"), # mm
            "max_compression": tk.StringVar(value="0.01"),    # mm
        }

        self.outputs = {}
        self.status_var = tk.StringVar()

        self.build_layout()
        self.schedule_update()

    def build_layout(self):
        root = self.root
        root.columnconfigure(0, weight=1)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(0, weight=1)

        controls = ttk.Frame(root, padding=16)
        controls.grid(row=0, column=0, sticky="nsew")
        controls.columnconfigure(1, weight=1)

        outputs = ttk.Frame(root, padding=16)
        outputs.grid(row=0, column=1, sticky="nsew")
        outputs.columnconfigure(1, weight=1)

        ttk.Label(controls, text="Inputs", font=("Segoe UI", 14, "bold")).grid(
            row=0, column=0, columnspan=4, sticky="w", pady=(0, 12)
        )

        self.add_slider(
            controls,
            row=1,
            label="Radius R",
            value_var=self.radius_var,
            text_var=self.radius_text,
            minimum=1.0,
            maximum=50.0,
            unit="mm",
        )
        self.add_slider(
            controls,
            row=2,
            label="Angle",
            value_var=self.angle_var,
            text_var=self.angle_text,
            minimum=5.0,
            maximum=60.0,
            unit="deg",
        )

        ttk.Separator(controls).grid(row=3, column=0, columnspan=4, sticky="ew", pady=14)

        setting_rows = [
            ("Vehicle mass", "mass", "kg"),
            ("Leg length", "length", "mm"),
            ("Safety factor", "safety_factor", ""),
            ("Effective length K", "effective_length_factor", ""),
            ("Tip deflection limit", "max_tip_deflection", "mm"),
            ("Compression limit", "max_compression", "mm"),
        ]

        for offset, (label, key, unit) in enumerate(setting_rows, start=4):
            ttk.Label(controls, text=label).grid(row=offset, column=0, sticky="w", pady=4)
            entry = ttk.Entry(controls, textvariable=self.settings[key], width=12)
            entry.grid(row=offset, column=1, sticky="ew", padx=(8, 6), pady=4)
            ttk.Label(controls, text=unit).grid(row=offset, column=2, sticky="w", pady=4)
            entry.bind("<KeyRelease>", self.schedule_update)
            entry.bind("<FocusOut>", self.schedule_update)

        optimize_button = ttk.Button(
            controls,
            text="Find Minimum Mass",
            command=self.find_minimum_mass,
        )
        optimize_button.grid(row=11, column=0, columnspan=3, sticky="ew", pady=(18, 0))

        ttk.Label(
            controls,
            textvariable=self.status_var,
            wraplength=320,
        ).grid(row=12, column=0, columnspan=3, sticky="ew", pady=(12, 0))

        ttk.Label(outputs, text="Outputs", font=("Segoe UI", 14, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 12)
        )

        output_rows = [
            ("leg_mass", "Leg mass"),
            ("area", "Cross-section area"),
            ("inertia", "Second moment I"),
            ("bending_force", "Bending force"),
            ("axial_force", "Axial force"),
            ("tip_deflection", "Tip deflection"),
            ("longitudinal_deflection", "Longitudinal deflection"),
            ("bending_stress", "Bending stress"),
            ("compressive_stress", "Compressive stress"),
            ("governing_stress", "Governing stress"),
            ("yield_stress", "Yield stress"),
            ("buckling_load", "Euler buckling load"),
            ("buckling_margin", "Buckling margin"),
            ("feasible", "Feasible"),
        ]

        for row, (key, label) in enumerate(output_rows, start=1):
            ttk.Label(outputs, text=label).grid(row=row, column=0, sticky="w", pady=4)
            self.outputs[key] = tk.StringVar(value="-")
            ttk.Label(outputs, textvariable=self.outputs[key]).grid(
                row=row, column=1, sticky="e", pady=4
            )

    def add_slider(self, parent, row, label, value_var, text_var, minimum, maximum, unit):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=8)

        slider = ttk.Scale(
            parent,
            from_=minimum,
            to=maximum,
            variable=value_var,
            command=lambda _value: self.slider_changed(text_var, value_var),
        )
        slider.grid(row=row, column=1, sticky="ew", padx=(8, 8), pady=8)

        entry = ttk.Entry(parent, textvariable=text_var, width=10)
        entry.grid(row=row, column=2, sticky="ew", padx=(0, 6), pady=8)
        ttk.Label(parent, text=unit).grid(row=row, column=3, sticky="w", pady=8)

        entry.bind(
            "<KeyRelease>",
            lambda _event: self.entry_changed(text_var, value_var, minimum, maximum),
        )
        entry.bind(
            "<FocusOut>",
            lambda _event: self.entry_changed(
                text_var, value_var, minimum, maximum, clamp_and_format=True
            ),
        )

    def slider_changed(self, text_var, value_var):
        text_var.set(f"{value_var.get():.3f}")
        self.schedule_update()

    def entry_changed(self, text_var, value_var, minimum, maximum, clamp_and_format=False):
        try:
            value = float(text_var.get())
        except ValueError:
            self.status_var.set("Enter numeric values for the active inputs.")
            return

        if clamp_and_format:
            value = max(minimum, min(maximum, value))
            text_var.set(f"{value:.3f}")

        value_var.set(value)
        self.schedule_update()

    def read_float(self, key):
        return float(self.settings[key].get())

    def calculate_design(self, radius_mm, angle_deg):
        radius = radius_mm / 1000
        angle = radians(angle_deg)
        mass_vehicle = self.read_float("mass")
        length = self.read_float("length") / 1000
        safety_factor = self.read_float("safety_factor")
        effective_length_factor = self.read_float("effective_length_factor")
        max_tip_deflection = self.read_float("max_tip_deflection") / 1000
        max_compression = self.read_float("max_compression") / 1000

        if (
            radius <= 0 or
            mass_vehicle <= 0 or
            length <= 0 or
            safety_factor <= 0 or
            effective_length_factor <= 0 or
            max_tip_deflection < 0 or
            max_compression < 0
        ):
            raise ValueError

        area = pi * radius**2
        inertia = (pi * radius**4) / 4
        bending_force = (
            mass_vehicle
            * materials.constants.g
            * sin(angle)
            * safety_factor
            / self.number_of_legs
        )
        axial_force = (
            mass_vehicle
            * materials.constants.g
            * cos(angle)
            * safety_factor
            / self.number_of_legs
        )
        bending_moment = bending_force * length

        tip_deflection = (bending_force * length**3) / (3 * self.material.youngs_modulus * inertia)
        bending_stress = (bending_moment / inertia) * radius
        compressive_stress = abs(axial_force) / area
        longitudinal_deflection = (
            abs(axial_force) * length / (self.material.youngs_modulus * area)
        )
        governing_stress = max(abs(bending_stress), compressive_stress)
        leg_mass = area * length * self.material.density * safety_factor

        effective_length = effective_length_factor * length
        buckling_load = (pi**2 * self.material.youngs_modulus * inertia) / effective_length**2
        buckling_margin = float("inf") if axial_force == 0 else buckling_load / abs(axial_force)

        feasible = (
            tip_deflection <= max_tip_deflection and
            longitudinal_deflection <= max_compression and
            governing_stress <= self.material.yield_strength and
            buckling_load >= abs(axial_force)
        )

        return {
            "area": area,
            "inertia": inertia,
            "bending_force": bending_force,
            "axial_force": axial_force,
            "tip_deflection": tip_deflection,
            "longitudinal_deflection": longitudinal_deflection,
            "bending_stress": bending_stress,
            "compressive_stress": compressive_stress,
            "governing_stress": governing_stress,
            "leg_mass": leg_mass,
            "buckling_load": buckling_load,
            "buckling_margin": buckling_margin,
            "feasible": feasible,
        }

    def schedule_update(self, _event=None):
        if self.pending_update is not None:
            self.root.after_cancel(self.pending_update)
        self.pending_update = self.root.after(80, self.update_outputs)

    def update_outputs(self):
        self.pending_update = None

        try:
            design = self.calculate_design(self.radius_var.get(), self.angle_var.get())
        except (ValueError, ZeroDivisionError):
            self.status_var.set("Enter valid positive numeric settings.")
            return

        self.outputs["leg_mass"].set(f"{design['leg_mass'] * 1000:.3f} g")
        self.outputs["area"].set(f"{design['area'] * 1e6:.3f} mm^2")
        self.outputs["inertia"].set(f"{design['inertia'] * 1e12:.3f} mm^4")
        self.outputs["bending_force"].set(f"{design['bending_force']:.3f} N")
        self.outputs["axial_force"].set(f"{design['axial_force']:.3f} N")
        self.outputs["tip_deflection"].set(f"{design['tip_deflection'] * 1000:.6f} mm")
        self.outputs["longitudinal_deflection"].set(
            f"{design['longitudinal_deflection'] * 1000:.6f} mm"
        )
        self.outputs["bending_stress"].set(f"{design['bending_stress'] / 1e6:.3f} MPa")
        self.outputs["compressive_stress"].set(
            f"{design['compressive_stress'] / 1e6:.3f} MPa"
        )
        self.outputs["governing_stress"].set(f"{design['governing_stress'] / 1e6:.3f} MPa")
        self.outputs["yield_stress"].set(f"{self.material.yield_strength / 1e6:.3f} MPa")
        self.outputs["buckling_load"].set(f"{design['buckling_load']:.3f} N")
        self.outputs["buckling_margin"].set(f"{design['buckling_margin']:.3f}")
        self.outputs["feasible"].set("yes" if design["feasible"] else "no")

        if design["feasible"]:
            self.status_var.set("Current leg design satisfies the active constraints.")
        else:
            self.status_var.set("Current leg design violates one or more constraints.")

    def find_minimum_mass(self):
        try:
            best = None
            checked_designs = 0

            angle_min = 5.0
            angle_max = 60.0
            angle_step = 1.0
            radius_min = 1.0
            radius_max = 50.0
            radius_step = 0.1

            angle_steps = int(round((angle_max - angle_min) / angle_step)) + 1
            radius_steps = int(round((radius_max - radius_min) / radius_step)) + 1

            for angle_index in range(angle_steps):
                angle_deg = angle_min + angle_index * angle_step
                for radius_index in range(radius_steps):
                    radius_mm = radius_min + radius_index * radius_step
                    design = self.calculate_design(radius_mm, angle_deg)
                    checked_designs += 1

                    if design["feasible"] and (
                        best is None or design["leg_mass"] < best["design"]["leg_mass"]
                    ):
                        best = {
                            "radius_mm": radius_mm,
                            "angle_deg": angle_deg,
                            "design": design,
                        }

        except (ValueError, ZeroDivisionError):
            self.status_var.set("Enter valid settings before running the optimizer.")
            return

        if best is None:
            self.status_var.set(
                f"No feasible design found after checking {checked_designs} combinations."
            )
            return

        self.radius_var.set(best["radius_mm"])
        self.angle_var.set(best["angle_deg"])
        self.radius_text.set(f"{best['radius_mm']:.3f}")
        self.angle_text.set(f"{best['angle_deg']:.3f}")
        self.update_outputs()
        self.status_var.set(
            "Minimum feasible mass found after checking "
            f"{checked_designs} combinations."
        )


def main():
    root = tk.Tk()
    LegOptimizerWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()
