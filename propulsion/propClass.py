"""
README

This module turns a propeller CSV table into Python objects.

Expected CSV columns:
- filename
- Diameter
- Pitch
- RPM min
- RPM max
- Blades
- Extension
- RPM
- Power
- Thrust

How it works:
- Each propeller appears in multiple CSV rows, one row per RPM value.
- `load_propeller_dict(...)` groups the rows by `filename`.
- Each group becomes one `Propeller` object.

Main attributes of each `Propeller`:
- `self.name`
- `self.Diameter`
- `self.Pitch`
- `self.RPMmin`
- `self.RPMmax`
- `self.Blades`
- `self.Extension`
- `self.Power`
- `self.Thrust`

`self.Power` and `self.Thrust` are dictionaries:
- key: integer RPM
- value: power or thrust at that RPM

Example:
    from propulsion.propClass import load_propeller_dict

    props = load_propeller_dict("propulsion/8.0_E.csv")
    prop = props["PER3_4x33E.dat"]

    print(prop.Diameter)
    print(prop.Power[10000])
    print(prop.Thrust[10000])

You can also load a single propeller directly:
    from propulsion.propClass import Propeller

    prop = Propeller.from_csv("propulsion/8.0_E.csv", "PER3_4x45E.dat")
    print(prop.Power[13000])
"""

from pathlib import Path

import pandas as pd


class Propeller:
	"""Container for one propeller's geometric and static-performance data."""

	def __init__(self, propeller_df: pd.DataFrame) -> None:
		required_columns = {
			"filename",
			"Diameter",
			"Pitch",
			"RPM min",
			"RPM max",
			"Blades",
			"Extension",
			"RPM",
			"Power",
			"Thrust",
		}
		missing_columns = required_columns.difference(propeller_df.columns)
		if missing_columns:
			missing = ", ".join(sorted(missing_columns))
			raise KeyError(f"Missing required columns: {missing}")

		if propeller_df.empty:
			raise ValueError("propeller_df cannot be empty")

		propeller_df = propeller_df.sort_values("RPM").reset_index(drop=True)
		first_row = propeller_df.iloc[0]

		self.name = str(first_row["filename"])
		self.Diameter = float(first_row["Diameter"])
		self.Pitch = float(first_row["Pitch"])
		self.RPMmin = int(first_row["RPM min"])
		self.RPMmax = int(first_row["RPM max"])
		self.Blades = int(first_row["Blades"])
		self.Extension = str(first_row["Extension"])
		self.Power = {
			int(row["RPM"]): float(row["Power"])
			for row in propeller_df[["RPM", "Power"]].to_dict("records")
		}
		self.Thrust = {
			int(row["RPM"]): float(row["Thrust"])
			for row in propeller_df[["RPM", "Thrust"]].to_dict("records")
		}

	@classmethod
	def from_csv(cls, csv_path: str | Path, propeller_name: str) -> "Propeller":
		"""Load a single propeller from a CSV file."""

		csv_path = Path(csv_path)
		propeller_df = pd.read_csv(csv_path)
		matched_df = propeller_df.loc[propeller_df["filename"] == propeller_name]
		if matched_df.empty:
			raise ValueError(f"Could not find propeller in CSV: {propeller_name}")

		return cls(matched_df)


def load_propeller_dict(csv_path: str | Path) -> dict[str, Propeller]:
	"""Load a CSV file into a dictionary of Propeller objects keyed by name."""

	csv_path = Path(csv_path)
	propeller_df = pd.read_csv(csv_path)

	return {
		filename: Propeller(group.reset_index(drop=True))
		for filename, group in propeller_df.groupby("filename", sort=False)
	}
if __name__ == "__main__":
    x45E = Propeller.from_csv("propulsion/8.0_E.csv", "PER3_4x45E.dat")

    print(x45E.name)
    print(x45E.Power)
    print(x45E.Thrust)
    print(x45E.Power[13000])
    print(x45E.Thrust[13000])
