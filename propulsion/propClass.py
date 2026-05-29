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

If the requested RPM is not available exactly:
- the closest valid RPM is used automatically
- if two RPM values are equally close, the lower RPM is used
- requests below `10000` or above `self.RPMmax` raise an error

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


class RPMLookup(dict):
	"""Dictionary-like RPM lookup that snaps missing RPM requests to the nearest valid RPM."""

	def __init__(
		self,
		rpm_to_value: dict[int, float],
		min_rpm: int,
		max_rpm: int,
	) -> None:
		super().__init__(sorted((int(rpm), float(value)) for rpm, value in rpm_to_value.items()))
		self.min_rpm = int(min_rpm)
		self.max_rpm = int(max_rpm)

	def _validate_rpm(self, rpm: int | float) -> float:
		requested_rpm = float(rpm)
		if requested_rpm < self.min_rpm or requested_rpm > self.max_rpm:
			raise ValueError(
				f"RPM {rpm} is outside the valid range [{self.min_rpm}, {self.max_rpm}]"
			)

		return requested_rpm

	def closest_rpm(self, rpm: int | float) -> int:
		"""Return the closest valid RPM key."""

		if not self:
			raise KeyError("RPMLookup is empty")

		requested_rpm = self._validate_rpm(rpm)
		valid_rpms = list(self.keys())
		return min(valid_rpms, key=lambda valid_rpm: (abs(valid_rpm - requested_rpm), valid_rpm))

	def __getitem__(self, rpm: int | float) -> float:
		closest_rpm = self.closest_rpm(rpm)
		return super().__getitem__(closest_rpm)

	def get(self, rpm: int | float, default: float | None = None) -> float | None:
		if not self:
			return default

		try:
			return self[rpm]
		except (KeyError, TypeError, ValueError):
			return default


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
		self.RPMmin = int(propeller_df["RPM"].min())
		self.RPMmax = int(first_row["RPM max"])
		self.Blades = int(first_row["Blades"])
		self.Extension = str(first_row["Extension"])
		self.Power = RPMLookup(
			{
				int(row["RPM"]): float(row["Power"])
				for row in propeller_df[["RPM", "Power"]].to_dict("records")
			},
			min_rpm=self.RPMmin,
			max_rpm=self.RPMmax,
		)
		self.Thrust = RPMLookup(
			{
				int(row["RPM"]): float(row["Thrust"])
				for row in propeller_df[["RPM", "Thrust"]].to_dict("records")
			},
			min_rpm=self.RPMmin,
			max_rpm=self.RPMmax,
		)

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
	print(x45E.Power[13500])
	print(x45E.Power.closest_rpm(13500))
