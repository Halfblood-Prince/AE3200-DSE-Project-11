from pathlib import Path
import re

import pandas as pd

PROJECT_ROOT = Path(__file__).resolve().parents[1]
TITLE_SPEC_PATTERN = re.compile(
	r"^\s*(?P<diameter>\d+(?:\.\d+)?)x(?P<pitch>\d+(?:\.\d+)?)(?P<suffix>.*)\s*$"
)
BLADE_SUFFIX_PATTERN = re.compile(r"^(?P<extension>.*?)(?:-(?P<blades>\d+))?$")
RPM_RANGE_PATTERN = re.compile(r"^\s*(?P<file_name>\S+)\s+(?P<rpm_min>\d+)\s+(?P<rpm_max>\d+)\s*$")
PROP_RPM_PATTERN = re.compile(r"^\s*PROP RPM =\s*(?P<rpm>\d+)\s*$")


def load_titledat(source_path: str | Path) -> pd.DataFrame:
	"""Load PER2_TITLEDAT.DAT into a DataFrame with file name and title columns."""

	source_path = Path(source_path)
	pattern = re.compile(r"^\s*(\S+)\s+(.*?)\s*$")
	rows: list[dict[str, str]] = []

	for line in source_path.read_text(encoding="utf-8").splitlines():
		if not line.strip():
			continue

		match = pattern.match(line)
		if match is None:
			continue

		file_name, title = match.groups()
		if not file_name.lower().endswith(".dat"):
			continue

		rows.append({"file name": file_name, "title": title})

	return pd.DataFrame(rows, columns=["file name", "title"])


def load_rpm_range(source_path: str | Path) -> pd.DataFrame:
	"""Load PER2_RPMRANGE.DAT into a DataFrame with file name, RPM min, and RPM max columns."""

	source_path = Path(source_path)
	rows: list[dict[str, int | str]] = []

	for line in source_path.read_text(encoding="utf-8").splitlines():
		match = RPM_RANGE_PATTERN.match(line)
		if match is None:
			continue

		file_name, rpm_min, rpm_max = match.groups()
		rows.append(
			{
				"file name": file_name,
				"RPM min": int(rpm_min),
				"RPM max": int(rpm_max),
			}
		)

	return pd.DataFrame(rows, columns=["file name", "RPM min", "RPM max"])


def load_propeller_data(rpmrange_path: str | Path) -> pd.DataFrame:
	"""Load and match title data with RPM range data using normalized propeller file names."""

	rpmrange_path = Path(rpmrange_path)
	titledat_path = rpmrange_path.with_name("PER2_TITLEDAT.DAT")

	title_df = load_titledat(titledat_path).copy()
	title_df["filename"] = title_df["file name"]
	title_df["match name"] = (
		title_df["filename"]
		.str.removeprefix("PER3_")
		.str.removesuffix(".dat")
	)

	rpm_df = load_rpm_range(rpmrange_path).rename(columns={"file name": "match name"})

	title_names = set(title_df["match name"])
	rpm_names = set(rpm_df["match name"])
	missing_from_rpm = sorted(title_names - rpm_names)
	missing_from_title = sorted(rpm_names - title_names)
	if missing_from_rpm or missing_from_title:
		raise ValueError(
			"Title and RPM files do not contain the same propeller set: "
			f"missing from RPM={missing_from_rpm[:5]}, missing from title={missing_from_title[:5]}"
		)

	propeller_df = title_df.merge(rpm_df, on="match name", how="inner", validate="one_to_one")

	return propeller_df[["filename", "title", "RPM min", "RPM max"]]


def extract_propeller_specs(rpmrange_path: str | Path) -> pd.DataFrame:
	"""Extract geometry, blade count, and RPM data for each propeller."""

	propeller_df = load_propeller_data(rpmrange_path)
	required_columns = {"filename", "title", "RPM min", "RPM max"}
	missing_columns = required_columns.difference(propeller_df.columns)
	if missing_columns:
		missing = ", ".join(sorted(missing_columns))
		raise KeyError(f"Missing required columns: {missing}")

	extracted = propeller_df["title"].str.extract(TITLE_SPEC_PATTERN)
	invalid_rows = extracted.isna().any(axis=1)
	if invalid_rows.any():
		invalid_titles = propeller_df.loc[invalid_rows, "title"].tolist()
		raise ValueError(f"Could not parse propeller title(s): {invalid_titles}")

	suffix_parts = extracted["suffix"].str.strip().str.extract(BLADE_SUFFIX_PATTERN)
	extension = suffix_parts["extension"].fillna("").str.strip()
	extension = extension.where(extension.str.contains(r"[A-Za-z]", na=False), "[-]")
	blades = pd.to_numeric(suffix_parts["blades"], errors="coerce").fillna(2).astype(int)

	specs_df = pd.DataFrame(
		{
			"filename": propeller_df["filename"],
			"Diameter": pd.to_numeric(extracted["diameter"]),
			"Pitch": pd.to_numeric(extracted["pitch"]),
			"RPM min": propeller_df["RPM min"],
			"RPM max": propeller_df["RPM max"],
			"Blades": blades,
			"Extension": extension,
		},
		index=propeller_df.index,
	)

	return specs_df.sort_values(["Diameter", "Pitch"], kind="mergesort").reset_index(drop=True)


def filter_propellers(
	propeller_df: pd.DataFrame,
	max_diameter: float | None = None,
	extensions: list[str] | None = None,
	max_pitch: float | None = None,
	blades: int | list[int] | None = None,
) -> pd.DataFrame:
	"""Filter propellers by optional diameter, pitch, extension, and blade-count rules."""

	required_columns = {"filename", "Diameter", "Pitch", "RPM min", "RPM max", "Blades", "Extension"}
	missing_columns = required_columns.difference(propeller_df.columns)
	if missing_columns:
		missing = ", ".join(sorted(missing_columns))
		raise KeyError(f"Missing required columns: {missing}")

	filter_mask = pd.Series(True, index=propeller_df.index)

	if max_diameter is not None:
		filter_mask &= propeller_df["Diameter"].le(max_diameter)

	if max_pitch is not None:
		filter_mask &= propeller_df["Pitch"].le(max_pitch)

	if extensions:
		normalized_extensions = {extension.strip().upper() for extension in extensions}
		filter_mask &= (
			propeller_df["Extension"].astype(str).str.strip().str.upper().isin(normalized_extensions)
		)

	if blades is not None:
		blade_values = [blades] if isinstance(blades, int) else blades
		if blade_values:
			filter_mask &= propeller_df["Blades"].isin(blade_values)

	filtered_df = propeller_df[filter_mask]

	return filtered_df.reset_index(drop=True)


def load_static_performance(
	filename: str,
	propeller_df: pd.DataFrame | None = None,
	rpm_start: int = 10000,
	rpm_step: int = 3000,
	strict: bool = True,
) -> pd.DataFrame:
	"""Load thrust and power at zero velocity from a PERFILES2 propeller data file."""

	if propeller_df is None:
		specs_path = Path(__file__).resolve().parent / "propellers_data.csv"
		if specs_path.exists():
			propeller_df = pd.read_csv(specs_path)
		else:
			rpmrange_path = PROJECT_ROOT / "airfoilsdat" / "PER2_RPMRANGE.DAT"
			propeller_df = extract_propeller_specs(rpmrange_path)

	required_columns = {"filename", "RPM max"}
	missing_columns = required_columns.difference(propeller_df.columns)
	if missing_columns:
		missing = ", ".join(sorted(missing_columns))
		raise KeyError(f"Missing required columns: {missing}")

	match_df = propeller_df.loc[propeller_df["filename"] == filename, ["filename", "RPM max"]]
	if match_df.empty:
		raise ValueError(f"Could not find propeller data for filename: {filename}")

	rpm_max = int(match_df.iloc[0]["RPM max"])
	target_rpms = list(range(rpm_start, rpm_max + 1, rpm_step))
	if rpm_max not in target_rpms:
		target_rpms.append(rpm_max)
	if not target_rpms:
		target_rpms = [rpm_max]

	target_rpm_set = set(target_rpms)
	file_path = PROJECT_ROOT / "airfoilsdat" / "PERFILES2" / filename
	if not file_path.exists():
		raise FileNotFoundError(f"Could not find performance file: {file_path}")

	rows: list[dict[str, float | int | str]] = []
	current_rpm: int | None = None

	for line in file_path.read_text(encoding="utf-8").splitlines():
		rpm_match = PROP_RPM_PATTERN.match(line)
		if rpm_match is not None:
			current_rpm = int(rpm_match.group("rpm"))
			continue

		if current_rpm not in target_rpm_set:
			continue

		tokens = line.split()
		if not tokens:
			continue

		if tokens[0] != "0.00" or len(tokens) < 11:
			continue

		rows.append(
			{
				"filename": filename,
				"RPM": current_rpm,
				"Power": float(tokens[8]),
				"Thrust": float(tokens[10]),
			}
		)
		target_rpm_set.remove(current_rpm)
		current_rpm = None

	if target_rpm_set and strict:
		missing_rpms = ", ".join(str(rpm) for rpm in sorted(target_rpm_set))
		raise ValueError(f"Could not find zero-velocity data for RPM values: {missing_rpms}")

	return pd.DataFrame(rows, columns=["filename", "RPM", "Power", "Thrust"]).sort_values(
		"RPM"
	).reset_index(drop=True)


def build_static_propeller_table(
	propeller_df: pd.DataFrame,
	max_diameter: float | None = None,
	extensions: list[str] | None = None,
	max_pitch: float | None = None,
	blades: int | list[int] | None = None,
	rpm_start: int = 10000,
	rpm_step: int = 3000,
	strict_static_data: bool = False,
) -> pd.DataFrame:
	"""Build a long-form table with propeller geometry and static performance data."""

	filtered_df = filter_propellers(
		propeller_df,
		max_diameter=max_diameter,
		extensions=extensions,
		max_pitch=max_pitch,
		blades=blades,
	)
	if filtered_df.empty:
		return pd.DataFrame(
			columns=[
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
			]
		)

	rows: list[pd.DataFrame] = []
	for propeller in filtered_df.to_dict("records"):
		static_df = load_static_performance(
			propeller["filename"],
			propeller_df=filtered_df,
			rpm_start=rpm_start,
			rpm_step=rpm_step,
			strict=strict_static_data,
		)
		static_df["Diameter"] = propeller["Diameter"]
		static_df["Pitch"] = propeller["Pitch"]
		static_df["RPM min"] = propeller["RPM min"]
		static_df["RPM max"] = propeller["RPM max"]
		static_df["Blades"] = propeller["Blades"]
		static_df["Extension"] = propeller["Extension"]
		rows.append(static_df)

	combined_df = pd.concat(rows, ignore_index=True)
	return combined_df[
		[
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
		]
	].sort_values(["Diameter", "Pitch", "RPM"], kind="mergesort").reset_index(drop=True)



if __name__ == "__main__":
	# default_path = PROJECT_ROOT / "airfoilsdat" / "PER2_RPMRANGE.DAT"
	# data = extract_propeller_specs(default_path)
	# output_path = Path(__file__).resolve().parent / "propellers_data.csv"
	# data.to_csv(output_path, index=False)
	# print(f"Saved propeller data to {output_path}")
    df = pd.read_csv("propulsion/propellers_data.csv")
    # print(filter_propellers(df, max_diameter=8.0, extensions=["E"]))
    df = build_static_propeller_table(df, max_diameter=6.5, extensions=["E"], strict_static_data=False, rpm_step=1000) 
    print(df)
    df.to_csv("propulsion/6.5_E_1000.csv", index=False)