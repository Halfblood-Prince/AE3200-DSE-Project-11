import pathlib
import sys

import math
import pandas as pd
import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from propulsion.propClass import Propeller, RPMLookup, load_propeller_dict


def _build_propeller_dataframe() -> pd.DataFrame:
    return pd.DataFrame(
        [
            {
                "filename": "PER3_10x4E.dat",
                "Diameter": 10.0,
                "Pitch": 4.0,
                "RPM min": 5000,
                "RPM max": 14000,
                "Blades": 2,
                "Extension": "E",
                "RPM": 12000,
                "Power": 2.0,
                "Torque": 0.7,
                "Thrust": 3.0,
            },
            {
                "filename": "PER3_10x4E.dat",
                "Diameter": 10.0,
                "Pitch": 4.0,
                "RPM min": 5000,
                "RPM max": 14000,
                "Blades": 2,
                "Extension": "E",
                "RPM": 10000,
                "Power": 1.5,
                "Torque": 0.6,
                "Thrust": 2.5,
            },
            {
                "filename": "PER3_10x4E.dat",
                "Diameter": 10.0,
                "Pitch": 4.0,
                "RPM min": 5000,
                "RPM max": 14000,
                "Blades": 2,
                "Extension": "E",
                "RPM": 14000,
                "Power": 2.5,
                "Torque": 0.8,
                "Thrust": 3.5,
            },
        ]
    )


def _build_multi_propeller_dataframe() -> pd.DataFrame:
    return pd.DataFrame(
        [
            {
                "filename": "PER3_10x4E.dat",
                "Diameter": 10.0,
                "Pitch": 4.0,
                "RPM min": 5000,
                "RPM max": 12000,
                "Blades": 2,
                "Extension": "E",
                "RPM": 10000,
                "Power": 1.5,
                "Torque": 0.6,
                "Thrust": 2.5,
            },
            {
                "filename": "PER3_10x4E.dat",
                "Diameter": 10.0,
                "Pitch": 4.0,
                "RPM min": 5000,
                "RPM max": 12000,
                "Blades": 2,
                "Extension": "E",
                "RPM": 12000,
                "Power": 2.0,
                "Torque": 0.7,
                "Thrust": 3.0,
            },
            {
                "filename": "PER3_12x6EP-3.dat",
                "Diameter": 12.0,
                "Pitch": 6.0,
                "RPM min": 7000,
                "RPM max": 14000,
                "Blades": 3,
                "Extension": "EP",
                "RPM": 10000,
                "Power": 2.5,
                "Torque": 0.9,
                "Thrust": 4.0,
            },
            {
                "filename": "PER3_12x6EP-3.dat",
                "Diameter": 12.0,
                "Pitch": 6.0,
                "RPM min": 7000,
                "RPM max": 14000,
                "Blades": 3,
                "Extension": "EP",
                "RPM": 14000,
                "Power": 3.5,
                "Torque": 1.1,
                "Thrust": 5.0,
            },
        ]
    )


def _write_propeller_csv(tmp_path: pathlib.Path, data: pd.DataFrame) -> pathlib.Path:
    csv_path = tmp_path / "propellers.csv"
    data.to_csv(csv_path, index=False)
    return csv_path


def test_SIZE_PROP_CLS_UT_01():
    # Test RPMLookup exact RPM access
    lookup = RPMLookup({10000: 1.5, 12000: 2.0}, min_rpm=10000, max_rpm=12000)

    assert lookup[12000] == 2.0


def test_SIZE_PROP_CLS_UT_02():
    # Test RPMLookup snaps to the nearest valid RPM
    lookup = RPMLookup({10000: 1.5, 12000: 2.0, 14000: 2.5}, min_rpm=10000, max_rpm=14000)

    assert lookup[13100] == 2.5


def test_SIZE_PROP_CLS_UT_03():
    # Test RPMLookup uses the lower RPM when two values are equally close
    lookup = RPMLookup({10000: 1.5, 12000: 2.0}, min_rpm=10000, max_rpm=12000)

    assert lookup.closest_rpm(11000) == 10000


def test_SIZE_PROP_CLS_UT_04():
    # Test RPMLookup rejects RPM requests outside the valid range
    lookup = RPMLookup({10000: 1.5, 12000: 2.0}, min_rpm=10000, max_rpm=12000)

    with pytest.raises(ValueError, match="outside the valid range"):
        _ = lookup[9000]


def test_SIZE_PROP_CLS_UT_05():
    # Test RPMLookup.get returns the provided default for invalid requests
    lookup = RPMLookup({10000: 1.5, 12000: 2.0}, min_rpm=10000, max_rpm=12000)

    assert lookup.get("not_a_number", default=9.9) == 9.9


def test_SIZE_PROP_CLS_UT_06():
    # Test Propeller builds attributes and RPM lookup tables from a DataFrame
    propeller = Propeller(_build_propeller_dataframe())

    assert propeller.name == "PER3_10x4E.dat"
    assert propeller.Diameter == 10.0
    assert propeller.Pitch == 4.0
    assert propeller.RPMmin == 10000
    assert propeller.RPMmax == 14000
    assert propeller.Blades == 2
    assert propeller.Extension == "E"
    assert propeller.Power[12000] == 2.0
    assert propeller.Torque[13000] == 0.7
    assert propeller.TorqueThrustRatio[13000] == pytest.approx(0.7 / 3.0)
    assert propeller.Thrust[13000] == 3.0


def test_SIZE_PROP_CLS_UT_07():
    # Test Propeller rejects DataFrames that miss required columns
    propeller_df = pd.DataFrame([{"filename": "PER3_10x4E.dat", "Diameter": 10.0}])

    with pytest.raises(KeyError, match="Missing required columns"):
        Propeller(propeller_df)


def test_SIZE_PROP_CLS_UT_08():
    # Test Propeller computes torque when loading an older CSV without a Torque column
    propeller_df = _build_propeller_dataframe().drop(columns=["Torque"])

    propeller = Propeller(propeller_df)

    expected_torque = 2.0 / (math.tau * 12000 / 60.0)
    assert propeller.Torque[12000] == pytest.approx(expected_torque)
    assert propeller.TorqueThrustRatio[12000] == pytest.approx(expected_torque / 3.0)


def test_SIZE_PROP_CLS_UT_09():
    # Test Propeller rejects empty DataFrames even when columns are present
    propeller_df = pd.DataFrame(
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
            "Torque",
            "Thrust",
        ]
    )

    with pytest.raises(ValueError, match="cannot be empty"):
        Propeller(propeller_df)


def test_SIZE_PROP_CLS_UT_10(tmp_path):
    # Test Propeller.from_csv loads only the requested propeller rows
    csv_path = _write_propeller_csv(tmp_path, _build_multi_propeller_dataframe())

    propeller = Propeller.from_csv(csv_path, "PER3_12x6EP-3.dat")

    assert propeller.name == "PER3_12x6EP-3.dat"
    assert propeller.Blades == 3
    assert propeller.Torque[14000] == 1.1
    assert propeller.TorqueThrustRatio[14000] == pytest.approx(1.1 / 5.0)
    assert propeller.Power[14000] == 3.5


def test_SIZE_PROP_CLS_UT_11(tmp_path):
    # Test Propeller.from_csv raises an error for an unknown propeller name
    csv_path = _write_propeller_csv(tmp_path, _build_multi_propeller_dataframe())

    with pytest.raises(ValueError, match="Could not find propeller in CSV"):
        Propeller.from_csv(csv_path, "PER3_missing.dat")


def test_SIZE_PROP_CLS_UT_12(tmp_path):
    # Test load_propeller_dict groups rows into Propeller objects keyed by filename
    csv_path = _write_propeller_csv(tmp_path, _build_multi_propeller_dataframe())

    propeller_dict = load_propeller_dict(csv_path)

    assert list(propeller_dict) == ["PER3_10x4E.dat", "PER3_12x6EP-3.dat"]
    assert isinstance(propeller_dict["PER3_10x4E.dat"], Propeller)
    assert propeller_dict["PER3_10x4E.dat"].Torque[12000] == 0.7
    assert propeller_dict["PER3_10x4E.dat"].TorqueThrustRatio[12000] == pytest.approx(0.7 / 3.0)
    assert propeller_dict["PER3_10x4E.dat"].Thrust[12000] == 3.0
    assert propeller_dict["PER3_12x6EP-3.dat"].Power[12000] == 2.5


def test_SIZE_PROP_CLS_MT_01(tmp_path):
    # Test the full CSV-to-object workflow with dictionary loading and RPM snapping
    csv_path = _write_propeller_csv(tmp_path, _build_multi_propeller_dataframe())

    propeller_dict = load_propeller_dict(csv_path)
    selected_propeller = Propeller.from_csv(csv_path, "PER3_10x4E.dat")

    assert set(propeller_dict) == {"PER3_10x4E.dat", "PER3_12x6EP-3.dat"}
    assert selected_propeller.name == "PER3_10x4E.dat"
    assert selected_propeller.Power[11000] == 1.5
    assert selected_propeller.Torque[11000] == 0.6
    assert selected_propeller.TorqueThrustRatio[11000] == pytest.approx(0.6 / 2.5)
    assert propeller_dict["PER3_12x6EP-3.dat"].Thrust[13000] == 5.0


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
