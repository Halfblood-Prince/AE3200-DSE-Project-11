import pathlib
import sys

import pandas as pd
import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

import propulsion.propellersData2CSV as propellers_data


def _write_metadata_files(tmp_path, titledat_text, rpmrange_text):
    titledat_path = tmp_path / "PER2_TITLEDAT.DAT"
    rpmrange_path = tmp_path / "PER2_RPMRANGE.DAT"
    titledat_path.write_text(titledat_text, encoding="utf-8")
    rpmrange_path.write_text(rpmrange_text, encoding="utf-8")
    return titledat_path, rpmrange_path


def _write_static_performance_file(root_path, filename, rpm_rows):
    performance_dir = root_path / "airfoilsdat" / "PERFILES2"
    performance_dir.mkdir(parents=True, exist_ok=True)
    file_path = performance_dir / filename

    lines = []
    for rpm_row in rpm_rows:
        if len(rpm_row) == 3:
            rpm, power, thrust = rpm_row
            torque = 0.0
        else:
            rpm, power, torque, thrust = rpm_row
        lines.extend(
            [
                f"PROP RPM = {rpm}",
                "header line to skip",
                f"0.00 0 0 0 0 0 0 0 {power:.2f} {torque:.2f} {thrust:.2f}",
            ]
        )

    file_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return file_path


def test_SIZE_PROP_D2C_UT_01(tmp_path):
    # Test load_titledat function with a simple valid input
    source_path = tmp_path / "PER2_TITLEDAT.DAT"
    source_path.write_text(
        "PER3_10x4.dat 10x4E\n"
        "PER3_12x6.DAT    12x6 sport prop\n",
        encoding="utf-8",
    )

    result = propellers_data.load_titledat(source_path)

    expected = pd.DataFrame(
        [
            {"file name": "PER3_10x4.dat", "title": "10x4E"},
            {"file name": "PER3_12x6.DAT", "title": "12x6 sport prop"},
        ],
        columns=["file name", "title"],
    )

    pd.testing.assert_frame_equal(result, expected)


def test_SIZE_PROP_D2C_UT_02(tmp_path):
    # Test load_titledat function with invalid lines and extra whitespace
    source_path = tmp_path / "PER2_TITLEDAT.DAT"
    source_path.write_text(
        "\n"
        "NOT_A_DAT_FILE title that should be skipped\n"
        "invalid_line_without_title\n"
        "PER3_9x4.dat      9x4 valid title\n"
        "PER3_11x5.txt     wrong extension\n"
        "   PER3_13x6.dat   another valid title   \n",
        encoding="utf-8",
    )

    result = propellers_data.load_titledat(source_path)

    expected = pd.DataFrame(
        [
            {"file name": "PER3_9x4.dat", "title": "9x4 valid title"},
            {"file name": "PER3_13x6.dat", "title": "another valid title"},
        ],
        columns=["file name", "title"],
    )

    pd.testing.assert_frame_equal(result, expected)


def test_SIZE_PROP_D2C_UT_03(tmp_path):
    # Test load_rpm_range function with a simple valid input
    source_path = tmp_path / "PER2_RPMRANGE.DAT"
    source_path.write_text(
        "10x4e 5000 12000\n"
        "12x6sport 7000 15000\n",
        encoding="utf-8",
    )

    result = propellers_data.load_rpm_range(source_path)

    expected = pd.DataFrame(
        [
            {"file name": "10x4e", "RPM min": 5000, "RPM max": 12000},
            {"file name": "12x6sport", "RPM min": 7000, "RPM max": 15000},
        ],
        columns=["file name", "RPM min", "RPM max"],
    )

    pd.testing.assert_frame_equal(result, expected)


def test_SIZE_PROP_D2C_UT_04(tmp_path):
    # Test load_rpm_range function with invalid lines and extra whitespace
    source_path = tmp_path / "PER2_RPMRANGE.DAT"
    source_path.write_text(
        "\n"
        "10x4e 5000 12000\n"
        "invalid line that should be skipped\n"
        "11x5e 6500 not_a_number\n"
        "   12x6sport    7000    15000   \n"
        "13x6e 8000\n",
        encoding="utf-8",
    )

    result = propellers_data.load_rpm_range(source_path)

    expected = pd.DataFrame(
        [
            {"file name": "10x4e", "RPM min": 5000, "RPM max": 12000},
            {"file name": "12x6sport", "RPM min": 7000, "RPM max": 15000},
        ],
        columns=["file name", "RPM min", "RPM max"],
    )

    pd.testing.assert_frame_equal(result, expected)


def test_SIZE_PROP_D2C_UT_05(tmp_path):
    # Test load_propeller_data function with matching title and RPM files
    _, rpmrange_path = _write_metadata_files(
        tmp_path,
        "PER3_12x6ep-3.dat 12x6EP-3\nPER3_10x4.dat 10x4\n",
        "12x6ep-3 7000 15000\n10x4 5000 12000\n",
    )

    result = propellers_data.load_propeller_data(rpmrange_path)

    expected = pd.DataFrame(
        [
            {"filename": "PER3_12x6ep-3.dat", "title": "12x6EP-3", "RPM min": 7000, "RPM max": 15000},
            {"filename": "PER3_10x4.dat", "title": "10x4", "RPM min": 5000, "RPM max": 12000},
        ],
        columns=["filename", "title", "RPM min", "RPM max"],
    )

    pd.testing.assert_frame_equal(result, expected)


def test_SIZE_PROP_D2C_UT_06(tmp_path):
    # Test load_propeller_data function when title and RPM files do not match
    _, rpmrange_path = _write_metadata_files(
        tmp_path,
        "PER3_10x4.dat 10x4\nPER3_12x6.dat 12x6E\n",
        "10x4 5000 12000\n",
    )

    with pytest.raises(ValueError, match="do not contain the same propeller set"):
        propellers_data.load_propeller_data(rpmrange_path)


def test_SIZE_PROP_D2C_UT_07(tmp_path):
    # Test extract_propeller_specs function with blade and extension parsing
    _, rpmrange_path = _write_metadata_files(
        tmp_path,
        "PER3_12x6ep-3.dat 12x6EP-3\nPER3_10x4.dat 10x4\n",
        "12x6ep-3 7000 15000\n10x4 5000 12000\n",
    )

    result = propellers_data.extract_propeller_specs(rpmrange_path)

    expected = pd.DataFrame(
        [
            {
                "filename": "PER3_10x4.dat",
                "Diameter": 10,
                "Pitch": 4,
                "RPM min": 5000,
                "RPM max": 12000,
                "Blades": 2,
                "Extension": "[-]",
            },
            {
                "filename": "PER3_12x6ep-3.dat",
                "Diameter": 12,
                "Pitch": 6,
                "RPM min": 7000,
                "RPM max": 15000,
                "Blades": 3,
                "Extension": "EP",
            },
        ],
        columns=["filename", "Diameter", "Pitch", "RPM min", "RPM max", "Blades", "Extension"],
    )
    pd.testing.assert_frame_equal(result, expected)


def test_SIZE_PROP_D2C_UT_08(tmp_path):
    # Test extract_propeller_specs function with an invalid title format
    _, rpmrange_path = _write_metadata_files(
        tmp_path,
        "PER3_bad.dat invalid_title\n",
        "bad 5000 12000\n",
    )

    with pytest.raises(ValueError, match="Could not parse propeller title"):
        propellers_data.extract_propeller_specs(rpmrange_path)


def test_SIZE_PROP_D2C_UT_09():
    # Test filter_propellers function with multiple filters applied together
    propeller_df = pd.DataFrame(
        [
            {"filename": "PER3_10x4.dat", "Diameter": 10.0, "Pitch": 4.0, "RPM min": 5000, "RPM max": 12000, "Blades": 2, "Extension": "E"},
            {"filename": "PER3_11x6.dat", "Diameter": 11.0, "Pitch": 6.0, "RPM min": 6000, "RPM max": 13000, "Blades": 3, "Extension": "EP"},
            {"filename": "PER3_9x4.dat", "Diameter": 9.0, "Pitch": 4.0, "RPM min": 5500, "RPM max": 11000, "Blades": 2, "Extension": "[-]"},
        ]
    )

    result = propellers_data.filter_propellers(
        propeller_df,
        max_diameter=10.0,
        max_pitch=4.0,
        extensions=[" e "],
        blades=[2, 3],
    )

    expected = pd.DataFrame(
        [
            {"filename": "PER3_10x4.dat", "Diameter": 10.0, "Pitch": 4.0, "RPM min": 5000, "RPM max": 12000, "Blades": 2, "Extension": "E"}
        ]
    ).reset_index(drop=True)

    pd.testing.assert_frame_equal(result, expected)


def test_SIZE_PROP_D2C_UT_10():
    # Test filter_propellers function with missing required columns
    propeller_df = pd.DataFrame([{"filename": "PER3_10x4.dat", "Diameter": 10.0}])

    with pytest.raises(KeyError, match="Missing required columns"):
        propellers_data.filter_propellers(propeller_df, max_diameter=10.0)


def test_SIZE_PROP_D2C_UT_11(tmp_path, monkeypatch):
    # Test load_static_performance function with matching zero-velocity RPM rows
    monkeypatch.setattr(propellers_data, "PROJECT_ROOT", tmp_path)
    _write_static_performance_file(
        tmp_path,
        "PER3_10x4.dat",
        [(10000, 1.50, 0.60, 2.50), (12000, 2.00, 0.70, 3.00), (14000, 2.50, 0.80, 3.50)],
    )

    propeller_df = pd.DataFrame([{"filename": "PER3_10x4.dat", "RPM max": 14000}])
    result = propellers_data.load_static_performance(
        "PER3_10x4.dat",
        propeller_df=propeller_df,
        rpm_start=10000,
        rpm_step=2000,
        strict=True,
    )

    expected = pd.DataFrame(
        [
            {"filename": "PER3_10x4.dat", "RPM": 10000, "Power": 1.50, "Torque": 0.60, "Thrust": 2.50},
            {"filename": "PER3_10x4.dat", "RPM": 12000, "Power": 2.00, "Torque": 0.70, "Thrust": 3.00},
            {"filename": "PER3_10x4.dat", "RPM": 14000, "Power": 2.50, "Torque": 0.80, "Thrust": 3.50},
        ],
        columns=["filename", "RPM", "Power", "Torque", "Thrust"],
    )

    pd.testing.assert_frame_equal(result, expected)


def test_SIZE_PROP_D2C_UT_12(tmp_path, monkeypatch):
    # Test load_static_performance function when a target RPM value is missing
    monkeypatch.setattr(propellers_data, "PROJECT_ROOT", tmp_path)
    _write_static_performance_file(
        tmp_path,
        "PER3_10x4.dat",
        [(10000, 1.50, 0.60, 2.50), (14000, 2.50, 0.80, 3.50)],
    )

    propeller_df = pd.DataFrame([{"filename": "PER3_10x4.dat", "RPM max": 14000}])

    with pytest.raises(ValueError, match="12000"):
        propellers_data.load_static_performance(
            "PER3_10x4.dat",
            propeller_df=propeller_df,
            rpm_start=10000,
            rpm_step=2000,
            strict=True,
        )


def test_SIZE_PROP_D2C_UT_13(tmp_path, monkeypatch):
    # Test build_static_propeller_table function with filtered propellers and dummy performance files
    monkeypatch.setattr(propellers_data, "PROJECT_ROOT", tmp_path)
    _write_static_performance_file(
        tmp_path,
        "PER3_10x4.dat",
        [(10000, 1.50, 0.60, 2.50), (12000, 2.00, 0.70, 3.00)],
    )
    _write_static_performance_file(
        tmp_path,
        "PER3_12x6ep-3.dat",
        [(10000, 2.50, 0.90, 4.00), (12000, 3.00, 1.00, 4.50), (14000, 3.50, 1.10, 5.00)],
    )

    propeller_df = pd.DataFrame(
        [
            {"filename": "PER3_12x6ep-3.dat", "Diameter": 12.0, "Pitch": 6.0, "RPM min": 7000, "RPM max": 14000, "Blades": 3, "Extension": "EP"},
            {"filename": "PER3_10x4.dat", "Diameter": 10.0, "Pitch": 4.0, "RPM min": 5000, "RPM max": 12000, "Blades": 2, "Extension": "E"},
        ]
    )

    result = propellers_data.build_static_propeller_table(
        propeller_df,
        max_diameter=12.0,
        extensions=["E", "EP"],
        max_pitch=6.0,
        blades=[2, 3],
        rpm_start=10000,
        rpm_step=2000,
        strict_static_data=True,
    )

    expected = pd.DataFrame(
        [
            {
                "filename": "PER3_10x4.dat",
                "Diameter": 10.0,
                "Pitch": 4.0,
                "RPM min": 5000,
                "RPM max": 12000,
                "Blades": 2,
                "Extension": "E",
                "RPM": 10000,
                "Power": 1.50,
                "Torque": 0.60,
                "Thrust": 2.50,
            },
            {
                "filename": "PER3_10x4.dat",
                "Diameter": 10.0,
                "Pitch": 4.0,
                "RPM min": 5000,
                "RPM max": 12000,
                "Blades": 2,
                "Extension": "E",
                "RPM": 12000,
                "Power": 2.00,
                "Torque": 0.70,
                "Thrust": 3.00,
            },
            {
                "filename": "PER3_12x6ep-3.dat",
                "Diameter": 12.0,
                "Pitch": 6.0,
                "RPM min": 7000,
                "RPM max": 14000,
                "Blades": 3,
                "Extension": "EP",
                "RPM": 10000,
                "Power": 2.50,
                "Torque": 0.90,
                "Thrust": 4.00,
            },
            {
                "filename": "PER3_12x6ep-3.dat",
                "Diameter": 12.0,
                "Pitch": 6.0,
                "RPM min": 7000,
                "RPM max": 14000,
                "Blades": 3,
                "Extension": "EP",
                "RPM": 12000,
                "Power": 3.00,
                "Torque": 1.00,
                "Thrust": 4.50,
            },
            {
                "filename": "PER3_12x6ep-3.dat",
                "Diameter": 12.0,
                "Pitch": 6.0,
                "RPM min": 7000,
                "RPM max": 14000,
                "Blades": 3,
                "Extension": "EP",
                "RPM": 14000,
                "Power": 3.50,
                "Torque": 1.10,
                "Thrust": 5.00,
            },
        ],
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
        ],
    )

    pd.testing.assert_frame_equal(result, expected)


def test_SIZE_PROP_D2C_UT_14():
    # Test build_static_propeller_table function when filters remove every propeller
    propeller_df = pd.DataFrame(
        [
            {"filename": "PER3_10x4.dat", "Diameter": 10.0, "Pitch": 4.0, "RPM min": 5000, "RPM max": 12000, "Blades": 2, "Extension": "E"}
        ]
    )

    result = propellers_data.build_static_propeller_table(
        propeller_df,
        max_diameter=8.0,
        rpm_start=10000,
        rpm_step=2000,
    )

    expected = pd.DataFrame(
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

    pd.testing.assert_frame_equal(result, expected)


def test_SIZE_PROP_D2C_MT_01(tmp_path, monkeypatch):
    # Test the end-to-end metadata and static-performance workflow for one filtered propeller
    monkeypatch.setattr(propellers_data, "PROJECT_ROOT", tmp_path)
    _, rpmrange_path = _write_metadata_files(
        tmp_path,
        "PER3_10x4e.dat 10x4E\nPER3_12x6ep-3.dat 12x6EP-3\n",
        "10x4e 5000 12000\n12x6ep-3 7000 14000\n",
    )
    _write_static_performance_file(
        tmp_path,
        "PER3_10x4e.dat",
        [(10000, 1.50, 0.60, 2.50), (12000, 2.00, 0.70, 3.00)],
    )
    _write_static_performance_file(
        tmp_path,
        "PER3_12x6ep-3.dat",
        [(10000, 2.50, 0.90, 4.00), (12000, 3.00, 1.00, 4.50), (14000, 3.50, 1.10, 5.00)],
    )

    propeller_df = propellers_data.extract_propeller_specs(rpmrange_path)
    result = propellers_data.build_static_propeller_table(
        propeller_df,
        max_diameter=10.0,
        extensions=["E"],
        max_pitch=4.0,
        blades=2,
        rpm_start=10000,
        rpm_step=2000,
        strict_static_data=True,
    )

    expected = pd.DataFrame(
        [
            {
                "filename": "PER3_10x4e.dat",
                "Diameter": 10,
                "Pitch": 4,
                "RPM min": 5000,
                "RPM max": 12000,
                "Blades": 2,
                "Extension": "E",
                "RPM": 10000,
                "Power": 1.50,
                "Torque": 0.60,
                "Thrust": 2.50,
            },
            {
                "filename": "PER3_10x4e.dat",
                "Diameter": 10,
                "Pitch": 4,
                "RPM min": 5000,
                "RPM max": 12000,
                "Blades": 2,
                "Extension": "E",
                "RPM": 12000,
                "Power": 2.00,
                "Torque": 0.70,
                "Thrust": 3.00,
            },
        ],
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
        ],
    )

    pd.testing.assert_frame_equal(result, expected)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
