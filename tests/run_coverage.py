import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
import tempfile

from coverage import Coverage

import test_main

REPO_ROOT = Path(__file__).resolve().parent.parent


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Run the repository test suite with coverage reporting."
    )
    parser.add_argument(
        "--fast",
        action="store_true",
        help="Skip the slow system test test_SIZE_ST_04.",
    )
    parser.add_argument(
        "--html",
        action="store_true",
        help="Generate an HTML report in htmlcov/.",
    )
    parser.add_argument(
        "--xml",
        action="store_true",
        help="Generate a coverage.xml report.",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=None,
        help="Fail if total coverage drops below this percentage.",
    )
    parser.add_argument(
        "pytest_args",
        nargs=argparse.REMAINDER,
        help="Optional pytest arguments after '--'.",
    )
    args = parser.parse_args()
    if args.pytest_args and args.pytest_args[0] == "--":
        args.pytest_args = args.pytest_args[1:]
    return args


def _coverage_group(relative_path):
    path = Path(relative_path)
    parts = path.parts

    if path.name == "Sizing_tool.py":
        return "Sizing_tool"
    if len(parts) >= 2 and parts[0] == "structures" and parts[1] == "bending":
        return "structures.bending"
    if parts:
        return parts[0]
    return str(path)


def _build_group_summary(report_data):
    grouped = defaultdict(
        lambda: {
            "covered_lines": 0,
            "num_statements": 0,
            "covered_branches": 0,
            "num_branches": 0,
        }
    )

    for file_name, file_data in report_data["files"].items():
        group = _coverage_group(file_name)
        summary = file_data["summary"]
        grouped[group]["covered_lines"] += summary["covered_lines"]
        grouped[group]["num_statements"] += summary["num_statements"]
        grouped[group]["covered_branches"] += summary["covered_branches"]
        grouped[group]["num_branches"] += summary["num_branches"]

    return grouped


def _coverage_percent(summary):
    covered = summary["covered_lines"] + summary["covered_branches"]
    total = summary["num_statements"] + summary["num_branches"]
    if total == 0:
        return 100.0
    return 100.0 * covered / total


def _print_group_summary(report_data):
    grouped = _build_group_summary(report_data)
    print("\nCoverage by module:")
    for group_name in sorted(grouped):
        summary = grouped[group_name]
        percent = _coverage_percent(summary)
        print(
            f"- {group_name}: "
            f"statements={summary['num_statements']}, "
            f"branches={summary['num_branches']}, "
            f"coverage={percent:.2f}%"
        )

    totals = report_data["totals"]
    print(
        "\nTotal coverage: "
        f"{totals['percent_covered']:.2f}% "
        f"(statements={totals['num_statements']}, branches={totals['num_branches']})"
    )


def main():
    args = _parse_args()
    os.environ.setdefault("MPLBACKEND", "Agg")

    coverage = Coverage(config_file=str(REPO_ROOT / ".coveragerc"))
    coverage.erase()
    coverage.start()

    try:
        test_exit_code = test_main.main(fast=args.fast, pytest_args=args.pytest_args)
    finally:
        coverage.stop()
        coverage.save()

    coverage.report(sort="Cover")

    if args.html:
        coverage.html_report()
        print("HTML coverage report written to htmlcov/index.html")

    if args.xml:
        coverage.xml_report()
        print("XML coverage report written to coverage.xml")

    with tempfile.NamedTemporaryFile(
        mode="w+",
        suffix=".json",
        delete=False,
        encoding="utf-8",
    ) as temp_report:
        temp_report_path = Path(temp_report.name)

    try:
        coverage.json_report(outfile=str(temp_report_path))
        report_data = json.loads(temp_report_path.read_text(encoding="utf-8"))
    finally:
        temp_report_path.unlink(missing_ok=True)

    _print_group_summary(report_data)

    if args.threshold is not None:
        total_coverage = report_data["totals"]["percent_covered"]
        if total_coverage < args.threshold:
            print(
                f"Coverage threshold not met: "
                f"{total_coverage:.2f}% < {args.threshold:.2f}%"
            )
            return 1 if test_exit_code == 0 else test_exit_code

    return test_exit_code


if __name__ == "__main__":
    raise SystemExit(main())
