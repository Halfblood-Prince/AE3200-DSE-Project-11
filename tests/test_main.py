import pathlib
import shutil
import sys
import tempfile
import uuid

import pytest

FAST = False

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
TESTS_ROOT = pathlib.Path(__file__).resolve().parent
BENDING_TESTS_ROOT = REPO_ROOT / "structures" / "bending" / "tests"
BENDING_MODULE_ROOT = REPO_ROOT / "structures" / "bending"
THIS_FILE = pathlib.Path(__file__).resolve()
RUN_ID = uuid.uuid4().hex
TMP_ROOT = pathlib.Path(tempfile.gettempdir()) / "codex_pytest_tmp"


class TestRunCollector:
    def __init__(self):
        self.test_statuses = {}
        self.collection_failures = 0
        self.deselected = 0

    def pytest_runtest_logreport(self, report):
        if report.failed:
            outcome = "failed"
        elif report.skipped:
            outcome = "skipped"
        elif report.passed and report.when == "call":
            outcome = "passed"
        else:
            return

        current_outcome = self.test_statuses.get(report.nodeid)
        if current_outcome == "failed":
            return
        if current_outcome == "skipped" and outcome == "passed":
            return

        self.test_statuses[report.nodeid] = outcome

    def build_summary(self):
        total = len(self.test_statuses) + self.collection_failures + self.deselected
        passed = sum(outcome == "passed" for outcome in self.test_statuses.values())
        failed = (
            sum(outcome == "failed" for outcome in self.test_statuses.values())
            + self.collection_failures
        )
        skipped = (
            sum(outcome == "skipped" for outcome in self.test_statuses.values())
            + self.deselected
        )
        return {
            "total": total,
            "passed": passed,
            "failed": failed,
            "skipped": skipped,
        }

    def pytest_collectreport(self, report):
        if report.failed:
            self.collection_failures += 1

    def pytest_deselected(self, items):
        self.deselected += len(items)


def _display_path(path):
    return str(pathlib.Path(path).resolve().relative_to(REPO_ROOT))


def _is_test_script(path):
    if path.suffix != ".py":
        return False
    if path.resolve() == THIS_FILE:
        return False

    stem = path.stem
    return stem.startswith("test") or stem.endswith("_tests")


def _discover_test_files():
    discovered = set()
    for root in (TESTS_ROOT, BENDING_TESTS_ROOT):
        if not root.exists():
            continue
        for path in root.rglob("*.py"):
            if "__pycache__" in path.parts:
                continue
            if _is_test_script(path):
                discovered.add(str(path.resolve()))

    return sorted(discovered)


def _build_pytest_args(user_args=None):
    if user_args is None:
        user_args = sys.argv[1:]
    user_args = list(user_args)
    output_flags = {"-q", "-qq", "-v", "-vv", "--quiet", "--verbose"}
    if not any(flag in user_args for flag in output_flags):
        user_args.insert(0, "-q")
    return user_args


def _build_basetemp(test_file):
    display_path = _display_path(test_file)
    safe_name = display_path.replace("\\", "__").replace("/", "__").replace(" ", "_")
    return TMP_ROOT / RUN_ID / safe_name


def _is_bending_test(test_file):
    return BENDING_TESTS_ROOT in pathlib.Path(test_file).resolve().parents


def _is_system_test_file(test_file):
    return pathlib.Path(test_file).resolve() == (
        REPO_ROOT / "tests" / "System tests" / "System_tests.py"
    ).resolve()


def main(fast=FAST, pytest_args=None):
    test_files = _discover_test_files()
    if not test_files:
        print("No test files were found.")
        return 1

    shared_args = _build_pytest_args(pytest_args)
    exit_code = 0
    file_summaries = []
    total_tests = 0
    total_passed = 0
    total_failed = 0
    total_skipped = 0

    try:
        for test_file in test_files:
            display_path = _display_path(test_file)
            print(f"Running test script: {display_path}")
            pytest_args = [test_file, *shared_args]
            if fast and _is_system_test_file(test_file):
                pytest_args.extend(["-k", "not test_SIZE_ST_04"])
            if "--basetemp" not in shared_args:
                basetemp = _build_basetemp(test_file)
                basetemp.parent.mkdir(parents=True, exist_ok=True)
                pytest_args.extend(["--basetemp", str(basetemp)])

            collector = TestRunCollector()
            added_bending_path = False
            if _is_bending_test(test_file):
                bending_path = str(BENDING_MODULE_ROOT)
                if bending_path not in sys.path:
                    sys.path.insert(0, bending_path)
                    added_bending_path = True

            try:
                result = pytest.main(pytest_args, plugins=[collector])
            finally:
                if added_bending_path:
                    sys.path.remove(str(BENDING_MODULE_ROOT))
            summary = collector.build_summary()

            file_summaries.append((display_path, summary))
            total_tests += summary["total"]
            total_passed += summary["passed"]
            total_failed += summary["failed"]
            total_skipped += summary["skipped"]

            summary_line = (
                f"Summary for {display_path}: "
                f"total={summary['total']}, "
                f"passed={summary['passed']}, "
                f"failed={summary['failed']}, "
                f"skipped={summary['skipped']}"
            )
            print(summary_line)

            if result != 0 and exit_code == 0:
                exit_code = int(result)

        print("\nRun summary:")
        for file_run, summary in file_summaries:
            line = (
                f"- {file_run}: "
                f"total={summary['total']}, "
                f"passed={summary['passed']}, "
                f"failed={summary['failed']}, "
                f"skipped={summary['skipped']}"
            )
            print(line)
        print(f"Total tests: {total_tests}")
        print(f"Tests passed: {total_passed}")
        print(f"Tests failed: {total_failed}")
        print(f"Tests skipped: {total_skipped}")
    finally:
        if TMP_ROOT.exists():
            shutil.rmtree(TMP_ROOT, ignore_errors=True)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
