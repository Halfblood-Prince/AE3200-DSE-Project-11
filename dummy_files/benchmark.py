import argparse
import json
import subprocess
import sys
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import a as astar_demo
import cpp_astar


def _path_from_json(raw_path):
    if raw_path is None:
        return None

    return [tuple(int(value) for value in position) for position in raw_path]


def benchmark_python(iterations):
    path = None
    started = time.perf_counter_ns()

    for _ in range(iterations):
        path = astar_demo.astar(astar_demo.grid, astar_demo.start, astar_demo.goal)

    duration_ns = time.perf_counter_ns() - started

    return {
        "implementation": "python",
        "iterations": iterations,
        "duration_ns": duration_ns,
        "path": path,
    }


def _benchmark_cpp_library(iterations, cpp_library=None):
    solver = cpp_astar.CppAStarSolver(astar_demo.grid, cpp_library=cpp_library)
    path = None
    started = time.perf_counter_ns()

    for _ in range(iterations):
        path = solver.astar(astar_demo.start, astar_demo.goal)

    duration_ns = time.perf_counter_ns() - started

    return {
        "implementation": "c++-dll",
        "iterations": iterations,
        "duration_ns": duration_ns,
        "path": path,
        "binary": str(solver.library_path),
    }


def _benchmark_cpp_executable(iterations, cpp_binary=None):
    executable = cpp_astar.find_cpp_binary(cpp_binary)
    completed = subprocess.run(
        [str(executable), "--benchmark", str(iterations)],
        check=True,
        capture_output=True,
        text=True,
    )
    result = json.loads(completed.stdout)

    return {
        "implementation": "c++-exe",
        "iterations": int(result["iterations"]),
        "duration_ns": int(result["duration_ns"]),
        "path": _path_from_json(result["result"]["path"]),
        "binary": str(executable),
    }


def benchmark_cpp(iterations, cpp_library=None, cpp_binary=None):
    try:
        return _benchmark_cpp_library(iterations, cpp_library=cpp_library)
    except (FileNotFoundError, OSError) as library_error:
        try:
            return _benchmark_cpp_executable(iterations, cpp_binary=cpp_binary)
        except (FileNotFoundError, OSError) as binary_error:
            raise FileNotFoundError(
                f"{library_error}\n\nExecutable benchmark fallback also unavailable:\n{binary_error}"
            ) from binary_error


def benchmark_astar(iterations=1_000, cpp_library=None, cpp_binary=None):
    python_result = benchmark_python(iterations)
    results = [python_result]
    errors = {}

    try:
        results.append(
            benchmark_cpp(
                iterations,
                cpp_library=cpp_library,
                cpp_binary=cpp_binary,
            )
        )
    except (FileNotFoundError, OSError) as error:
        errors["C++"] = str(error)

    return results, errors


def print_benchmark_results(results, errors=None):
    errors = errors or {}
    print("Benchmark results:")

    for result in results:
        total_ms = result["duration_ns"] / 1_000_000
        average_us = result["duration_ns"] / result["iterations"] / 1_000
        print(
            f"{result['implementation']:>8}: "
            f"{result['iterations']} runs, "
            f"{total_ms:.3f} ms total, "
            f"{average_us:.3f} us/run"
        )

    python_result = next(
        (result for result in results if result["implementation"] == "python"),
        None,
    )

    if python_result:
        for result in results:
            if result["implementation"] == "python":
                continue

            if python_result["path"] != result["path"]:
                print(f"\nWarning: {result['implementation']} path does not match Python path.")

            if result["duration_ns"] > 0:
                speed_ratio = python_result["duration_ns"] / result["duration_ns"]
                print(f"\n{result['implementation']}/Python speed ratio: {speed_ratio:.2f}x")

    for name, error in errors.items():
        print(f"\n{name} benchmark skipped:")
        print(error)


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Benchmark 3D A* pathfinding.")
    parser.add_argument(
        "--iterations",
        type=int,
        default=1_000,
        help="Number of benchmark iterations.",
    )
    parser.add_argument(
        "--cpp-lib",
        type=Path,
        default=None,
        help="Path to the compiled C++ DLL/shared library.",
    )
    parser.add_argument(
        "--cpp-bin",
        type=Path,
        default=None,
        help="Path to the compiled C++ executable fallback.",
    )

    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.iterations < 1:
        parser.error("--iterations must be at least 1")

    results, errors = benchmark_astar(
        iterations=args.iterations,
        cpp_library=args.cpp_lib,
        cpp_binary=args.cpp_bin,
    )
    print_benchmark_results(results, errors=errors)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, OSError, subprocess.CalledProcessError) as error:
        print(error, file=sys.stderr)
        raise SystemExit(1)
