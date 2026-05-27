import argparse
import ctypes
import json
import os
import subprocess
import sys
import time
from functools import lru_cache
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent

# 0 = free space
# 1 = obstacle
grid = np.array(
    [
        # Layer 0
        [
            [0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 0, 1, 0],
            [0, 0, 0, 1, 0, 1, 0],
            [1, 1, 0, 0, 0, 1, 0],
            [0, 0, 0, 1, 0, 0, 0],
            [0, 1, 0, 1, 1, 1, 0],
            [0, 1, 0, 0, 0, 0, 0],
        ],
        # Layer 1
        [
            [0, 0, 0, 1, 0, 0, 0],
            [0, 1, 0, 1, 0, 1, 0],
            [0, 1, 0, 0, 0, 1, 0],
            [0, 1, 1, 1, 0, 0, 0],
            [0, 0, 0, 1, 1, 1, 0],
            [1, 1, 0, 0, 0, 1, 0],
            [0, 0, 0, 1, 0, 0, 0],
        ],
        # Layer 2
        [
            [0, 0, 0, 0, 0, 1, 0],
            [1, 1, 1, 1, 0, 1, 0],
            [0, 0, 0, 1, 0, 0, 0],
            [0, 1, 0, 1, 1, 1, 0],
            [0, 1, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0],
        ],
    ],
    dtype=np.uint8,
)

# Position format: (layer, row, column)
start = (0, 0, 0)
goal = (2, 6, 6)


def g(g_score, current):
    """
    Actual cost from start to current node.
    """
    return g_score[current]


def h(current, goal):
    """
    Estimated cost from current node to goal.
    3D Manhattan distance is used because movement is only in 6 directions.
    """
    return (
        abs(goal[0] - current[0])
        + abs(goal[1] - current[1])
        + abs(goal[2] - current[2])
    )


def f(g_score, current, goal):
    """
    Total A* score.
    f(n) = g(n) + h(n)
    """
    return g(g_score, current) + h(current, goal)


def get_neighbors(grid, current):
    layer, row, col = current

    moves = [
        (0, -1, 0),  # row up
        (0, 1, 0),  # row down
        (0, 0, -1),  # column left
        (0, 0, 1),  # column right
        (1, 0, 0),  # one layer up
        (-1, 0, 0),  # one layer down
    ]

    neighbors = []

    for d_layer, d_row, d_col in moves:
        new_layer = layer + d_layer
        new_row = row + d_row
        new_col = col + d_col

        inside_grid = (
            0 <= new_layer < grid.shape[0]
            and 0 <= new_row < grid.shape[1]
            and 0 <= new_col < grid.shape[2]
        )

        if inside_grid and grid[new_layer, new_row, new_col] == 0:
            neighbors.append((new_layer, new_row, new_col))

    return neighbors


def reconstruct_path(came_from, current):
    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path


def astar(grid, start, goal):
    open_set = [start]
    closed_set = set()

    came_from = {}

    g_score = {}
    g_score[start] = 0

    while open_set:
        # Choose the node with the lowest f score.
        current = min(open_set, key=lambda node: f(g_score, node, goal))

        if current == goal:
            return reconstruct_path(came_from, current)

        open_set.remove(current)
        closed_set.add(current)

        for neighbor in get_neighbors(grid, current):
            if neighbor in closed_set:
                continue

            new_g = g_score[current] + 1

            if neighbor not in g_score or new_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = new_g

                if neighbor not in open_set:
                    open_set.append(neighbor)

    return None


def _native_search_dirs():
    return [
        SCRIPT_DIR,
        REPO_ROOT / "dist" / "windows-amd64",
        REPO_ROOT / "dist" / "windows_amd_x64",
        REPO_ROOT / "bin" / "windows-amd64",
        REPO_ROOT / "bin" / "windows_amd_x64",
    ]


def _rust_library_names():
    if os.name == "nt":
        return ["a_star_rust.dll"]

    if sys.platform == "darwin":
        return ["liba_star_rust.dylib", "a_star_rust.dylib"]

    return ["liba_star_rust.so", "a_star_rust.so"]


def _cpp_library_names():
    if os.name == "nt":
        return ["a_star_cpp.dll"]

    if sys.platform == "darwin":
        return ["liba_star_cpp.dylib", "a_star_cpp.dylib"]

    return ["liba_star_cpp.so", "a_star_cpp.so"]


def _rust_executable_names():
    if os.name == "nt":
        return ["a_star_rust.exe"]

    return ["a_star_rust"]


def _cpp_executable_names():
    if os.name == "nt":
        return ["a_star_cpp.exe"]

    return ["a_star_cpp"]


def _candidate_paths(explicit_path, env_var, names):
    paths = []

    if explicit_path is not None:
        paths.append(Path(explicit_path).expanduser())

    env_value = os.environ.get(env_var)
    if env_value:
        paths.append(Path(env_value).expanduser())

    for directory in _native_search_dirs():
        for name in names:
            paths.append(directory / name)

    return paths


def _find_existing_path(explicit_path, env_var, names, label):
    candidates = _candidate_paths(explicit_path, env_var, names)

    for path in candidates:
        if path.is_file():
            return path.resolve()

    searched = "\n".join(f"  - {path}" for path in candidates)
    raise FileNotFoundError(f"{label} not found. Searched:\n{searched}")


def find_rust_library(rust_library=None):
    return _find_existing_path(
        rust_library,
        "ASTAR_RUST_LIB",
        _rust_library_names(),
        "Rust A* library",
    )


def find_rust_binary(rust_binary=None):
    return _find_existing_path(
        rust_binary,
        "ASTAR_RUST_BIN",
        _rust_executable_names(),
        "Rust A* executable",
    )


def find_cpp_library(cpp_library=None):
    return _find_existing_path(
        cpp_library,
        "ASTAR_CPP_LIB",
        _cpp_library_names(),
        "C++ A* library",
    )


def find_cpp_binary(cpp_binary=None):
    return _find_existing_path(
        cpp_binary,
        "ASTAR_CPP_BIN",
        _cpp_executable_names(),
        "C++ A* executable",
    )


def _configure_native_astar_function(library, function_name):
    function = getattr(library, function_name)
    function.argtypes = [
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_size_t,
        ctypes.c_size_t,
        ctypes.c_size_t,
        ctypes.c_size_t,
        ctypes.c_size_t,
        ctypes.c_size_t,
        ctypes.c_size_t,
        ctypes.c_size_t,
        ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_size_t),
        ctypes.c_size_t,
    ]
    function.restype = ctypes.c_ssize_t

    return function


@lru_cache(maxsize=None)
def _load_rust_library(library_path_text):
    library_path = Path(library_path_text)

    if os.name == "nt" and hasattr(os, "add_dll_directory"):
        with os.add_dll_directory(str(library_path.parent)):
            library = ctypes.CDLL(str(library_path))
    else:
        library = ctypes.CDLL(str(library_path))

    _configure_native_astar_function(library, "astar_path")

    return library


@lru_cache(maxsize=None)
def _load_cpp_library(library_path_text):
    library_path = Path(library_path_text)

    if os.name == "nt" and hasattr(os, "add_dll_directory"):
        with os.add_dll_directory(str(library_path.parent)):
            library = ctypes.CDLL(str(library_path))
    else:
        library = ctypes.CDLL(str(library_path))

    _configure_native_astar_function(library, "cpp_astar_path")

    return library


def _position_tuple(position):
    if len(position) != 3:
        raise ValueError("Position must have exactly three values: layer, row, column.")

    return tuple(int(value) for value in position)


class RustAStarSolver:
    def __init__(self, grid_data, rust_library=None):
        self.library_path = find_rust_library(rust_library)
        self.library = _load_rust_library(str(self.library_path))
        self.grid = np.ascontiguousarray(grid_data, dtype=np.uint8)

        if self.grid.ndim != 3:
            raise ValueError("Rust A* expects a 3D grid.")

        if self.grid.size == 0:
            raise ValueError("Rust A* expects a non-empty grid.")

        self.layers, self.rows, self.cols = (int(value) for value in self.grid.shape)
        self.output_capacity = int(self.grid.size)
        self.output = (ctypes.c_size_t * (self.output_capacity * 3))()
        self.grid_pointer = self.grid.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))

    def astar(self, start_pos, goal_pos):
        start_layer, start_row, start_col = _position_tuple(start_pos)
        goal_layer, goal_row, goal_col = _position_tuple(goal_pos)

        count = self.library.astar_path(
            self.grid_pointer,
            self.layers,
            self.rows,
            self.cols,
            start_layer,
            start_row,
            start_col,
            goal_layer,
            goal_row,
            goal_col,
            self.output,
            self.output_capacity,
        )

        if count == -1:
            return None

        if count == -2:
            raise RuntimeError("Rust A* received invalid input pointers or dimensions.")

        if count == -3:
            raise RuntimeError("Rust A* output buffer was too small.")

        if count < 0:
            raise RuntimeError(f"Rust A* failed with error code {count}.")

        return [
            (
                int(self.output[index * 3]),
                int(self.output[index * 3 + 1]),
                int(self.output[index * 3 + 2]),
            )
            for index in range(count)
        ]


class CppAStarSolver:
    def __init__(self, grid_data, cpp_library=None):
        self.library_path = find_cpp_library(cpp_library)
        self.library = _load_cpp_library(str(self.library_path))
        self.grid = np.ascontiguousarray(grid_data, dtype=np.uint8)

        if self.grid.ndim != 3:
            raise ValueError("C++ A* expects a 3D grid.")

        if self.grid.size == 0:
            raise ValueError("C++ A* expects a non-empty grid.")

        self.layers, self.rows, self.cols = (int(value) for value in self.grid.shape)
        self.output_capacity = int(self.grid.size)
        self.output = (ctypes.c_size_t * (self.output_capacity * 3))()
        self.grid_pointer = self.grid.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))

    def astar(self, start_pos, goal_pos):
        start_layer, start_row, start_col = _position_tuple(start_pos)
        goal_layer, goal_row, goal_col = _position_tuple(goal_pos)

        count = self.library.cpp_astar_path(
            self.grid_pointer,
            self.layers,
            self.rows,
            self.cols,
            start_layer,
            start_row,
            start_col,
            goal_layer,
            goal_row,
            goal_col,
            self.output,
            self.output_capacity,
        )

        if count == -1:
            return None

        if count == -2:
            raise RuntimeError("C++ A* received invalid input pointers or dimensions.")

        if count == -3:
            raise RuntimeError("C++ A* output buffer was too small.")

        if count < 0:
            raise RuntimeError(f"C++ A* failed with error code {count}.")

        return [
            (
                int(self.output[index * 3]),
                int(self.output[index * 3 + 1]),
                int(self.output[index * 3 + 2]),
            )
            for index in range(count)
        ]


def _is_default_problem(grid_data, start_pos, goal_pos):
    return (
        np.array_equal(np.asarray(grid_data, dtype=np.uint8), grid)
        and _position_tuple(start_pos) == start
        and _position_tuple(goal_pos) == goal
    )


def _path_from_json(raw_path):
    if raw_path is None:
        return None

    return [tuple(int(value) for value in position) for position in raw_path]


def astar_rust_executable(rust_binary=None):
    executable = find_rust_binary(rust_binary)
    completed = subprocess.run(
        [str(executable), "--json"],
        check=True,
        capture_output=True,
        text=True,
    )
    result = json.loads(completed.stdout)

    return _path_from_json(result["path"])


def astar_cpp_executable(cpp_binary=None):
    executable = find_cpp_binary(cpp_binary)
    completed = subprocess.run(
        [str(executable), "--json"],
        check=True,
        capture_output=True,
        text=True,
    )
    result = json.loads(completed.stdout)

    return _path_from_json(result["path"])


def astar_rust(grid_data=grid, start_pos=start, goal_pos=goal, rust_library=None, rust_binary=None):
    try:
        solver = RustAStarSolver(grid_data, rust_library=rust_library)
        return solver.astar(start_pos, goal_pos)
    except (FileNotFoundError, OSError) as library_error:
        if not _is_default_problem(grid_data, start_pos, goal_pos):
            raise FileNotFoundError(
                "Rust DLL was not found, and the executable fallback only supports "
                "the default grid embedded in dummy_files/a.rs."
            ) from library_error

        try:
            return astar_rust_executable(rust_binary=rust_binary)
        except (FileNotFoundError, OSError) as binary_error:
            raise FileNotFoundError(
                f"{library_error}\n\nExecutable fallback also unavailable:\n{binary_error}"
            ) from binary_error


def astar_cpp(grid_data=grid, start_pos=start, goal_pos=goal, cpp_library=None, cpp_binary=None):
    try:
        solver = CppAStarSolver(grid_data, cpp_library=cpp_library)
        return solver.astar(start_pos, goal_pos)
    except (FileNotFoundError, OSError) as library_error:
        if not _is_default_problem(grid_data, start_pos, goal_pos):
            raise FileNotFoundError(
                "C++ DLL was not found, and the executable fallback only supports "
                "the default grid embedded in dummy_files/a.cpp."
            ) from library_error

        try:
            return astar_cpp_executable(cpp_binary=cpp_binary)
        except (FileNotFoundError, OSError) as binary_error:
            raise FileNotFoundError(
                f"{library_error}\n\nExecutable fallback also unavailable:\n{binary_error}"
            ) from binary_error


def print_grid_with_path(grid, path, start, goal):
    display = grid.astype(str)

    display[display == "0"] = "."
    display[display == "1"] = "#"

    if path is not None:
        for layer, row, col in path:
            display[layer, row, col] = "*"

    display[start] = "S"
    display[goal] = "G"

    for layer_index in range(display.shape[0]):
        print(f"\nLayer {layer_index}:")
        for row in display[layer_index]:
            print(" ".join(row))


def print_path_result(path):
    if path is None:
        print("No path found.")
        return

    print("Path found:")
    print(path)
    print("\nPath length:", len(path) - 1)
    print("\nGrid with path:")
    print_grid_with_path(grid, path, start, goal)


def benchmark_python(iterations):
    path = None
    started = time.perf_counter_ns()

    for _ in range(iterations):
        path = astar(grid, start, goal)

    duration_ns = time.perf_counter_ns() - started

    return {
        "implementation": "python",
        "iterations": iterations,
        "duration_ns": duration_ns,
        "path": path,
    }


def _benchmark_rust_library(iterations, rust_library=None):
    solver = RustAStarSolver(grid, rust_library=rust_library)
    path = None
    started = time.perf_counter_ns()

    for _ in range(iterations):
        path = solver.astar(start, goal)

    duration_ns = time.perf_counter_ns() - started

    return {
        "implementation": "rust-dll",
        "iterations": iterations,
        "duration_ns": duration_ns,
        "path": path,
        "binary": str(solver.library_path),
    }


def _benchmark_rust_executable(iterations, rust_binary=None):
    executable = find_rust_binary(rust_binary)
    completed = subprocess.run(
        [str(executable), "--benchmark", str(iterations)],
        check=True,
        capture_output=True,
        text=True,
    )
    result = json.loads(completed.stdout)

    return {
        "implementation": "rust-exe",
        "iterations": int(result["iterations"]),
        "duration_ns": int(result["duration_ns"]),
        "path": _path_from_json(result["result"]["path"]),
        "binary": str(executable),
    }


def _benchmark_cpp_library(iterations, cpp_library=None):
    solver = CppAStarSolver(grid, cpp_library=cpp_library)
    path = None
    started = time.perf_counter_ns()

    for _ in range(iterations):
        path = solver.astar(start, goal)

    duration_ns = time.perf_counter_ns() - started

    return {
        "implementation": "c++-dll",
        "iterations": iterations,
        "duration_ns": duration_ns,
        "path": path,
        "binary": str(solver.library_path),
    }


def _benchmark_cpp_executable(iterations, cpp_binary=None):
    executable = find_cpp_binary(cpp_binary)
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


def benchmark_rust(iterations, rust_library=None, rust_binary=None):
    try:
        return _benchmark_rust_library(iterations, rust_library=rust_library)
    except (FileNotFoundError, OSError) as library_error:
        try:
            return _benchmark_rust_executable(iterations, rust_binary=rust_binary)
        except (FileNotFoundError, OSError) as binary_error:
            raise FileNotFoundError(
                f"{library_error}\n\nExecutable benchmark fallback also unavailable:\n{binary_error}"
            ) from binary_error


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


def benchmark_astar(
    iterations=1_000,
    rust_library=None,
    rust_binary=None,
    cpp_library=None,
    cpp_binary=None,
):
    python_result = benchmark_python(iterations)
    results = [python_result]
    errors = {}

    try:
        results.append(
            benchmark_rust(
                iterations,
                rust_library=rust_library,
                rust_binary=rust_binary,
            )
        )
    except (FileNotFoundError, OSError) as error:
        errors["Rust"] = str(error)

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
    parser = argparse.ArgumentParser(description="Run and benchmark 3D A* pathfinding.")
    parser.add_argument(
        "--implementation",
        choices=("python", "rust", "cpp"),
        default="python",
        help="Implementation to use for the normal path run.",
    )
    parser.add_argument(
        "--benchmark",
        action="store_true",
        help="Benchmark Python plus the Rust and C++ implementations if available.",
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=1_000,
        help="Number of benchmark iterations.",
    )
    parser.add_argument(
        "--rust-lib",
        type=Path,
        default=None,
        help="Path to the compiled Rust DLL/shared library.",
    )
    parser.add_argument(
        "--rust-bin",
        type=Path,
        default=None,
        help="Path to the compiled Rust executable fallback.",
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

    if args.benchmark:
        results, errors = benchmark_astar(
            iterations=args.iterations,
            rust_library=args.rust_lib,
            rust_binary=args.rust_bin,
            cpp_library=args.cpp_lib,
            cpp_binary=args.cpp_bin,
        )
        print_benchmark_results(results, errors=errors)
        return 0

    if args.implementation == "rust":
        path = astar_rust(
            grid,
            start,
            goal,
            rust_library=args.rust_lib,
            rust_binary=args.rust_bin,
        )
    elif args.implementation == "cpp":
        path = astar_cpp(
            grid,
            start,
            goal,
            cpp_library=args.cpp_lib,
            cpp_binary=args.cpp_bin,
        )
    else:
        path = astar(grid, start, goal)

    print_path_result(path)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(error, file=sys.stderr)
        raise SystemExit(1)
