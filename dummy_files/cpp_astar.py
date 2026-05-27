import ctypes
import json
import os
import subprocess
import sys
from functools import lru_cache
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent


def _native_search_dirs():
    return [
        SCRIPT_DIR,
        REPO_ROOT / "dist" / "windows-amd64",
        REPO_ROOT / "dist" / "windows_amd_x64",
        REPO_ROOT / "bin" / "windows-amd64",
        REPO_ROOT / "bin" / "windows_amd_x64",
    ]


def _cpp_library_names():
    if os.name == "nt":
        return ["a_star_cpp.dll"]

    if sys.platform == "darwin":
        return ["liba_star_cpp.dylib", "a_star_cpp.dylib"]

    return ["liba_star_cpp.so", "a_star_cpp.so"]


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


@lru_cache(maxsize=None)
def _load_cpp_library(library_path_text):
    library_path = Path(library_path_text)

    if os.name == "nt" and hasattr(os, "add_dll_directory"):
        with os.add_dll_directory(str(library_path.parent)):
            library = ctypes.CDLL(str(library_path))
    else:
        library = ctypes.CDLL(str(library_path))

    function = library.cpp_astar_path
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

    return library


def _position_tuple(position):
    if len(position) != 3:
        raise ValueError("Position must have exactly three values: layer, row, column.")

    return tuple(int(value) for value in position)


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


def path_from_json(raw_path):
    if raw_path is None:
        return None

    return [tuple(int(value) for value in position) for position in raw_path]


def astar_cpp_executable(cpp_binary=None):
    executable = find_cpp_binary(cpp_binary)
    completed = subprocess.run(
        [str(executable), "--json"],
        check=True,
        capture_output=True,
        text=True,
    )
    result = json.loads(completed.stdout)

    return path_from_json(result["path"])


def astar_cpp(grid_data, start_pos, goal_pos, cpp_library=None):
    solver = CppAStarSolver(grid_data, cpp_library=cpp_library)
    return solver.astar(start_pos, goal_pos)
