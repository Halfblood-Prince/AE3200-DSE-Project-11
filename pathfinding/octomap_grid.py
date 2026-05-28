"""Convert OctoMap .bt files into NumPy grids for the bundled A* planner."""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, floor
from pathlib import Path
from typing import Iterable

import numpy as np


TREE_DEPTH = 16
TREE_MAX_VAL = 1 << (TREE_DEPTH - 1)
BINARY_FILE_HEADER = "# Octomap OcTree binary file"


@dataclass(frozen=True)
class OctomapLeaf:
    """One known OctoMap leaf node."""

    key: tuple[int, int, int]
    depth: int
    occupied: bool


@dataclass(frozen=True)
class OctomapBinary:
    """Parsed .bt metadata and leaf nodes."""

    resolution: float
    tree_id: str
    expected_size: int
    leaves: tuple[OctomapLeaf, ...]


@dataclass(frozen=True)
class OctomapGrid:
    """Dense grid plus the transform needed to move between grid/world frames."""

    grid: np.ndarray
    origin: tuple[float, float, float]
    resolution: float
    occupied_leaves: int
    free_leaves: int

    def world_to_grid(self, point: tuple[float, float, float]) -> tuple[int, int, int]:
        """Convert a world-frame point into an A* (x, y, z) grid index."""
        return tuple(
            int(floor((point[index] - self.origin[index]) / self.resolution))
            for index in range(3)
        )

    def grid_to_world(self, index: tuple[int, int, int]) -> tuple[float, float, float]:
        """Convert an A* (x, y, z) grid index to the voxel-center world point."""
        return tuple(
            self.origin[axis] + (index[axis] + 0.5) * self.resolution
            for axis in range(3)
        )

    def contains_index(self, index: tuple[int, int, int]) -> bool:
        """Return true when an A* index lies inside the dense grid."""
        x, y, z = index
        return 0 <= z < self.grid.shape[0] and 0 <= y < self.grid.shape[1] and 0 <= x < self.grid.shape[2]


def key_to_coord(key: int, depth: int, resolution: float) -> float:
    """Match OctoMap keyToCoord for one coordinate axis."""
    if depth == 0:
        return 0.0
    if depth == TREE_DEPTH:
        return (key - TREE_MAX_VAL + 0.5) * resolution

    node_size = node_size_at_depth(depth, resolution)
    divisor = 1 << (TREE_DEPTH - depth)
    return (floor((key - TREE_MAX_VAL) / divisor) + 0.5) * node_size


def node_size_at_depth(depth: int, resolution: float) -> float:
    """Return the cubic voxel size represented by an OctoMap node depth."""
    return resolution * float(1 << (TREE_DEPTH - depth))


def compute_child_key(parent_key: tuple[int, int, int], child_index: int, child_depth: int) -> tuple[int, int, int]:
    """Compute the child key using OctoMap's child-index convention."""
    if not 1 <= child_depth <= TREE_DEPTH:
        raise ValueError(f"child_depth must be between 1 and {TREE_DEPTH}")
    center_offset = 1 << (TREE_DEPTH - child_depth - 1) if child_depth < TREE_DEPTH else 0
    child_key = list(parent_key)
    for axis, bit in enumerate((1, 2, 4)):
        if child_index & bit:
            child_key[axis] = parent_key[axis] + center_offset
        else:
            child_key[axis] = parent_key[axis] - center_offset - (0 if center_offset else 1)
    return tuple(child_key)


def parse_bt_file(path: str | Path) -> OctomapBinary:
    """Read an OctoMap binary .bt file and return known leaf nodes."""
    with Path(path).open("rb") as stream:
        header = _read_header(stream)
        leaves = tuple(_read_binary_node(stream, (TREE_MAX_VAL, TREE_MAX_VAL, TREE_MAX_VAL), 0))
        return OctomapBinary(
            resolution=header["resolution"],
            tree_id=header["tree_id"],
            expected_size=header["size"],
            leaves=leaves,
        )


def bt_file_to_numpy_grid(
    path: str | Path,
    *,
    planning_resolution: float | None = None,
    bounds: tuple[tuple[float, float, float], tuple[float, float, float]] | None = None,
    padding: float = 0.5,
    unknown_is_occupied: bool = True,
    include_points: Iterable[tuple[float, float, float]] = (),
) -> OctomapGrid:
    """Convert a .bt file into the dense 0-free / 1-occupied grid used by A*."""
    octomap = parse_bt_file(path)
    resolution = float(planning_resolution or octomap.resolution)
    if resolution <= 0.0:
        raise ValueError("planning_resolution must be positive")

    origin, shape_xyz = _grid_bounds(
        octomap.leaves,
        octomap.resolution,
        resolution,
        bounds,
        padding,
        include_points,
    )
    fill_value = 1 if unknown_is_occupied else 0
    grid = np.full((shape_xyz[2], shape_xyz[1], shape_xyz[0]), fill_value, dtype=np.uint8)

    # Free is applied first; occupied wins where observations conflict.
    for desired_state in (False, True):
        value = 1 if desired_state else 0
        for leaf in octomap.leaves:
            if leaf.occupied is desired_state:
                _paint_leaf(grid, origin, resolution, leaf, octomap.resolution, value)

    return OctomapGrid(
        grid=grid,
        origin=origin,
        resolution=resolution,
        occupied_leaves=sum(1 for leaf in octomap.leaves if leaf.occupied),
        free_leaves=sum(1 for leaf in octomap.leaves if not leaf.occupied),
    )


def _read_header(stream) -> dict[str, float | int | str]:
    first_line = stream.readline().decode("utf-8", errors="replace").strip()
    if not first_line.startswith(BINARY_FILE_HEADER):
        raise ValueError(f"Not an OctoMap .bt file: missing '{BINARY_FILE_HEADER}' header")

    header: dict[str, float | int | str] = {"tree_id": "", "size": 0, "resolution": 0.0}
    while True:
        raw_line = stream.readline()
        if not raw_line:
            raise ValueError("OctoMap .bt file ended before the data section")
        line = raw_line.decode("utf-8", errors="replace").strip()
        if not line or line.startswith("#"):
            continue
        if line == "data":
            break

        key, _, value = line.partition(" ")
        if key == "id":
            header["tree_id"] = value.strip()
        elif key == "size":
            header["size"] = int(value)
        elif key == "res":
            header["resolution"] = float(value)

    if not header["tree_id"]:
        raise ValueError("OctoMap .bt header is missing an id field")
    if header["resolution"] <= 0.0:
        raise ValueError("OctoMap .bt header has an invalid resolution")
    return header


def _read_binary_node(stream, parent_key: tuple[int, int, int], depth: int):
    if depth >= TREE_DEPTH:
        raise ValueError("OctoMap .bt data contains children deeper than the tree depth")

    raw = stream.read(2)
    if len(raw) != 2:
        raise ValueError("OctoMap .bt data ended inside a node")

    child_depth = depth + 1
    child_statuses = []
    for byte_index, child_bits in enumerate(raw):
        for bit_pair in range(4):
            child_index = byte_index * 4 + bit_pair
            low_bit = (child_bits >> (bit_pair * 2)) & 1
            high_bit = (child_bits >> (bit_pair * 2 + 1)) & 1
            child_statuses.append((child_index, low_bit, high_bit))

    for child_index, low_bit, high_bit in child_statuses:
        child_key = compute_child_key(parent_key, child_index, child_depth)
        if (low_bit, high_bit) == (1, 0):
            yield OctomapLeaf(child_key, child_depth, False)
        elif (low_bit, high_bit) == (0, 1):
            yield OctomapLeaf(child_key, child_depth, True)
        elif (low_bit, high_bit) == (1, 1):
            yield from _read_binary_node(stream, child_key, child_depth)


def _grid_bounds(
    leaves: tuple[OctomapLeaf, ...],
    map_resolution: float,
    grid_resolution: float,
    bounds: tuple[tuple[float, float, float], tuple[float, float, float]] | None,
    padding: float,
    include_points: Iterable[tuple[float, float, float]],
) -> tuple[tuple[float, float, float], tuple[int, int, int]]:
    if bounds is not None:
        lower, upper = bounds
    else:
        lower = [float("inf"), float("inf"), float("inf")]
        upper = [float("-inf"), float("-inf"), float("-inf")]

        for leaf in leaves:
            leaf_lower, leaf_upper = _leaf_bounds(leaf, map_resolution)
            for axis in range(3):
                lower[axis] = min(lower[axis], leaf_lower[axis])
                upper[axis] = max(upper[axis], leaf_upper[axis])

        for point in include_points:
            for axis in range(3):
                lower[axis] = min(lower[axis], point[axis])
                upper[axis] = max(upper[axis], point[axis])

        if not all(np.isfinite(value) for value in lower + upper):
            raise ValueError("Cannot build a grid from an empty OctoMap without explicit bounds")

        lower = tuple(value - padding for value in lower)
        upper = tuple(value + padding for value in upper)

    origin = tuple(floor(value / grid_resolution) * grid_resolution for value in lower)
    shape_xyz = tuple(max(1, int(ceil((upper[axis] - origin[axis]) / grid_resolution))) for axis in range(3))
    return origin, shape_xyz


def _leaf_bounds(leaf: OctomapLeaf, map_resolution: float) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    size = node_size_at_depth(leaf.depth, map_resolution)
    half_size = size / 2.0
    center = tuple(key_to_coord(leaf.key[axis], leaf.depth, map_resolution) for axis in range(3))
    lower = tuple(center[axis] - half_size for axis in range(3))
    upper = tuple(center[axis] + half_size for axis in range(3))
    return lower, upper


def _paint_leaf(
    grid: np.ndarray,
    origin: tuple[float, float, float],
    grid_resolution: float,
    leaf: OctomapLeaf,
    map_resolution: float,
    value: int,
) -> None:
    lower, upper = _leaf_bounds(leaf, map_resolution)
    mins = [int(floor((lower[axis] - origin[axis]) / grid_resolution)) for axis in range(3)]
    maxs = [int(ceil((upper[axis] - origin[axis]) / grid_resolution)) for axis in range(3)]

    x0 = max(0, mins[0])
    y0 = max(0, mins[1])
    z0 = max(0, mins[2])
    x1 = min(grid.shape[2], maxs[0])
    y1 = min(grid.shape[1], maxs[1])
    z1 = min(grid.shape[0], maxs[2])
    if x0 < x1 and y0 < y1 and z0 < z1:
        grid[z0:z1, y0:y1, x0:x1] = value
