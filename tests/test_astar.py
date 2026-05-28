"""Unit tests for the standalone 3D A* implementation."""

import numpy as np

from pathfinding import astar as astar_module


def test_default_grid_path_reaches_goal():
    """The bundled example grid should have a valid path to the goal."""
    path = astar_module.astar(astar_module.grid, astar_module.start, astar_module.goal)

    assert path is not None
    assert path[0] == astar_module.start
    assert path[-1] == astar_module.goal
    assert len(path) - 1 == 14


def test_neighbors_use_xyz_coordinates_over_zyx_storage():
    """Neighbor lookup should expose x/y/z while indexing NumPy as z/y/x."""
    grid = np.zeros((2, 3, 4), dtype=np.uint8)
    grid[0, 1, 2] = 1

    neighbors = astar_module.get_neighbors(grid, (1, 1, 0))

    assert (2, 1, 0) not in neighbors
    assert (1, 1, 1) in neighbors
    assert astar_module.grid_index((3, 2, 1)) == (1, 2, 3)


def test_unreachable_goal_returns_none():
    """A fully blocked middle column should make the goal unreachable."""
    grid = np.zeros((1, 3, 3), dtype=np.uint8)
    grid[0, :, 1] = 1

    assert astar_module.astar(grid, (0, 1, 0), (2, 1, 0)) is None
