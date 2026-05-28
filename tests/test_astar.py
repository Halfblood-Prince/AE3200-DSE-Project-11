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


def test_cost_helpers_use_unit_moves_and_manhattan_distance():
    """The A* score helpers should combine actual and estimated costs."""
    g_score = {(1, 2, 0): 4}

    assert astar_module.g(g_score, (1, 2, 0)) == 4
    assert astar_module.h((1, 2, 0), (4, 0, 2)) == 7
    assert astar_module.f(g_score, (1, 2, 0), (4, 0, 2)) == 11


def test_reconstruct_path_walks_back_to_start():
    """came_from links should be reversed into start-to-goal order."""
    came_from = {
        (1, 0, 0): (0, 0, 0),
        (2, 0, 0): (1, 0, 0),
    }

    assert astar_module.reconstruct_path(came_from, (2, 0, 0)) == [
        (0, 0, 0),
        (1, 0, 0),
        (2, 0, 0),
    ]


def test_unreachable_goal_returns_none():
    """A fully blocked middle column should make the goal unreachable."""
    grid = np.zeros((1, 3, 3), dtype=np.uint8)
    grid[0, :, 1] = 1

    assert astar_module.astar(grid, (0, 1, 0), (2, 1, 0)) is None


def test_print_grid_with_path_draws_symbols(capsys):
    """Grid printing should show layers, obstacles, start, goal, and path."""
    grid = np.array([[[0, 1], [0, 0]]], dtype=np.uint8)
    path = [(0, 0, 0), (0, 1, 0), (1, 1, 0)]

    astar_module.print_grid_with_path(grid, path, path[0], path[-1])

    output = capsys.readouterr().out
    assert "Layer 0:" in output
    assert "S #" in output
    assert "* G" in output


def test_print_grid_without_path_keeps_start_and_goal(capsys):
    """Grid printing should still mark endpoints when no path is available."""
    grid = np.zeros((1, 1, 2), dtype=np.uint8)

    astar_module.print_grid_with_path(grid, None, (0, 0, 0), (1, 0, 0))

    assert "S G" in capsys.readouterr().out


def test_print_path_result_handles_missing_path(capsys):
    """A missing path should produce a short no-path message."""
    astar_module.print_path_result(None)

    assert capsys.readouterr().out == "No path found.\n"


def test_print_path_result_reports_found_path(monkeypatch, capsys):
    """A found path should print the path, length, and grid heading."""
    calls = []
    monkeypatch.setattr(
        astar_module,
        "print_grid_with_path",
        lambda *args: calls.append(args),
    )

    astar_module.print_path_result([(0, 0, 0), (1, 0, 0)])

    output = capsys.readouterr().out
    assert "Path found:" in output
    assert "Path length: 1" in output
    assert "Grid with path:" in output
    assert calls


def test_main_runs_default_search(monkeypatch):
    """main should run A* on the bundled grid and print the result."""
    calls = []
    expected_path = [(0, 0, 0), (1, 0, 0)]
    monkeypatch.setattr(astar_module, "astar", lambda *args: expected_path)
    monkeypatch.setattr(astar_module, "print_path_result", calls.append)

    assert astar_module.main() is None
    assert calls == [expected_path]