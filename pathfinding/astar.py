"""
Run a simple 3D A* pathfinding example on a layered occupancy grid.

The user-facing coordinate system is (x, y, z):
- x is the column number inside a layer.
- y is the row number inside a layer.
- z is the layer number.

NumPy stores the grid as grid[z, y, x], so helper functions translate between
the coordinate convention and the array indexing convention where needed.
"""

import numpy as np

# The grid is a 3D occupancy map with shape:
#   number of layers x number of rows x number of columns.
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

# Start and goal are written in the public (x, y, z) coordinate format.
start = (0, 0, 0)
goal = (6, 6, 2)


def grid_index(position):
    """
    Convert an (x, y, z) position to NumPy grid indexing.

    The pathfinding code uses positions as (x, y, z), but the grid array is
    indexed as grid[layer, row, column], which is the same as grid[z, y, x].
    """
    x, y, z = position
    return z, y, x


def g(g_score, current):
    """
    Actual cost from start to current node.

    A* stores this value in the g_score dictionary. Since every move costs 1,
    this is the number of moves taken to reach current from start.
    """
    return g_score[current]


def h(current, goal):
    """
    Estimated cost from current node to goal.

    3D Manhattan distance is used because movement is only in 6 directions.
    The heuristic is admissible here because it never overestimates the number
    of axis-aligned moves needed to reach the goal.
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

    A* chooses the open node with the smallest f score as the next node to
    explore.
    """
    return g(g_score, current) + h(current, goal)


def get_neighbors(grid, current):
    """
    Return all valid free-space neighbors of current.

    Movement is allowed in exactly 6 directions: left/right along x, up/down
    along y, and up/down through z layers. Diagonal movement is not allowed.
    """
    x, y, z = current

    # Each move is expressed as a change in (x, y, z).
    moves = [
        (0, -1, 0),  # row up
        (0, 1, 0),  # row down
        (-1, 0, 0),  # column left
        (1, 0, 0),  # column right
        (0, 0, 1),  # one layer up
        (0, 0, -1),  # one layer down
    ]

    neighbors = []

    for d_x, d_y, d_z in moves:
        # Apply the move to produce a candidate neighbor coordinate.
        new_x = x + d_x
        new_y = y + d_y
        new_z = z + d_z

        # Check bounds using grid shape order: layers, rows, columns.
        inside_grid = (
            0 <= new_z < grid.shape[0]
            and 0 <= new_y < grid.shape[1]
            and 0 <= new_x < grid.shape[2]
        )

        # Only free cells can be visited.
        if inside_grid and grid[new_z, new_y, new_x] == 0:
            neighbors.append((new_x, new_y, new_z))

    return neighbors


def reconstruct_path(came_from, current):
    """
    Rebuild the final path after A* reaches the goal.

    came_from maps each visited node to the node that led to it. Starting at
    the goal and walking backward through this map produces the path in reverse,
    so the list is reversed before returning.
    """
    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path


def astar(grid, start, goal):
    """
    Find the shortest path from start to goal using the A* algorithm.

    The returned path is a list of (x, y, z) positions. If the goal cannot be
    reached, None is returned.
    """
    # Nodes discovered but not fully explored yet.
    open_set = [start]

    # Nodes already explored.
    closed_set = set()

    # Back-pointer map used to reconstruct the final path.
    came_from = {}

    # Best known distance from start to each discovered node.
    g_score = {start: 0}

    while open_set:
        # Choose the node with the lowest f score.
        current = min(open_set, key=lambda node: f(g_score, node, goal))

        # The goal was reached; rebuild and return the path.
        if current == goal:
            return reconstruct_path(came_from, current)

        # Move current from open to closed because it is now being expanded.
        open_set.remove(current)
        closed_set.add(current)

        for neighbor in get_neighbors(grid, current):
            # Ignore nodes that have already been fully explored.
            if neighbor in closed_set:
                continue

            # Every valid move has cost 1.
            new_g = g_score[current] + 1

            # Keep this route if neighbor is new or this route is cheaper.
            if neighbor not in g_score or new_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = new_g

                # Queue the neighbor for future exploration.
                if neighbor not in open_set:
                    open_set.append(neighbor)

    # No route could connect start to goal.
    return None


def print_grid_with_path(grid, path, start, goal):
    """
    Print each grid layer with the path drawn over it.

    Display symbols:
    - "." means free space.
    - "#" means obstacle.
    - "*" means path.
    - "S" means start.
    - "G" means goal.
    """
    # Convert numeric grid values to printable strings.
    display = grid.astype(str)

    display[display == "0"] = "."
    display[display == "1"] = "#"

    # Draw the path first, then draw start/goal so they remain visible.
    if path is not None:
        for position in path:
            display[grid_index(position)] = "*"

    display[grid_index(start)] = "S"
    display[grid_index(goal)] = "G"

    for layer_index in range(display.shape[0]):
        print(f"\nLayer {layer_index}:")
        for row in display[layer_index]:
            print(" ".join(row))


def print_path_result(path):
    """
    Print the A* result in both coordinate-list and grid-display formats.
    """
    if path is None:
        print("No path found.")
        return

    print("Path found:")
    print(path)
    print("\nPath length:", len(path) - 1)
    print("\nGrid with path:")
    print_grid_with_path(grid, path, start, goal)


def main():
    """
    Run the example A* search using the default grid, start, and goal.
    """
    path = astar(grid, start, goal)
    print_path_result(path)


if __name__ == "__main__":
    # Run the example only when this file is executed directly.
    raise SystemExit(main())