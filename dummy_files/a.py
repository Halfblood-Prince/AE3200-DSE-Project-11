import numpy as np


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
    g_score = {start: 0}

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


def main():
    path = astar(grid, start, goal)
    print_path_result(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
