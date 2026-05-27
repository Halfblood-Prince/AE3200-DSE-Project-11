#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#if defined(_WIN32)
#define ASTAR_API __declspec(dllexport)
#else
#define ASTAR_API __attribute__((visibility("default")))
#endif

struct Position {
    std::size_t layer;
    std::size_t row;
    std::size_t col;

    bool operator==(const Position& other) const {
        return layer == other.layer && row == other.row && col == other.col;
    }
};

struct PositionHash {
    std::size_t operator()(const Position& position) const {
        std::size_t value = position.layer;
        value = value * 31 + position.row;
        value = value * 31 + position.col;
        return value;
    }
};

struct GridView {
    const std::uint8_t* cells;
    std::size_t layers;
    std::size_t rows;
    std::size_t cols;

    bool in_bounds(const Position& position) const {
        return position.layer < layers && position.row < rows && position.col < cols;
    }

    std::size_t index(const Position& position) const {
        return (position.layer * rows + position.row) * cols + position.col;
    }

    std::uint8_t cell(const Position& position) const {
        return cells[index(position)];
    }

    bool is_free(const Position& position) const {
        return in_bounds(position) && cell(position) == 0;
    }
};

constexpr std::size_t DEFAULT_LAYERS = 3;
constexpr std::size_t DEFAULT_ROWS = 7;
constexpr std::size_t DEFAULT_COLS = 7;
constexpr Position DEFAULT_START{0, 0, 0};
constexpr Position DEFAULT_GOAL{2, 6, 6};

const std::uint8_t DEFAULT_GRID[DEFAULT_LAYERS][DEFAULT_ROWS][DEFAULT_COLS] = {
    {
        {0, 0, 0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0, 1, 0},
        {0, 0, 0, 1, 0, 1, 0},
        {1, 1, 0, 0, 0, 1, 0},
        {0, 0, 0, 1, 0, 0, 0},
        {0, 1, 0, 1, 1, 1, 0},
        {0, 1, 0, 0, 0, 0, 0},
    },
    {
        {0, 0, 0, 1, 0, 0, 0},
        {0, 1, 0, 1, 0, 1, 0},
        {0, 1, 0, 0, 0, 1, 0},
        {0, 1, 1, 1, 0, 0, 0},
        {0, 0, 0, 1, 1, 1, 0},
        {1, 1, 0, 0, 0, 1, 0},
        {0, 0, 0, 1, 0, 0, 0},
    },
    {
        {0, 0, 0, 0, 0, 1, 0},
        {1, 1, 1, 1, 0, 1, 0},
        {0, 0, 0, 1, 0, 0, 0},
        {0, 1, 0, 1, 1, 1, 0},
        {0, 1, 0, 0, 0, 0, 0},
        {0, 1, 1, 1, 1, 1, 0},
        {0, 0, 0, 0, 0, 0, 0},
    },
};

std::vector<std::uint8_t> default_grid_flat() {
    std::vector<std::uint8_t> cells;
    cells.reserve(DEFAULT_LAYERS * DEFAULT_ROWS * DEFAULT_COLS);

    for (const auto& layer : DEFAULT_GRID) {
        for (const auto& row : layer) {
            for (std::uint8_t value : row) {
                cells.push_back(value);
            }
        }
    }

    return cells;
}

std::size_t h(const Position& current, const Position& goal) {
    const auto layer = current.layer > goal.layer ? current.layer - goal.layer : goal.layer - current.layer;
    const auto row = current.row > goal.row ? current.row - goal.row : goal.row - current.row;
    const auto col = current.col > goal.col ? current.col - goal.col : goal.col - current.col;

    return layer + row + col;
}

std::size_t f(
    const std::unordered_map<Position, std::size_t, PositionHash>& g_score,
    const Position& current,
    const Position& goal
) {
    return g_score.at(current) + h(current, goal);
}

std::vector<Position> get_neighbors(const GridView& grid, const Position& current) {
    const int moves[6][3] = {
        {0, -1, 0},
        {0, 1, 0},
        {0, 0, -1},
        {0, 0, 1},
        {1, 0, 0},
        {-1, 0, 0},
    };

    std::vector<Position> neighbors;

    for (const auto& move : moves) {
        const auto new_layer = static_cast<long long>(current.layer) + move[0];
        const auto new_row = static_cast<long long>(current.row) + move[1];
        const auto new_col = static_cast<long long>(current.col) + move[2];

        if (new_layer < 0 || new_row < 0 || new_col < 0) {
            continue;
        }

        Position position{
            static_cast<std::size_t>(new_layer),
            static_cast<std::size_t>(new_row),
            static_cast<std::size_t>(new_col),
        };

        if (grid.is_free(position)) {
            neighbors.push_back(position);
        }
    }

    return neighbors;
}

std::vector<Position> reconstruct_path(
    const std::unordered_map<Position, Position, PositionHash>& came_from,
    Position current
) {
    std::vector<Position> path{current};

    while (came_from.find(current) != came_from.end()) {
        current = came_from.at(current);
        path.push_back(current);
    }

    std::reverse(path.begin(), path.end());
    return path;
}

std::optional<std::vector<Position>> astar(const GridView& grid, const Position& start, const Position& goal) {
    if (!grid.is_free(start) || !grid.is_free(goal)) {
        return std::nullopt;
    }

    std::vector<Position> open_set{start};
    std::unordered_set<Position, PositionHash> closed_set;
    std::unordered_map<Position, Position, PositionHash> came_from;
    std::unordered_map<Position, std::size_t, PositionHash> g_score;

    g_score[start] = 0;

    while (!open_set.empty()) {
        const auto current_iter = std::min_element(
            open_set.begin(),
            open_set.end(),
            [&](const Position& left, const Position& right) {
                return f(g_score, left, goal) < f(g_score, right, goal);
            }
        );
        const Position current = *current_iter;

        if (current == goal) {
            return reconstruct_path(came_from, current);
        }

        open_set.erase(current_iter);
        closed_set.insert(current);

        for (const auto& neighbor : get_neighbors(grid, current)) {
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }

            const auto new_g = g_score[current] + 1;

            if (g_score.find(neighbor) == g_score.end() || new_g < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = new_g;

                if (std::find(open_set.begin(), open_set.end(), neighbor) == open_set.end()) {
                    open_set.push_back(neighbor);
                }
            }
        }
    }

    return std::nullopt;
}

extern "C" ASTAR_API std::ptrdiff_t cpp_astar_path(
    const std::uint8_t* grid_ptr,
    std::size_t layers,
    std::size_t rows,
    std::size_t cols,
    std::size_t start_layer,
    std::size_t start_row,
    std::size_t start_col,
    std::size_t goal_layer,
    std::size_t goal_row,
    std::size_t goal_col,
    std::size_t* output_ptr,
    std::size_t output_positions
) {
    if (grid_ptr == nullptr || output_ptr == nullptr || layers == 0 || rows == 0 || cols == 0) {
        return -2;
    }

    const GridView grid{grid_ptr, layers, rows, cols};
    const auto path = astar(
        grid,
        Position{start_layer, start_row, start_col},
        Position{goal_layer, goal_row, goal_col}
    );

    if (!path.has_value()) {
        return -1;
    }

    if (path->size() > output_positions) {
        return -3;
    }

    for (std::size_t index = 0; index < path->size(); ++index) {
        output_ptr[index * 3] = (*path)[index].layer;
        output_ptr[index * 3 + 1] = (*path)[index].row;
        output_ptr[index * 3 + 2] = (*path)[index].col;
    }

    return static_cast<std::ptrdiff_t>(path->size());
}

std::string path_to_json(const std::vector<Position>& path) {
    std::string json = "{\"path\":[";

    for (std::size_t index = 0; index < path.size(); ++index) {
        if (index > 0) {
            json += ",";
        }

        json += "[" + std::to_string(path[index].layer) + ",";
        json += std::to_string(path[index].row) + ",";
        json += std::to_string(path[index].col) + "]";
    }

    json += "],\"path_length\":" + std::to_string(path.empty() ? 0 : path.size() - 1) + "}";
    return json;
}

void print_path_json(const std::optional<std::vector<Position>>& path) {
    if (path.has_value()) {
        std::cout << path_to_json(*path) << '\n';
    } else {
        std::cout << "{\"path\":null,\"path_length\":null}\n";
    }
}

void print_grid_with_path(
    const GridView& grid,
    const std::optional<std::vector<Position>>& path,
    const Position& start,
    const Position& goal
) {
    std::vector<char> display(grid.layers * grid.rows * grid.cols, '.');

    for (std::size_t layer = 0; layer < grid.layers; ++layer) {
        for (std::size_t row = 0; row < grid.rows; ++row) {
            for (std::size_t col = 0; col < grid.cols; ++col) {
                Position position{layer, row, col};

                if (grid.cell(position) == 1) {
                    display[grid.index(position)] = '#';
                }
            }
        }
    }

    if (path.has_value()) {
        for (const auto& position : *path) {
            display[grid.index(position)] = '*';
        }
    }

    display[grid.index(start)] = 'S';
    display[grid.index(goal)] = 'G';

    for (std::size_t layer = 0; layer < grid.layers; ++layer) {
        std::cout << "\nLayer " << layer << ":\n";

        for (std::size_t row = 0; row < grid.rows; ++row) {
            for (std::size_t col = 0; col < grid.cols; ++col) {
                if (col > 0) {
                    std::cout << ' ';
                }

                std::cout << display[grid.index(Position{layer, row, col})];
            }

            std::cout << '\n';
        }
    }
}

void run_default() {
    const auto cells = default_grid_flat();
    const GridView grid{cells.data(), DEFAULT_LAYERS, DEFAULT_ROWS, DEFAULT_COLS};
    const auto path = astar(grid, DEFAULT_START, DEFAULT_GOAL);

    if (!path.has_value()) {
        std::cout << "No path found.\n";
        return;
    }

    std::cout << "Path found:\n[";

    for (std::size_t index = 0; index < path->size(); ++index) {
        if (index > 0) {
            std::cout << ", ";
        }

        const auto& position = (*path)[index];
        std::cout << "(" << position.layer << ", " << position.row << ", " << position.col << ")";
    }

    std::cout << "]\n\nPath length: " << path->size() - 1 << "\n\nGrid with path:\n";
    print_grid_with_path(grid, path, DEFAULT_START, DEFAULT_GOAL);
}

void run_json() {
    const auto cells = default_grid_flat();
    const GridView grid{cells.data(), DEFAULT_LAYERS, DEFAULT_ROWS, DEFAULT_COLS};
    const auto path = astar(grid, DEFAULT_START, DEFAULT_GOAL);

    print_path_json(path);
}

void run_benchmark(std::size_t iterations) {
    iterations = std::max<std::size_t>(iterations, 1);
    const auto cells = default_grid_flat();
    const GridView grid{cells.data(), DEFAULT_LAYERS, DEFAULT_ROWS, DEFAULT_COLS};
    std::optional<std::vector<Position>> last_path;
    volatile std::size_t observed_path_size = 0;
    const auto started = std::chrono::steady_clock::now();

    for (std::size_t index = 0; index < iterations; ++index) {
        last_path = astar(grid, DEFAULT_START, DEFAULT_GOAL);
        observed_path_size += last_path.has_value() ? last_path->size() : 0;
    }

    const auto stopped = std::chrono::steady_clock::now();
    const auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(stopped - started).count();

    if (observed_path_size == 0 && last_path.has_value()) {
        std::cerr << "Unexpected empty benchmark observation.\n";
    }

    std::cout << "{\"implementation\":\"cpp\",\"iterations\":" << iterations;
    std::cout << ",\"duration_ns\":" << elapsed_ns << ",\"result\":";

    if (last_path.has_value()) {
        std::cout << path_to_json(*last_path);
    } else {
        std::cout << "{\"path\":null,\"path_length\":null}";
    }

    std::cout << "}\n";
}

void print_usage(const char* program) {
    std::cout << "Usage:\n";
    std::cout << "  " << program << "              Run the default A* demo\n";
    std::cout << "  " << program << " --json       Print the default A* path as JSON\n";
    std::cout << "  " << program << " --benchmark [iterations]\n";
}

int main(int argc, char** argv) {
    if (argc == 1) {
        run_default();
        return 0;
    }

    const std::string option = argv[1];

    if (option == "--json") {
        run_json();
        return 0;
    }

    if (option == "--benchmark") {
        const auto iterations = argc >= 3 ? std::strtoull(argv[2], nullptr, 10) : 1000;
        run_benchmark(static_cast<std::size_t>(iterations));
        return 0;
    }

    if (option == "--help" || option == "-h") {
        print_usage(argv[0]);
        return 0;
    }

    std::cerr << "Unknown option: " << option << '\n';
    print_usage(argv[0]);
    return 2;
}
