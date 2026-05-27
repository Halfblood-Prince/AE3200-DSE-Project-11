use std::collections::{HashMap, HashSet};
use std::env;
use std::process;
use std::slice;
use std::time::Instant;

type Position = (usize, usize, usize);

const DEFAULT_LAYERS: usize = 3;
const DEFAULT_ROWS: usize = 7;
const DEFAULT_COLS: usize = 7;
const DEFAULT_START: Position = (0, 0, 0);
const DEFAULT_GOAL: Position = (2, 6, 6);

const DEFAULT_GRID: [[[u8; DEFAULT_COLS]; DEFAULT_ROWS]; DEFAULT_LAYERS] = [
    // Layer 0
    [
        [0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0, 1, 0],
        [0, 0, 0, 1, 0, 1, 0],
        [1, 1, 0, 0, 0, 1, 0],
        [0, 0, 0, 1, 0, 0, 0],
        [0, 1, 0, 1, 1, 1, 0],
        [0, 1, 0, 0, 0, 0, 0],
    ],
    // Layer 1
    [
        [0, 0, 0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0, 1, 0],
        [0, 1, 0, 0, 0, 1, 0],
        [0, 1, 1, 1, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 0],
        [1, 1, 0, 0, 0, 1, 0],
        [0, 0, 0, 1, 0, 0, 0],
    ],
    // Layer 2
    [
        [0, 0, 0, 0, 0, 1, 0],
        [1, 1, 1, 1, 0, 1, 0],
        [0, 0, 0, 1, 0, 0, 0],
        [0, 1, 0, 1, 1, 1, 0],
        [0, 1, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0],
    ],
];

struct GridView<'a> {
    cells: &'a [u8],
    layers: usize,
    rows: usize,
    cols: usize,
}

impl GridView<'_> {
    fn in_bounds(&self, position: Position) -> bool {
        position.0 < self.layers && position.1 < self.rows && position.2 < self.cols
    }

    fn index(&self, position: Position) -> usize {
        (position.0 * self.rows + position.1) * self.cols + position.2
    }

    fn cell(&self, position: Position) -> u8 {
        self.cells[self.index(position)]
    }

    fn is_free(&self, position: Position) -> bool {
        self.in_bounds(position) && self.cell(position) == 0
    }
}

fn default_grid_flat() -> Vec<u8> {
    DEFAULT_GRID
        .iter()
        .flat_map(|layer| layer.iter())
        .flat_map(|row| row.iter())
        .copied()
        .collect()
}

fn default_grid_view(cells: &[u8]) -> GridView<'_> {
    GridView {
        cells,
        layers: DEFAULT_LAYERS,
        rows: DEFAULT_ROWS,
        cols: DEFAULT_COLS,
    }
}

fn g(g_score: &HashMap<Position, usize>, current: Position) -> usize {
    g_score[&current]
}

fn h(current: Position, goal: Position) -> usize {
    current.0.abs_diff(goal.0) + current.1.abs_diff(goal.1) + current.2.abs_diff(goal.2)
}

fn f(g_score: &HashMap<Position, usize>, current: Position, goal: Position) -> usize {
    g(g_score, current) + h(current, goal)
}

fn get_neighbors(grid: &GridView<'_>, current: Position) -> Vec<Position> {
    let (layer, row, col) = current;
    let moves = [
        (0, -1, 0), // row up
        (0, 1, 0),  // row down
        (0, 0, -1), // column left
        (0, 0, 1),  // column right
        (1, 0, 0),  // one layer up
        (-1, 0, 0), // one layer down
    ];

    let mut neighbors = Vec::new();

    for (d_layer, d_row, d_col) in moves {
        let new_layer = layer as isize + d_layer;
        let new_row = row as isize + d_row;
        let new_col = col as isize + d_col;

        if new_layer < 0 || new_row < 0 || new_col < 0 {
            continue;
        }

        let position = (new_layer as usize, new_row as usize, new_col as usize);

        if grid.is_free(position) {
            neighbors.push(position);
        }
    }

    neighbors
}

fn reconstruct_path(came_from: &HashMap<Position, Position>, mut current: Position) -> Vec<Position> {
    let mut path = vec![current];

    while let Some(previous) = came_from.get(&current) {
        current = *previous;
        path.push(current);
    }

    path.reverse();
    path
}

fn astar(grid: &GridView<'_>, start: Position, goal: Position) -> Option<Vec<Position>> {
    if !grid.is_free(start) || !grid.is_free(goal) {
        return None;
    }

    let mut open_set = vec![start];
    let mut closed_set = HashSet::new();
    let mut came_from = HashMap::new();
    let mut g_score = HashMap::new();

    g_score.insert(start, 0);

    while !open_set.is_empty() {
        let current_index = open_set
            .iter()
            .enumerate()
            .min_by_key(|(_, node)| f(&g_score, **node, goal))
            .map(|(index, _)| index)
            .expect("open set is not empty");

        let current = open_set[current_index];

        if current == goal {
            return Some(reconstruct_path(&came_from, current));
        }

        open_set.remove(current_index);
        closed_set.insert(current);

        for neighbor in get_neighbors(grid, current) {
            if closed_set.contains(&neighbor) {
                continue;
            }

            let new_g = g_score[&current] + 1;

            if !g_score.contains_key(&neighbor) || new_g < g_score[&neighbor] {
                came_from.insert(neighbor, current);
                g_score.insert(neighbor, new_g);

                if !open_set.contains(&neighbor) {
                    open_set.push(neighbor);
                }
            }
        }
    }

    None
}

#[no_mangle]
pub extern "C" fn astar_path(
    grid_ptr: *const u8,
    layers: usize,
    rows: usize,
    cols: usize,
    start_layer: usize,
    start_row: usize,
    start_col: usize,
    goal_layer: usize,
    goal_row: usize,
    goal_col: usize,
    output_ptr: *mut usize,
    output_positions: usize,
) -> isize {
    if grid_ptr.is_null() || output_ptr.is_null() {
        return -2;
    }

    let cell_count = match layers.checked_mul(rows).and_then(|value| value.checked_mul(cols)) {
        Some(value) if value > 0 => value,
        _ => return -2,
    };

    let cells = unsafe { slice::from_raw_parts(grid_ptr, cell_count) };
    let grid = GridView {
        cells,
        layers,
        rows,
        cols,
    };

    let path = match astar(
        &grid,
        (start_layer, start_row, start_col),
        (goal_layer, goal_row, goal_col),
    ) {
        Some(path) => path,
        None => return -1,
    };

    if path.len() > output_positions || path.len() > isize::MAX as usize {
        return -3;
    }

    let output_values = match path.len().checked_mul(3) {
        Some(value) => value,
        None => return -3,
    };
    let output = unsafe { slice::from_raw_parts_mut(output_ptr, output_values) };

    for (index, (layer, row, col)) in path.iter().copied().enumerate() {
        output[index * 3] = layer;
        output[index * 3 + 1] = row;
        output[index * 3 + 2] = col;
    }

    path.len() as isize
}

fn path_to_json(path: &[Position]) -> String {
    let values = path
        .iter()
        .map(|(layer, row, col)| format!("[{},{},{}]", layer, row, col))
        .collect::<Vec<_>>()
        .join(",");

    format!(
        "{{\"path\":[{}],\"path_length\":{}}}",
        values,
        path.len().saturating_sub(1)
    )
}

fn print_path_json(path: Option<&[Position]>) {
    match path {
        Some(path) => println!("{}", path_to_json(path)),
        None => println!("{{\"path\":null,\"path_length\":null}}"),
    }
}

fn print_grid_with_path(grid: &GridView<'_>, path: Option<&[Position]>, start: Position, goal: Position) {
    let mut display = vec!['.'; grid.cells.len()];

    for layer in 0..grid.layers {
        for row in 0..grid.rows {
            for col in 0..grid.cols {
                let position = (layer, row, col);

                if grid.cell(position) == 1 {
                    display[grid.index(position)] = '#';
                }
            }
        }
    }

    if let Some(path) = path {
        for &position in path {
            display[grid.index(position)] = '*';
        }
    }

    display[grid.index(start)] = 'S';
    display[grid.index(goal)] = 'G';

    for layer in 0..grid.layers {
        println!("\nLayer {}:", layer);

        for row in 0..grid.rows {
            let mut line = String::new();

            for col in 0..grid.cols {
                if col > 0 {
                    line.push(' ');
                }

                line.push(display[grid.index((layer, row, col))]);
            }

            println!("{}", line);
        }
    }
}

fn run_default() {
    let cells = default_grid_flat();
    let grid = default_grid_view(&cells);
    let path = astar(&grid, DEFAULT_START, DEFAULT_GOAL);

    match path {
        None => println!("No path found."),
        Some(path) => {
            println!("Path found:");
            println!("{:?}", path);
            println!("\nPath length: {}", path.len() - 1);
            println!("\nGrid with path:");
            print_grid_with_path(&grid, Some(&path), DEFAULT_START, DEFAULT_GOAL);
        }
    }
}

fn run_json() {
    let cells = default_grid_flat();
    let grid = default_grid_view(&cells);
    let path = astar(&grid, DEFAULT_START, DEFAULT_GOAL);

    print_path_json(path.as_deref());
}

fn run_benchmark(iterations: usize) {
    let iterations = iterations.max(1);
    let cells = default_grid_flat();
    let grid = default_grid_view(&cells);
    let mut last_path = None;
    let started = Instant::now();

    for _ in 0..iterations {
        last_path = astar(&grid, DEFAULT_START, DEFAULT_GOAL);
        std::hint::black_box(&last_path);
    }

    let elapsed_ns = started.elapsed().as_nanos();
    let path_json = match last_path.as_deref() {
        Some(path) => path_to_json(path),
        None => "{\"path\":null,\"path_length\":null}".to_string(),
    };

    println!(
        "{{\"implementation\":\"rust\",\"iterations\":{},\"duration_ns\":{},\"result\":{}}}",
        iterations, elapsed_ns, path_json
    );
}

fn print_usage(program: &str) {
    println!("Usage:");
    println!("  {}              Run the default A* demo", program);
    println!("  {} --json       Print the default A* path as JSON", program);
    println!("  {} --benchmark [iterations]", program);
}

fn main() {
    let args = env::args().collect::<Vec<_>>();
    let program = args.first().map(String::as_str).unwrap_or("a_star_rust");

    match args.get(1).map(String::as_str) {
        None => run_default(),
        Some("--json") => run_json(),
        Some("--benchmark") => {
            let iterations = args
                .get(2)
                .and_then(|value| value.parse::<usize>().ok())
                .unwrap_or(1_000);

            run_benchmark(iterations);
        }
        Some("--help") | Some("-h") => print_usage(program),
        Some(option) => {
            eprintln!("Unknown option: {}", option);
            print_usage(program);
            process::exit(2);
        }
    }
}
