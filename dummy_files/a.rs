use std::collections::{HashMap, HashSet};

type Grid = [[[u8; 7]; 7]; 3];
type Position = (usize, usize, usize);

fn g(g_score: &HashMap<Position, usize>, current: Position) -> usize {
    g_score[&current]
}

fn h(current: Position, goal: Position) -> usize {
    current.0.abs_diff(goal.0) + current.1.abs_diff(goal.1) + current.2.abs_diff(goal.2)
}

fn f(g_score: &HashMap<Position, usize>, current: Position, goal: Position) -> usize {
    g(g_score, current) + h(current, goal)
}

fn get_neighbors(grid: &Grid, current: Position) -> Vec<Position> {
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

        let inside_grid = new_layer >= 0
            && new_layer < grid.len() as isize
            && new_row >= 0
            && new_row < grid[0].len() as isize
            && new_col >= 0
            && new_col < grid[0][0].len() as isize;

        if inside_grid {
            let position = (new_layer as usize, new_row as usize, new_col as usize);

            if grid[position.0][position.1][position.2] == 0 {
                neighbors.push(position);
            }
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

fn astar(grid: &Grid, start: Position, goal: Position) -> Option<Vec<Position>> {
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

fn print_grid_with_path(grid: &Grid, path: Option<&[Position]>, start: Position, goal: Position) {
    let mut display = [[['.'; 7]; 7]; 3];

    for layer in 0..grid.len() {
        for row in 0..grid[layer].len() {
            for col in 0..grid[layer][row].len() {
                if grid[layer][row][col] == 1 {
                    display[layer][row][col] = '#';
                }
            }
        }
    }

    if let Some(path) = path {
        for &(layer, row, col) in path {
            display[layer][row][col] = '*';
        }
    }

    display[start.0][start.1][start.2] = 'S';
    display[goal.0][goal.1][goal.2] = 'G';

    for (layer_index, layer) in display.iter().enumerate() {
        println!("\nLayer {}:", layer_index);

        for row in layer {
            let line = row
                .iter()
                .map(char::to_string)
                .collect::<Vec<_>>()
                .join(" ");

            println!("{}", line);
        }
    }
}

fn main() {
    // 0 = free space
    // 1 = obstacle
    let grid: Grid = [
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

    // Position format: (layer, row, column)
    let start = (0, 0, 0);
    let goal = (2, 6, 6);

    let path = astar(&grid, start, goal);

    match path {
        None => println!("No path found."),
        Some(path) => {
            println!("Path found:");
            println!("{:?}", path);
            println!("\nPath length: {}", path.len() - 1);
            println!("\nGrid with path:");
            print_grid_with_path(&grid, Some(&path), start, goal);
        }
    }
}
