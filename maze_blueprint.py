#!/usr/bin/env python3
# maze_blueprint.py - Create and visualize maze blueprints

import json
import numpy as np
from geometry import CELL_CM, COLS, ROWS, START_CELL, EXIT_CELL

# Bit values for walls: 1=North, 2=East, 4=South, 8=West
# Example: 5 = North(1) + South(4) = walls on North and South sides

# Empty maze (just outer walls)
def create_empty_maze():
    # Initialize with zeros
    blueprint = [[0 for _ in range(COLS)] for _ in range(ROWS)]
    
    # Add outer walls
    for x in range(COLS):
        # Bottom row has South walls
        blueprint[0][x] |= 4
        # Top row has North walls
        blueprint[ROWS-1][x] |= 1
    
    for y in range(ROWS):
        # Left column has West walls
        blueprint[y][0] |= 8
        # Right column has East walls
        blueprint[y][COLS-1] |= 2
    
    return blueprint

# Competition maze with internal walls
def create_competition_maze():
    """Create the competition maze based on the provided wall grids"""
    # Initialize with zeros
    blueprint = [[0 for _ in range(COLS)] for _ in range(ROWS)]
    
    # Convert the h_walls and v_walls to our bitmap representation
    h_walls = np.array([
        [0, 1, 1, 1, 1, 1, 1, 1, 1, 1],   # top border (open at entry, col 0)
        [0, 1, 0, 0, 1, 0, 0, 1, 1, 0],
        [0, 1, 1, 1, 0, 1, 0, 0, 0, 1],
        [1, 0, 0, 1, 0, 0, 1, 1, 0, 0],
        [0, 1, 1, 0, 0, 1, 1, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 0]    # bottom border (open at exit, col 9)
    ], dtype=bool)

    v_walls = np.array([
        [1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1],
        [1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1],
        [1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1],
        [1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1],
        [1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1]
    ], dtype=bool)
    
    # Convert wall grids to our bitmap format
    for y in range(ROWS):
        for x in range(COLS):
            # North wall (from h_walls)
            if h_walls[y, x]:
                blueprint[y][x] |= 1
                
            # South wall (from h_walls of row below)
            if h_walls[y+1, x]:
                blueprint[y][x] |= 4
                
            # West wall (from v_walls)
            if v_walls[y, x]:
                blueprint[y][x] |= 8
                
            # East wall (from v_walls of column to the right)
            if v_walls[y, x+1]:
                blueprint[y][x] |= 2
    
    return blueprint

def blueprint_to_wall_grids(blueprint):
    """Convert a blueprint to horizontal and vertical wall grids"""
    h_walls = np.zeros((ROWS + 1, COLS), dtype=bool)
    v_walls = np.zeros((ROWS, COLS + 1), dtype=bool)
    
    for y in range(ROWS):
        for x in range(COLS):
            cell = blueprint[y][x]
            
            # North wall
            if cell & 1:
                h_walls[y, x] = True
                
            # South wall
            if cell & 4:
                h_walls[y+1, x] = True
                
            # West wall
            if cell & 8:
                v_walls[y, x] = True
                
            # East wall
            if cell & 2:
                v_walls[y, x+1] = True
    
    return h_walls, v_walls

def wall_grids_to_occupancy(h_walls, v_walls):
    """Convert wall grids to an occupancy grid (1=wall, 0=free space)"""
    rows, cols = ROWS, COLS
    occ = np.ones((rows * 2 + 1, cols * 2 + 1), dtype=int)

    for r in range(rows):
        for c in range(cols):
            occ[2*r + 1, 2*c + 1] = 0               # cell center is free
            if not h_walls[r, c]:     occ[2*r,     2*c + 1] = 0   # north opening
            if not h_walls[r + 1, c]: occ[2*r + 2, 2*c + 1] = 0   # south opening
            if not v_walls[r, c]:     occ[2*r + 1, 2*c    ] = 0   # west opening
            if not v_walls[r, c + 1]: occ[2*r + 1, 2*c + 2] = 0   # east opening
    
    return occ

def blueprint_to_occupancy(blueprint):
    """Convert a blueprint directly to an occupancy grid"""
    h_walls, v_walls = blueprint_to_wall_grids(blueprint)
    return wall_grids_to_occupancy(h_walls, v_walls)

def print_maze(blueprint):
    """Print a text representation of the maze"""
    # Print top border
    print(' ' + '_' * (COLS * 2 - 1))
    
    for y in range(ROWS-1, -1, -1):  # Print from top to bottom
        # Print west wall and cell contents
        line = '|'
        for x in range(COLS):
            # Check for south wall
            if blueprint[y][x] & 4:  # Has south wall
                line += '_'
            else:
                line += ' '
                
            # Check for east wall
            if x < COLS-1 and blueprint[y][x] & 2:  # Has east wall
                line += '|'
            else:
                line += ' '
                
        # Print east wall of rightmost cell
        line += '|'
        
        # Mark start and exit cells
        line_with_markers = list(line)
        
        if y == START_CELL[1]:
            # Mark start with S
            pos = START_CELL[0] * 2 + 1
            line_with_markers[pos] = 'S'
            
        if y == EXIT_CELL[1]:
            # Mark exit with E
            pos = EXIT_CELL[0] * 2 + 1
            line_with_markers[pos] = 'E'
            
        print(''.join(line_with_markers))

def print_occupancy_grid(occ):
    """Print the occupancy grid as ASCII art"""
    print(f"Occupancy grid ({occ.shape[0]} × {occ.shape[1]}):")
    for row in occ:
        print("".join('█' if cell else ' ' for cell in row))
    
def save_blueprint(blueprint, filename='maze_blueprint.json'):
    """Save blueprint to a JSON file"""
    with open(filename, 'w') as f:
        json.dump(blueprint, f)
    print(f"Blueprint saved to {filename}")
    
def load_blueprint(filename='maze_blueprint.json'):
    """Load blueprint from a JSON file"""
    try:
        with open(filename, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Blueprint file {filename} not found. Creating empty maze.")
        return create_empty_maze()

def save_wall_grids(h_walls, v_walls, filename='wall_grids.npz'):
    """Save horizontal and vertical wall grids to a NumPy file"""
    np.savez(filename, h_walls=h_walls, v_walls=v_walls)
    print(f"Wall grids saved to {filename}")

def load_wall_grids(filename='wall_grids.npz'):
    """Load horizontal and vertical wall grids from a NumPy file"""
    try:
        data = np.load(filename)
        return data['h_walls'], data['v_walls']
    except FileNotFoundError:
        print(f"Wall grids file {filename} not found.")
        return None, None

# Visualization with matplotlib (optional)
def plot_maze(h_walls, v_walls, show=True):
    """Visual representation of the maze using matplotlib"""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("Matplotlib is required for visualization")
        return
        
    rows, cols = h_walls.shape[0] - 1, h_walls.shape[1]
    fig, ax = plt.subplots(figsize=(cols, rows * 0.8))

    # horizontal walls
    for r in range(rows + 1):
        for c in range(cols):
            if h_walls[r, c]:
                ax.plot([c, c + 1], [rows - r, rows - r], lw=4, color='k')
    # vertical walls
    for r in range(rows):
        for c in range(cols + 1):
            if v_walls[r, c]:
                ax.plot([c, c], [rows - r - 1, rows - r], lw=4, color='k')

    ax.set_aspect('equal')
    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.axis('off')
    ax.set_title(f'Maze grid representation ({COLS} × {ROWS})')
    
    # Mark start and exit
    ax.plot(START_CELL[0] + 0.5, rows - START_CELL[1] - 0.5, 'go', markersize=10)
    ax.plot(EXIT_CELL[0] + 0.5, rows - EXIT_CELL[1] - 0.5, 'ro', markersize=10)

    if show:
        plt.show()
    return fig, ax

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Maze Blueprint Creator')
    parser.add_argument('--create', choices=['empty', 'competition'], 
                        help='Create a new blueprint')
    parser.add_argument('--save', action='store_true', 
                        help='Save the blueprint to file')
    parser.add_argument('--file', type=str, default='maze_blueprint.json',
                        help='Blueprint filename')
    parser.add_argument('--plot', action='store_true',
                        help='Plot the maze using matplotlib')
    parser.add_argument('--occupancy', action='store_true',
                        help='Print the occupancy grid')
    
    args = parser.parse_args()
    
    if args.create == 'empty':
        blueprint = create_empty_maze()
        print("Created empty maze blueprint")
    elif args.create == 'competition':
        blueprint = create_competition_maze()
        print("Created competition maze blueprint")
    else:
        # Try to load existing blueprint
        blueprint = load_blueprint(args.file)
    
    # Print the ASCII maze
    print_maze(blueprint)
    
    # Convert to wall grids and occupancy grid if needed
    if args.plot or args.occupancy:
        h_walls, v_walls = blueprint_to_wall_grids(blueprint)
        
        if args.plot:
            try:
                plot_maze(h_walls, v_walls)
            except ImportError:
                print("Matplotlib not available for plotting")
        
        if args.occupancy:
            occ = wall_grids_to_occupancy(h_walls, v_walls)
            print_occupancy_grid(occ)
    
    # Save if requested
    if args.save:
        save_blueprint(blueprint, args.file) 