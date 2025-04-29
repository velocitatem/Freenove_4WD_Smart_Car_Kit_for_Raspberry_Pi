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

# Example competition maze with internal walls
def create_competition_maze():
    blueprint = create_empty_maze()
    
    # Define internal walls
    # Format: (y, x, direction)
    # Direction: 1=North, 2=East, 4=South, 8=West
    internal_walls = [
        # Some example walls
        (1, 2, 1),  # Cell (2,1) has North wall
        (1, 2, 8),  # Cell (2,1) has West wall
        (2, 3, 2),  # Cell (3,2) has East wall
        (2, 3, 4),  # Cell (3,2) has South wall
        (3, 6, 1),  # Cell (6,3) has North wall
        (3, 6, 2),  # Cell (6,3) has East wall
        # Add more walls as needed
    ]
    
    for y, x, wall in internal_walls:
        blueprint[y][x] |= wall
        
        # Add the corresponding wall to the adjacent cell
        if wall == 1 and y < ROWS-1:  # North wall
            blueprint[y+1][x] |= 4     # South wall of cell above
        elif wall == 2 and x < COLS-1:  # East wall
            blueprint[y][x+1] |= 8      # West wall of cell to the right
        elif wall == 4 and y > 0:       # South wall
            blueprint[y-1][x] |= 1      # North wall of cell below
        elif wall == 8 and x > 0:       # West wall
            blueprint[y][x-1] |= 2      # East wall of cell to the left
    
    return blueprint

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

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Maze Blueprint Creator')
    parser.add_argument('--create', choices=['empty', 'competition'], 
                        help='Create a new blueprint')
    parser.add_argument('--save', action='store_true', 
                        help='Save the blueprint to file')
    parser.add_argument('--file', type=str, default='maze_blueprint.json',
                        help='Blueprint filename')
    
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
    
    # Print the maze
    print_maze(blueprint)
    
    # Save if requested
    if args.save:
        save_blueprint(blueprint, args.file) 