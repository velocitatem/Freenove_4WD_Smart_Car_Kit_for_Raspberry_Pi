#!/usr/bin/env python3
# run_solver.py - Simple script to run the maze solver with various options

import os
import sys
import argparse
import time

# Add the Code/Server directory to the path
current_dir = os.path.dirname(os.path.abspath(__file__))
server_dir = os.path.join(current_dir, 'Code', 'Server')
sys.path.append(server_dir)

# Import the MazeSolver
from maze_solver import MazeSolver
from geometry import START_CELL, EXIT_CELL

def main():
    parser = argparse.ArgumentParser(description='Run the Maze Solver with various options')
    parser.add_argument('--sim', action='store_true', help='Run in simulation mode (no hardware)')
    parser.add_argument('--explore', action='store_true', help='Run exploration mode')
    parser.add_argument('--steps', type=int, default=20, help='Number of exploration steps')
    parser.add_argument('--calibrate', action='store_true', help='Run calibration procedure')
    parser.add_argument('--exit', action='store_true', help='Navigate to exit')
    parser.add_argument('--load', type=str, help='Load maze map from file')
    parser.add_argument('--save', type=str, default='maze_map.json', help='Save maze map to file')
    parser.add_argument('--path', action='store_true', help='Only calculate and display path to exit')
    
    args = parser.parse_args()
    
    # Create maze solver
    solver = MazeSolver()
    
    # Set simulation mode if requested
    if args.sim:
        print("Running in simulation mode")
        solver.set_simulation_mode(True)
    
    # Run calibration if requested
    if args.calibrate:
        print("Running calibration...")
        solver.calibrate()
        return
    
    # Load maze map if requested
    if args.load:
        print(f"Loading maze map from {args.load}")
        solver.load_maze_map(args.load)
    else:
        # Preload competition maze as default
        print("Preloading competition maze")
        solver.preload_maze()
    
    try:
        # If path mode, just calculate and display path
        if args.path:
            print("Calculating path to exit...")
            path = solver.a_star_on_occupancy_grid(START_CELL, EXIT_CELL)
            if path:
                print(f"Path found: {path}")
                print(f"Path length: {len(path)} steps")
            else:
                print("No path found!")
            return
        
        # Run exploration if requested
        if args.explore:
            print(f"Running exploration for {args.steps} steps...")
            solver.run_exploration(args.steps)
            print("Exploration complete")
        
        # Navigate to exit if requested
        if args.exit or not args.explore:
            print(f"Navigating to exit at {EXIT_CELL}...")
            success = solver.run_to_target(EXIT_CELL)
            
            if success:
                print("Successfully reached the exit!")
            else:
                print("Failed to reach the exit")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        # Save maze map
        print(f"Saving maze map to {args.save}")
        solver.save_maze_map(args.save)
        
        # Stop motors if not in simulation mode
        if not args.sim:
            print("Stopping motors")
            solver.pwm.set_motor_model(0, 0, 0, 0)
        
        print("Done!")

if __name__ == "__main__":
    main() 