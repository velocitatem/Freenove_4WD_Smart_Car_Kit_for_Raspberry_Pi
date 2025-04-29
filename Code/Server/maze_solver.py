import time
import math
import threading
import numpy as np
from motor import *
from infrared import Infrared
from ultrasonic import Ultrasonic
from buzzer import Buzzer
from servo import Servo
from motor import Ordinary_Car
from queue import PriorityQueue
import json
from enum import IntEnum
from collections import deque

# Direction enum for consistent direction handling
class Dir(IntEnum):
    STRAIGHT = 0
    LEFT = 1
    RIGHT = 2
    BACK = -1

# Initialize sensors
infrared = Infrared()
ultrasonic = Ultrasonic()
buzzer = Buzzer()
servo = Servo()
PWM = Ordinary_Car()

class MazeSolver:
    def __init__(self):
        self.pwm = PWM
        
        # Constants for movement
        self.BASE_SPEED = 800
        self.TURN_SPEED = 1000
        self.TURN_DURATION_90 = 0.55  # Calibrate for exact 90 degrees
        self.TURN_DURATION_45 = self.TURN_DURATION_90 / 2
        self.TURN_DURATION_180 = self.TURN_DURATION_90 * 2
        self.CELL_DURATION = 0.5  # seconds to travel one grid cell
        self.CLEAR_DISTANCE = 50  # cm threshold for obstacle detection
        
        # Maze solving variables
        self.grid_size = 20  # cm per grid cell
        self.current_position = (0, 0)  # (x, y) in grid coordinates
        self.current_direction = 0  # 0: North, 1: East, 2: South, 3: West
        self.maze_map = {}  # Dictionary to store discovered maze cells: {(x,y): [north_wall, east_wall, south_wall, west_wall]}
        self.visited_cells = set()  # Set of visited cells
        self.path = []  # Current A* path
        self.target_position = None  # Target position for pathfinding
        self.in_maze = False  # Whether we're in the maze or still following the line
        
        # Maze dimensions
        self.min_x = 0
        self.max_x = 0
        self.min_y = 0
        self.max_y = 0
        
        # Stuck detection
        self.last_positions = deque(maxlen=8)
        
        # Servo scanning parameters
        self.scan_angles = [0, 90, 180]  # Servo angles to scan (left, front, right)
        self.scan_results = {}  # Store scan results
        self.observations = []
        
        # Wall detection threshold
        self.wall_threshold = 30  # cm
        
        # For simulation mode (no hardware)
        self.simulation_mode = False
        
    def set_simulation_mode(self, mode):
        """Set simulation mode (True for no hardware, False for hardware)"""
        self.simulation_mode = mode
    
    def median_distance(self):
        """Get median distance reading, ignoring zeros"""
        samples = [d for d in (ultrasonic.get_distance() for _ in range(5)) if d]
        if not samples:
            return 0  # treat as blocked
        samples.sort()
        return samples[len(samples)//2]
        
    def scan_environment(self):
        """Scan the environment using the servo-mounted ultrasonic sensor"""
        self.scan_results = {}  # Clear previous results
        
        if self.simulation_mode:
            # In simulation mode, generate fake scan results
            self.scan_results = {
                0: 50,   # Left
                90: 30,  # Front
                180: 40  # Right
            }
            time.sleep(0.5)  # Simulate scan time
            return self.scan_results
            
        # Move servo and take measurements at each angle
        SERVO_MOVE_TIME = 0.25
        
        for angle in self.scan_angles:
            # Move servo to scan angle
            servo.set_servo_pwm('0', angle)
            time.sleep(SERVO_MOVE_TIME)  # Wait for servo to move
            
            # Get median distance
            self.scan_results[angle] = self.median_distance()
            print(f"{angle}: {self.scan_results[angle]}")
            
        # Reset servo to center position
        servo.set_servo_pwm('0', 90)
        return self.scan_results
    
    def get_direction(self, scan_results):
        """Determine the direction to move based on the scan results"""
        # FIXED: Corrected direction mapping to match move() expectations
        if scan_results.get(90, 0) > self.wall_threshold:
            return Dir.STRAIGHT
        if scan_results.get(0, 0) > self.wall_threshold:
            return Dir.LEFT      # LEFT is 1
        if scan_results.get(180, 0) > self.wall_threshold:
            return Dir.RIGHT     # RIGHT is 2
        return Dir.BACK
    
    def update_maze_map(self):
        """Update the maze map based on the current position and scan results"""
        x, y = self.current_position
        
        # Update visited cells
        self.visited_cells.add(self.current_position)
        
        # Update maze boundaries
        self.min_x = min(self.min_x, x)
        self.max_x = max(self.max_x, x)
        self.min_y = min(self.min_y, y)
        self.max_y = max(self.max_y, y)
        
        # Initialize cell in maze_map if not already present
        if self.current_position not in self.maze_map:
            # [north_wall, east_wall, south_wall, west_wall]
            self.maze_map[self.current_position] = [False, False, False, False]
        
        # Update walls based on scan results and current direction
        # Wall presence is indicated by distance less than threshold
        for angle, distance in self.scan_results.items():
            wall_present = distance < self.wall_threshold
            
            # Convert servo angle to wall direction based on current orientation
            if angle == 90:  # Front scan
                wall_dir = self.current_direction
            elif angle == 0:  # Left scan
                wall_dir = (self.current_direction - 1) % 4
            elif angle == 180:  # Right scan
                wall_dir = (self.current_direction + 1) % 4
            else:
                continue  # Skip invalid angles
                
            # Update the wall in the current cell
            self.maze_map[self.current_position][wall_dir] = wall_present
            
            # Update the corresponding wall in adjacent cell
            if wall_present:
                # Calculate adjacent cell position
                if wall_dir == 0:  # North
                    adj_pos = (x, y + 1)
                    adj_wall_dir = 2  # South wall of adjacent cell
                elif wall_dir == 1:  # East
                    adj_pos = (x + 1, y)
                    adj_wall_dir = 3  # West wall of adjacent cell
                elif wall_dir == 2:  # South
                    adj_pos = (x, y - 1)
                    adj_wall_dir = 0  # North wall of adjacent cell
                elif wall_dir == 3:  # West
                    adj_pos = (x - 1, y)
                    adj_wall_dir = 1  # East wall of adjacent cell
                
                # Initialize adjacent cell if not already mapped
                if adj_pos not in self.maze_map:
                    self.maze_map[adj_pos] = [False, False, False, False]
                    
                # Update wall in adjacent cell
                self.maze_map[adj_pos][adj_wall_dir] = True
    
    def a_star_pathfinding(self, start, goal):
        """Find a path from start to goal using A* algorithm"""
        if goal not in self.maze_map:
            # If goal isn't in the maze map yet, we can't path to it
            return []
            
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {start: None}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        open_set_hash = {start}
        
        while not open_set.empty():
            current = open_set.get()[1]
            open_set_hash.remove(current)
            
            if current == goal:
                # Reconstruct path
                path = []
                while current:
                    path.append(current)
                    current = came_from[current]
                return path[::-1]  # Return reversed path
                
            # Get current cell x, y
            x, y = current
            
            # Check all four directions
            for i, (dx, dy) in enumerate([(0, 1), (1, 0), (0, -1), (-1, 0)]):
                # If there's a wall in this direction, skip
                if self.maze_map.get(current, [False, False, False, False])[i]:
                    continue
                
                neighbor = (x + dx, y + dy)
                
                # Skip if the neighbor isn't in the maze map
                if neighbor not in self.maze_map:
                    continue
                    
                # Calculate tentative g_score
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # This path is better
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    
                    if neighbor not in open_set_hash:
                        open_set.put((f_score[neighbor], neighbor))
                        open_set_hash.add(neighbor)
                        
        return []  # No path found
                
    def heuristic(self, a, b):
        """Manhattan distance heuristic for A*"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def get_next_exploration_target(self):
        """Choose the next unexplored cell to target"""
        # Find cells that are in the maze_map but not visited
        unexplored = set(self.maze_map.keys()) - self.visited_cells
        
        if not unexplored:
            # If all known cells are visited, return None
            return None
            
        # Find the closest unexplored cell
        current = self.current_position
        closest = min(unexplored, key=lambda pos: self.heuristic(current, pos))
        
        return closest
    
    # Movement primitives
    def go_straight(self, cells=1):
        """Move straight ahead for the specified number of cells"""
        if self.simulation_mode:
            self._update_position_after_straight()
            time.sleep(self.CELL_DURATION * cells)  # Simulate movement time
            return
            
        PWM.set_motor_model(self.BASE_SPEED, self.BASE_SPEED, 
                            self.BASE_SPEED, self.BASE_SPEED)
        time.sleep(self.CELL_DURATION * cells)
        PWM.set_motor_model(0, 0, 0, 0)
        
        # Update position
        self._update_position_after_straight()
        
    def turn(self, left=True, degrees=90):
        """Turn in place by the specified degrees"""
        if self.simulation_mode:
            if left:
                self.current_direction = (self.current_direction - (degrees // 90)) % 4
            else:
                self.current_direction = (self.current_direction + (degrees // 90)) % 4
            time.sleep(self.TURN_DURATION_90 * (degrees / 90))  # Simulate turning time
            return
            
        # Determine turn duration based on degrees
        if degrees == 45:
            duration = self.TURN_DURATION_45
        elif degrees == 90:
            duration = self.TURN_DURATION_90
        elif degrees == 180:
            duration = self.TURN_DURATION_180
        else:
            duration = self.TURN_DURATION_90 * (degrees / 90)
            
        # Set motors based on turn direction
        if left:
            PWM.set_motor_model(-self.TURN_SPEED, -self.TURN_SPEED,
                                self.TURN_SPEED, self.TURN_SPEED)
            # Update direction
            self.current_direction = (self.current_direction - (degrees // 90)) % 4
        else:
            PWM.set_motor_model(self.TURN_SPEED, self.TURN_SPEED,
                                -self.TURN_SPEED, -self.TURN_SPEED)
            # Update direction
            self.current_direction = (self.current_direction + (degrees // 90)) % 4
            
        time.sleep(duration)
        PWM.set_motor_model(0, 0, 0, 0)
    
    def move(self, direction):
        """
        Move the car in a specified direction.
        
        Args:
            direction: Dir enum value (STRAIGHT, LEFT, RIGHT, BACK)
        """
        # Record position for stuck detection
        self.last_positions.append(self.current_position)
        
        # Record current position as visited
        self.visited_cells.add(self.current_position)
        
        try:
            if direction is Dir.STRAIGHT:
                self.go_straight()
                
            elif direction is Dir.LEFT:
                self.turn(left=True)
                self.go_straight()
                
            elif direction is Dir.RIGHT:
                self.turn(left=False)
                self.go_straight()
                
            elif direction is Dir.BACK:
                self.turn(left=True, degrees=180)
                self.go_straight()
                
            # Check if we're stuck
            if len(self.last_positions) == 8 and len(set(self.last_positions)) <= 2:
                self.recovery_spin()
                
        except Exception as e:
            print(f"Error during movement: {e}")
            # Ensure car stops if there's an error
            PWM.set_motor_model(0, 0, 0, 0)
    
    def recovery_spin(self):
        """Recovery behavior when stuck in a loop"""
        print("Detected stuck in a loop - performing recovery spin")
        # Turn 90° right and rescan
        self.turn(left=False)
        # Clear stuck detection
        self.last_positions.clear()
    
    def _update_position_after_straight(self):
        """Update position after moving straight"""
        x, y = self.current_position
        if self.current_direction == 0:  # North
            self.current_position = (x, y + 1)
        elif self.current_direction == 1:  # East
            self.current_position = (x + 1, y)
        elif self.current_direction == 2:  # South
            self.current_position = (x, y - 1)
        elif self.current_direction == 3:  # West
            self.current_position = (x - 1, y)
                
        # Update maze boundaries
        x, y = self.current_position
        self.min_x = min(self.min_x, x)
        self.max_x = max(self.max_x, x)
        self.min_y = min(self.min_y, y)
        self.max_y = max(self.max_y, y)
    
    def follow_path(self, path):
        """Follow a given path using the robot's movement capabilities"""
        if not path or len(path) < 2:
            print("Path too short to follow")
            return
            
        # Start from the second point (first is current position)
        for next_pos in path[1:]:
            # Calculate direction to move
            curr_x, curr_y = self.current_position
            next_x, next_y = next_pos
            
            # Calculate relative direction
            if next_x > curr_x:  # East
                target_direction = 1
            elif next_x < curr_x:  # West
                target_direction = 3
            elif next_y > curr_y:  # North
                target_direction = 0
            else:  # South
                target_direction = 2
                
            # Calculate turn needed
            turn = (target_direction - self.current_direction) % 4
            
            # Convert turn to movement command
            if turn == 0:  # No turn needed
                move_command = Dir.STRAIGHT
            elif turn == 1:  # 90 degrees right
                move_command = Dir.RIGHT
            elif turn == 2:  # 180 degrees
                move_command = Dir.BACK
            elif turn == 3:  # 90 degrees left
                move_command = Dir.LEFT
                
            # Execute movement
            self.move(move_command)
    
    def run_step(self):
        """Run a single step of the maze solving algorithm"""
        # Scan the environment
        scan_results = self.scan_environment()
        print("Scan results:", scan_results)
        
        # Update the maze map based on scan results
        self.update_maze_map()
        
        # Choose next direction based on simple algorithm
        direction = self.get_direction(scan_results)
        direction_strings = {
            Dir.STRAIGHT: 'straight', 
            Dir.LEFT: 'left', 
            Dir.RIGHT: 'right', 
            Dir.BACK: 'back'
        }
        print(f"Moving {direction_strings[direction]}")
        
        # Move in the chosen direction
        self.move(direction)
        
        return direction
    
    def run_exploration(self, steps=10):
        """Run exploration for a set number of steps"""
        for _ in range(steps):
            self.run_step()
            time.sleep(0.5)  # Small delay between steps
    
    def run_to_target(self, target_position):
        """Run to a specific target position using A* pathfinding"""
        # Find path to target
        path = self.a_star_pathfinding(self.current_position, target_position)
        
        if not path:
            print(f"No path found to target {target_position}")
            return False
            
        # Save and display the path
        self.path = path
        print(f"Path found: {path}")
        
        # Follow the path
        self.follow_path(path)
        
        return True
    
    def save_maze_map(self, filename):
        """Save the maze map to a JSON file"""
        # Convert maze_map keys to strings for JSON serialization
        serializable_map = {str(k): v for k, v in self.maze_map.items()}
        
        data = {
            "maze_map": serializable_map,
            "visited_cells": list(map(list, self.visited_cells)),
            "min_x": self.min_x,
            "max_x": self.max_x,
            "min_y": self.min_y,
            "max_y": self.max_y
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f)
        
        print(f"Maze map saved to {filename}")
    
    def load_maze_map(self, filename):
        """Load the maze map from a JSON file"""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
                
            # Convert maze_map keys back to tuples
            self.maze_map = {eval(k): v for k, v in data["maze_map"].items()}
            self.visited_cells = set(map(tuple, data["visited_cells"]))
            self.min_x = data["min_x"]
            self.max_x = data["max_x"]
            self.min_y = data["min_y"]
            self.max_y = data["max_y"]
            
            print(f"Maze map loaded from {filename}")
            return True
        except Exception as e:
            print(f"Error loading maze map: {e}")
            return False


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Maze Solver')
    parser.add_argument('--sim', action='store_true', help='Run in simulation mode (no hardware)')
    parser.add_argument('--load', type=str, help='Load maze map from file')
    parser.add_argument('--steps', type=int, default=20, help='Number of exploration steps')
    args = parser.parse_args()
    
    # Create maze solver
    maze_solver = MazeSolver()
    
    # Set simulation mode if requested
    if args.sim:
        print("Running in simulation mode")
        maze_solver.set_simulation_mode(True)
    
    # Load maze map if requested
    if args.load:
        maze_solver.load_maze_map(args.load)
    
    try:
        # Run exploration
        maze_solver.run_exploration(args.steps)
        
        # Save the map
        maze_solver.save_maze_map('maze_map.json')
        
    except KeyboardInterrupt:
        print("Interrupted by user")
        # Save the map
        maze_solver.save_maze_map('maze_map.json')
