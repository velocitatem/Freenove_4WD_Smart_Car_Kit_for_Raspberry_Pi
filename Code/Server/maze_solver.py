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
        self.BASE_SPEED = 2000
        self.TURN_SPEED = 2500
        self.TURN_DURATION_90 = 0.65  # Increased from 0.55 for more accurate turns
        self.CELL_DURATION = 0.5  # seconds to travel one grid cell
        
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
        # self.scan_angles = [0, 90, 180]  # Old angles
        self.SCAN_ANGLES = [15, 45, 75, 90, 105, 135, 165] # New angles (7 points)
        self.SERVO_MOVE_TIME = 0.5# Increased pause time for servo to settle
        self.READING_PAUSE = 0.05 # Small pause after reading
        # Keep old angle names for reference in logic if needed, though direct angles used now
        self.ANG_LEFT_FAR = 15
        self.ANG_LEFT_MID = 45
        self.ANG_LEFT_NEAR = 75
        self.ANG_FRONT = 90
        self.ANG_RIGHT_NEAR = 105
        self.ANG_RIGHT_MID = 135
        self.ANG_RIGHT_FAR = 165
        self.scan_results = {}  # Store scan results {angle: distance}
        self.observations = []
        
        # Wall detection threshold - ADJUSTED
        self.FRONT_CLEAR_CM = 45     # start the turn sooner
        self.SIDE_CLEAR_CM  = 35     # sides may be slightly closer
        self.MAP_WALL_THRESHOLD_CM = 30 # Threshold for marking walls in the map
        
        # For simulation mode (no hardware)
        self.simulation_mode = False
        
    def set_simulation_mode(self, mode):
        """Set simulation mode (True for no hardware, False for hardware)"""
        self.simulation_mode = mode
    
    def median_distance(self):
        """Get median distance reading, ignoring zeros"""
        samples = [d for d in (ultrasonic.get_distance() for _ in range(5)) if d > 0] # Ignore 0 explicitly
        if not samples:
            return 1  # Treat as blocked (return a small non-zero number)
        samples.sort()
        return samples[len(samples)//2]
        
    def scan_environment(self):
        """Scan the environment using the servo-mounted ultrasonic sensor across defined angles."""
        self.scan_results = {}  # Clear previous results

        if self.simulation_mode:
            # In simulation mode, generate fake scan results matching new angles
            self.scan_results = {
                15: 50, 45: 60, 75: 70, # Left side
                90: 25, # Front (blocked)
                105: 70, 135: 60, 165: 50 # Right side
            }
            print("Simulated Scan:", self.scan_results)
            time.sleep(0.5)  # Simulate scan time
            return self.scan_results

        # Ensure servo starts at a known position (e.g., first angle)
        servo.set_servo_pwm('0', self.SCAN_ANGLES[0])
        time.sleep(0.3) # Allow time to reach start angle

        # Gradually move servo and take measurements
        for angle in self.SCAN_ANGLES:
            servo.set_servo_pwm('0', angle)
            time.sleep(self.SERVO_MOVE_TIME)  # Wait for servo to settle

            # Get median distance
            distance = self.median_distance()
            self.scan_results[angle] = distance
            time.sleep(self.READING_PAUSE) # Add pause after reading
            # print(f"Scan Angle {angle}: {distance:.1f} cm") # Optional debug print

        # Reset servo to center position after scan
        servo.set_servo_pwm('0', self.ANG_FRONT) # Reset to front (90)
        time.sleep(0.2)
        # print("Scan Results:", {a: f"{d:.1f}" for a, d in self.scan_results.items()}) # Optional summary print
        return self.scan_results
    
    def get_direction(self, scan):
        """Decide where to go next based on multi-point scan results."""
        # Check front first
        fwd_dist = scan.get(self.ANG_FRONT, 0)
        if fwd_dist > self.FRONT_CLEAR_CM:
            return Dir.STRAIGHT

        # Get max distances for left and right sectors
        left_readings = [scan.get(a, 0) for a in self.SCAN_ANGLES if a < self.ANG_FRONT]
        right_readings = [scan.get(a, 0) for a in self.SCAN_ANGLES if a > self.ANG_FRONT]

        max_left = max(left_readings) if left_readings else 0
        max_right = max(right_readings) if right_readings else 0

        # Check if either side is clear enough
        left_clear = max_left > self.SIDE_CLEAR_CM
        right_clear = max_right > self.SIDE_CLEAR_CM

        if left_clear or right_clear:
            # Prefer the side with more clearance
            if max_left >= max_right:
                return Dir.LEFT
            else:
                return Dir.RIGHT

        # If neither front nor sides are clear, go back
        return Dir.BACK
    
    def update_maze_map(self):
        """Update the maze map based on the current position and multi-point scan results."""
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

        # --- Determine wall presence based on minimum distances in sectors ---
        scan = self.scan_results

        # Front wall based on 90-degree reading
        front_dist = scan.get(self.ANG_FRONT, 0)
        front_wall_present = front_dist < self.MAP_WALL_THRESHOLD_CM

        # Left wall based on minimum of left-side readings
        left_readings = [scan.get(a, 0) for a in self.SCAN_ANGLES if a < self.ANG_FRONT and a in scan]
        min_left = min(left_readings) if left_readings else 999 # Use large number if no readings
        left_wall_present = min_left < self.MAP_WALL_THRESHOLD_CM

        # Right wall based on minimum of right-side readings
        right_readings = [scan.get(a, 0) for a in self.SCAN_ANGLES if a > self.ANG_FRONT and a in scan]
        min_right = min(right_readings) if right_readings else 999 # Use large number if no readings
        right_wall_present = min_right < self.MAP_WALL_THRESHOLD_CM

        # --- Map relative walls (front, left, right) to absolute directions (N, E, S, W) ---
        relative_walls = {
            0: front_wall_present,  # Relative Front
            1: right_wall_present,  # Relative Right
            # 2: No direct reading for back
            3: left_wall_present    # Relative Left
        }

        for relative_dir, wall_present in relative_walls.items():
            # Calculate absolute wall direction
            # Relative dir 0 (Front) maps to current_direction
            # Relative dir 1 (Right) maps to (current_direction + 1) % 4
            # Relative dir 3 (Left) maps to (current_direction - 1 + 4) % 4
            if relative_dir == 0:
                abs_wall_dir = self.current_direction
            elif relative_dir == 1:
                abs_wall_dir = (self.current_direction + 1) % 4
            elif relative_dir == 3:
                abs_wall_dir = (self.current_direction - 1 + 4) % 4 # Ensure positive modulo
            else:
                continue # Skip other relative directions

            # Update the wall in the current cell
            self.maze_map[self.current_position][abs_wall_dir] = wall_present

            # Update the corresponding wall in adjacent cell if a wall was detected
            if wall_present:
                adj_pos = None
                adj_wall_dir = -1
                # Calculate adjacent cell position and the wall to update there
                if abs_wall_dir == 0:  # North wall in current cell
                    adj_pos = (x, y + 1)
                    adj_wall_dir = 2  # South wall of adjacent cell
                elif abs_wall_dir == 1:  # East wall in current cell
                    adj_pos = (x + 1, y)
                    adj_wall_dir = 3  # West wall of adjacent cell
                elif abs_wall_dir == 2:  # South wall in current cell
                    adj_pos = (x, y - 1)
                    adj_wall_dir = 0  # North wall of adjacent cell
                elif abs_wall_dir == 3:  # West wall in current cell
                    adj_pos = (x - 1, y)
                    adj_wall_dir = 1  # East wall of adjacent cell

                if adj_pos is not None:
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
        
    def turn(self, left=True):
        """Turn 90 degrees in place"""
        if self.simulation_mode:
            if left:
                self.current_direction = (self.current_direction - 1) % 4
            else:
                self.current_direction = (self.current_direction + 1) % 4
            time.sleep(self.TURN_DURATION_90)  # Simulate turning time
            return

        duration = self.TURN_DURATION_90

        # Set motors based on turn direction
        if left:
            PWM.set_motor_model(-self.TURN_SPEED, -self.TURN_SPEED,
                                self.TURN_SPEED, self.TURN_SPEED)
            # Update direction
            self.current_direction = (self.current_direction - 1) % 4
        else:
            PWM.set_motor_model(self.TURN_SPEED, self.TURN_SPEED,
                                -self.TURN_SPEED, -self.TURN_SPEED)
            # Update direction
            self.current_direction = (self.current_direction + 1) % 4

        time.sleep(duration)
        PWM.set_motor_model(0, 0, 0, 0)
    
    def safety_check(self):
        """Perform a safety check to avoid collisions"""
        # Quick scan directly in front to check if there's an obstacle
        servo.set_servo_pwm('0', self.ANG_FRONT)  # Center the servo
        time.sleep(0.2)  # Brief delay for servo to move
        
        distance = self.median_distance()
        if distance < 15:  # Emergency stop if obstacle is very close
            print(f"EMERGENCY STOP - Obstacle detected at {distance}cm")
            return False
        return True

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
            # Safety check before straight moves
            if direction is Dir.STRAIGHT and not self.safety_check():
                print("Safety check failed, choosing a different direction")
                # Attempt to find a safe direction
                left_safe = True
                right_safe = True
                
                # Check left
                servo.set_servo_pwm('0', self.ANG_LEFT_FAR)
                time.sleep(0.2)
                if self.median_distance() < 15:
                    left_safe = False
                
                # Check right
                servo.set_servo_pwm('0', self.ANG_RIGHT_FAR)
                time.sleep(0.2)
                if self.median_distance() < 15:
                    right_safe = False
                
                # Reset servo position
                servo.set_servo_pwm('0', self.ANG_FRONT)
                
                # Choose a safe direction
                if left_safe:
                    direction = Dir.LEFT
                elif right_safe:
                    direction = Dir.RIGHT
                else:
                    direction = Dir.BACK
            
            if direction is Dir.STRAIGHT:
                self.go_straight()
                
            elif direction is Dir.LEFT:
                self.turn(left=True)
                if self.safety_check():
                    self.go_straight()
                else:
                    print("Obstacle detected after turn, stopping")
                
            elif direction is Dir.RIGHT:
                self.turn(left=False)
                if self.safety_check():
                    self.go_straight()
                else:
                    print("Obstacle detected after turn, stopping")
                
            elif direction is Dir.BACK:
                self.turn(left=True)  # Turn 90 degrees left
                self.turn(left=True)  # Turn another 90 degrees left (total 180)
                if self.safety_check():
                    self.go_straight()
                else:
                    print("Obstacle detected after turn, stopping")
                
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
        #print("Scan results:", scan_results) # Original print

        # Update the maze map based on scan results
        self.update_maze_map()

        # Choose next direction based on new algorithm
        direction = self.get_direction(scan_results)

        # Debug print for real-time insight
        scan = scan_results # Use the local variable for clarity
        front_dist = scan.get(self.ANG_FRONT, 0)
        left_readings = [scan.get(a, 0) for a in self.SCAN_ANGLES if a < self.ANG_FRONT and a in scan]
        right_readings = [scan.get(a, 0) for a in self.SCAN_ANGLES if a > self.ANG_FRONT and a in scan]
        max_left = max(left_readings) if left_readings else 0
        max_right = max(right_readings) if right_readings else 0

        print(
            f"F:{front_dist:>5.1f} | L max:{max_left:>5.1f} ({len(left_readings)}) | R max:{max_right:>5.1f} ({len(right_readings)}) -> {direction.name}"
        )

        # direction_strings = {
        #     Dir.STRAIGHT: 'straight',
        #     Dir.LEFT: 'left',
        #     Dir.RIGHT: 'right',
        #     Dir.BACK: 'back'
        # }
        # print(f"Moving {direction_strings[direction]}") # Original print

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
