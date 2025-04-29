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
import os
from pathlib import Path
import sys

# Add parent directory to path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

# Import new components
from geometry import CELL_CM, COLS, ROWS, WALL_THICKCM, START_CELL, EXIT_CELL, DX, DY
from calibration import load_calibration, save_calibration, calibrate_straight_movement
from pose import Pose
from maze_blueprint import create_competition_maze, blueprint_to_wall_grids, wall_grids_to_occupancy, blueprint_to_occupancy

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
        
        # Load calibration values
        calib = load_calibration()
        
        # Constants for movement from calibration
        self.BASE_SPEED = calib["BASE_SPEED"]
        self.TURN_SPEED = calib["TURN_SPEED"]
        self.TURN_DURATION_90 = calib["TURN_DURATION_90"]
        self.sec_per_cell = calib["SEC_PER_CELL"]
        
        # Use Pose class for tracking position
        self.pose = Pose(START_CELL, 0)  # Start at (0,4) facing North (0)
        
        # Maze solving variables
        self.maze_map = {}  # Dictionary: {(x,y): [north_wall, east_wall, south_wall, west_wall]}
        self.visited_cells = set()  # Set of visited cells
        self.path = []  # Current A* path
        self.target_position = None  # Target position for pathfinding
        
        # Wall grid representation (for advanced navigation)
        self.h_walls = None  # Horizontal walls
        self.v_walls = None  # Vertical walls
        self.occupancy_grid = None  # Occupancy grid
        
        # Stuck detection
        self.last_positions = deque(maxlen=8)
        
        # Servo scanning parameters
        self.SCAN_ANGLES = [15, 45, 75, 90, 105, 135, 165] # 7 points
        self.SERVO_MOVE_TIME = 0.5 # seconds for servo to settle
        self.READING_PAUSE = 0.05 # seconds after reading
        
        # Define constant angle names for clarity in code
        self.ANG_LEFT_FAR = 15
        self.ANG_LEFT_MID = 45
        self.ANG_LEFT_NEAR = 75
        self.ANG_FRONT = 90
        self.ANG_RIGHT_NEAR = 105
        self.ANG_RIGHT_MID = 135
        self.ANG_RIGHT_FAR = 165
        self.scan_results = {}  # Store scan results {angle: distance}
        
        # Wall detection thresholds based on cell size
        self.FRONT_CLEAR_CM = 0.9 * CELL_CM  # ~30 cm
        self.SIDE_CLEAR_CM = 0.6 * CELL_CM   # ~20 cm
        self.MAP_WALL_THRESHOLD_CM = 0.7 * CELL_CM  # ~23 cm
        
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
        
    def align_to_walls(self):
        """Align the robot perpendicular to walls before scanning"""
        if self.simulation_mode:
            time.sleep(0.2)  # Simulate alignment time
            return
            
        # Look left
        servo.set_servo_pwm('0', self.ANG_LEFT_FAR)
        time.sleep(0.3)
        left = self.median_distance()
        
        # Look right
        servo.set_servo_pwm('0', self.ANG_RIGHT_FAR)
        time.sleep(0.3)
        right = self.median_distance()
        
        # Center servo
        servo.set_servo_pwm('0', self.ANG_FRONT)
        time.sleep(0.2)
        
        # Check if alignment is needed
        if abs(left - right) > 3:  # >3 cm skew
            print(f"Aligning: Left={left:.1f}cm, Right={right:.1f}cm")
            if left > right:
                self.nudge_left()
            else:
                self.nudge_right()
                
    def nudge_left(self):
        """Small correction to the left"""
        if self.simulation_mode:
            time.sleep(0.1)
            return
            
        # Short low-speed pulse on right wheels
        PWM.set_motor_model(0, 0, 800, 800)
        time.sleep(0.1)
        PWM.set_motor_model(0, 0, 0, 0)
        
    def nudge_right(self):
        """Small correction to the right"""
        if self.simulation_mode:
            time.sleep(0.1)
            return
            
        # Short low-speed pulse on left wheels
        PWM.set_motor_model(800, 800, 0, 0)
        time.sleep(0.1)
        PWM.set_motor_model(0, 0, 0, 0)
        
    def scan_environment(self):
        """Scan the environment using the servo-mounted ultrasonic sensor across defined angles."""
        # First align to walls for more accurate scanning
        self.align_to_walls()
        
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

        # Reset servo to center position after scan
        servo.set_servo_pwm('0', self.ANG_FRONT) # Reset to front (90)
        time.sleep(0.2)
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
        x, y = self.pose.cell
        
        # Update visited cells
        self.visited_cells.add(self.pose.cell)
        
        # Initialize cell in maze_map if not already present
        if self.pose.cell not in self.maze_map:
            # [north_wall, east_wall, south_wall, west_wall]
            self.maze_map[self.pose.cell] = [False, False, False, False]
            
        scan = self.scan_results
        
        # Determine wall presence based on scan results and thresholds
        # Front wall
        front_dist = scan.get(self.ANG_FRONT, 0)
        front_wall = front_dist < self.MAP_WALL_THRESHOLD_CM
        
        # Left wall (minimum of left readings)
        left_readings = [scan.get(a, 0) for a in self.SCAN_ANGLES if a < self.ANG_FRONT and a in scan]
        min_left = min(left_readings) if left_readings else float('inf')
        left_wall = min_left < self.MAP_WALL_THRESHOLD_CM
        
        # Right wall (minimum of right readings)
        right_readings = [scan.get(a, 0) for a in self.SCAN_ANGLES if a > self.ANG_FRONT and a in scan]
        min_right = min(right_readings) if right_readings else float('inf')
        right_wall = min_right < self.MAP_WALL_THRESHOLD_CM
        
        # Map relative walls to absolute directions
        walls = {
            0: front_wall,  # Front
            1: right_wall,  # Right
            3: left_wall    # Left
        }
        
        # Update walls in current cell and adjacent cells
        for rel_dir, has_wall in walls.items():
            # Calculate absolute direction
            abs_dir = (self.pose.heading + rel_dir) % 4
            
            # Update wall in current cell
            self.maze_map[self.pose.cell][abs_dir] = has_wall
            
            # If there's a wall, update the adjacent cell too
            if has_wall:
                # Calculate adjacent cell coordinates
                adj_x = x + DX[abs_dir]
                adj_y = y + DY[abs_dir]
                adj_pos = (adj_x, adj_y)
                
                # Initialize adjacent cell if needed
                if adj_pos not in self.maze_map:
                    self.maze_map[adj_pos] = [False, False, False, False]
                
                # Update the opposite wall in the adjacent cell
                opposite_dir = (abs_dir + 2) % 4
                self.maze_map[adj_pos][opposite_dir] = True
        
        # Update wall grid representation if available
        if self.h_walls is not None and self.v_walls is not None:
            self._update_wall_grids()
    
    def _update_wall_grids(self):
        """Update the wall grid representation from the maze map"""
        # For simplicity, just regenerate the wall grids
        blueprint = [[0 for _ in range(COLS)] for _ in range(ROWS)]
        
        for (x, y), walls in self.maze_map.items():
            if 0 <= x < COLS and 0 <= y < ROWS:
                cell_code = 0
                # Convert walls to bitmap
                if walls[0]: cell_code |= 1  # North
                if walls[1]: cell_code |= 2  # East
                if walls[2]: cell_code |= 4  # South
                if walls[3]: cell_code |= 8  # West
                blueprint[y][x] = cell_code
        
        # Convert to wall grids
        self.h_walls, self.v_walls = blueprint_to_wall_grids(blueprint)
        # Update occupancy grid
        self.occupancy_grid = wall_grids_to_occupancy(self.h_walls, self.v_walls)
    
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
    
    def a_star_on_occupancy_grid(self, start, goal):
        """A* pathfinding on the occupancy grid for more precise paths
        
        Args:
            start: Start position in grid coordinates (x, y)
            goal: Goal position in grid coordinates (x, y)
            
        Returns:
            List of coordinates forming the path
        """
        if self.occupancy_grid is None:
            print("Occupancy grid not available")
            return self.a_star_pathfinding(start, goal)
            
        # Convert cell coordinates to occupancy grid coordinates (center points)
        start_node = (start[1] * 2 + 1, start[0] * 2 + 1)  # (row, col) format
        goal_node = (goal[1] * 2 + 1, goal[0] * 2 + 1)     # (row, col) format
        
        # Check if start or goal is a wall (shouldn't happen)
        if self.occupancy_grid[start_node[0], start_node[1]] == 1:
            print("Start position is inside a wall!")
            return []
            
        if self.occupancy_grid[goal_node[0], goal_node[1]] == 1:
            print("Goal position is inside a wall!")
            return []
            
        # Set up A* search
        open_set = PriorityQueue()
        open_set.put((0, start_node))
        came_from = {start_node: None}
        g_score = {start_node: 0}
        f_score = {start_node: self.heuristic_grid(start_node, goal_node)}
        
        grid_shape = self.occupancy_grid.shape
        open_set_hash = {start_node}
        
        # Four-way connectivity (N, E, S, W)
        directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
        
        while not open_set.empty():
            current = open_set.get()[1]
            open_set_hash.remove(current)
            
            if current == goal_node:
                # Reconstruct path
                path = []
                while current:
                    # Convert grid coordinates back to cell coordinates
                    cell_coord = (current[1] // 2, current[0] // 2)
                    if not path or cell_coord != path[-1]:  # Avoid duplicates
                        path.append(cell_coord)
                    current = came_from[current]
                return path[::-1]  # Return reversed path
                
            # Explore neighbors
            for dr, dc in directions:
                nr, nc = current[0] + dr, current[1] + dc
                
                # Check bounds
                if not (0 <= nr < grid_shape[0] and 0 <= nc < grid_shape[1]):
                    continue
                    
                # Skip walls
                if self.occupancy_grid[nr, nc] == 1:
                    continue
                    
                neighbor = (nr, nc)
                
                # Calculate tentative g_score
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # This path is better
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic_grid(neighbor, goal_node)
                    
                    if neighbor not in open_set_hash:
                        open_set.put((f_score[neighbor], neighbor))
                        open_set_hash.add(neighbor)
                        
        return []  # No path found
                
    def heuristic(self, a, b):
        """Manhattan distance heuristic for A*"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def heuristic_grid(self, a, b):
        """Manhattan distance heuristic for grid coordinates"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def get_next_exploration_target(self):
        """Choose the next unexplored cell to target"""
        # Find cells that are in the maze_map but not visited
        unexplored = set(self.maze_map.keys()) - self.visited_cells
        
        if not unexplored:
            # If all known cells are visited, return None
            return None
            
        # Find the closest unexplored cell
        current = self.pose.cell
        closest = min(unexplored, key=lambda pos: self.heuristic(current, pos))
        
        return closest
    
    # Movement primitives
    def go_straight(self, cells=1):
        """Move straight ahead for the specified number of cells"""
        if self.simulation_mode:
            self.pose.update_after_straight(cells)
            time.sleep(self.sec_per_cell * cells)  # Simulate movement time
            return
            
        # Calculate movement time based on calibration
        run_time = self.sec_per_cell * cells
        
        # Move forward at calibrated speed
        self.pwm.set_motor_model(self.BASE_SPEED, self.BASE_SPEED, 
                                 self.BASE_SPEED, self.BASE_SPEED)
        time.sleep(run_time)
        self.pwm.set_motor_model(0, 0, 0, 0)  # Stop
        
        # Update position
        self.pose.update_after_straight(cells)
        
    def turn(self, left=True):
        """Turn 90 degrees in place"""
        if self.simulation_mode:
            self.pose.update_after_turn(left)
            time.sleep(self.TURN_DURATION_90)  # Simulate turning time
            return

        # Set motors based on turn direction
        if left:
            self.pwm.set_motor_model(-self.TURN_SPEED, -self.TURN_SPEED,
                                     self.TURN_SPEED, self.TURN_SPEED)
        else:
            self.pwm.set_motor_model(self.TURN_SPEED, self.TURN_SPEED,
                                    -self.TURN_SPEED, -self.TURN_SPEED)

        # Wait for turn duration
        time.sleep(self.TURN_DURATION_90 * 0.8 if left else self.TURN_DURATION_90 * 1.2)
        self.pwm.set_motor_model(0, 0, 0, 0)  # Stop
        
        # Update pose heading
        self.pose.update_after_turn(left)
    
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
        self.last_positions.append(self.pose.cell)
        
        # Record current position as visited
        self.visited_cells.add(self.pose.cell)
        
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
            self.pwm.set_motor_model(0, 0, 0, 0)
    
    def recovery_spin(self):
        """Recovery behavior when stuck in a loop"""
        print("Detected stuck in a loop - performing recovery spin")
        # Turn 90° right and rescan
        self.turn(left=False)
        # Clear stuck detection
        self.last_positions.clear()
    
    def follow_path(self, path):
        """Follow a given path using the robot's movement capabilities"""
        if not path or len(path) < 2:
            print("Path too short to follow")
            return
            
        # Start from the second point (first is current position)
        for next_pos in path[1:]:
            # Get command to move to the next position
            move_command = self._command_to(next_pos)
                
            # Execute movement
            self.move(move_command)
            
    def _command_to(self, next_pos):
        """Calculate the command needed to move to the next position"""
        # Calculate direction to move
        curr_x, curr_y = self.pose.cell
        next_x, next_y = next_pos
        
        # Calculate target absolute direction
        if next_x > curr_x:  # East
            target_direction = 1
        elif next_x < curr_x:  # West
            target_direction = 3
        elif next_y > curr_y:  # North
            target_direction = 0
        else:  # South
            target_direction = 2
            
        # Calculate turn needed
        turn = (target_direction - self.pose.heading) % 4
        
        # Convert turn to movement command
        if turn == 0:  # No turn needed
            move_command = Dir.STRAIGHT
        elif turn == 1:  # 90 degrees right
            move_command = Dir.RIGHT
        elif turn == 2:  # 180 degrees
            move_command = Dir.BACK
        elif turn == 3:  # 90 degrees left
            move_command = Dir.LEFT
            
        return move_command
    
    def run_step(self):
        """Run a single step of the maze solving algorithm"""
        # Scan the environment
        scan_results = self.scan_environment()

        # Update the maze map based on scan results
        self.update_maze_map()

        # Choose next direction based on scan results
        direction = self.get_direction(scan_results)

        # Debug print for real-time insight
        scan = scan_results
        front_dist = scan.get(self.ANG_FRONT, 0)
        left_readings = [scan.get(a, 0) for a in self.SCAN_ANGLES if a < self.ANG_FRONT and a in scan]
        right_readings = [scan.get(a, 0) for a in self.SCAN_ANGLES if a > self.ANG_FRONT and a in scan]
        max_left = max(left_readings) if left_readings else 0
        max_right = max(right_readings) if right_readings else 0

        print(
            f"POS:{self.pose.cell} | F:{front_dist:>5.1f} | L:{max_left:>5.1f} | R:{max_right:>5.1f} -> {direction.name}"
        )

        # Move in the chosen direction
        self.move(direction)

        return direction
    
    def run_exploration(self, steps=10):
        """Run exploration for a set number of steps"""
        for i in range(steps):
            print(f"Step {i+1}/{steps}")
            self.run_step()
            time.sleep(0.5)  # Small delay between steps
    
    def run_to_target(self, target_position):
        """Run to a specific target position using A* pathfinding"""
        # Find path to target
        if self.occupancy_grid is not None:
            # Use grid-based A* if available
            print(f"Using occupancy grid A* to find path to {target_position}")
            path = self.a_star_on_occupancy_grid(self.pose.cell, target_position)
        else:
            # Fall back to regular A*
            print(f"Using regular A* to find path to {target_position}")
            path = self.a_star_pathfinding(self.pose.cell, target_position)
        
        if not path:
            print(f"No path found to target {target_position}")
            return False
            
        # Save and display the path
        self.path = path
        print(f"Path found: {path}")
        
        # Follow the path
        self.follow_path(path)
        
        return True
    
    def preload_maze(self, blueprint=None):
        """Preload a known maze layout from a blueprint or the competition maze"""
        if blueprint is None:
            # Load the competition maze layout
            blueprint = create_competition_maze()
            
        # Convert to wall grids
        self.h_walls, self.v_walls = blueprint_to_wall_grids(blueprint)
        self.occupancy_grid = wall_grids_to_occupancy(self.h_walls, self.v_walls)
        
        # Populate the maze_map from the blueprint
        for y in range(ROWS):
            for x in range(COLS):
                code = blueprint[y][x]
                walls = [False] * 4
                
                # Convert bitmap to wall array
                if code & 1: walls[0] = True  # North
                if code & 2: walls[1] = True  # East
                if code & 4: walls[2] = True  # South
                if code & 8: walls[3] = True  # West
                
                self.maze_map[(x, y)] = walls
        
        print(f"Maze preloaded with {len(self.maze_map)} cells")
    
    def save_maze_map(self, filename):
        """Save the maze map to a JSON file"""
        # Convert maze_map keys to strings for JSON serialization
        serializable_map = {str(k): v for k, v in self.maze_map.items()}
        
        data = {
            "maze_map": serializable_map,
            "visited_cells": list(map(list, self.visited_cells)),
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
            
            # Regenerate the wall grids
            self._update_wall_grids()
            
            print(f"Maze map loaded from {filename}")
            return True
        except Exception as e:
            print(f"Error loading maze map: {e}")
            return False
    
    def calibrate(self):
        """Run calibration for movement parameters"""
        if self.simulation_mode:
            print("Cannot calibrate in simulation mode")
            return
            
        print("Starting movement calibration...")
        
        # Load current calibration
        calib = load_calibration()
        
        # Calibrate straight movement (time per cell)
        print("\n=== Straight Movement Calibration ===")
        sec_per_cell = calibrate_straight_movement(self.pwm, calib["BASE_SPEED"])
        
        # Save updated calibration
        calib["SEC_PER_CELL"] = sec_per_cell
        save_calibration(calib)
        
        # Update local parameters
        self.sec_per_cell = sec_per_cell
        
        print(f"Calibration complete! SEC_PER_CELL = {sec_per_cell:.3f}s")
        

if __name__ == '__main__':
    import argparse
    try:
        import RPi.GPIO as GPIO
        HAS_GPIO = True
    except (ImportError, RuntimeError):
        print("RPi.GPIO not available. Assuming simulation or testing environment.")
        HAS_GPIO = False

    parser = argparse.ArgumentParser(description='Maze Solver')
    parser.add_argument('--sim', action='store_true', help='Run in simulation mode (no hardware)')
    parser.add_argument('--load', type=str, help='Load maze map from file')
    parser.add_argument('--steps', type=int, default=20, help='Number of exploration steps')
    parser.add_argument('--calibrate', action='store_true', help='Run calibration procedure')
    parser.add_argument('--exit', action='store_true', help='Go directly to exit after exploration')
    parser.add_argument('--preload', action='store_true', help='Preload the competition maze')
    
    args = parser.parse_args()
    
    # Create maze solver
    maze_solver = MazeSolver()
    
    # Set simulation mode if requested
    if args.sim:
        print("Running in simulation mode")
        maze_solver.set_simulation_mode(True)
    
    # Run calibration if requested
    if args.calibrate:
        maze_solver.calibrate()
        exit(0)
    
    # Preload maze if requested
    if args.preload:
        print("Preloading competition maze")
        maze_solver.preload_maze()
    
    # Load maze map if requested
    elif args.load:
        maze_solver.load_maze_map(args.load)
    
    try:
        # If we have a preloaded maze, just go to the exit
        if args.preload or (args.load and args.exit):
            print(f"Navigating directly to exit at {EXIT_CELL}...")
            maze_solver.run_to_target(EXIT_CELL)
            print("Exit reached!")
        else:
            # Run exploration
            print("Starting maze exploration...")
            maze_solver.run_exploration(args.steps)
            print("Exploration finished.")
            
            # Go to exit if requested
            if args.exit:
                print(f"Navigating to exit at {EXIT_CELL}...")
                maze_solver.run_to_target(EXIT_CELL)
                print("Exit reached!")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        print("Saving final maze map...")
        # Save the map regardless of how we exit
        maze_solver.save_maze_map('maze_map.json')

        # Cleanup hardware resources
        if not args.sim and HAS_GPIO:
            print("Stopping motors and cleaning up GPIO...")
            try:
                # Ensure motors are stopped first
                maze_solver.pwm.set_motor_model(0, 0, 0, 0)
                time.sleep(0.1) # Short pause
                # Clean up all GPIO channels
                GPIO.cleanup()
                print("GPIO cleanup complete.")
            except Exception as cleanup_error:
                print(f"Error during GPIO cleanup: {cleanup_error}")
        elif args.sim:
            print("Simulation mode: No GPIO cleanup needed.")
        else:
            print("GPIO library not available: Skipping cleanup.")

        print("Program terminated.")
