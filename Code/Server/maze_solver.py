import time
from motor import *
from gpiozero import LineSensor
from ultrasonic import Ultrasonic
from buzzer import Buzzer
import math

# Define sensor pins
IR01 = 14  # Left sensor
IR02 = 15  # Middle sensor
IR03 = 23  # Right sensor

# Initialize sensors
IR01_sensor = LineSensor(IR01)
IR02_sensor = LineSensor(IR02)
IR03_sensor = LineSensor(IR03)
ultrasonic = Ultrasonic()
buzzer = Buzzer()

class MazeSolver:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.set_pwm_freq(50)
        
        # Movement parameters (using proven values from your code)
        self.base_speed = 800  # Base speed for line following
        self.turn_speed = 1500  # Speed for turns
        self.sharp_turn_speed = 2000  # Speed for sharp turns
        self.max_turn_speed = 4000  # Maximum turn speed
        
        # Maze solving variables
        self.grid_size = 20  # cm per grid cell
        self.current_position = (0, 0)  # (x, y) in grid coordinates
        self.current_direction = 0  # 0: North, 1: East, 2: South, 3: West
        self.maze_map = {}  # Stores discovered walls
        self.path = []  # Current A* path
        self.in_maze = False  # Whether we're in the maze or still following the line
        self.line_lost_count = 0  # Count how many times we've lost the line
        self.max_line_lost = 5  # Number of times to try finding line before entering maze mode

    def duty_range(self, duty1, duty2, duty3, duty4):
        if duty1 > 4095:
            duty1 = 4095
        elif duty1 < -4095:
            duty1 = -4095        
        if duty2 > 4095:
            duty2 = 4095
        elif duty2 < -4095:
            duty2 = -4095  
        if duty3 > 4095:
            duty3 = 4095
        elif duty3 < -4095:
            duty3 = -4095
        if duty4 > 4095:
            duty4 = 4095
        elif duty4 < -4095:
            duty4 = -4095
        return duty1, duty2, duty3, duty4

    def left_upper_wheel(self, duty):
        if duty > 0:
            self.pwm.set_motor_pwm(0, 0)
            self.pwm.set_motor_pwm(1, duty)
        elif duty < 0:
            self.pwm.set_motor_pwm(1, 0)
            self.pwm.set_motor_pwm(0, abs(duty))
        else:
            self.pwm.set_motor_pwm(0, 4095)
            self.pwm.set_motor_pwm(1, 4095)

    def left_lower_wheel(self, duty):
        if duty > 0:
            self.pwm.set_motor_pwm(3, 0)
            self.pwm.set_motor_pwm(2, duty)
        elif duty < 0:
            self.pwm.set_motor_pwm(2, 0)
            self.pwm.set_motor_pwm(3, abs(duty))
        else:
            self.pwm.set_motor_pwm(2, 4095)
            self.pwm.set_motor_pwm(3, 4095)

    def right_upper_wheel(self, duty):
        if duty > 0:
            self.pwm.set_motor_pwm(6, 0)
            self.pwm.set_motor_pwm(7, duty)
        elif duty < 0:
            self.pwm.set_motor_pwm(7, 0)
            self.pwm.set_motor_pwm(6, abs(duty))
        else:
            self.pwm.set_motor_pwm(6, 4095)
            self.pwm.set_motor_pwm(7, 4095)

    def right_lower_wheel(self, duty):
        if duty > 0:
            self.pwm.set_motor_pwm(4, 0)
            self.pwm.set_motor_pwm(5, duty)
        elif duty < 0:
            self.pwm.set_motor_pwm(5, 0)
            self.pwm.set_motor_pwm(4, abs(duty))
        else:
            self.pwm.set_motor_pwm(4, 4095)
            self.pwm.set_motor_pwm(5, 4095)

    def set_motor_model(self, duty1, duty2, duty3, duty4):
        duty1, duty2, duty3, duty4 = self.duty_range(duty1, duty2, duty3, duty4)
        self.left_upper_wheel(duty1)
        self.left_lower_wheel(duty2)
        self.right_upper_wheel(duty3)
        self.right_lower_wheel(duty4)

    def manhattan_distance(self, pos1, pos2):
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def get_neighbors(self, pos):
        x, y = pos
        neighbors = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            new_pos = (x + dx, y + dy)
            if new_pos not in self.maze_map.get(pos, []):
                neighbors.append(new_pos)
        return neighbors

    def a_star_search(self, start, goal):
        frontier = [(0, start)]
        came_from = {start: None}
        g_score = {start: 0}
        f_score = {start: self.manhattan_distance(start, goal)}

        while frontier:
            current = frontier.pop(0)[1]
            if current == goal:
                path = []
                while current is not None:
                    path.append(current)
                    current = came_from[current]
                return path[::-1]

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.manhattan_distance(neighbor, goal)
                    if neighbor not in [i[1] for i in frontier]:
                        frontier.append((f_score[neighbor], neighbor))
            frontier.sort()

        return None

    def follow_line(self):
        """Follow the black line using the proven line following logic"""
        # Check for obstacles first
        d = ultrasonic.get_distance()
        if d is not None and d <= 50:
            buzzer.run('1')
            time.sleep(0.2)
            buzzer.run('0')
            self.set_motor_model(0, 0, 0, 0)
            time.sleep(0.5)
            return

        # Read infrared sensors using your proven logic
        self.LMR = 0x00
        if IR01_sensor.value:
            self.LMR = (self.LMR | 4)
        if IR02_sensor.value:
            self.LMR = (self.LMR | 2)
        if IR03_sensor.value:
            self.LMR = (self.LMR | 1)

        # Use your proven motor control logic
        if self.LMR == 2:  # Only middle sensor detects line
            self.set_motor_model(self.base_speed, self.base_speed, self.base_speed, self.base_speed)
            self.line_lost_count = 0
        elif self.LMR == 4:  # Middle and right sensors detect line
            self.set_motor_model(-self.turn_speed, -self.turn_speed, self.turn_speed, self.turn_speed)
        elif self.LMR == 6:  # Right sensor detects line
            self.set_motor_model(-self.sharp_turn_speed, -self.sharp_turn_speed, self.max_turn_speed, self.max_turn_speed)
        elif self.LMR == 1:  # Middle and left sensors detect line
            self.set_motor_model(self.turn_speed, self.turn_speed, -self.turn_speed, -self.turn_speed)
        elif self.LMR == 3:  # Left sensor detects line
            self.set_motor_model(self.max_turn_speed, self.max_turn_speed, -self.sharp_turn_speed, -self.sharp_turn_speed)
        elif self.LMR == 7:  # All sensors detect line (intersection)
            self.set_motor_model(0, 0, 0, 0)  # Stop at intersection
            time.sleep(0.5)
            self.in_maze = True  # Transition to maze solving
            print("Transitioning to maze solving mode")
        else:  # No line detected
            self.line_lost_count += 1
            if self.line_lost_count < self.max_line_lost:
                # Search for line by turning right
                self.set_motor_model(-self.turn_speed, -self.turn_speed, self.turn_speed, self.turn_speed)
            else:
                # Line lost for too long, transition to maze solving
                self.in_maze = True
                print("Transitioning to maze solving mode")
                self.set_motor_model(0, 0, 0, 0)
                time.sleep(1)

    def update_maze_map(self):
        """Update the maze map based on ultrasonic sensor readings"""
        # Check left
        ultrasonic.trigger_pin = 27
        ultrasonic.echo_pin = 22
        left_dist = ultrasonic.get_distance()
        
        # Check front
        ultrasonic.trigger_pin = 5
        ultrasonic.echo_pin = 6
        front_dist = ultrasonic.get_distance()
        
        # Check right
        ultrasonic.trigger_pin = 13
        ultrasonic.echo_pin = 19
        right_dist = ultrasonic.get_distance()
        
        # Update maze map with detected walls
        if left_dist is not None and left_dist < 30:
            wall_pos = (self.current_position[0] - 1, self.current_position[1])
            if self.current_position not in self.maze_map:
                self.maze_map[self.current_position] = []
            if wall_pos not in self.maze_map[self.current_position]:
                self.maze_map[self.current_position].append(wall_pos)
            
        if front_dist is not None and front_dist < 30:
            wall_pos = (self.current_position[0], self.current_position[1] + 1)
            if self.current_position not in self.maze_map:
                self.maze_map[self.current_position] = []
            if wall_pos not in self.maze_map[self.current_position]:
                self.maze_map[self.current_position].append(wall_pos)
            
        if right_dist is not None and right_dist < 30:
            wall_pos = (self.current_position[0] + 1, self.current_position[1])
            if self.current_position not in self.maze_map:
                self.maze_map[self.current_position] = []
            if wall_pos not in self.maze_map[self.current_position]:
                self.maze_map[self.current_position].append(wall_pos)

    def solve_maze(self):
        """Navigate through the maze using A* algorithm"""
        # First check for immediate obstacles
        distance = ultrasonic.get_distance()
        if distance is not None and distance < 30:
            self.set_motor_model(0, 0, 0, 0)
            buzzer.run('1')
            time.sleep(0.2)
            buzzer.run('0')
            self.set_motor_model(-self.turn_speed, -self.turn_speed, self.turn_speed, self.turn_speed)
            time.sleep(0.5)
            return

        # Update maze map and find path
        self.update_maze_map()
        
        if not self.path:
            goal_position = (5, 5)  # Example goal position
            self.path = self.a_star_search(self.current_position, goal_position)
        
        if self.path:
            next_pos = self.path[0]
            dx = next_pos[0] - self.current_position[0]
            dy = next_pos[1] - self.current_position[1]
            
            # Determine required turn
            required_direction = 0
            if dx == 1: required_direction = 1
            elif dx == -1: required_direction = 3
            elif dy == 1: required_direction = 0
            elif dy == -1: required_direction = 2
            
            # Execute turn if needed
            turn_diff = (required_direction - self.current_direction) % 4
            if turn_diff == 1:  # Turn right
                self.set_motor_model(-self.turn_speed, -self.turn_speed, self.turn_speed, self.turn_speed)
                time.sleep(0.3)
            elif turn_diff == 3:  # Turn left
                self.set_motor_model(self.turn_speed, self.turn_speed, -self.turn_speed, -self.turn_speed)
                time.sleep(0.3)
            elif turn_diff == 2:  # Turn around
                self.set_motor_model(-self.turn_speed, -self.turn_speed, self.turn_speed, self.turn_speed)
                time.sleep(0.6)
            
            self.current_direction = required_direction
            
            # Move forward one grid cell
            self.set_motor_model(self.base_speed, self.base_speed, self.base_speed, self.base_speed)
            time.sleep(0.8)
            
            # Update position
            self.current_position = next_pos
            self.path.pop(0)
        else:
            # No path found, turn right to search
            self.set_motor_model(-self.turn_speed, -self.turn_speed, self.turn_speed, self.turn_speed)
            time.sleep(0.3)

    def run(self):
        try:
            print("Starting maze solver...")
            print("First following the line...")
            
            while True:
                if not self.in_maze:
                    self.follow_line()
                else:
                    self.solve_maze()
                time.sleep(0.05)  # Small delay to prevent overwhelming the system

        except KeyboardInterrupt:
            print("\nStopping maze solver...")
            self.set_motor_model(0, 0, 0, 0)
            self.pwm.close()

if __name__ == '__main__':
    print('Program is starting...')
    maze_solver = MazeSolver()
    maze_solver.run() 