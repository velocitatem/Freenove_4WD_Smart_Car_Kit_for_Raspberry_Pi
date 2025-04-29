import time
from motor import *
from infrared import Infrared
from ultrasonic import Ultrasonic
from buzzer import Buzzer
from servo import Servo
from motor import Ordinary_Car
import math

# Initialize sensors
infrared = Infrared()
ultrasonic = Ultrasonic()
buzzer = Buzzer()
servo = Servo()
PWM = Ordinary_Car()
class MazeSolver:
    def __init__(self):
        self.pwm = PWM
        
        # Maze solving variables
        self.grid_size = 20  # cm per grid cell
        self.current_position = (0, 0)  # (x, y) in grid coordinates
        self.current_direction = 0  # 0: North, 1: East, 2: South, 3: West
        self.maze_map = {}  # Stores discovered walls
        self.path = []  # Current A* path
        self.in_maze = False  # Whether we're in the maze or still following the line
        
        # Line tracking variables
        self.last_direction = None
        self.stuck_counter = 0
        self.max_stuck_count = 5
        
        # Servo scanning parameters
        self.scan_angles = [0, 90, 180]  # Servo angles to scan
        self.scan_results = {}  # Store scan results
        self.current_scan_index = 0

    def scan_environment(self):
        """Scan the environment using the servo-mounted ultrasonic sensor"""
        self.scan_results = {}  # Clear previous results
        # results format: {angle: distance}

        
        for angle in self.scan_angles:
            # Move servo to scan angle
            servo.set_servo_pwm('0', angle)
            time.sleep(0.5)  # Wait for servo to move
            
            # Take multiple measurements and average them
            measurements = []
            for _ in range(3):  # Take 3 measurements
                distance = ultrasonic.get_distance()
                if distance is not None:
                    measurements.append(distance)
                    print(f"{angle}: {distance}")
            
            # Calculate average distance
            average_distance = sum(measurements) / len(measurements)
            
            # Store the result
            self.scan_results[angle] = average_distance
            
            # Move servo to next angle
            time.sleep(0.5)  # Wait for servo to move
            
        # Reset servo to center position
        servo.set_servo_pwm('0', 90)
        time.sleep(0.2)

        return self.scan_results
    
    def get_direction(self, scan_results):
        """Determine the direction to move based on the scan results"""
        # see if we can go straight
        DISTANCE = 50
        if scan_results[90] > DISTANCE:
            return 0
        # see if we can go right
        if scan_results[180] > DISTANCE:
            return 1
        # see if we can go left
        if scan_results[0] > DISTANCE:
            return 2
        # if no clear path, move back
        return -1
    
    def move(self, direction: int, distance: int = 1):
        """
        Move the car in a specified direction for a given distance.
        
        Args:
            direction: 0=straight, 1=left, 2=right, -1=back
            distance: Number of grid cells to move (default=1)
        """
        # Base speeds for different movements
        BASE_SPEED = 800
        TURN_SPEED = 1000  # Reduced for more precise turns
        SHARP_TURN_SPEED = 1500
        MAX_TURN_SPEED = 2000
        
        # Movement duration based on distance
        MOVE_DURATION = 0.5 * distance  # seconds per grid cell
        TURN_DURATION = 0.125  # seconds for 22.5-degree turn (0.5s / 4)
        
        try:
            if direction == 0:  # Straight
                # Use proven forward speed from car.py
                PWM.set_motor_model(BASE_SPEED, BASE_SPEED, BASE_SPEED, BASE_SPEED)
                time.sleep(MOVE_DURATION)
                
            elif direction == 2:  # Left # THIS IS RIGHT
                # Use proven left turn pattern from car.py
                PWM.set_motor_model(TURN_SPEED, TURN_SPEED, -TURN_SPEED, -TURN_SPEED)
                time.sleep(TURN_DURATION)
                # Move forward after turn
                PWM.set_motor_model(BASE_SPEED, BASE_SPEED, BASE_SPEED, BASE_SPEED)
                time.sleep(MOVE_DURATION)
                
            elif direction == 1:  # Right
                # Use proven right turn pattern from car.py
                PWM.set_motor_model(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED)
                time.sleep(TURN_DURATION)
                # Move forward after turn
                PWM.set_motor_model(BASE_SPEED, BASE_SPEED, BASE_SPEED, BASE_SPEED)
                time.sleep(MOVE_DURATION)
                
            elif direction == -1:  # Back
                # Use proven backward pattern from car.py
                PWM.set_motor_model(-BASE_SPEED, -BASE_SPEED, -BASE_SPEED, -BASE_SPEED)
                time.sleep(MOVE_DURATION)
                
            # Stop the car after movement
            PWM.set_motor_model(0, 0, 0, 0)
            time.sleep(0.1)  # Small delay to ensure complete stop
            
        except Exception as e:
            print(f"Error during movement: {e}")
            # Ensure car stops if there's an error
            PWM.set_motor_model(0, 0, 0, 0)
            
    def run(self):
        # scan 
        scan_results = self.scan_environment()
        print(scan_results)
        direction = self.get_direction(scan_results)
        direction_strings = ['straight', 'left', 'right', 'back']
        print(direction_strings[direction])
        
        # Move in the chosen direction
        self.move(direction)
        
        # update position and direction
        if direction == 0:  # straight
            if self.current_direction == 0:  # north
                self.current_position = (self.current_position[0], self.current_position[1] + 1)
            elif self.current_direction == 1:  # east
                self.current_position = (self.current_position[0] + 1, self.current_position[1])
            elif self.current_direction == 2:  # south
                self.current_position = (self.current_position[0], self.current_position[1] - 1)
            else:  # west
                self.current_position = (self.current_position[0] - 1, self.current_position[1])
        elif direction == 1:  # left
            self.current_direction = (self.current_direction - 1) % 8  # Now using 8 directions
        elif direction == 2:  # right
            self.current_direction = (self.current_direction + 1) % 8  # Now using 8 directions
        elif direction == -1:  # back
            if self.current_direction == 0:  # north
                self.current_position = (self.current_position[0], self.current_position[1] - 1)
            elif self.current_direction == 1:  # east
                self.current_position = (self.current_position[0] - 1, self.current_position[1])
            elif self.current_direction == 2:  # south
                self.current_position = (self.current_position[0], self.current_position[1] + 1)
            else:  # west
                self.current_position = (self.current_position[0] + 1, self.current_position[1])

    def run_loop(self):
        while True:
            self.run()



if __name__ == '__main__':
    maze_solver = MazeSolver()
    maze_solver.run()
    #maze_solver.run_loop()



