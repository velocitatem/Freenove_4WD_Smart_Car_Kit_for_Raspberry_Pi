import time
from motor import *
from infrared import Infrared
from ultrasonic import Ultrasonic
from buzzer import Buzzer
from servo import Servo
import math

# Initialize sensors
infrared = Infrared()
ultrasonic = Ultrasonic()
buzzer = Buzzer()
servo = Servo()

class MazeSolver:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.set_pwm_freq(50)
        
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
        self.scan_angles = [50, 70, 90, 110, 130]  # Servo angles to scan
        self.scan_results = {}  # Store scan results
        self.current_scan_index = 0

    def scan_environment(self):
        """Scan the environment using the servo-mounted ultrasonic sensor"""
        self.scan_results = {}  # Clear previous results
        # results format: {angle: distance}
        
        for angle in self.scan_angles:
            # Move servo to scan angle
            servo.set_servo_pwm('0', angle)
            time.sleep(0.2)  # Wait for servo to move
            
            # Take multiple measurements and average them
            measurements = []
            for _ in range(3):  # Take 3 measurements
                distance = ultrasonic.get_distance()
                if distance is not None:
                    measurements.append(distance)
            
            # Calculate average distance
            average_distance = sum(measurements) / len(measurements)
            
            # Store the result
            self.scan_results[angle] = average_distance
            
            # Move servo to next angle
            servo.set_servo_pwm('0', angle + 10)
            time.sleep(0.2)  # Wait for servo to move
            
        # Reset servo to center position
        servo.set_servo_pwm('0', 90)
        time.sleep(0.2)

        return self.scan_results
    
    def get_direction(self, scan_results):
        """Determine the direction to move based on the scan results"""
        # see if we can go straight
        if scan_results[90] > 10:
            return 0
        # see if we can go right
        if scan_results[130] > 10:
            return 1
        # see if we can go left
        if scan_results[50] > 10:
            return 2
    

    def run(self):

        # scan 
        scan_results = self.scan_environment()
        print(scan_results)
        direction = self.get_direction(scan_results)
        direction_strings = ['straight', 'right', 'left']
        print(direction_strings[direction])
        move_distance_cm = 2
        if direction == 0:
            self.pwm.set_motor_pwm(0, 1000)
            self.pwm.set_motor_pwm(1, 1000)
            time.sleep(move_distance_cm / 1000)
        elif direction == 1:
            self.pwm.set_motor_pwm(0, 1000)
            self.pwm.set_motor_pwm(1, 1000)
            time.sleep(move_distance_cm / 1000)
        elif direction == 2:
            self.pwm.set_motor_pwm(0, 1000)
            self.pwm.set_motor_pwm(1, 1000)
            time.sleep(move_distance_cm / 1000)
        else:
            self.pwm.set_motor_pwm(0, 0)
            self.pwm.set_motor_pwm(1, 0)
            time.sleep(move_distance_cm / 1000)
        time.sleep(0.1)

        # update position
        if direction == 0:
            self.current_position = (self.current_position[0], self.current_position[1] + move_distance_cm)
        elif direction == 1:
            self.current_position = (self.current_position[0] + move_distance_cm, self.current_position[1])
        elif direction == 2:
            self.current_position = (self.current_position[0] - move_distance_cm, self.current_position[1])
        else:
            self.current_position = (self.current_position[0], self.current_position[1] - move_distance_cm)

        # update maze map
        self.maze_map[self.current_position] = direction


if __name__ == '__main__':
    maze_solver = MazeSolver()
    maze_solver.run()



