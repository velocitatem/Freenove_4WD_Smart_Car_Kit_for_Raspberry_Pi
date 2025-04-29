from ultrasonic import Ultrasonic
from motor import Ordinary_Car
from servo import Servo
from infrared import Infrared
from adc import ADC
import time
import math

class Car:
    def __init__(self):
        self.servo = None
        self.sonic = None
        self.motor = None
        self.infrared = None
        self.adc = None
        self.car_record_time = time.time()
        self.car_sonic_servo_angle = 30
        self.car_sonic_servo_dir = 1
        self.car_sonic_distance = [30, 30, 30]
        self.time_compensate = 3 #Depend on your own car,If you want to get the best out of the rotation mode, change the value by experimenting.
        # Maze solving variables
        self.grid_size = 20  # cm per grid cell
        self.current_position = (0, 0)  # (x, y) in grid coordinates
        self.current_direction = 0  # 0: North, 1: East, 2: South, 3: West
        self.maze_map = {}  # Stores discovered walls
        self.path = []  # Current A* path
        self.start()
    def start(self):  
        if self.servo is None:
            self.servo = Servo()
        if self.sonic is None:
            self.sonic = Ultrasonic()
        if self.motor is None:
            self.motor = Ordinary_Car()
        if self.infrared is None:
            self.infrared = Infrared()
        if self.adc is None:
            self.adc = ADC() 

    def close(self):
        self.motor.set_motor_model(0,0,0,0)
        self.sonic.close()
        self.motor.close()
        self.infrared.close()
        self.adc.close_i2c()
        self.servo = None
        self.sonic = None
        self.motor = None
        self.infrared = None
        self.adc = None

    def run_motor_ultrasonic(self, distance):
        if (distance[0] < 30 and distance[1] < 30 and distance[2] <30) or distance[1] < 30 :
            self.motor.set_motor_model(-1450,-1450,-1450,-1450) 
            time.sleep(0.1)   
            if distance[0] < distance[2]:
                self.motor.set_motor_model(1450,1450,-1450,-1450)
            else :
                self.motor.set_motor_model(-1450,-1450,1450,1450)
        elif distance[0] < 30 and distance[1] < 30:
            self.motor.set_motor_model(1500,1500,-1500,-1500)
        elif distance[2] < 30 and distance[1] < 30:
            self.motor.set_motor_model(-1500,-1500,1500,1500)
        elif distance[0] < 20 :
            self.motor.set_motor_model(2000,2000,-500,-500)
            if distance[0] < 10 :
                self.motor.set_motor_model(1500,1500,-1000,-1000)
        elif distance[2] < 20 :
            self.motor.set_motor_model(-500,-500,2000,2000)
            if distance[2] < 10 :
                self.motor.set_motor_model(-1500,-1500,1500,1500)
        else :
            self.motor.set_motor_model(600,600,600,600)

    def mode_ultrasonic(self):
        if (time.time() - self.car_record_time) > 0.2:
            self.car_record_time = time.time()
            self.servo.set_servo_pwm('0', self.car_sonic_servo_angle)
            if self.car_sonic_servo_angle == 30:
                self.car_sonic_distance[0] = self.sonic.get_distance()
            elif self.car_sonic_servo_angle == 90:
                self.car_sonic_distance[1] = self.sonic.get_distance()
            elif self.car_sonic_servo_angle == 150:
                self.car_sonic_distance[2] = self.sonic.get_distance()
            #print("L:{}, M:{}, R:{}".format(self.car_sonic_distance[0], self.car_sonic_distance[1], self.car_sonic_distance[2]))
            self.run_motor_ultrasonic(self.car_sonic_distance)
            if self.car_sonic_servo_angle <= 30:
                self.car_sonic_servo_dir = 1
            elif self.car_sonic_servo_angle >= 150:
                self.car_sonic_servo_dir = 0
            if self.car_sonic_servo_dir == 1:
                self.car_sonic_servo_angle += 60
            elif self.car_sonic_servo_dir == 0:
                self.car_sonic_servo_angle -= 60

    def mode_infrared(self):
        if (time.time() - self.car_record_time) > 0.2:
            self.car_record_time = time.time()
            infrared_value = self.infrared.read_all_infrared()
            #print("infrared_value: " + str(infrared_value))
            if infrared_value == 2:
                self.motor.set_motor_model(800,800,800,800)
            elif infrared_value == 4:
                self.motor.set_motor_model(-1500,-1500,2500,2500)
            elif infrared_value == 6:
                self.motor.set_motor_model(-2000,-2000,4000,4000)
            elif infrared_value == 1:
                self.motor.set_motor_model(2500,2500,-1500,-1500)
            elif infrared_value == 3:
                self.motor.set_motor_model(4000,4000,-2000,-2000)
            elif infrared_value == 7:
                self.motor.set_motor_model(0,0,0,0)

    def mode_light(self):
        if (time.time() - self.car_record_time) > 0.2:
            self.car_record_time = time.time()
            self.motor.set_motor_model(0,0,0,0)
            L = self.adc.read_adc(0)
            R = self.adc.read_adc(1)
            #print("L: {}, R: {}".format(L, R))
            if L < 2.99 and R < 2.99 :
                self.motor.set_motor_model(600,600,600,600)
            elif abs(L-R)<0.15:
                self.motor.set_motor_model(0,0,0,0)
            elif L > 3 or R > 3:
                if L > R :
                    self.motor.set_motor_model(-1200,-1200,1400,1400)
                elif R > L :
                    self.motor.set_motor_model(1400,1400,-1200,-1200)

    def mode_rotate(self, n):
        angle = n
        bat_compensate = 7.5 / (self.adc.read_adc(2) * (3 if self.adc.pcb_version == 1 else 2))
        while True:
            W = 2000
            VY = int(2000 * math.cos(math.radians(angle)))
            VX = -int(2000 * math.sin(math.radians(angle)))
            FR = VY - VX + W
            FL = VY + VX - W
            BL = VY - VX - W
            BR = VY + VX + W
            print("rotating")
            self.motor.set_motor_model(FL, BL, FR, BR)
            time.sleep(5*self.time_compensate*bat_compensate/1000)
            angle -= 5

    def manhattan_distance(self, pos1, pos2):
        """Calculate Manhattan distance between two positions."""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def get_neighbors(self, pos):
        """Get valid neighboring positions."""
        x, y = pos
        neighbors = []
        # Check all four directions
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            new_pos = (x + dx, y + dy)
            # Check if there's a wall in this direction
            if new_pos not in self.maze_map.get(pos, []):
                neighbors.append(new_pos)
        return neighbors

    def a_star_search(self, start, goal):
        """Implement A* pathfinding algorithm."""
        frontier = [(0, start)]  # Priority queue: (f_score, position)
        came_from = {start: None}
        g_score = {start: 0}
        f_score = {start: self.manhattan_distance(start, goal)}

        while frontier:
            current = frontier.pop(0)[1]
            if current == goal:
                # Reconstruct path
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
            frontier.sort()  # Sort by f_score

        return None  # No path found

    def update_maze_map(self):
        """Update the maze map based on ultrasonic sensor readings."""
        # Get distances in all three directions
        self.servo.set_servo_pwm('0', 30)
        time.sleep(0.1)
        left_dist = self.sonic.get_distance()
        
        self.servo.set_servo_pwm('0', 90)
        time.sleep(0.1)
        front_dist = self.sonic.get_distance()
        
        self.servo.set_servo_pwm('0', 150)
        time.sleep(0.1)
        right_dist = self.sonic.get_distance()
        
        # Reset servo to forward position
        self.servo.set_servo_pwm('0', 90)
        
        # Convert distances to grid positions
        if left_dist is not None and left_dist < 30:
            wall_pos = (self.current_position[0] - 1, self.current_position[1])
            if self.current_position not in self.maze_map:
                self.maze_map[self.current_position] = []
            self.maze_map[self.current_position].append(wall_pos)
            
        if front_dist is not None and front_dist < 30:
            wall_pos = (self.current_position[0], self.current_position[1] + 1)
            if self.current_position not in self.maze_map:
                self.maze_map[self.current_position] = []
            self.maze_map[self.current_position].append(wall_pos)
            
        if right_dist is not None and right_dist < 30:
            wall_pos = (self.current_position[0] + 1, self.current_position[1])
            if self.current_position not in self.maze_map:
                self.maze_map[self.current_position] = []
            self.maze_map[self.current_position].append(wall_pos)

    def mode_advanced_line_follower(self):
        if (time.time() - self.car_record_time) > 0.2:
            self.car_record_time = time.time()
            
            # First check ultrasonic for obstacles
            distance = self.sonic.get_distance()
            if distance is not None and distance < 30:
                # Stop the car
                self.motor.set_motor_model(0, 0, 0, 0)
                time.sleep(0.5)
                # Turn right to find a new path
                self.motor.set_motor_model(-1500, -1500, 1500, 1500)
                time.sleep(0.5)
                return
            
            # Read infrared sensors
            ir_value = self.infrared.read_all_infrared()
            
            # If we're still following the line
            if ir_value != 0:
                # Use the proven line tracking logic
                if ir_value == 2:  # Only middle sensor detects line
                    self.motor.set_motor_model(800, 800, 800, 800)  # Go straight
                elif ir_value == 4:  # Middle and right sensors detect line
                    self.motor.set_motor_model(-1500, -1500, 2500, 2500)  # Turn right
                elif ir_value == 6:  # Right sensor detects line
                    self.motor.set_motor_model(-2000, -2000, 4000, 4000)  # Sharp right
                elif ir_value == 1:  # Middle and left sensors detect line
                    self.motor.set_motor_model(2500, 2500, -1500, -1500)  # Turn left
                elif ir_value == 3:  # Left sensor detects line
                    self.motor.set_motor_model(4000, 4000, -2000, -2000)  # Sharp left
                elif ir_value == 7:  # All sensors detect line (intersection)
                    # At intersection, continue straight
                    self.motor.set_motor_model(800, 800, 800, 800)
            else:
                # We've entered the maze, start maze solving
                self.update_maze_map()
                
                # If we don't have a path or need to recalculate
                if not self.path:
                    # Define goal position (this should be set based on maze layout)
                    goal_position = (5, 5)  # Example goal
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
                    
                    # Turn if needed
                    turn_diff = (required_direction - self.current_direction) % 4
                    if turn_diff == 1:  # Turn right
                        self.motor.set_motor_model(-1500, -1500, 1500, 1500)
                        time.sleep(0.5)
                    elif turn_diff == 3:  # Turn left
                        self.motor.set_motor_model(1500, 1500, -1500, -1500)
                        time.sleep(0.5)
                    elif turn_diff == 2:  # Turn around
                        self.motor.set_motor_model(-1500, -1500, 1500, 1500)
                        time.sleep(1)
                    
                    self.current_direction = required_direction
                    
                    # Move forward
                    self.motor.set_motor_model(800, 800, 800, 800)
                    time.sleep(1)  # Move one grid cell
                    
                    # Update position
                    self.current_position = next_pos
                    self.path.pop(0)
                else:
                    # No path found, search for new path
                    self.motor.set_motor_model(-1500, -1500, 1500, 1500)
                    time.sleep(0.5)

def test_car_sonic():
    car = Car()
    try:
        while True:
            car.mode_ultrasonic()
    except KeyboardInterrupt:
        car.close()
        print("\nEnd of program")

def test_car_infrared():
    car = Car()
    try:
        while True:
            car.mode_infrared()
    except KeyboardInterrupt:
        car.close()
        print("\nEnd of program")

def test_car_light():
    car = Car()
    try:
        print("Program is starting...")
        while True:
            car.mode_light()
    except KeyboardInterrupt:
        car.close()
        print("\nEnd of program")

def test_car_rotate():
    car = Car()
    print("Program is starting...")
    try:
        car.mode_rotate(0)
    except KeyboardInterrupt:
        print ("\nEnd of program")
        car.motor.set_motor_model(0,0,0,0)
        car.close()

def test_car_advanced_line_follower():
    car = Car()
    print("Program is starting...")
    try:
        while True:
            car.mode_advanced_line_follower()
    except KeyboardInterrupt:
        print("\nEnd of program")
        car.motor.set_motor_model(0,0,0,0)
        car.close()

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print("Parameter error: Please assign the device")
        exit()
    if sys.argv[1] == 'Sonic' or sys.argv[1] == 'sonic':
        test_car_sonic()
    elif sys.argv[1] == 'Infrared' or sys.argv[1] == 'infrared':
        test_car_infrared()
    elif sys.argv[1] == 'Light' or sys.argv[1] == 'light':
        test_car_light()
    elif sys.argv[1] == 'Rotate' or sys.argv[1] == 'rotate':
        test_car_rotate()
    elif sys.argv[1] == 'Advanced' or sys.argv[1] == 'advanced':
        test_car_advanced_line_follower()