import time
import sys
import select
import math
from motor import *
from gpiozero import LineSensor
from ultrasonic import Ultrasonic
from buzzer import Buzzer

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
        self.in_maze = False  # Whether we're in the maze or still following the line
        self.scan_positions = [0, 22.5, 45, 67.5, 90]  # Scan positions in degrees
        self.current_scan_index = 0
        self.scan_results = {}  # Store scan results

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

    def check_keyboard(self):
        """Check if a key has been pressed"""
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == 'q':  # Press 'q' to stop line following
                print("Manual transition to maze solving mode")
                self.in_maze = True
                return True
        return False

    def follow_line(self):
        """Follow the black line using the proven line following logic"""
        # Check for keyboard input
        if self.check_keyboard():
            return

        # Check for obstacles first
        d = ultrasonic.get_distance()
        if d is not None and d <= 50:
            buzzer.set_state(1)  # Turn buzzer on
            time.sleep(0.2)
            buzzer.set_state(0)  # Turn buzzer off
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

    def scan_ultrasonic(self):
        """Scan the ultrasonic sensor across a 90-degree field"""
        self.scan_results = {}  # Clear previous results
        
        # Configure ultrasonic sensor for scanning
        ultrasonic.trigger_pin = 5
        ultrasonic.echo_pin = 6
        
        # Scan at each position
        for angle in self.scan_positions:
            # Turn the car to face the scan angle
            self.turn_to_angle(angle)
            time.sleep(0.2)  # Wait for turn to complete
            
            # Take multiple measurements and average them
            measurements = []
            for _ in range(3):  # Take 3 measurements
                distance = ultrasonic.get_distance()
                if distance is not None:
                    measurements.append(distance)
                time.sleep(0.1)
            
            if measurements:
                avg_distance = sum(measurements) / len(measurements)
                self.scan_results[angle] = avg_distance
                print(f"Scan at {angle}°: {avg_distance:.1f}cm")
            
            time.sleep(0.1)  # Small delay between scans

    def find_best_direction(self):
        """Find the direction with the most space ahead"""
        if not self.scan_results:
            return None

        # Find the angle with the maximum distance
        best_angle = max(self.scan_results, key=self.scan_results.get)
        best_distance = self.scan_results[best_angle]
        
        print(f"Best direction: {best_angle}° with {best_distance:.1f}cm")
        return best_angle, best_distance

    def turn_to_angle(self, target_angle):
        """Turn the car to face the target angle"""
        # Calculate turn duration based on angle difference
        turn_duration = abs(target_angle) / 90.0  # Normalize to 90 degrees
        
        if target_angle > 0:
            # Turn right
            self.set_motor_model(-self.turn_speed, -self.turn_speed, self.turn_speed, self.turn_speed)
        else:
            # Turn left
            self.set_motor_model(self.turn_speed, self.turn_speed, -self.turn_speed, -self.turn_speed)
        
        time.sleep(turn_duration)
        self.set_motor_model(0, 0, 0, 0)

    def solve_maze(self):
        """Navigate through the maze using ultrasonic scanning"""
        # First check for immediate obstacles
        distance = ultrasonic.get_distance()
        if distance is not None and distance < 30:
            self.set_motor_model(0, 0, 0, 0)
            buzzer.set_state(1)  # Turn buzzer on
            time.sleep(0.2)
            buzzer.set_state(0)  # Turn buzzer off
            time.sleep(0.5)
            
            print("Obstacle detected, scanning environment...")
            # Scan for best direction
            self.scan_ultrasonic()
            best_angle, best_distance = self.find_best_direction()
            
            if best_angle is not None:
                # Turn to face the best direction
                print(f"Turning to {best_angle}°")
                self.turn_to_angle(best_angle)
                time.sleep(0.5)
                
                # Move forward if there's enough space
                if best_distance > 30:
                    print(f"Moving forward {best_distance:.1f}cm")
                    self.set_motor_model(self.base_speed, self.base_speed, self.base_speed, self.base_speed)
                    time.sleep(1.0)
            else:
                # No good direction found, turn right
                print("No clear path found, turning right")
                self.set_motor_model(-self.turn_speed, -self.turn_speed, self.turn_speed, self.turn_speed)
                time.sleep(0.5)
        else:
            # No immediate obstacle, move forward
            self.set_motor_model(self.base_speed, self.base_speed, self.base_speed, self.base_speed)
            time.sleep(0.5)

    def run(self):
        try:
            print("Starting maze solver...")
            print("First following the line...")
            print("Press 'q' to manually transition to maze solving mode")
            
            # Enable non-blocking keyboard input
            import tty
            import termios
            old_settings = termios.tcgetattr(sys.stdin)
            try:
                tty.setcbreak(sys.stdin.fileno())
                
                while True:
                    if not self.in_maze:
                        self.follow_line()
                    else:
                        self.solve_maze()
                    time.sleep(0.05)  # Small delay to prevent overwhelming the system

            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        except KeyboardInterrupt:
            print("\nStopping maze solver...")
            self.set_motor_model(0, 0, 0, 0)
            self.pwm.close()

if __name__ == '__main__':
    print('Program is starting...')
    maze_solver = MazeSolver()
    maze_solver.run() 