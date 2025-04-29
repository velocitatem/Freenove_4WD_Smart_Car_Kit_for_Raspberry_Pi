import time
import RPi.GPIO as GPIO
from motor import *
from ultrasonic import Ultrasonic
from buzzer import Buzzer
import numpy as np
from maze_solver import MazeSolver

print("Starting line follower initialization...")

# Sensor pins
IR01 = 14  # Left sensor
IR02 = 15  # Center sensor
IR03 = 23  # Right sensor

print(f"Setting up GPIO pins: Left={IR01}, Center={IR02}, Right={IR03}")

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(IR01, GPIO.IN)
GPIO.setup(IR02, GPIO.IN)
GPIO.setup(IR03, GPIO.IN)

print("GPIO setup complete")

# Initialize other components
print("Initializing motor controller...")
try:
    motor = Ordinary_Car()
    print("Motor controller initialized successfully")
except Exception as e:
    print(f"Error initializing motor controller: {e}")
    GPIO.cleanup()
    exit(1)

print("Initializing ultrasonic sensor...")
try:
    ultrasonic = Ultrasonic()
    print("Ultrasonic sensor initialized successfully")
except Exception as e:
    print(f"Error initializing ultrasonic sensor: {e}")
    motor.close()
    GPIO.cleanup()
    exit(1)

print("Initializing buzzer...")
try:
    buzzer = Buzzer()
    print("Buzzer initialized successfully")
except Exception as e:
    print(f"Error initializing buzzer: {e}")
    motor.close()
    GPIO.cleanup()
    exit(1)

# PID Controller parameters
KP = 12   # Increased for faster corrections
KI = 0.1  # Small integral term to eliminate steady-state error
KD = 8    # Increased for quicker response

# Motor speed parameters
BASE_SPEED = 1200        # Increased base speed
TURN_SPEED = 2500       # Increased turn speed
MAX_SPEED_CHANGE = 800   # Increased for faster acceleration

# Line following state variables
last_error = 0
integral = 0
last_time = time.time()
lost_line_counter = 0
LOST_LINE_THRESHOLD = 2  # Reduced for faster turn detection
turn_state = 'STRAIGHT'
turn_start_time = 0
TURN_TIMEOUT = 0.8  # Reduced timeout for faster recovery

# Speed ramping variables
current_left_speed = 0
current_right_speed = 0

# Add global variable for sensor inversion
invert_sensors = False

def calibrate_sensors():
    """Calibrate sensors and verify they're working"""
    print("\nStarting sensor calibration...")
    print("Please position the car so that:")
    print("1. All sensors are OVER the line")
    print("2. Press Enter when ready")
    input()
    
    print("\nReading sensor values when over the line...")
    # Read all sensors multiple times
    readings = []
    for i in range(10):
        left = GPIO.input(IR01)
        center = GPIO.input(IR02)
        right = GPIO.input(IR03)
        readings.append((left, center, right))
        print(f"Reading {i+1}/10: L={left}, C={center}, R={right}")
        time.sleep(0.1)
    
    # Calculate most common reading
    most_common = max(set(readings), key=readings.count)
    print(f"\nMost common reading when over line: L={most_common[0]}, C={most_common[1]}, R={most_common[2]}")
    
    print("\nNow position the car so that:")
    print("1. All sensors are OFF the line")
    print("2. Press Enter when ready")
    input()
    
    print("\nReading sensor values when off the line...")
    # Read all sensors multiple times
    readings = []
    for i in range(10):
        left = GPIO.input(IR01)
        center = GPIO.input(IR02)
        right = GPIO.input(IR03)
        readings.append((left, center, right))
        print(f"Reading {i+1}/10: L={left}, C={center}, R={right}")
        time.sleep(0.1)
    
    # Calculate most common reading
    most_common_off = max(set(readings), key=readings.count)
    print(f"\nMost common reading when off line: L={most_common_off[0]}, C={most_common_off[1]}, R={most_common_off[2]}")
    
    # Determine if we need to invert the readings
    global invert_sensors
    invert_sensors = (most_common[0] == 0)  # If 0 means "on line", we need to invert
    
    print(f"\nSensor logic {'will' if invert_sensors else 'will not'} be inverted")
    print("Calibration complete!")
    print("\nPosition the car on the line and press Enter to start")
    input()
    
    # Test motor response
    print("\nTesting motor response...")
    try:
        print("Setting all motors to 0...")
        motor.set_motor_model(0, 0, 0, 0)
        time.sleep(1)
        
        print("Testing forward motion...")
        motor.set_motor_model(500, 500, 500, 500)
        time.sleep(0.5)
        
        print("Testing stop...")
        motor.set_motor_model(0, 0, 0, 0)
        time.sleep(0.5)
        
        print("Motor test complete!")
    except Exception as e:
        print(f"Error during motor test: {e}")
        return False
    
    return True

def read_sensors():
    """Read and optionally invert sensor values"""
    left = GPIO.input(IR01)
    center = GPIO.input(IR02)
    right = GPIO.input(IR03)
    
    if invert_sensors:
        left = 1 - left
        center = 1 - center
        right = 1 - right
        
    print(f"Raw: L={GPIO.input(IR01)}, C={GPIO.input(IR02)}, R={GPIO.input(IR03)}")
    print(f"Processed: L={left}, C={center}, R={right}")
    return left, center, right

def calculate_error(left, center, right):
    # Debug print
    print(f"Calculating error - L:{left} C:{center} R:{right}")
    
    if center == 1 and left == 0 and right == 0:  # Center on line
        return 0
    elif left == 1 and center == 1 and right == 0:  # Line slightly to the left
        return -1
    elif left == 1 and center == 0 and right == 0:  # Line far to the left
        return -2
    elif right == 1 and center == 1 and left == 0:  # Line slightly to the right
        return 1
    elif right == 1 and center == 0 and left == 0:  # Line far to the right
        return 2
    elif left == 1 and center == 1 and right == 1:  # T-junction or crossroad
        return 0
    elif left == 1 and right == 1:  # Wide line or special case
        return 0
    else:  # Line lost
        return None

def smooth_speed_change(current_speed, target_speed):
    """Gradually change speed to avoid jerky movements"""
    diff = target_speed - current_speed
    if abs(diff) > MAX_SPEED_CHANGE:
        if diff > 0:
            return current_speed + MAX_SPEED_CHANGE
        else:
            return current_speed - MAX_SPEED_CHANGE
    return target_speed

def handle_sharp_turn(left, center, right):
    global turn_state, turn_start_time, lost_line_counter, current_left_speed, current_right_speed
    
    current_time = time.time()
    
    # Reset turn state if we're back on the line
    if center == 1 and turn_state != 'STRAIGHT':
        print("Back on line - resetting turn state")
        turn_state = 'STRAIGHT'
        lost_line_counter = 0
        return False

    # Check for sharp turn conditions - more sensitive detection
    if (center == 0 and (left == 1 or right == 1)) or (left == 0 and right == 1) or (left == 1 and right == 0):
        # First stop the motors briefly
        try:
            motor.set_motor_model(0, 0, 0, 0)
            time.sleep(0.02)  # Even shorter pause
        except Exception as e:
            print(f"Error stopping motors: {e}")
            return False
        
        if right == 1:  # Sensor on right sees line - turn right
            print("Sharp right turn detected")
            turn_state = 'TURNING_RIGHT'
            # More aggressive turn speeds
            current_left_speed = int(TURN_SPEED * 1.2)  # Boost inside wheel
            current_right_speed = int(-TURN_SPEED * 0.8)
        elif left == 1:  # Sensor on left sees line - turn left
            print("Sharp left turn detected")
            turn_state = 'TURNING_LEFT'
            # More aggressive turn speeds
            current_left_speed = int(-TURN_SPEED * 0.8)
            current_right_speed = int(TURN_SPEED * 1.2)  # Boost inside wheel
            
        try:
            motor.set_motor_model(current_left_speed, current_left_speed, 
                                current_right_speed, current_right_speed)
        except Exception as e:
            print(f"Error setting motor speeds: {e}")
            return False
        
        turn_start_time = current_time
        return True
        
    # If we've lost the line completely
    if left == 0 and center == 0 and right == 0:
        lost_line_counter += 1
        if lost_line_counter > LOST_LINE_THRESHOLD:
            # Continue turning in the last known direction with increased speed
            if turn_state == 'TURNING_LEFT':
                current_left_speed = int(-TURN_SPEED * 0.8)
                current_right_speed = int(TURN_SPEED * 1.2)
            elif turn_state == 'TURNING_RIGHT':
                current_left_speed = int(TURN_SPEED * 1.2)
                current_right_speed = int(-TURN_SPEED * 0.8)
            else:
                # If we don't know which way to turn, try turning right
                print("Line lost - attempting recovery turn right")
                current_left_speed = int(TURN_SPEED * 1.2)
                current_right_speed = int(-TURN_SPEED * 0.8)
                turn_state = 'TURNING_RIGHT'
                
            try:
                motor.set_motor_model(current_left_speed, current_left_speed, 
                                    current_right_speed, current_right_speed)
            except Exception as e:
                print(f"Error setting motor speeds during line loss: {e}")
                return False
            return True
            
    # Check if we've been turning too long
    if turn_state != 'STRAIGHT' and (current_time - turn_start_time) > TURN_TIMEOUT:
        print("Turn timeout - resetting")
        turn_state = 'STRAIGHT'
        lost_line_counter = 0
        try:
            motor.set_motor_model(0, 0, 0, 0)  # Stop motors on timeout
        except Exception as e:
            print(f"Error stopping motors on timeout: {e}")
        
    return False

def check_line_end(left, center, right, consecutive_readings=20):
    """Check if we've reached the end of the line"""
    print("\nChecking for line end...")
    all_zeros = 0
    max_consecutive_zeros = 0
    current_consecutive_zeros = 0
    
    # First, stop and back up slightly to ensure we're not at an intersection
    motor.set_motor_model(-1000, -1000, -1000, -1000)
    time.sleep(0.2)  # Back up briefly
    motor.set_motor_model(0, 0, 0, 0)
    time.sleep(0.5)  # Wait for stability
    
    # Do a rotation check to look for lines in other directions
    print("Performing rotation check for intersections...")
    for angle in [45, 90, 135, 180, 225, 270, 315, 360]:
        # Turn to the angle
        motor.set_motor_model(1000, 1000, -1000, -1000)  # Turn right
        time.sleep(angle/360.0)  # Approximate time for degree turn
        motor.set_motor_model(0, 0, 0, 0)
        time.sleep(0.1)
        
        # Check sensors
        l, c, r = read_sensors()
        print(f"Rotation check at {angle}°: L={l}, C={c}, R={r}")
        if l == 1 or c == 1 or r == 1:
            print(f"Found line at {angle}° rotation - this is an intersection")
            return False
    
    # If no lines found in rotation, check for true end
    print("No lines found in rotation, checking for true end...")
    for i in range(consecutive_readings):
        l, c, r = read_sensors()
        print(f"End check reading {i+1}/{consecutive_readings}: L={l}, C={c}, R={r}")
        if l == 0 and c == 0 and r == 0:
            current_consecutive_zeros += 1
            max_consecutive_zeros = max(max_consecutive_zeros, current_consecutive_zeros)
            all_zeros += 1
        else:
            current_consecutive_zeros = 0
            print("Found line signal - not at end")
            return False
        time.sleep(0.1)
    
    # Only return True if we had enough consecutive zero readings
    if all_zeros == consecutive_readings and max_consecutive_zeros >= 15:
        print("Confirmed end of line - all readings showed no line")
        return True
    return False

def cleanup_components():
    """Clean up all components properly"""
    print("\nCleaning up components...")
    try:
        motor.set_motor_model(0, 0, 0, 0)
        motor.close()
        print("Motor cleaned up")
    except Exception as e:
        print(f"Error cleaning up motor: {e}")
    
    try:
        ultrasonic.cleanup()
        print("Ultrasonic sensor cleaned up")
    except Exception as e:
        print(f"Error cleaning up ultrasonic: {e}")
    
    try:
        buzzer.cleanup()
        print("Buzzer cleaned up")
    except Exception as e:
        print(f"Error cleaning up buzzer: {e}")
    
    try:
        GPIO.cleanup()
        print("GPIO cleaned up")
    except Exception as e:
        print(f"Error cleaning up GPIO: {e}")

def follow_line():
    global last_error, integral, last_time, current_left_speed, current_right_speed
    
    try:
        print("Starting improved line follower...")
        print("Motor controller initialized")
        
        consecutive_line_lost = 0
        LINE_END_THRESHOLD = 40  # Increased threshold for more certainty
        last_turn_time = time.time()
        MIN_TIME_BETWEEN_CHECKS = 5  # Minimum seconds between end checks
        
        while True:
            left, center, right = read_sensors()
            current_time = time.time()
            
            # Check if we've reached the end of the line
            if left == 0 and center == 0 and right == 0:
                consecutive_line_lost += 1
                if consecutive_line_lost >= LINE_END_THRESHOLD:
                    # Only do end check if enough time has passed since last check
                    if current_time - last_turn_time >= MIN_TIME_BETWEEN_CHECKS:
                        # Stop the motors before checking
                        motor.set_motor_model(0, 0, 0, 0)
                        time.sleep(0.5)  # Wait for complete stop
                        
                        # Do a thorough check for line end
                        if check_line_end(left, center, right):
                            print("End of line detected - switching to maze solver")
                            motor.set_motor_model(0, 0, 0, 0)
                            # Clean up components before switching to maze solver
                            cleanup_components()
                            return True  # Signal to switch to maze solving
                        else:
                            consecutive_line_lost = 0  # Reset counter if it wasn't actually the end
                            last_turn_time = current_time
                    else:
                        print("Too soon since last check, continuing line following")
                        consecutive_line_lost = 0
            else:
                consecutive_line_lost = 0
            
            # Check for sharp turn first
            if handle_sharp_turn(left, center, right):
                continue
                
            # Calculate error for PID control
            error = calculate_error(left, center, right)
            
            if error is not None:
                # PID calculations
                current_time = time.time()
                dt = current_time - last_time
                dt = max(dt, 0.001)  # Prevent division by zero
                
                # Calculate PID terms
                proportional = KP * error
                integral += KI * error * dt
                derivative = KD * (error - last_error) / dt
                
                # Calculate motor adjustment
                adjustment = int(proportional + integral + derivative)
                
                # Calculate target speeds
                target_left = int(BASE_SPEED - adjustment)
                target_right = int(BASE_SPEED + adjustment)
                
                # Smooth speed transitions
                current_left_speed = smooth_speed_change(current_left_speed, target_left)
                current_right_speed = smooth_speed_change(current_right_speed, target_right)
                
                # Clamp speeds to valid range
                current_left_speed = max(-4095, min(4095, current_left_speed))
                current_right_speed = max(-4095, min(4095, current_right_speed))
                
                # Apply motor speeds
                try:
                    motor.set_motor_model(current_left_speed, current_left_speed,
                                        current_right_speed, current_right_speed)
                except Exception as e:
                    print(f"Error setting motor speeds: {e}")
                    continue
                
                # Update for next iteration
                last_error = error
                last_time = current_time
            
            time.sleep(0.005)  # Reduced delay for faster processing
            
    except KeyboardInterrupt:
        print("\nStopping motors...")
        try:
            motor.set_motor_model(0, 0, 0, 0)
        except Exception as e:
            print(f"Error stopping motors: {e}")
        return False
    except Exception as e:
        print(f"\nUnexpected error in line following: {e}")
        return False

if __name__ == "__main__":
    try:
        print("Starting line follower program...")
        print("First, let's calibrate the sensors...")
        
        # Run calibration
        if not calibrate_sensors():
            print("Calibration failed! Exiting...")
            cleanup_components()
            exit(1)
        
        print("Calibration complete! Starting line following...")
        if follow_line():  # If line following ends normally (reached end of line)
            print("Switching to maze solving mode...")
            # Initialize new GPIO for maze solver
            GPIO.setmode(GPIO.BCM)
            maze_solver = MazeSolver()
            maze_solver.explore_maze()
            
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
    finally:
        # Clean stop
        print("Cleaning up...")
        cleanup_components()