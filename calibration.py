#!/usr/bin/env python3
# calibration.py - Handles calibration data

import json
import os
import time
from pathlib import Path

# Default calibration values
DEFAULT_CALIBRATION = {
    "SEC_PER_CELL": 0.5,  # Initial guess, will be overridden by calibration
    "TURN_DURATION_90": 0.65,
    "BASE_SPEED": 2000,
    "TURN_SPEED": 2500
}

def load_calibration(file_path="~/maze_solver/calibration.json"):
    """Load calibration data from a JSON file"""
    path = Path(os.path.expanduser(file_path))
    
    # Create directory if it doesn't exist
    path.parent.mkdir(parents=True, exist_ok=True)
    
    try:
        if path.exists():
            with open(path, 'r') as f:
                return json.load(f)
        else:
            # Create default calibration file if it doesn't exist
            save_calibration(DEFAULT_CALIBRATION, file_path)
            return DEFAULT_CALIBRATION
    except Exception as e:
        print(f"Error loading calibration: {e}")
        return DEFAULT_CALIBRATION

def save_calibration(data, file_path="~/maze_solver/calibration.json"):
    """Save calibration data to a JSON file"""
    path = Path(os.path.expanduser(file_path))
    
    # Create directory if it doesn't exist
    path.parent.mkdir(parents=True, exist_ok=True)
    
    try:
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Calibration saved to {path}")
        return True
    except Exception as e:
        print(f"Error saving calibration: {e}")
        return False

def calibrate_straight_movement(pwm, speed=2000):
    """
    Helper function to calibrate straight movement
    Returns the time it takes to travel one cell
    
    Note: This is interactive and requires the user to manually stop the robot
    """
    input("Place the robot at the start of a cell facing straight. Press Enter to begin.")
    pwm.set_motor_model(speed, speed, speed, speed)
    start = time.perf_counter()
    
    input("Press Enter when the robot's front bumper touches the next grid line...")
    pwm.set_motor_model(0, 0, 0, 0)  # Stop motors
    elapsed = time.perf_counter() - start
    
    print(f"Measured time: {elapsed:.3f} seconds")
    return elapsed 