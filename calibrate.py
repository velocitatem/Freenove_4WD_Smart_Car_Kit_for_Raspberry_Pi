#!/usr/bin/env python3
# calibrate.py - Command-line tool for calibrating maze solver movement

import time
import sys
from calibration import load_calibration, save_calibration
from geometry import CELL_CM
from motor import *

# Initialize motors
try:
    from motor import Ordinary_Car
    PWM = Ordinary_Car()
    has_hardware = True
except (ImportError, RuntimeError):
    print("Warning: Hardware not available. Running in simulation mode.")
    has_hardware = False

def calibrate_straight():
    """Interactive calibration of straight line movement"""
    if not has_hardware:
        print("Error: Hardware required for calibration")
        return False
        
    try:
        # Load current calibration
        calib = load_calibration()
        speed = calib['BASE_SPEED']
        
        print("\n=== Straight Movement Calibration ===")
        print(f"Cell size: {CELL_CM} cm")
        print(f"Current speed: {speed}")
        print(f"Current time per cell: {calib['SEC_PER_CELL']:.3f} seconds")
        
        # Ask for confirmation
        input("\nPosition the robot at the start of a cell, facing straight ahead.")
        input("Press Enter to start motors...")
        
        # Start motors
        PWM.set_motor_model(speed, speed, speed, speed)
        start_time = time.perf_counter()
        
        # Wait for user to press Enter when robot reaches the next cell
        input("Press Enter when the robot's front center reaches the next cell line...")
        elapsed = time.perf_counter() - start_time
        
        # Stop motors
        PWM.set_motor_model(0, 0, 0, 0)
        
        print(f"Measured time: {elapsed:.3f} seconds")
        
        # Ask user if they're satisfied
        confirm = input("Save this calibration? (y/n): ").strip().lower()
        if confirm == 'y':
            calib['SEC_PER_CELL'] = elapsed
            save_calibration(calib)
            print("Calibration saved!")
            return True
        else:
            print("Calibration aborted.")
            return False
            
    except KeyboardInterrupt:
        # Ensure motors are stopped on Ctrl+C
        if has_hardware:
            PWM.set_motor_model(0, 0, 0, 0)
        print("\nCalibration interrupted.")
        return False

def calibrate_turn():
    """Interactive calibration of 90-degree turns"""
    if not has_hardware:
        print("Error: Hardware required for calibration")
        return False
        
    try:
        # Load current calibration
        calib = load_calibration()
        turn_speed = calib['TURN_SPEED']
        current_turn_time = calib['TURN_DURATION_90']
        
        print("\n=== Turn Movement Calibration ===")
        print(f"Current turn speed: {turn_speed}")
        print(f"Current 90° turn time: {current_turn_time:.3f} seconds")
        
        # Ask for confirmation
        input("\nPosition the robot on a straight line. Press Enter to continue...")
        
        # Let user choose turn direction
        direction = input("Calibrate (l)eft or (r)ight turn? ").strip().lower()
        left_turn = direction.startswith('l')
        
        input(f"Press Enter to perform a {90 if left_turn else 90}° {'left' if left_turn else 'right'} turn...")
        
        # Perform turn
        if left_turn:
            PWM.set_motor_model(-turn_speed, -turn_speed, turn_speed, turn_speed)
        else:
            PWM.set_motor_model(turn_speed, turn_speed, -turn_speed, -turn_speed)
            
        start_time = time.perf_counter()
        
        # Wait for user to press Enter when turn is complete
        input("Press Enter when the robot has turned exactly 90°...")
        elapsed = time.perf_counter() - start_time
        
        # Stop motors
        PWM.set_motor_model(0, 0, 0, 0)
        
        print(f"Measured turn time: {elapsed:.3f} seconds")
        
        # Ask user if they're satisfied
        confirm = input("Save this calibration? (y/n): ").strip().lower()
        if confirm == 'y':
            calib['TURN_DURATION_90'] = elapsed
            save_calibration(calib)
            print("Calibration saved!")
            return True
        else:
            print("Calibration aborted.")
            return False
            
    except KeyboardInterrupt:
        # Ensure motors are stopped on Ctrl+C
        if has_hardware:
            PWM.set_motor_model(0, 0, 0, 0)
        print("\nCalibration interrupted.")
        return False

def adjust_speeds():
    """Interactive adjustment of motor speeds"""
    if not has_hardware:
        print("Error: Hardware required for speed adjustment")
        return False
        
    try:
        # Load current calibration
        calib = load_calibration()
        base_speed = calib['BASE_SPEED']
        turn_speed = calib['TURN_SPEED']
        
        print("\n=== Speed Adjustment ===")
        print(f"Current base speed: {base_speed}")
        print(f"Current turn speed: {turn_speed}")
        
        # Get new values
        new_base = input(f"Enter new base speed [{base_speed}]: ").strip()
        if new_base and new_base.isdigit():
            base_speed = int(new_base)
            
        new_turn = input(f"Enter new turn speed [{turn_speed}]: ").strip()
        if new_turn and new_turn.isdigit():
            turn_speed = int(new_turn)
        
        # Ask for confirmation
        print(f"\nNew base speed: {base_speed}")
        print(f"New turn speed: {turn_speed}")
        confirm = input("Save these values? (y/n): ").strip().lower()
        
        if confirm == 'y':
            calib['BASE_SPEED'] = base_speed
            calib['TURN_SPEED'] = turn_speed
            save_calibration(calib)
            print("Speed values saved!")
            return True
        else:
            print("Speed adjustment aborted.")
            return False
            
    except KeyboardInterrupt:
        print("\nSpeed adjustment interrupted.")
        return False

def show_help():
    """Display help information"""
    print("\n=== Maze Solver Calibration Tool ===")
    print("Commands:")
    print("  s - Calibrate straight movement")
    print("  t - Calibrate turn movement")
    print("  v - Adjust speed values")
    print("  h - Show this help")
    print("  q - Quit")
    print("\nRecommended calibration order:")
    print("1. Adjust speeds (v)")
    print("2. Calibrate straight movement (s)")
    print("3. Calibrate turn movement (t)")

if __name__ == "__main__":
    if not has_hardware:
        print("Error: Hardware required for calibration")
        sys.exit(1)
        
    show_help()
    
    while True:
        command = input("\nEnter command (h for help): ").strip().lower()
        
        if command == 'q':
            print("Exiting calibration tool.")
            break
        elif command == 'h':
            show_help()
        elif command == 's':
            calibrate_straight()
        elif command == 't':
            calibrate_turn()
        elif command == 'v':
            adjust_speeds()
        else:
            print("Unknown command. Type 'h' for help.")
    
    # Ensure motors are stopped when exiting
    if has_hardware:
        PWM.set_motor_model(0, 0, 0, 0) 