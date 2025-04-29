from servo import Servo
from ultrasonic import Ultrasonic
from motor import Ordinary_Car
import time
class ContinousScanner:
    def __init__(self):
        self.ultrasonic = Ultrasonic()
        self.servo = Servo()
        self.PWM = Ordinary_Car()
        self.last_scan = 0 # 0: left to right, 1: right to left
        self.MOVING_TIME = 0.2
        self.MOVING_SPEED = 1500
    def scan(self):
        # smoothly move the servo from 0 to 180 degrees
        distribution = []
        rng = range(0, 180, 1) if self.last_scan == 0 else range(180, 0, -1)
        for angle in rng:
            self.servo.set_servo_pwm('0', angle)
            time.sleep(0.01)
            distance = self.ultrasonic.get_distance()
            distribution.append(distance)
        # read the ultrasonic sensor
        self.last_scan = 1 - self.last_scan 
        return distribution

    def nudge_right(self):
        # Small correction to the right (pulse left wheels)
        self.PWM.set_motor_model(self.MOVING_SPEED, self.MOVING_SPEED, 0, 0)
        time.sleep(self.MOVING_TIME)
        self.PWM.set_motor_model(0, 0, 0, 0)

    def nudge_left(self):
        # Small correction to the left (pulse right wheels)
        self.PWM.set_motor_model(0, 0, self.MOVING_SPEED, self.MOVING_SPEED)
        time.sleep(self.MOVING_TIME)
        self.PWM.set_motor_model(0, 0, 0, 0)

if __name__ == "__main__":
    scanner = ContinousScanner()
    center_angle = 90
    threshold = 5  # degrees within which we consider the peak centered
    max_attempts = 100
    for attempt in range(max_attempts):
        distribution = scanner.scan()
        peak_angle = distribution.index(max(distribution))
        print(f"Attempt {attempt+1}: Peak at {peak_angle}")
        if abs(peak_angle - center_angle) <= threshold:
            print(f"Peak centered at {peak_angle} (within {threshold} degrees)")
            break
        elif peak_angle < center_angle:
            print("Nudging left to move peak right...")
            scanner.nudge_left()
        else:
            print("Nudging right to move peak left...")
            scanner.nudge_right()
    else:
        print("Failed to center peak after max attempts.")


    