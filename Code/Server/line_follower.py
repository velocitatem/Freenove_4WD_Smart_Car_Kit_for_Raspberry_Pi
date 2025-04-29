import time
from motor import *
from gpiozero import LineSensor
from ultrasonic import Ultrasonic
from buzzer import Buzzer
IR01 = 14
IR02 = 15
IR03 = 23
IR01_sensor = LineSensor(IR01)
IR02_sensor = LineSensor(IR02)
IR03_sensor = LineSensor(IR03)
ultrasonic=Ultrasonic()              
class Ordinary_Car:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.set_pwm_freq(50)
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
        return duty1,duty2,duty3,duty4
    def left_upper_wheel(self,duty):
        if duty>0:
            self.pwm.set_motor_pwm(0,0)
            self.pwm.set_motor_pwm(1,duty)
        elif duty<0:
            self.pwm.set_motor_pwm(1,0)
            self.pwm.set_motor_pwm(0,abs(duty))
        else:
            self.pwm.set_motor_pwm(0,4095)
            self.pwm.set_motor_pwm(1,4095)
    def left_lower_wheel(self,duty):
        if duty>0:
            self.pwm.set_motor_pwm(3,0)
            self.pwm.set_motor_pwm(2,duty)
        elif duty<0:
            self.pwm.set_motor_pwm(2,0)
            self.pwm.set_motor_pwm(3,abs(duty))
        else:
            self.pwm.set_motor_pwm(2,4095)
            self.pwm.set_motor_pwm(3,4095)
    def right_upper_wheel(self,duty):
        if duty>0:
            self.pwm.set_motor_pwm(6,0)
            self.pwm.set_motor_pwm(7,duty)
        elif duty<0:
            self.pwm.set_motor_pwm(7,0)
            self.pwm.set_motor_pwm(6,abs(duty))
        else:
            self.pwm.set_motor_pwm(4,4095)
            self.pwm.set_motor_pwm(5,4095)
    def right_lower_wheel(self,duty):
        if duty>0:
            self.pwm.set_motor_pwm(4,0)
            self.pwm.set_motor_pwm(5,duty)
        elif duty<0:
            self.pwm.set_motor_pwm(5,0)
            self.pwm.set_motor_pwm(4,abs(duty))
        else:
            self.pwm.set_motor_pwm(4,4095)
            self.pwm.set_motor_pwm(5,4095)
    def set_motor_model(self, duty1, duty2, duty3, duty4):
        duty1,duty2,duty3,duty4=self.duty_range(duty1,duty2,duty3,duty4)
        self.left_upper_wheel(duty1)
        self.left_lower_wheel(duty2)
        self.right_upper_wheel(duty3)
        self.right_lower_wheel(duty4)

    def close(self):
        self.set_motor_model(0,0,0,0)
        self.pwm.close()


PWM = Ordinary_Car()
class Line_Tracking:
    def __init__(self):
        pass

    def test_Infrared(self):
        try:
            while True:
                if IR01_sensor.value !=True and IR02_sensor.value == True and IR03_sensor.value !=True:
                    print ('Middle')
                elif IR01_sensor.value !=True and IR02_sensor.value != True and IR03_sensor.value ==True:
                    print ('Right')
                elif IR01_sensor.value ==True and IR02_sensor.value != True and IR03_sensor.value !=True:
                    print ('Left')
        except KeyboardInterrupt:
            print ("\nEnd of program")
        
    def run(self):
        while True:
            self.LMR=0x00
            if IR01_sensor.value == True:
                self.LMR=(self.LMR | 4)
            if IR02_sensor.value == True:
                self.LMR=(self.LMR | 2)
            if IR03_sensor.value == True:
                self.LMR=(self.LMR | 1)
            
            print(f"Sensors: IR01={IR01_sensor.value} IR02={IR02_sensor.value} IR03={IR03_sensor.value} LMR={self.LMR}")
            
            if self.LMR==2:
                print("Moving forward")
                PWM.set_motor_model(400,400,400,400)
            elif self.LMR==4:
                print("Turning right")
                PWM.set_motor_model(-750,-750,1250,1250)
            elif self.LMR==6:
                print("Turning hard right")
                PWM.set_motor_model(-1000,-1000,2000,2000)
            elif self.LMR==1:
                print("Turning left")
                PWM.set_motor_model(1250,1250,-750,-750)
            elif self.LMR==3:
                print("Turning hard left")
                PWM.set_motor_model(2000,2000,-1000,-1000)
            elif self.LMR==7:
                print("All sensors triggered - stopping")
                PWM.set_motor_model(0,0,0,0)
            time.sleep(0.1)  # Add a small delay to avoid flooding with debug output

infrared=Line_Tracking()
# Main program logic follows:
if __name__ == '__main__':
    print ('Program is starting ... ')
    try:
        Line_Tracking().run()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program  will be  executed.
        PWM.set_motor_model(0,0,0,0)
