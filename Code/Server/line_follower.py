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
            elif self.LMR==0:
                print("No sensors detecting line - stopping")
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
