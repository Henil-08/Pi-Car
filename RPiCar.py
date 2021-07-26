import pyfirmata 
from time import sleep
import logging as log


class RPiCar:
    '''
    This is the RPiCar Module that is used to control the car with a servo and two 5V DC motors
    Works on PyFirmata Library that controls the hardware through arduino connected to the RPi
    The Pins of the servo and motors are fixed but later on can be changed as per the preference
    '''    

    def __init__(self, port='/dev/ttyACM1'):
        self.board = pyfirmata.Arduino(port) # Giving the Port for Arduino 
        sleep(2) # Time for the Arduino to Connect to the RPi
        log.info("Connection Established Successfully")


    def setup(self):
        '''
        This sets up the Servo and Motors with their respective pins
        Servo is set as SERVO function in Pyfirmata
        Respective Motor Enable Pins are set to PWM in PyFirmata

        ServoPin = 11
        Motor (Enable, In1, In2) = 10, 9, 8
        '''

        self.servoPin = self.board.digital[11]
        self.servoPin.mode = pyfirmata.SERVO

        
        self.Motor1_Enable = self.board.digital[10]
        self.Motor1_IN1 = self.board.digital[9]
        self.Motor1_IN2 = self.board.digital[8]
        self.Motor1_Enable.mode = pyfirmata.PWM

        log.debug("Setup Successfull!")

    
    def motor(self, speed=0):
        '''
        Controls the Speed and Direction of the car
        If speed is between -100 to 0 it moves backwards
        If speed is betwenn 0 and 100 it moves forwards
        '''
        speed = speed / 100 # PWM ranges from 0.01 to 0.1 so we have to divide by 100
        if speed < 0:
            log.debug(f"Going Backward! with speed: {abs(speed)}")
            self.backward(abs(speed))
        
        else: 
            log.debug(f"Going Forward with speed: {speed}")
            self.forward(abs(speed))


    def servo(self, angle=90):
        
        log.debug(f"Rotating Servo to {angle} degrees")
        self.servoPin.write(angle)
        sleep(0.10)


    def forward(self, speed):
        ''' 
        Setting In1 to HIGH and In2 to LOW we can make the car go forward
        Giving the speed from 0 to 100% to Enable to set the speed
        '''
        self.Motor1_IN1.write(0) 
        self.Motor1_IN2.write(1)
        
        self.Motor1_Enable.write(speed)


    def backward(self, speed):
        '''
        Setting In1 to LOW and In2 to HIGH we can make the car go backward
        Giving the speed from 0 to 100% to Enable to set the speed
        '''
        self.Motor1_IN1.write(1)
        self.Motor1_IN2.write(0)
    
        self.Motor1_Enable.write(speed)

    
    def motor_check(self):
        log.debug("Motor check started!")
        
        for i in range(100, -1, -25):
            self.motor(i)
            sleep(2)

        self.motor(0)
        sleep(3)

        for i in range(0, -101, -25):
            self.motor(i)
            sleep(2)

        self.motor(0)
        sleep(3)

    
    def servo_check(self):
        log.debug("Servo check started!")
        for i in range(60, 121, 15):
            self.servo(i)
            sleep(0.5)
        
        self.servo(90)
        sleep(3)

        for i in range(120, 59, -15):
            self.servo(i)
            sleep(0.5)

        self.servo(90)
        sleep(3)


    def cleanup(self):
        log.info("Cleaning up the car, Reseting the Hardware!")
        self.motor()
        self.servo()


if __name__ == '__main__':
    car = RPiCar()

    car.setup()

    car.motor_check()
    car.servo_check()

    car.cleanup()

