# LEGO type:standard slot:1 autostart
import pybricks
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import math

hub = PrimeHub()



#Preperation for parallel code execution
accelerate = True
run_generator = True
runSmall = True
lastAngle = 0
oldAngle = 0
gyroValue = 0

# Create your objects here.
hub = PrimeHub()

#PID value Definition
pRegler = 0.0
iRegler = 0.0
dRegler = 0.0
pReglerLight = 0.0
iReglerLight = 0.0
dReglerLight = 0.0

"""
Initialize color Sensors
left sensor: port F
right sensor: port E
"""
colorE = ColorSensor('E') #adjust the sensor ports until they match your configuration, we recommend assigning your ports to the ones in the program for ease of use
colorF = ColorSensor('F')
smallMotorC = Motor('C')
smallMotorD = Motor('D')
circumference = 17.6 #circumference of the wheel powered by the robot in cm
sensordistance = 1 #distance between the two light sensors in cm. Used in Tangent alignment 6.4 in studs


cancel = False
inMain = True

db = DriveBase(hub, 'A', 'B')

class DriveBase:

    def __init__(self, hub, leftMotor, rightMotor):
        self.hub = hub
        self.leftMotor = Motor(leftMotor)
        self.rightMotor = Motor(rightMotor)
        self.movement_motors = MotorPair(leftMotor, rightMotor) 

    def gyroRotation(self, angle, startspeed, maxspeed, endspeed, addspeed = 0.3, brakeStart = 0.7, rotate_mode = 0, stopMethod = None, generator = None, stop = True):
        """
            This is the function that we use to make the robot turn the length of a specific angle or for the robot to turn until it senses a line. Even in this function the robot
            can accelerate and slow down. It also has Gyrosensor calibrations based on our experimental experience.
            Parameters
            -------------
            angle: The angle which the robot is supposed to turn. Use negative numbers to turn counterclockwise. Type: Integer. Default value: No default value
            startspeed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value
            maxspeed: The highest speed at which the robot drives. Type: Integer. Default: No default value
            endspeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value
            addspeed: The percentage after which the robot reaches the maxspeed. Type: Float. Default: No default value
            brakeStart: The percentage after which the robot starts slowing down until it reaches endspeed. Type: Float. Default: No default value
            rotate_mode: Different turning types. 0: Both motors turn, robot turns on the spot. 1: Only the outer motor turns, resulting in a corner. Type: Integer. Default: 0
            stopMethod: the Stopmethod the robot uses to stop. If no stopMethod is passed stopDistance is used instead. Default: stopDistance
            generator:  the generator that runs something parallel while driving. Default: No default value
            stop: the boolean that determines whether the robot should stop the motors after driving or not. Default: True
        """

        if cancel:
            return

        global run_generator, runSmall

        if generator == None:
            run_generator = False

        if rotate_mode == 0:
            startspeed = abs(startspeed)
            maxspeed = abs(maxspeed)
            endspeed = abs(endspeed)

        speed = startspeed

        #set standard variables
        rotatedDistance = 0
        steering = 1

        accelerateDistance = abs(angle * addspeed) 
        deccelerateDistance = abs(angle * (1 - brakeStart))

        #gyro sensor calibration
        angle = angle * (2400/2443) #experimental value based on 20 rotations of the robot

        #Setting variables based on inputs
        loop = True
        gyroStartValue = getGyroValue() #Yaw angle used due to orientation of the self.hub. This might need to be changed
        brakeStartValue = (angle + gyroStartValue) * brakeStart

        #Inversion of steering value for turning counter clockwise
        if angle < 0:
            steering = -1

        #Testing to see if turining is necessary, turns until loop = False

        while loop:
            if cancel:
                break

            if run_generator: #run parallel code execution
                next(generator)

            oldRotatedDistance = rotatedDistance
            rotatedDistance = getGyroValue() #Yaw angle used due to orientation of the self.hub. This might need to be changed
            speed = speedCalculation(speed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, abs(1), abs(0))
            
            
            #Checking for variants
            #Both Motors turn, robot moves on the spot
            if rotate_mode == 0:
                self.movement_motors.start_tank_at_power(int(speed) * steering, -int(speed) * steering)
            #Only outer motor turns, robot has a wide turning radius
            
            elif rotate_mode == 1:

                if angle * speed > 0:
                    self.leftMotor.start_at_power(- int(speed))
                else:
                    self.rightMotor.start_at_power(+ int(speed))

            if stopMethod != None:
                if stopMethod.loop():
                    loop = False
                    break
            elif abs(angle) <= abs(rotatedDistance - gyroStartValue):                   
                    loop = False
                    break



        #Stops movement motors for increased accuracy while stopping
        if stop:
            self.movement_motors.stop()

        run_generator = True
        runSmall = True

        return # End of gyroStraightDrive
