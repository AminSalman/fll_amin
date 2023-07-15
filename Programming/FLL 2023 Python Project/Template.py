# LEGO type:standard slot:1 autostart
import math
from spike import PrimeHub, Motor, MotorPair, ColorSensor
from spike.control import wait_for_seconds, Timer
from hub import battery
hub = PrimeHub()
import hub as hub2
import sys

accelerate = True   #enable acceleration
lastAngle = 0
oldAngle = 0
gyroValue = 0
hub = PrimeHub()
kP = 0.0
kI = 0.0
kD = 0.0
kPLight = 0.0
kILight = 0.0
kDLight = 0.0
kPLight = 1.6
kILight = 0.009
kDLight = 16
accelerate = True
colorE = ColorSensor('E') #adjust the sensor ports until they match your configuration, we recommend assigning your ports to the ones in the program for ease of use
colorF = ColorSensor('F')
smallMotorC = Motor('C')
smallMotorD = Motor('D')
circumference = 17.6 #circumference of wheel in cm
sensordistance = 1 #distance between the two light sensors in cm.
cancel = False
inMain = True

def resetGyroValue():
    global gyroValue
    hub.motion_sensor.reset_yaw_angle()
    gyroValue = 0

def getDrivenDistance(data):


    #print(str(abs(data.leftMotor.get_degrees_counted() - data.left_Startvalue)) + " .:. " + str(abs(data.rightMotor.get_degrees_counted() - data.right_Startvalue)))

    drivenDistance = (
                    abs(data.leftMotor.get_degrees_counted() - data.left_Startvalue) + 
                    abs(data.rightMotor.get_degrees_counted() - data.right_Startvalue)) / 2

    return drivenDistance
            
def speedCalculation(speed, startSpeed, maxSpeed, endSpeed, accelerateDistance, deccelerateDistance, deccPValue, drivenDistance, oldDrivenDistance):
    accPPerDegree = (maxSpeed - startSpeed) / accelerateDistance 
    subSpeedPerDegree = (maxSpeed - endSpeed) / deccelerateDistance
    

    subtraction = (abs(drivenDistance) - abs(oldDrivenDistance) if abs(drivenDistance) - abs(oldDrivenDistance) >= 1 else 1) * subSpeedPerDegree
    addition = (abs(drivenDistance) - abs(oldDrivenDistance) if abs(drivenDistance) - abs(oldDrivenDistance) >= 1 else 1) * accPPerDegree

    if abs(drivenDistance) > abs(deccPValue):

        if abs(speed) > abs(endSpeed):
            speed = speed - subtraction
            
    elif abs(speed) < abs(maxSpeed):

        speed = speed + addition

    return speed

def pidCalculation(speed):
    global kP
    global kI
    global kD
    if speed > 0:
        kP = -0.17 * speed + 12.83
        kI = 12
        kD = 1.94 * speed - 51.9
        if kP < 3.2:
            kP = 3.2
    else:
        kP = (11.1 * abs(speed))/(0.5 * abs(speed) -7) - 20
        kI = 10
        #kI = 0.02
        kD = 1.15**(- abs(speed)+49) + 88
    
def pidCalculationLight(speed):
    global kPLight
    global kDLight
    kPLight = -0.04 * speed + 4.11
    kDLight = 0.98 * speed - 34.2
    if kDLight < 5:
        kDLight = 5

def breakFunction(args):
    global cancel, inMain
    if not inMain:
        cancel = True

class DriveBase:

    def __init__(self, hub, leftMotor, rightMotor):
        self.hub = hub
        self.leftMotor = Motor(leftMotor)
        self.rightMotor = Motor(rightMotor)
        self.movement_motors = MotorPair(leftMotor, rightMotor) 

    def lineFollower(self, distance, startSpeed, maxSpeed, endSpeed, sensorPort, side, accP = 0.2, deccP = 0.7):
        """
            This is the function used to let the robot follow a line until either the entered distance has been achieved or the other sensor of the robot senses a line.
            Like all functions that drive the robot this function has linear acceleration and breaking. It also uses PID values that are automatically set depending on the
            current speed of the robot (See function PIDCalculationLight)
            Parameters
            -------------
            distance: The value tells the program the distance the robot has to drive. Type: Integer. Default: No default value
            speed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value
            maxSpeed: The highest speed at which the robot drives. Type: Integer. Default: No default value
            endSpeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value
            accP: The percentage after which the robot reaches its maxSpeed. Type: Float. Default: No default value
            deccP: The value which we use to tell the robot after what percentage of the distance we need to slow down. Type: Float. Default: No default value
        """        
        if cancel:
            return
        
        speed = startSpeed      #set the speed the robot starts at
        change = 0
        old_change = 0      #reset PID values to eliminate bugs
        integral = 0
        #reset the driven distance of the robot to eliminate bugs
        colorsensor = ColorSensor(sensorPort)        #specifies the color sensor
        loop = True
        finalDistance = (distance / 17.6) * 360             #Calculate target values for the motors to turn to       #Get degrees of motors turned before robot has moved, allows for distance calculation without resetting motors
        accelerateDistance = finalDistance * accP        #Calculate after what distance the robot has to reach max speed
        deccelerateDistance = finalDistance * (1 - deccP)

        invert = 1

        #Calculation of steering factor, depending on which side of the line we are on
        if side == "left":
            invert = 1
        elif side == "right":
            invert = -1
        
        #Calculation of the start of the robot slowing down
        self.left_Startvalue = self.leftMotor.get_degrees_counted()
        self.right_Startvalue = self.rightMotor.get_degrees_counted()
        drivenDistance = getDrivenDistance(self)
        deccPValue = deccP * finalDistance

        while loop:

            if cancel:
                print("cancel")
                break
        
            #Checks the driven distance as an average of both motors for increased accuracy
            oldDrivenDistance = drivenDistance
            drivenDistance = getDrivenDistance(self)
            #Calculates target value for Robot as the edge of black and white lines
            old_change = change

            change = colorsensor.get_reflected_light() - 50
      
            steering = (((change * kPLight) + (integral * kILight) + (kDLight * (change - old_change)))) * invert
            integral = change + integral
            speed = speedCalculation(speed, startSpeed, maxSpeed, endSpeed, accelerateDistance, deccelerateDistance, deccPValue, drivenDistance, oldDrivenDistance)
            pidCalculationLight(speed)            
            steering = max(-100, min(steering, 100))
            #Driving using speed values calculated with PID and acceleration for steering, use of distance check
            
            self.movement_motors.start_at_power(int(speed), int(steering))

            if finalDistance < drivenDistance:
                break
        self.movement_motors.stop()
        return

    def gyroRotation(self, targetAngle, startSpeed, maxSpeed, endSpeed, accP = 0.3, deccP = 0.7, rotate_mode = 0):    
        global cancel

        direction = targetAngle/abs(targetAngle)
        rotatedDistance = 0
        accelerateDistance = abs(targetAngle * accP) 
        deccelerateDistance = abs(targetAngle * (1 - deccP))
        speed = startSpeed  
        targetAngle = targetAngle * (2400/2443) #experimental value based on 20 rotations of the robot  #gyro sensor calibration
        gyroStartValue = hub.motion_sensor.get_yaw_angle() #Yaw angle used due to orientation of the self.hub. This might need to be changed
        deccPValue = targetAngle* deccP
        loop = True     

        while loop:
            
            if hub.right_button.is_pressed():
                cancel = True
        
            if cancel:
                break
            
            oldRotatedDistance = rotatedDistance
            rotatedDistance = hub.motion_sensor.get_yaw_angle()
            speed = speedCalculation(speed, startSpeed, maxSpeed, endSpeed, accelerateDistance, deccelerateDistance, deccPValue, rotatedDistance, oldRotatedDistance)
        
           
            if rotate_mode == 0: #Both Motors turn, robot moves on the spot
                self.movement_motors.start_tank_at_power(int(abs(speed) * direction), -int(abs(speed) * direction))
        
            elif rotate_mode == 1:      #Only outer motor turns, robot has a wide turning radius
                if targetAngle * speed > 0:
                    self.leftMotor.start_at_power(int(speed)*-1)
                else:
                    self.rightMotor.start_at_power(int(speed))

            if (rotatedDistance >= targetAngle and direction == 1) or (rotatedDistance <= targetAngle and direction == -1):
                loop = False
                break
        self.movement_motors.stop()
        return #END

    def gyroStraightDrive(self, distance, startSpeed, maxSpeed, endSpeed, accP = 0.3, deccP = 0.7):
        """
           
            -------------
            distance: the distance that the robot is supposed to drive. Type: Integer. Default: No default value
            speed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value
            maxSpeed: The highest speed at which the robot drives. Type: Integer. Default: No default value
            endSpeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value
            accP: The speed which the robot adds in order to accelerate. Type: Float. Default: 0.2
            deccP: The value which we use to tell the robot after what percentage of the distance we need to slow down. Type: Float. Default: 0.8
            offset: The value sends the robot in a direction which is indicated by the value entered. Type: Integer. Default: 0

        """
        global kP, kI, kD, cancel
        if cancel:
            return        
        speed = startSpeed
        change = 0
        old_change = 0
        integral = 0
        steeringSum = 0
        invert = -1*abs(speed)/speed               
        #Sets values based on user inputs   
        gyroStartValue = hub.motion_sensor.get_yaw_angle()
        distance = abs(distance)
        rotateDistance = (distance / 17.6) * 360
        accelerateDistance = rotateDistance * accP
        deccelerateDistance = rotateDistance * (1 - deccP)
        #Calculation of braking point
        self.left_Startvalue = self.leftMotor.get_degrees_counted()
        self.right_Startvalue = self.rightMotor.get_degrees_counted()
        deccPValue = deccP * rotateDistance
        drivenDistance = getDrivenDistance(self)
        loop = True
        while loop:
            if hub.right_button.is_pressed():
                cancel = True
            if cancel:
                break
            oldDrivenDistance = drivenDistance  #Calculation of driven distance and PID values
            drivenDistance = getDrivenDistance(self)
            pidCalculation(speed)
            change = hub.motion_sensor.get_yaw_angle() - gyroStartValue #yaw angle used due to orientation of the self.hub
            currenSteering = (change * kP + integral * kI + kD * (change - old_change)) + steeringSum*0.02
            currenSteering = max(-100, min(currenSteering, 100))
            print("steering: " + str(currenSteering) + " gyro: " + str(change) + " integral: " + str(integral))
            steeringSum += change
            integral += change - old_change
            old_change = change  #Calculation of speed based on acceleration and braking, calculation of steering value for robot to drive perfectly straight:
            speed = speedCalculation(speed, startSpeed,maxSpeed, endSpeed, accelerateDistance, deccelerateDistance, deccPValue, drivenDistance, oldDrivenDistance)
            
            self.movement_motors.start_at_power(int(speed), int(invert * currenSteering))

            if rotateDistance < drivenDistance:                   
                loop = False

        self.movement_motors.stop()
    
        return #End of gyroStraightDrive

hub2.button.right.callback(breakFunction)
resetGyroValue()
smallMotorC.set_stall_detection(stop_when_stalled=True)
smallMotorD.set_stall_detection(stop_when_stalled=True)
accelerate = True
db = DriveBase(hub, 'A', 'B') #(A: left driver; B: right driver)
db.movement_motors.set_stop_action("hold") #hold motors on wait for increased reliability

db.gyroStraightDrive(50, 15, 40, 25, 0.3, 0.7)
sys.exit("ended")
        

