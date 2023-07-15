# LEGO type:standard slot:1 autostart
import math
from spike import PrimeHub, Motor, MotorPair, ColorSensor
from spike.control import wait_for_seconds, Timer
from hub import battery
hub = PrimeHub()
import hub as hub2
import sys

#######################      Resets    ###############################################
accelerate = True   #enable acceleration
lastAngle = 0
oldAngle = 0
gyroValue = 0
hub = PrimeHub()
pRegler = 0.0
iRegler = 0.0
dRegler = 0.0
pReglerLight = 0.0
iReglerLight = 0.0
dReglerLight = 0.0
pReglerLight = 1.6
iReglerLight = 0.009
dReglerLight = 16
accelerate = True
colorE = ColorSensor('E') #adjust the sensor ports until they match your configuration, we recommend assigning your ports to the ones in the program for ease of use
colorF = ColorSensor('F')
smallMotorC = Motor('C')
smallMotorD = Motor('D')
circumference = 17.6 #circumference of wheel in cm
sensordistance = 1 #distance between the two light sensors in cm.
cancel = False
inMain = True
####################################################################################


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
            
def speedCalculation(speed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, drivenDistance, oldDrivenDistance):
    addSpeedPerDegree = (maxspeed - startspeed) / accelerateDistance 
    subSpeedPerDegree = (maxspeed - endspeed) / deccelerateDistance
    

    subtraction = (abs(drivenDistance) - abs(oldDrivenDistance) if abs(drivenDistance) - abs(oldDrivenDistance) >= 1 else 1) * subSpeedPerDegree
    addition = (abs(drivenDistance) - abs(oldDrivenDistance) if abs(drivenDistance) - abs(oldDrivenDistance) >= 1 else 1) * addSpeedPerDegree

    if abs(drivenDistance) > abs(brakeStartValue):

        if abs(speed) > abs(endspeed):
            speed = speed - subtraction
            
    elif abs(speed) < abs(maxspeed):

        speed = speed + addition

    return speed

def pidCalculation(speed):
    global pRegler
    global iRegler
    global dRegler
    if speed > 0:
        pRegler = -0.17 * speed + 12.83
        iRegler = 12
        dRegler = 1.94 * speed - 51.9
        if pRegler < 3.2:
            pRegler = 3.2
    else:
        pRegler = (11.1 * abs(speed))/(0.5 * abs(speed) -7) - 20
        iRegler = 10
        #iRegler = 0.02
        dRegler = 1.15**(- abs(speed)+49) + 88
    
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

    def gyroRotation(self, angle, startspeed, maxspeed, endspeed, addspeed = 0.3, brakeStart = 0.7, rotate_mode = 0):

        rotatedDistance = 0
        steering = -1
        if angle>0:
            steering = 1
        accelerateDistance = abs(angle * addspeed) 
        deccelerateDistance = abs(angle * (1 - brakeStart))
        speed = startspeed  
        angle = angle * (2400/2443) #experimental value based on 20 rotations of the robot  #gyro sensor calibration
        gyroStartValue = hub.motion_sensor.get_yaw_angle() #Yaw angle used due to orientation of the self.hub. This might need to be changed
        brakeStartValue = (angle + gyroStartValue) * brakeStart
        oldRotatedDistance = rotatedDistance
        rotatedDistance = hub.motion_sensor.get_yaw_angle()
        speed = speedCalculation(speed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, rotatedDistance, oldRotatedDistance)
        if rotate_mode == 0: #Both Motors turn, robot moves on the spot
            self.movement_motors.start_tank_at_power(int(speed * steering), int(-1 * speed * steering))
        elif rotate_mode == 1:      #Only outer motor turns, robot has a wide turning radius
            if angle * speed > 0:
                self.leftMotor.start_at_power(int(speed)*-1)
            else:
                self.rightMotor.start_at_power(int(speed))

        if abs(angle) <= abs(rotatedDistance - gyroStartValue):
            loop = False
            break
        self.movement_motors.stop()
        return #END

    def gyroStraightDrive(self, distance, startspeed, maxspeed, endspeed, addspeed = 0.3, brakeStart = 0.7):
        """
           
            -------------
            distance: the distance that the robot is supposed to drive. Type: Integer. Default: No default value
            speed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value
            maxspeed: The highest speed at which the robot drives. Type: Integer. Default: No default value
            endspeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value
            addspeed: The speed which the robot adds in order to accelerate. Type: Float. Default: 0.2
            brakeStart: The value which we use to tell the robot after what percentage of the distance we need to slow down. Type: Float. Default: 0.8
            offset: The value sends the robot in a direction which is indicated by the value entered. Type: Integer. Default: 0

        """
        if cancel:
            return
        global pRegler, iRegler, dRegler
        speed = startspeed
        change = 0
        old_change = 0
        integral = 0
        steeringSum = 0
        invert = -1*abs(speed)/speed               
        #Sets values based on user inputs   
        gyroStartValue = hub.motion_sensor.get_yaw_angle()
        distance = abs(distance)
        rotateDistance = (distance / 17.6) * 360
        accelerateDistance = rotateDistance * addspeed
        deccelerateDistance = rotateDistance * (1 - brakeStart)
        #Calculation of braking point
        self.left_Startvalue = self.leftMotor.get_degrees_counted()
        self.right_Startvalue = self.rightMotor.get_degrees_counted()
        brakeStartValue = brakeStart * rotateDistance
        drivenDistance = getDrivenDistance(self)
        loop = True
        while loop:
            if cancel:
                break
            oldDrivenDistance = drivenDistance  #Calculation of driven distance and PID values
            drivenDistance = getDrivenDistance(self)
            pidCalculation(speed)
            change = hub.motion_sensor.get_yaw_angle() - gyroStartValue #yaw angle used due to orientation of the self.hub
            currenSteering = (change * pRegler + integral * iRegler + dRegler * (change - old_change)) + steeringSum*0.02
            currenSteering = max(-100, min(currenSteering, 100))
            print("steering: " + str(currenSteering) + " gyro: " + str(change) + " integral: " + str(integral))
            steeringSum += change
            integral += change - old_change
            old_change = change  #Calculation of speed based on acceleration and braking, calculation of steering value for robot to drive perfectly straight:
            speed = speedCalculation(speed, startspeed,maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, drivenDistance, oldDrivenDistance)
            
            self.movement_motors.start_at_power(int(speed), int(invert * currenSteering))

            if rotateDistance < drivenDistance:                   
                loop = False

        self.movement_motors.stop()
    
        return #End of gyroStraightDrive


#####################   Resets and Declarations   #################

hub2.button.right.callback(breakFunction)

smallMotorC.set_stall_detection(stop_when_stalled=True)
smallMotorD.set_stall_detection(stop_when_stalled=True)
accelerate = True
db = DriveBase(hub, 'A', 'B') #(A: left driver; B: right driver)
db.movement_motors.set_stop_action("hold") #hold motors on wait for increased reliability

#####################   RUN 2   #################

hub.right_button.wait_until_pressed()
hub.right_button.wait_until_released()
wait_for_seconds(0.2)
resetGyroValue()
db.gyroRotation(-40, 20, 25, 15, 0.25, 0.8, rotate_mode = 0)
db.gyroRotation(80, 20, 25, 15, 0.25, 0.8, rotate_mode = 0)


sys.exit("ended program successfully")

##################### END ####################################
