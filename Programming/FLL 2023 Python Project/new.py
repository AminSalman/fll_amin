# LEGO type:standard slot:1 autostart

import math
from spike import PrimeHub, Motor, MotorPair, ColorSensor
from spike.control import wait_for_seconds, Timer
from hub import battery
import hub as hub2
import sys

#Preperation for parallel code execution
accelerate = True
run_generator = True
runSmall = True

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
colorE = ColorSensor('E')
colorF = ColorSensor('F')
smallMotorC = Motor('C')
smallMotorD = Motor('D')
circumference = 17.6
sensordistance = 1 
cancel = False
inMain = True
hub.status_light.on('blue')

class DriveBase:

    def __init__(self, hub, leftMotor, rightMotor):
        self.hub = hub
        self.leftMotor = Motor(leftMotor)
        self.rightMotor = Motor(rightMotor)
        self.movement_motors = MotorPair(leftMotor, rightMotor) 

    def lineFollower(self, distance, startspeed, maxspeed, endspeed, sensorPort, side, addspeed = 0.2, brakeStart = 0.7 , stopMethod=None, generator = None, stop = True):
        """
            This is the function used to let the robot follow a line until either the entered distance has been achieved or the other sensor of the robot senses a line.
            Like all functions that drive the robot this function has linear acceleration and breaking. It also uses PID values that are automatically set depending on the
            current speed of the robot (See function PIDCalculationLight)
            Parameters
            -------------
            distance: The value tells the program the distance the robot has to drive. Type: Integer. Default: No default value
            speed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value
            maxspeed: The highest speed at which the robot drives. Type: Integer. Default: No default value
            endspeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value
            addspeed: The percentage after which the robot reaches its maxspeed. Type: Float. Default: No default value
            brakeStart: The value which we use to tell the robot after what percentage of the distance we need to slow down. Type: Float. Default: No default value
            stopMethod: the Stopmethod the robot uses to stop. If no stopMethod is passed stopDistance is used instead. Default: stopDistance
            generator:  the generator that runs something parallel while driving. Default: No default value
            stop: the boolean that determines whether the robot should stop the motors after driving or not. Default: True
        """
        
        if cancel:
            return

        global run_generator, runSmall

        if generator == None:
            run_generator = False

        #set the speed the robot starts at
        speed = startspeed
        #reset PID values to eliminate bugs
        change = 0
        old_change = 0
        integral = 0
        #reset the driven distance of the robot to eliminate bugs

        #specifies the color sensor
        colorsensor = ColorSensor(sensorPort)
        #Get degrees of motors turned before robot has moved, allows for distance calculation without resetting motors
        loop = True
        #Going backwards is not supported on our robot due to the motors then being in front of the colour sensors and the program not working
        if distance < 0:
            print('ERR: distance < 0')
            distance = abs(distance)
        #Calculate target values for the motors to turn to
        finalDistance = (distance / 17.6) * 360
        #Calculate after what distance the robot has to reach max speed
        accelerateDistance = finalDistance * addspeed
        deccelerateDistance = finalDistance * (1 - brakeStart)

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

        brakeStartValue = brakeStart * finalDistance
        while loop:
            if cancel:
                print("cancel")
                break
        
            if run_generator: #run parallel code execution
                next(generator)

            #Checks the driven distance as an average of both motors for increased accuracy
            oldDrivenDistance = drivenDistance
            drivenDistance = getDrivenDistance(self)
            #Calculates target value for Robot as the edge of black and white lines
            old_change = change

            change = colorsensor.get_reflected_light() - 50


            #Steering factor calculation using PID, sets new I value

        
            steering = (((change * pReglerLight) + (integral * iReglerLight) + (dReglerLight * (change - old_change)))) * invert
            integral = change + integral
            #Calculation of current speed for robot, used for acceleratiion, braking etc.
            speed = speedCalculation(speed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, drivenDistance, oldDrivenDistance)

            pidCalculationLight(speed)
            #PID value updates
            steering = max(-100, min(steering, 100))

            #Driving using speed values calculated with PID and acceleration for steering, use of distance check
            self.movement_motors.start_at_power(int(speed), int(steering))

            if stopMethod != None:
                if stopMethod.loop():
                    loop = False
            else:   
                if finalDistance < drivenDistance:
                    break

        if stop:
            self.movement_motors.stop()
            
        run_generator = True
        runSmall = True
        generator = 0
        return

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

    def gyroStraightDrive(self, distance, startspeed, maxspeed, endspeed, addspeed = 0.3, brakeStart = 0.7, stopMethod=None, offset = 0, generator = None, stop = True):
        """
            This is the function that we use to make the robot go forwards or backwards without drifting. It can accelerate, it can slow down and there's also PID. You can set the values
            in a way where you can either drive until the entered distance has been achieved or until the robot senses a line.
            Parameters
            -------------
            distance: the distance that the robot is supposed to drive. Type: Integer. Default: No default value
            speed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value
            maxspeed: The highest speed at which the robot drives. Type: Integer. Default: No default value
            endspeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value
            addspeed: The speed which the robot adds in order to accelerate. Type: Float. Default: 0.2
            brakeStart: The value which we use to tell the robot after what percentage of the distance we need to slow down. Type: Float. Default: 0.8
            port: This value tells the program whether the robot is supposed to check for a black line with the specified light snsor. Type: String. Default: 0
            lightValue: This value tells the program the value the robot should stop at if port sees it. Type: Integer. Default: 0
            align_variant: Tells the robot to align itself to a line if it sees one. 0: No alignment. 1: standard alignment. 2: tangent based alignment Type: Integer. Default: 0
            detectLineStart: The value which we use to tell the robot after what percentage of the distance we need to look for the line to drive to. Type: Float. Default: 0
            offset: The value sends the robot in a direction which is indicated by the value entered. Type: Integer. Default: 0
            generator: Function executed while robot is executing gyroStraightDrive. Write the wanted function and its parameters here. Type: . Default: 0
            stopMethod: the Stopmethod the robot uses to stop. If no stopMethod is passed stopDistance is used instead. Default: stopDistance
            generator:  the generator that runs something parallel while driving. Default: No default value
            stop: the boolean that determines whether the robot should stop the motors after driving or not. Default: True
        """

        if cancel:
            return

        global run_generator, runSmall
        global pRegler, iRegler, dRegler
        
        if generator == None:
            run_generator = False

        #Set starting speed of robot
        speed = startspeed
        #Sets PID values

        change = 0
        old_change = 0
        integral = 0
        steeringSum = 0

        invert = -1

        #Sets values based on user inputs
        loop = True


        gyroStartValue = getGyroValue()

        #Error check for distance
        if distance < 0:
            print('ERR: distance < 0')
            distance = abs(distance)

        #Calulation of degrees the motors should turn to
        #17.6 is wheel circumference in cm. You might need to adapt it
        rotateDistance = (distance / 17.6) * 360
        accelerateDistance = rotateDistance * addspeed
        deccelerateDistance = rotateDistance * (1 - brakeStart)

        #Inversion of target rotation value for negative values
        if speed < 0:
            invert = 1

        #Calculation of braking point
        self.left_Startvalue = self.leftMotor.get_degrees_counted()
        self.right_Startvalue = self.rightMotor.get_degrees_counted()
        brakeStartValue = brakeStart * rotateDistance
        drivenDistance = getDrivenDistance(self)

        while loop:
            if cancel:
                break
            if run_generator: #run parallel code execution
                next(generator)

            #Calculation of driven distance and PID values
            oldDrivenDistance = drivenDistance
            drivenDistance = getDrivenDistance(self)

            pidCalculation(speed)
            change = getGyroValue() - gyroStartValue #yaw angle used due to orientation of the self.hub


            currenSteering = (change * pRegler + integral * iRegler + dRegler * (change - old_change)) + offset + steeringSum*0.02

            currenSteering = max(-100, min(currenSteering, 100))
            #print("steering: " + str(currenSteering) + " gyro: " + str(change) + " integral: " + str(integral))

            steeringSum += change
            integral += change - old_change
            old_change = change

            #Calculation of speed based on acceleration and braking, calculation of steering value for robot to drive perfectly straight
            speed = speedCalculation(speed, startspeed,maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, drivenDistance, oldDrivenDistance)
            self.movement_motors.start_at_power(int(speed), invert * int(currenSteering))


            if stopMethod != None:
                if stopMethod.loop():
                    loop = False
            elif rotateDistance < drivenDistance:                   
                    loop = False


        if stop:
            self.movement_motors.stop()
    
        run_generator = True
        runSmall = True

        return #End of gyroStraightDrive

def resetGyroValue():
    global gyroValue
    hub2.motion.yaw_pitch_roll(0)

    gyroValue = 0

def getGyroValue():

    #this method is used to return the absolute gyro Angle and the angle returned by this method doesn't reset at 180 degree
    global lastAngle
    global oldAngle
    global gyroValue

    #gets the angle returned by the spike prime program. The problem is the default get_yaw_angle resets at 180 and -179 back to 0
    angle = hub.motion_sensor.get_yaw_angle()

    if angle != lastAngle:
        oldAngle = lastAngle
        
    lastAngle = angle

    if angle == 179 and oldAngle == 178:
        hub2.motion.yaw_pitch_roll(0)#reset
        gyroValue += 179
        angle = 0
    
    if angle == -180 and oldAngle == -179:
        hub2.motion.yaw_pitch_roll(0) #reset
        gyroValue -= 180   
        angle = 0

    return gyroValue + angle

def getDrivenDistance(data):


    #print(str(abs(data.leftMotor.get_degrees_counted() - data.left_Startvalue)) + " .:. " + str(abs(data.rightMotor.get_degrees_counted() - data.right_Startvalue)))

    drivenDistance = (
                    abs(data.leftMotor.get_degrees_counted() - data.left_Startvalue) + 
                    abs(data.rightMotor.get_degrees_counted() - data.right_Startvalue)) / 2

    return drivenDistance

def defaultClass(object, db):
    object.db = db
    object.leftMotor = db.leftMotor
    object.rightMotor = db.rightMotor

    object.left_Startvalue = abs(db.leftMotor.get_degrees_counted())
    object.right_Startvalue = abs(db.rightMotor.get_degrees_counted())
    return object

class stopMethods(): #This class has all our stopmethods for easier coding and less redundancy
    
    class stopLine():
        """
            Drive until a Line is detected
            Parameters
            -------------
            db: the drivebase of the robot
            port: Port to detect line on
            lightvalue: Value of the light to detect
            detectLineDistance: Distance until start detecting a line
            """
        def __init__(self, db, port, lightvalue, detectLineDistance):
            self = defaultClass(self, db)            

            self.port = port
            self.detectLineDistance = (detectLineDistance / 17.6) * 360

            #if lightvalue bigger 50 stop when lightvalue is higher
            self.lightvalue = lightvalue


        def loop(self):


            drivenDistance = getDrivenDistance(self)

            if abs(self.detectLineDistance) < abs(drivenDistance):
                if self.lightvalue > 50:
                    if ColorSensor(self.port).get_reflected_light() > self.lightvalue:
                        return True
                else:
                    if ColorSensor(self.port).get_reflected_light() < self.lightvalue:
                        return True

            return False
    
    class stopAlign():
        """
            Drive until a Line is detected
            Parameters
            -------------
            db: the drivebase of the robot
            port: Port to detect line on
            lightvalue: Value of the light to detect
            speed: speed at which the robot searches for other line
            """
        def __init__(self, db, lightvalue, speed):
            self = defaultClass(self, db)    
            self.speed = speed


            #if lightvalue bigger 50 stop when lightvalue is higher
            self.lightValue = lightvalue


        def loop(self):

            if colorE.get_reflected_light() < self.lightValue:
                self.rightMotor.stop()
                #Turning robot so that other colour sensor is over line
                while True:

                    self.leftMotor.start_at_power(-int(self.speed))

                    #Line detection and stopping
                    if colorF.get_reflected_light() < self.lightValue or cancel:
                        self.leftMotor.stop()
                        return True
                

            #Colour sensor F sees line first
            elif colorF.get_reflected_light() < self.lightValue:

                self.leftMotor.stop()

                #Turning robot so that other colour sensor is over line
                while True:
                    self.rightMotor.start_at_power(int(self.speed))

                    #Line detection and stopping
                    if colorE.get_reflected_light() < self.lightValue or cancel:
                        self.rightMotor.stop()
                        return True
            

            return False

    class stopTangens():
        """
            Drive until a Line is detected
            Parameters
            -------------
            db: the drivebase of the robot
            port: Port to detect line on
            lightvalue: Value of the light to detect
            speed: Distance until start detecting a line
            """
        def __init__(self, db, lightvalue, speed):
            self.count = 0
            self = defaultClass(self, db)    
            self.speed = speed
            #if lightvalue bigger 50 stop when lightvalue is higher
            self.lightValue = lightvalue
            self.detectedLineDistance = 0

            self.invert = 1
            if speed < 0:
                self.invert = -1
            
        def loop(self):
            drivenDistance = getDrivenDistance(self)
            if colorE.get_reflected_light() < self.lightValue:
                    #measuring the distance the robot has driven since it has seen the line
                    if(self.detectedLineDistance == 0):
                        self.detectedLineDistance = getDrivenDistance(self)
                        self.detectedPort = 'E'

                    elif self.detectedPort == 'F':
                        db.movement_motors.stop() #Stops robot with sensor F on the line
                        angle = math.degrees(math.atan(((drivenDistance - self.detectedLineDistance) / 360 * circumference) / sensordistance)) #Calculating angle that needs to be turned using tangent
                        #print("angle: " + str(angle))
                        db.gyroRotation(-angle, self.invert * self.speed, self.invert * self.speed, self.invert * self.speed, rotate_mode=1) #Standard gyrorotation for alignment, but inverting speed values if necessary

                        db.movement_motors.stop() #Stopping robot for increased reliability
                        return True

                #Colour sensor F sees line first
            elif colorF.get_reflected_light() < self.lightValue:
                #measuring the distnace the robot has driven since it has seen the line
                if(self.detectedLineDistance == 0):
                    self.detectedLineDistance = drivenDistance
                    self.detectedPort = 'F'

                elif self.detectedPort == 'E':
                    db.movement_motors.stop() #Stops robot with sensor E on the line
                    angle = math.degrees(math.atan(((drivenDistance - self.detectedLineDistance) / 360 * circumference) / sensordistance)) #Calculation angle that needs to be turned using tangent
                    db.gyroRotation(angle, self.invert * self.speed, self.invert * self.speed, self.invert * self.speed, rotate_mode=1) #Standard gyrorotation for alignment, but inverting speed values if necessary
                    db.movement_motors.stop() #Stopping robot for increased reliablity
                    return True

            return False
    class stopDegree():
        """
            Roates until a certain degree is reached
            Parameters            
            -------------
            db: the drivebase of the robot
            angle: the angle to rotate
        """
        def __init__(self, db, angle):
            self.angle = angle * (336/360)
            
            self.gyroStartValue = getGyroValue() #Yaw angle used due to orientation of the self.hub.
            

        def loop(self):
            rotatedDistance = getGyroValue() #Yaw angle used due to orientation of the self.hub. 

            if abs(self.angle) <= abs(rotatedDistance - self.gyroStartValue):
                return True
            else:
                return False

    class stopTime():

        """
            Drive until a certain time is reached
            Parameters
            -------------
            db: the drivebase of the robot
            time: the time to drive
        """

        def __init__(self, db, time) -> None:
            self = defaultClass(self, db)
            self.time = time
            self.timer = Timer()
            self.startTime = self.timer.now()

        def loop(self):
            if self.timer.now() > self.startTime + self.time:
                return True
            else:
                return False
       
    class stopResistance():

        """
            Drive until the Robot doesn't move anymore
            Parameters
            -------------
            db: the drivebase of the robot
            restistance: the value the resistance has to be below to stop              
        """
        def __init__(self, db, resistance):
            self = defaultClass(self, db)
            self.resistance = resistance
            self.timer = Timer()
            
            self.startTime = 0
            self.lower = False
            self.runs = 0

        def loop(self):

            self.runs += 1
            motion = abs(hub2.motion.accelerometer(True)[2])

            if motion < self.resistance:
                self.lower = True

            if self.runs > 15:
                if self.lower:
                    if self.startTime == 0:
                        self.startTime = self.timer.now()

                    if self.timer.now() > self.startTime:
                        return True

                else:
                    self.lower = False
                    return False
                
def motorResistance(speed, port, resistancevalue):
    """
    lets the motor stop when it hits an obstacle
    """
    if abs(resistancevalue) > abs(speed):
        return

        
    if cancel:
        return

    if port == "A":
        smallMotorC.start_at_power(speed)
        while True:
            old_position = smallMotorC.get_position()
            wait_for_seconds(0.4)
            if abs(old_position - smallMotorC.get_position())<resistancevalue or cancel:
                smallMotorC.stop()
                print("detected stalling")
                return

    elif port == "D":
        smallMotorD.start_at_power(speed)
        while True:
            old_position = smallMotorD.get_position()
            wait_for_seconds(0.4)
            if abs(old_position - smallMotorD.get_position())<resistancevalue or cancel:
                smallMotorD.stop()
                print("detected stalling")
                return
    else:
        print("wrong port selected. Select A or D")
        return

def speedCalculation(speed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, drivenDistance, oldDrivenDistance):
    """
        Used to calculate all the speeds in out programs. Done seperatly to reduce redundancy. Brakes and accelerates
        Parameters
        -------------
        speed: The current speed the robot has
        startspeed: Speed the robot starts at. Type: Integer. Default: No default value.
        maxspeed: The maximum speed the robot reaches. Type: Integer. Default: No default value.
        endspeed: Speed the robot aims for while braking, minimum speed at the end of the program. Type: Integer. Default: No default value.
        addspeed: Percentage of the distance after which the robot reaches the maximum speed. Type: Integer. Default: No default value.
        brakeStartValue: Percentage of the driven distance after which the robot starts braking. Type: Integer. Default: No default value.
        drivenDistance: Calculation of the driven distance in degrees. Type: Integer. Default: No default value.
    """    

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

def breakFunction(args):
    """
    Allows you to manually stop currently executing round but still stays in main. 
    This is much quicker and more reliable than pressing the center button.
    """
    global cancel, inMain
    if not inMain:
        cancel = True

def pidCalculation(speed):
    #golbally sets PID values based on current speed of the robot, allows for fast and accurate driving
    global pRegler
    global iRegler
    global dRegler
    #Important note: These PID values are experimental and based on our design for the robot. You will need to adjust them manually. You can also set them statically as you can see below
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
    
def pidCalculationLight(speed):
    #Sets the PID values for the lineFollower based on current speed. Allows for accurate and fast driving
    #Important note: these PID values are experimental and based on our design for the robot. You will need to adjust them. See above on how to do so
    global pReglerLight
    global dReglerLight

    pReglerLight = -0.04 * speed + 4.11
    dReglerLight = 0.98 * speed - 34.2
    #set hard bottom for d value, as otherwise the values don't work
    if dReglerLight < 5:
        dReglerLight = 5

def driveMotor(rotations, speed, port):
    """
    Allows you to drive a small motor in parallel to driving with gyroStraightDrive
    Parameters
    -------------
    rotations: the rotations the motor turns
    speed: the speed at which the motor turns
    port: the motor used. Note: this cannot be the same motors as configured in the motor Drivebase
    """
           
    global runSmall
    global run_generator

    if cancel:
        runSmall = False
        run_generator = False

    while runSmall:
        smallMotor = Motor(port)
        smallMotor.set_degrees_counted(0)

        loop_small = True
        while loop_small:
            drivenDistance = smallMotor.get_degrees_counted()
            smallMotor.start_at_power(speed)
            if (abs(drivenDistance) > abs(rotations) * 360):
                loop_small = False
            if cancel:
                loop_small = False
            yield

        smallMotor.stop()
        runSmall = False
        run_generator = False
    yield

hub2.motion.yaw_pitch_roll(0)

db = DriveBase(hub, 'A', 'B') #this lets us conveniently hand over our motors (B: left driver; C: right driver). This is necessary for the cancel function

def exampleOne():
    db.gyroRotation(-45, 10, 20, 10, addspeed = 0.3, brakeStart = 0.7, rotate_mode = 1)
    return

class bcolors:
    BATTERY = '\033[32m'
    BATTERY_LOW = '\033[31m'

    ENDC = '\033[0m'

pReglerLight = 1.6
iReglerLight = 0.009
dReglerLight = 16

accelerate = True

hub2.button.right.callback(breakFunction)
gyroValue = 0

#Battery voltage printout in console for monitoring charge
if battery.voltage() < 8000: #set threshold for battery level
    print(bcolors.BATTERY_LOW + "battery voltage is too low: " + str(battery.voltage()) + " \n ----------------------------- \n >>>> please charge robot <<<< \n ----------------------------- \n"+ bcolors.ENDC)
else:
    print(bcolors.BATTERY + "battery voltage: " + str(battery.voltage()) + bcolors.ENDC)


#User Interface in Program for competition and instant program loading
main = True


programselect = 1 #Set the attachment the selection program starts on
hub.light_matrix.write(programselect)
db.movement_motors.set_stop_action("hold") #hold motors on wait for increased reliability



while main:
    cancel = False
    inMain = True

    #Program selection
    
    if hub.right_button.is_pressed(): #press right button to cycle through programs. cycling back isn't supported yet, but we are working on reallocating the buttons in the file system
        hub.right_button.wait_until_released()
        programselect = programselect + 1
        hub.light_matrix.write(programselect) #show current selcted program
        hub.speaker.beep(45, 0.05) #give audio feedback for user

        if programselect == 1:
            hub.status_light.on('blue')
        elif programselect == 2:
            hub.status_light.on('green')
        elif programselect == 3:
            hub.status_light.on('yellow')
        elif programselect == 4:
            hub.status_light.on('white')
        elif programselect == 5:
            hub.status_light.on('red')        
        elif programselect == 6:
            hub.status_light.on('orange')
        elif programselect == 7:
            hub.status_light.on('pink')
        elif programselect == 8:
            hub.status_light.on('green')
        #cycle to start of stack
        if programselect == 9:
            programselect = 1
            hub.light_matrix.write(programselect)
            hub.status_light.on('blue')

    #Program start
    if hub.left_button.is_pressed():
        
        inMain = False
        hub.light_matrix.show_image("DUCK")
        hub.left_button.wait_until_released()
        
        if programselect == 1:
            hub.status_light.off()            
            db.gyroStraightDrive(40,10,40,10)
            cancel = True
            programselect = 1
            hub.light_matrix.write(programselect)
        elif programselect == 2:
            hub.status_light.off()
            exampleOne()
            programselect = 2
            hub.light_matrix.write(programselect)
        elif programselect == 3:
            hub.status_light.off()
            exampleOne()
            programselect = 3
            hub.light_matrix.write(programselect)
        elif programselect == 4:
            hub.status_light.off()
            exampleOne()
            programselect = 4
            hub.light_matrix.write(programselect)
        elif programselect == 5:
            hub.status_light.off()
            exampleOne()
            programselect = 5
            hub.light_matrix.write(programselect)
        elif programselect == 6:
            hub.status_light.off()
            exampleOne() 
            programselect = 6
            hub.light_matrix.write(programselect)

sys.exit("ended program successfully")