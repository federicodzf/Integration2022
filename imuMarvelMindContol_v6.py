#! /usr/bin/python3

baudRate = 115200
port = "/dev/ttyACM1"

import sys
import time

#Localization with the MarvelMind Beacon
from marvelmind import MarvelmindHedge

#Motor libraries
sys.path.append("../src/open_motor/")# This line points the python path to the open_motor module.
#This will need to be changed based on the location of your code.
from open_motor_serial import open_motor
comms = open_motor()
comms.init_serial_port(port,baudRate,0.5)

#Import IMU data
import board
import adafruit_bno055
i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)
last_val = 0xFFFF

sys.path.append('/home/pi/Desktop/PID')
import PID_Diff as Diff

import PID_Test1 as PID

global i
i = 0

theta=0

import numpy as np

def forward():
    comms.send_pwm_goal(0,-204, -200, 0)
    
def back():
    comms.send_pwm_goal(0,204, 200, 0)
    
def goLeft():
    comms.send_pwm_goal(0,150, -150, 0)
    
def goRight():
    comms.send_pwm_goal(0,-150, 150, 0)

def stop():
    comms.send_pwm_goal(0,0, 0, 0)
    
previous_sensor = 0
imuSensor = 0

def imuAngle(): # Makes it such that the code shouldn't get any non-float operand errors by ignoring any values that are non-float
    global previous_sensor 
    global imuSensor 
    
    imuSensor = sensor.euler[0] * (np.pi/180)
    # print("type", type(imuSensor))
    instance = isinstance(imuSensor, float)
    
    if instance == True:
        imuSensor = imuSensor
        # print("float")
    else:
        imuSensor = previous_sensor
        
    previous_sensor = imuSensor
    
    return imuSensor

#theta is orientation relative to its initial position
def initialOrientation(initialX, initialY, currX, currY):
    vectorX = currX - initialX
    vectorY = currY - initialY
    initialToCurrMag = np.sqrt(vectorX**2 + vectorY**2)
    theta = np.arccos((vectorX)/(initialToCurrMag))
    return theta
    
def rotate(theta, hedgeX, hedgeY, desX, desY): 
    n = 1 # Arbitrary magnitude of imu header (aligned with MarvelMind)
#     header = (hedgeX + n*np.cos(theta), hedgeY + n*np.sin(theta)) # Compute heading second point
    header = (np.cos(theta), np.sin(theta)) # Compute heading second point
    # print("header", header)
    # print("header", header)
#     headX = header[0] - hedgeX
#     headY = header[1] - hedgeY
#     headerMag = np.sqrt(headX**2 + headY**2)
    headerMag = np.sqrt(header[0]**2 + header[1]**2)
    desiredPathX = desX - hedgeX
    desiredPathY = desY - hedgeY
    desiredPathMag = np.sqrt(desiredPathX**2 + desiredPathY**2)
#     print("num", (header[0]*desiredPathX)*(header[1]*desiredPathY)) - print numerator
#     print("deno", desiredPathMag*headerMag) - print denominator
    IMURot = np.arccos(((header[0]*desiredPathX)+(header[1]*desiredPathY))/(desiredPathMag*headerMag))
    # IMURot = int(IMURot * (180/np.pi)) - Uncomment
    IMURotSign = np.cross([header[0], header[1]],[desiredPathX, desiredPathY]) # will  help determine the direction the robot has to rotate in
    
    # conditional for determining whether the robot has to turn a negative or positive amount
    if IMURotSign < 0:
        print("Negative")
        IMURot = -1*IMURot # changes sign of imu rotation amount to account for the cross product results. If negative could turn left
    else:
        print("Positive")
        IMURot = IMURot # If positive could turn right

    print("Turn IMU This Much (rad): ", IMURot, "(deg)", IMURot*(180/np.pi))
    print("The cross produced: ", IMURotSign)
    time.sleep(2)
    
    return IMURot

def main():
    hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=None, debug=False) # create MarvelmindHedge thread
    count = 0 #For Calibration of the Beacon
    
    if (len(sys.argv)>1): # Counts the number of arugements
        hedge.tty= sys.argv[1] # Command line arguments passed to script
    
    hedge.start() # start thread
    
    robot = Diff.robot()
            #Calibrate the initial position before entering the actual loop
    while count != 5:
        if(hedge.positionUpdated):
            initialX = hedge.position()[1]
            initialY = hedge.position()[2]
            print(initialX, initialY)
            count = count + 1
            
    # Below tells robot to move certain amount to get next position to compute orientation angle
    time.sleep(5)
    forward()
    time.sleep(2)
    #Move a certain distance
    stop()
    time.sleep(5)
    
    #Get new position from Marvelmind
    if(hedge.positionUpdated):
        xPos = hedge.position()[1]
        yPos = hedge.position()[2]
    
    #calculate the initial orientation
    orientAngle = initialOrientation(0, 0, 1, 1) # changed for testing orientAngle = initialOrientation(initialX, initialY, xPos, yPos) ;function calculates initialOrientation of robot. SHOULD RUN ONLY ONCE
    print("Initial Orientation", orientAngle)
    run = True
    
    #Calculate initial robot heading
    heading = robot.get_heading(orientAngle)
    print("Heading: ", heading)
    
    iteration = 1 #To run only once
    
    while run:
        
        DesPWMFB = 400 # Desired PWM value forward backward
        DesPWMLR = 250 # Desired PWM value left right

        try:

            endLocations = [[4.32,4.57],[4.2,4.9],[3.3,4.9]]

            # Determine the amount the imu must rotate after orienting it
            hedge.dataEvent.clear()  
            hedge.dataEvent.wait(1)
            if (hedge.positionUpdated):
                for location in endLocations:
                    # print("Now here")
                    time.sleep(1)
                    currX = hedge.position()[1]
                    currY = hedge.position()[2]
                    time.sleep(1)
                    
                    robot.set_locations(heading, currX, currY, location[0], location[1])
                    
                    if iteration == 1:
                        robot.set_velocity()
        
        except KeyboardInterrupt:
            stop()
            hedge.stop()  # stop and close serial port
            sys.exit()
                            
#         if run == True:    
#             print(speed)
#             robot_drive.set_velocity(speed[0], speed[1])
#             
#         # turn off drive if motors not running pwm = 0 -- determines how long after a motor has stopped until pid cleared
#         if speed[0] == 0 and speed[1] == 0:
#             global i
#             i += 1
#             print(i)
#             if i > 5: # Time before pid resets - tinker with values; have tried 40, 25, 20, 10. 40 too long, same with 25. Depends on how long we want the pid to "                                                      decelerate".
#                 robot_drive.clear_pid() # Sets pid values equal to zero
#                 i = 0
#         else:
#             i = 0 # Otherwise pid is not cleared
             
if __name__ == "__main__":
    main()

        



                  
            


 


