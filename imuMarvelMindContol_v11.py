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
import PID_Diff_v2 as Diff
import PID_Test1_Diff as PID

import pigpio
from os import system # Crucial do not remove. Necessary for loading in the sudo pigpiod --- pigpio library launched as a demon, hence why we use "sudo pigpiod".
system("sudo pigpiod") # Do not remove, if removed you must manually enter "sudo pigpiod" into the terminal each time the code is ran.

#Initializing servos
servo_o = 24 # pigpio uses the GPIO pins and not 
pwm_o = pigpio.pi() 
pwm_o.set_mode(servo_o, pigpio.OUTPUT)
pwm_o.set_PWM_frequency(servo_o, 50)
#Setup the servo in the base to GPIO 24, or Pin 18
servo = 23
pwm = pigpio.pi() 
pwm.set_mode(servo, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo, 50)

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

pi = np.pi

def imuAngle(): # Makes it such that the code shouldn't get any non-float operand errors by ignoring any values that are non-float
    global previous_sensor 
    global imuSensor 
    
    imuSensor = sensor.euler[0]
    
    print("angle", imuSensor)
    # print("type", type(imuSensor))
    instance = isinstance(imuSensor, float)
    
    if instance == True:
        imuSensor = imuSensor
        # print("float")
    else:
        imuSensor = previous_sensor
        
    previous_sensor = imuSensor
    
    return (imuSensor * (np.pi/180))

#theta is orientation relative to its initial position
def initialOrientation(initialX, initialY, currX, currY):
    vectorX = currX - initialX
    print("vectorX", vectorX)
    vectorY = currY - initialY
    print("vectorY", vectorY)
    initialToCurrMag = np.sqrt(vectorX**2 + vectorY**2)
    print("mag", initialToCurrMag)
    theta = np.arccos((vectorX)/(initialToCurrMag))
    sign = np.cross([1, 0], [vectorX, vectorY])
    print("sign", sign)
    
    if sign < 0:
        theta = -1*theta

    print("theta", theta)
    return theta


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
    time.sleep(3)
    forward()
    time.sleep(1.5)
    #Move a certain distance
    stop()
    time.sleep(5)
    
    #Get new position from Marvelmind
    if(hedge.positionUpdated):
        xPos = hedge.position()[1]
        yPos = hedge.position()[2]
        
    print("currX ", xPos,"currY ", yPos)
    #calculate the initial orientation
    orientAngle = initialOrientation(initialX, initialY, xPos, yPos) # changed for testing orientAngle = initialOrientation(initialX, initialY, xPos, yPos) ;function calculates initialOrientation of robot. SHOULD RUN ONLY ONCE
    print("Initial Orientation", orientAngle)
    run = True
    #Set IMU angle to that initial orientation
    imu = imuAngle() + orientAngle
    print("imu angle", imu)
    if imu > 2*np.pi:
        imu = imu - 2*np.pi
        
    #Calculate initial robot heading
    heading = robot.get_heading(imu)
    print("heading:", heading)
    iteration = 1 #To run only once
    initX = [0.279, 0.304] # What is this used for?
    initY = [4.394, 4.405]
    
    global j
    j = 0
    
    global c
    c = 0
    
    while run:
        
        DesPWMFB = 400 # Desired PWM value forward backward
        DesPWMLR = 250 # Desired PWM value left right
        
        try:
            fX = [3.70, 3.80]
            fY = [4.51, 4.80]
            sX = [3.70, 3.80]
            sY = [1.67, 1.757]
            # endLocations = [[3.80, 4.50] ,[3.67, 1.83], [7.02, 1.48],[3.67, 1.83], [3.80, 4.50]]
            endLocations = [[2.66, 4.07] ,[3.62, 4.79], [5.00, 4.01], [5.97, 4.49], [5.00, 4.01], [3.62, 4.79], [2.66, 4.07]]

            # Determine the amount the imu must rotate after orienting it
            hedge.dataEvent.clear()  
            hedge.dataEvent.wait(1)
            if (hedge.positionUpdated):
                
                if j < len(endLocations):
                    time.sleep(0.01)
                    
                    print(j)
                    currX = hedge.position()[1]
                    currY = hedge.position()[2]
                    print("vectorX between curr and goal:", (endLocations[j][0] - currX)) # What if we tried setting an i-value? The i initally set to 0. After the while loop has exited, we add one to i? The i value is passed into endLocations[i][#]
                    print("vectorY between curr and goal:", (endLocations[j][1] - currY))
                    
                    robot.set_locations(heading, currX, currY, endLocations[j][0],endLocations[j][1]) # robot.set_locations(heading, currX, currY, location[0], location[1])
                        
                    # Constantly update the heding using the IMU
                    heading = robot.get_heading(imuAngle() + orientAngle)
                    print("heading", heading)
                    robot.set_velocity()
                    
                    if (currX > (endLocations[j][0] - 0.15) and currX < (endLocations[j][0] + 0.15) and currY > (endLocations[j][1] - 0.15) and currY < (endLocations[j][1] + 0.15)):
                        print("goal")
#                         if j == len(endLocations):
#                             print("No Go")
#                             stop()
#                             hedge.stop()
#                         else:
                        j += 1
                        print(j)
                        print("Here")
                    
                    if (c == 0 and currX > (endLocations[3][0] - 0.15) and currX < (endLocations[3][0] + 0.15) and currY > (endLocations[3][1] - 0.15) and currY < (endLocations[3][1] + 0.15)):
                        stop()
                        time.sleep(2)
                        print('waiting')
                        pwm_o.set_servo_pulsewidth( servo_o, 1800 ); # Trial and error for determining the pwm values.
                        time.sleep(1)
                        pwm.set_servo_pulsewidth( servo, 1650 );
                        time.sleep(1)
                        pwm_o.set_servo_pulsewidth( servo_o, 1525 );
                        time.sleep(2)
                        pwm.set_servo_pulsewidth( servo, 550 );
                        
                        c = 1
                        print(c)
                    
                    print(c)
                    
                    if j == len(endLocations):
                        stop()
                        time.sleep(1)
                        pwm.set_servo_pulsewidth( servo, 1500 );
                        time.sleep(1)
                        pwm_o.set_servo_pulsewidth( servo_o, 1800 );
                        hedge.stop()
                
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

        



                  
            


 




