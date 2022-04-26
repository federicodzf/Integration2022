#! /usr/bin/python3

baudRate = 115200
port = "/dev/ttyACM0"

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

#PyGame module
import pygame
pygame.init()
display = pygame.display.set_mode((300, 300))

sys.path.append('/home/pi/Desktop/PID')
import PID_Test1 as PID

global i
i = 0

#Motor Control Comands
# def forward():
#     speed = (200, -200)
# #     
# def back():
#     speed = (-200, 200)
# #     
# def goLeft():
#     speed = (-200, -200)
# #     
# def goRight():
#     speed = (200, 200)
# #     
# def stop():
#    comms.send_pwm_goal(0,0, 0, 0)

def main():
    hedge = MarvelmindHedge(tty = "/dev/ttyACM1", adr=None, debug=False) # create MarvelmindHedge thread
    count = 0 #For Calibration of the Beacon
    
    if (len(sys.argv)>1): # Counts the number of arugements
        hedge.tty= sys.argv[1] # Command line arguments passed to script
    
    hedge.start() # start thread
    
    robot_drive = PID.drive()
    speed = (0, 0)
    
    run = True
    
    while run:
        
        DesPWMFB = 400 # Desired PWM value forward backward
        DesPWMLR = 250 # Desired PWM value left right
    
        try:
            hedge.dataEvent.wait(1)
            hedge.dataEvent.clear()
            
            #Desired locations delimited by a box 
            startX = [3.3, 3.4]
            startY = [3.7, 4.7]
            fX = [4.3, 4.7]
            fY = [3.7, 4.7]
            sX = [4.2, 4.40]
            sY = [4.90, 5.1]
            tX = [3.3, 3.5]
            tY = [4.90, 5.1]
            
            #Calibrate the initial position before entering the actual loop
            while count != 5:
                if(hedge.positionUpdated):
                    initialPos = hedge.position()[1]
                    count = count + 1
                    
            if (hedge.positionUpdated):
                xPos = hedge.position()[1]
                yPos = hedge.position()[2]
                print ("X position: ", xPos, "Y Pos: ", yPos)
                
                speed = (-100, -100) # What is this for?
                
                if(hedge.position()[1] > fX[0] and hedge.position()[1] < fX[1] and hedge.position()[2] > fY[0] and hedge.position()[2] < fY[1]):
                    speed = (0,0)
                    #time.sleep(4)
                    #speed = (200, -200)
                    #time.sleep(4)
                    #speed = (0, 0)
#             
#                 elif(hedge.position()[1] > sX[0] and hedge.position()[1] < sX[1] and hedge.position()[2] > sY[0] and hedge.position()[2] < sY[1]):
#                     speed = (0,0)
#                     time.sleep(1)
#                     speed = (200, -200)
#                     time.sleep(1)
#                     speed = (200, 200)
#                     
#                 elif(hedge.position()[1] > tX[0] and hedge.position()[1] < tX[1] and hedge.position()[2] > tY[0] and hedge.position()[2] < tY[1]):
#                     speed = (0,0)
#                     time.sleep(1)
#                     speed = (200, -200)
#                     time.sleep(1)
#                     speed = (200, 200)
#                     
#                 elif(hedge.position()[1] > startX[0] and hedge.position()[1] < startX[1] and hedge.position()[2] > startY[0] and hedge.position()[2] < startY[1]):
#                     speed = (0,0)
#                     time.sleep(1)
#                     speed = (200, -200)
#                     time.sleep(1)
#                     speed = (200, 200)

        except KeyboardInterrupt:
            hedge.stop()  # stop and close serial port
            sys.exit()
                                
        if run == True:    
            print(speed)
            robot_drive.set_velocity(speed[0], speed[1])
        
                      
if __name__ == "__main__":
    main()

        



                  
            


 