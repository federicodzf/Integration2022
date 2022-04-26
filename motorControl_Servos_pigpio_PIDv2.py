# Comment from Carter: When you run the code, make sure it says something similar to "can't initialize pigpio".
# This tells us that the proper tools needed for pigpio - namely "sudo pigpiod" - is running. To verify that the
# library is properly functioning, enter "pigs t" into the terminal; if a large number is returned everything SHOULD BE fine.

#! /usr/bin/python3

import sys
sys.path.append('/home/pi/Desktop/PID')
import PID_Test1 as PID

baudRate = 115200
port = "/dev/ttyACM1"

import sys
sys.path.append("../src/open_motor/")
# This line points the python path to the open_motor module.
# This will need to be changed based on the location of your code.

from open_motor_serial import open_motor
import time

# comms = open_motor()
# comms.init_serial_port(port,baudRate,0.5)

# importing pygame module
import pygame 

# importing sys module
import sys

# initialising pygame
pygame.init()

# creating display
display = pygame.display.set_mode((300, 300)) 

# import the GPIO module for the servos
import RPi.GPIO as GPIO
from time import sleep

# import the pigpio library for the servos
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

def main():

    # specify self to be motor.drive
    robot_drive = PID.drive()
    speed = (0, 0)
    
    run = True
    
    while run:
    
        DesPWMFB = 400 # Desired rpm value forward backward --- not corrected to GoBilda motors
        DesPWMLR = 250 # Desired rpm value left right
        
        # Cycles through all the events currently occuring
        for event in pygame.event.get():
            pressed_keys = pygame.key.get_pressed()
            # Condition becomes true when keyboard is pressed   
            if event.type == pygame.KEYDOWN:
     
              if event.key == pygame.K_w:
                  # Go forward
                  speed = (DesPWMFB, -DesPWMFB)
                  
              elif event.key == pygame.K_a:
                  # Go left
                  speed = (-DesPWMLR, -DesPWMLR)
                  
              elif event.key == pygame.K_d:
                  # Go right
                  speed = (DesPWMLR, DesPWMLR)
                  
              elif event.key == pygame.K_s:
                  # Go backwards
                   speed = (-DesPWMFB, DesPWMFB)
                  
              elif event.key == pygame.K_p:
                  # back()
                   speed = (-0, 0)
                  
              elif event.key == pygame.K_SPACE:
                  print( "0 deg" )
                  pwm_o.set_servo_pulsewidth( servo_o, 1800 ) ; # Trial and error for determining the pwm values.
                  sleep(1)
                  
              elif event.key == pygame.K_u: # Temporarily switched from KP_ENTER to U
                  print( "50 deg" )
                  pwm_o.set_servo_pulsewidth( servo_o, 1525 ) ;
                  sleep(1)
                  
              elif event.key == pygame.K_UP:
                  print( "0 deg" )
                  pwm.set_servo_pulsewidth( servo, 550 ) ;
                  sleep(1)
                  
              elif event.key == pygame.K_DOWN:
                  print( "100 deg" )
                  pwm.set_servo_pulsewidth( servo, 1500 ) ;
                  sleep(1)
                  
              elif event.key == pygame.K_ESCAPE:
                  sys.exit()

            if event.type == pygame.KEYUP: # Sets PWM equal to zero when the key is released
                speed = (0,0)
                            
        if run == True:    
            print(speed)
            robot_drive.set_velocity(speed[0], speed[1])
        
        # turn off drive if motors not running pwm = 0 -- determines how long after a motor has stopped until pid cleared
        if speed[0] == 0 and speed[1] == 0:
            global i
            i += 1
            print(i)
            if i > 5: # Time before pid resets - tinker with values; have tried 40, 25, 20, 10. 40 too long, same with 25. Depends on how long we want the pid to "                                                      decelerate".
                robot_drive.clear_pid() # Sets pid values equal to zero
                i = 0
        else:
            i = 0 # Otherwise pid is not cleared
            
if __name__ == "__main__":
    main() 

