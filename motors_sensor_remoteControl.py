# Note: When you run the code, make sure it says something similar to "can't initialize pigpio".
# This tells us that the proper tools needed for pigpio - namely "sudo pigpiod" - is running. To verify that the
# library is properly functioning, enter "pigs t" into the terminal; if a large number is returned everything SHOULD BE fine.

#! /usr/bin/python3

baudRate = 115200
port = "/dev/ttyACM1"

import sys
sys.path.append("../src/open_motor/")
# This line points the python path to the open_motor module.
# This will need to be changed based on the location of your code.

from open_motor_serial import open_motor
import time

comms = open_motor()
comms.init_serial_port(port,baudRate,0.5)

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
# import the pigpio library for the servos
import pigpio
from os import system # Crucial do not remove. Necessary for loading in the sudo pigpiod --- pigpio library launched as a demon, hence why we use "sudo pigpiod".
system("sudo pigpiod") # Do not remove, if removed you must manually enter "sudo pigpiod" into the terminal each time the code is ran.

#Initializing servos
servo_o = 24 # pigpio uses the GPIO pins and not 
pwm_o = pigpio.pi() 
pwm_o.set_mode(servo_o, pigpio.OUTPUT)
pwm_o.set_PWM_frequency( servo_o, 50 )
#Setup the servo in the base to GPIO 24, or Pin 18
servo = 23
pwm = pigpio.pi() 
pwm.set_mode(servo, pigpio.OUTPUT)
pwm.set_PWM_frequency( servo, 50 )
 
#For Ultrasonic Sensors
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER_1= 17
GPIO_ECHO_1 = 27
GPIO_TRIGGER_2= 6
GPIO_ECHO_2 = 13
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER_1, GPIO.OUT)
GPIO.setup(GPIO_ECHO_1, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_2, GPIO.OUT)
GPIO.setup(GPIO_ECHO_2, GPIO.IN)

def distanceUS(echo, trig):
    # set Trigger to HIGH
    GPIO.output(trig, True)
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(trig, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(echo) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(echo) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s) and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance

# Motor fucntions without PID controller
#black cord of motor has to go on top 
def forward():
    comms.send_pwm_goal(0,-203, -200, 0)
    
def back():
    comms.send_pwm_goal(0,203, 200, 0)
    
def goLeft():
    comms.send_pwm_goal(0,150, -150, 0)
    
def goRight():
    comms.send_pwm_goal(0,-150, 150, 0)

def stop():
    comms.send_pwm_goal(0,0, 0, 0)
  
#Main function
def main():
    while True:
        dist1 = distanceUS(GPIO_ECHO_1, GPIO_TRIGGER_1)
        print ("Measured Distance from USR = %.1f cm" % dist1)
        dist2 = distanceUS(GPIO_ECHO_2, GPIO_TRIGGER_2)
        print ("Measured Distance from USL = %.1f cm" % dist2)
        time.sleep(1)
        
        if dist1 <= 40 or dist2 <= 40:
            back()
            time.sleep(1)
            goRight()
            time.sleep(2.5)
            stop()
        
        # Pygame module to handle keyboard input
        for event in pygame.event.get():
            pressed_keys = pygame.key.get_pressed()
            # Condition becomes true when keyboard is pressed   
            if event.type == pygame.KEYDOWN:
     
              if event.key == pygame.K_w:
                  forward()
                  
              elif event.key == pygame.K_a:
                  goLeft()
                  
              elif event.key == pygame.K_d:
                  goRight()
                  
              elif event.key == pygame.K_s:
                  back()
                  
              elif event.key == pygame.K_q:
                  stop()
                  sys.exit()
                  
              elif event.key == pygame.K_SPACE:
                  print( "0 deg" )
                  pwm_o.set_servo_pulsewidth( servo_o, 1800 ) ; # Trial and error for determining the pwm values.
                  time.sleep(1)
                  
              elif event.key == pygame.K_KP_ENTER:
                  print( "50 deg" )
                  pwm_o.set_servo_pulsewidth( servo_o, 1525 ) ;
                  time.sleep(1)
                  
              elif event.key == pygame.K_UP:
                  print( "0 deg" )
                  pwm.set_servo_pulsewidth( servo, 550 ) ;
                  time.sleep(1)
                  
              elif event.key == pygame.K_DOWN:
                  print( "100 deg" )
                  pwm.set_servo_pulsewidth( servo, 1600 ) ;
                  time.sleep(1)
                  
              elif event.key == pygame.K_ESCAPE:
                  stop()
                  sys.exit()

            if event.type == pygame.KEYUP:
                stop()            

if __name__ == "__main__":
    main()
