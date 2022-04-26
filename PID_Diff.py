from open_motor_serial import open_motor # Get data from motors
import time
import math
import numpy as np
# Port /dev/ttyACM0,1, or 2 is Motor Controller
port = "/dev/ttyACM1"
baudRate = 115200

#######################################################################################
# Translational PID used as reference for building/modifying go-to-place PID controller
#######################################################################################

# Additional info to be used somewhere
# N relates to the number of ticks of the robot's motors - GoBilda 117 RPM (1425.1 PPR - pulse per rev)
# R = wheel radius (0.045 m = 45 mm)
# L = distance between center of wheels (approximately 320 mm = 12.6 in - roughly measured with ruler)

class robot():
    def __init__(self): 
        # Gains based on trial and error - constant throughout PID process
        self.k_p = 1 # proportional gain
        self.k_d = 0 # derivative gain
        self.k_i = 0 # integral gain
        
        # Initializes serial port to transfer info from Teensy
        self.comms = open_motor()
        self.comms.init_serial_port(port, baudRate, 0.5)
        
        # Pulls info from motor
        self.response = []
        
        # Wheel characteristics
        self.R = 0.045 # Wheel radius in meters
        self.L = 0.320 # Distance between wheel centers in meters
        
        self.head_des = 0 # Set initial desired heading
        
        self.dt = 0 # change in time
        self.de = 0 # change in error
        self.last_time = time.time() # Note, time.time() is simply more accurate than other time-grabbing commands   
        
        self.error_h = 0 # heading error
        self.prev_error_h = 0
        self.sum_error_h = 0 # sum of heading errors
        self.change_error_h = 0 # change in heading error
        
        self.header = [];
        self.hedgeX = 0;
        self.hedgeY = 0;
        self.desX = 0;
        self.desY = 0;
        
        self.desiredPathX = 0
        self.desiredPathY = 0
        self.desiredPathMag = 0
        
        self.IMURot = 0
        self.IMURotSign = 0
        
        self.pid_motor = motor_pid(self.error_h, self.prev_error_h, self.sum_error_h, self.change_error_h) # Calls class motor_pid and inputs initial values to compute new ones as time progresses
    
    def set_locations(self, head, hedgeX, hedgeY, x, y):
        self.header = head
        self.hedgeX = hedgeX
        self.hedgeY = hedgeY
        self.desX = x
        self.desY = y
        
    def set_velocity(self): # Arugments are the velocity of left and right motor and used in function - ISSUES WITH PASSING COMPUTED RPM TO PWM
        # vel loop
#         print("vel left", (self.vel.l*240/1425.1), "(rpm)") # Display the velocity of the left motor based on the kinematic equation
#         print("vel right", (self.vel.r*240/1425.1), "(rpm)") # Display the velocity of the right motor based on the kinematic equation
        self.leftVel, self.rightVel = self.pid_motor.change_vel(self.header, self.hedgeX, self.hedgeY, self.desX, self.desY)
        self.comms.send_vel_goal( 0, self.leftVel, self.rightVel, 0) # Send computed velocity values to the motors - velocity calculated in change_vel function
       
    # stop robot - set PWM to zero and halt motion
    def stop(self):
        self.comms.send_pwm_goal( 0, 0, 0, 0) # Tells Teensy to send 0 to each port on motor controller
    
    def get_heading(self,theta):
        return motor_pid.calculateHead(self,theta)
        
    # Resets PID controller - Look into more    
    def clear_pid(self):
        self.pid_left.clear()
        self.pid_right.clear()
        self.comms.send_pwm_goal( 0, 0, 0, 0) 

# Used to get the speed the motor should be going at
class motor_pid:
    def __init__(self, error, prev_error, sum_error, change_error): #, error, prev_error, change_error):
        print("PID Initialized")
        
        # imports
        self.error = error
        self.prev_error = prev_error
        self.sum_error = sum_error
        self.change_error = change_error
        
        self.k_p = 1 # proportional gain -- alter proportional gain to increase the initial acceleration and deceleration rate --- set btwn 0.1-0.07; no right answer ---- current ideal: kp = 0.08; Kd = 0.001; ki = 0.0008
        self.k_d = 0 # derivative gain
        self.k_i = 0 # integral gain
        
        # Wheel characteristics
        self.R = 0.045 # Wheel radius in meters
        self.L = 0.320 # Distance between wheel centers in meters
        self.vel_Linear = 20
        
        # look at integral windup
        
#         self.error_h = 0 # heading error
#         self.prev_error_h = 0
#         self.sum_error_h = 0 # sum of heading errors
#         self.change_error_h = 0 # change in heading error
        
        self.dt = 0 # change in time
        self.de = 0 # change in error
        self.last_time = time.time()
        
        self.header = []
        self.pathX = 0
        self.pathY = 0
        self.desiredPathMag = 0
        
        self.IMURot = 0
        self.IMURotSign = 0
        
        self.u = 0 # value responsible for changing the motors values
        
        self.pwm = 0
        
        self.sum_limit = (2000, -2000) # integral windup reset at limit
        
    # Define the robot's header
    def calculateHead(self, theta):
        self.header.append(np.cos(theta))
        self.header.append(np.sin(theta)) # Already a vector
        return self.header
            
    def change_vel(self, header, hedgeX, hedgeY, desX, desY):        
        # Define time
        print("test", header[1])
        now = time.time()
        headerMag = np.sqrt((header[0])**2 + (header[1])**2)
        # Define vector between robot's current position and the goal position
#         self.pathX = (desX - hedgeX)
#         self.pathY = (desY - hedgeY)
        desiredPathMag = np.sqrt((desX - hedgeX)**2 + (desY - hedgeY)**2)
        
        self.IMURot = np.arccos((((header[0])*(desX - hedgeX))+((header[1])*(desY - hedgeY)))/(desiredPathMag*headerMag))
        # IMURot = int(IMURot * (180/np.pi)) - Uncomment
        self.IMURotSign = np.cross([header[0], header[1]],[(desX - hedgeX), (desY - hedgeY)]) # will  help determine the direction the robot has to rotate in
        
        # conditional for determining whether the robot has to turn a negative or positive amount
        if self.IMURotSign < 0:
            print("Negative")
            self.IMURot = -1*(self.IMURot) # changes sign of imu rotation amount to account for the cross product results. If negative could turn left
        else:
            print("Positive")
            self.IMURot = self.IMURot # If positive could turn right 
        
        # P - Proportional
        self.error = self.IMURot #ensure that error is within [-pi, pi]
        print("error", self.error)

        # I - Integral
        self.sum_error += self.error * self.dt# Integral component is sum of errors
        # print("Current sum error: ", self.sum_error)

        # D - Derivative
        # change in time
        if self.dt == 0:
            self.change_error = (self.error - self.prev_error)/now
        else:
            self.change_error = (self.error - self.prev_error)/self.dt # change in error
        self.dt = now - self.last_time # change in time
        
        # Define previous error
        self.prev_error = self.error
        self.last_time = now     
        
        self.u += (self.k_p * self.error) + (self.k_i * self.sum_error * self.dt) + (self.k_d * self.change_error / self.dt)
        print("u", self.u)
        #print("pwm", self.pwm)
        
        self.sum_error = self.clamp() # Clamp limits a value to a range.
        # print("sum error check", self.sum_error)
        
        self.vel_l = (2*self.vel_Linear - self.u*self.L)/(2*self.R) # Set velocity of left motor based on PID calc
        self.vel_r = (2*self.vel_Linear + self.u*self.L)/(2*self.R) # Set velocity of right motor based on PID calc
        
        print("left", self.vel_l)
        print("right", self.vel_r)
        
        return self.vel_l, self.vel_r
    
    # limit the range of intgral function -- for integral windup prevention
    def clamp(self):
        upper, lower = self.sum_limit
        value = self.sum_error
        
        if (value > upper):
            return upper
        elif (value < lower):
            return lower
        return value
        
    # Resets PID controller -- Look into    
    def clear_pid(self):
        self.pid_motor.clear()
        self.comms.send_pwm_goal( 0, 0, 0, 0) 
