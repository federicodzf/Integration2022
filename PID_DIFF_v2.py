# Import modules 
from open_motor_serial import open_motor # Get data from motors
import time
import math
import numpy as np

# Define port for motor controller
port = "/dev/ttyACM1"
baudRate = 115200

sys.path.append('/home/pi/Desktop/PID')
import PID_Test1 as PID

#######################################################################################
# Translational PID used as reference for building/modifying go-to-place PID controller
#######################################################################################

# Additional info to be used somewhere
# N relates to the number of ticks of the robot's motors - GoBilda 117 RPM (1425.1 PPR - pulse per rev)
# R = wheel radius (0.045 m = 45 mm)
# L = distance between center of wheels (approximately 320 mm = 12.6 in - roughly measured with ruler)

# Initialize values and regulate motor control
class robot():
    def __init__(self): 
        # Initialize gains based on trial and error - constant throughout PID process
        self.k_p = 1 # proportional gain
        self.k_d = 0 # derivative gain
        self.k_i = 0 # integral gain
        
        # Initializes serial port to transfer information from Teensy (via encoders connecting from the motor to motor driver)
        self.comms = open_motor()
        self.comms.init_serial_port(port, baudRate, 0.5)
        
        # Pulls information from motor
        self.response = []
        
        # Wheel characteristics necessary for kinematic equations
        self.R = 0.045 # Wheel radius in meters
        self.L = 0.320 # Distance between wheel centers in meters
        
        self.head_des = 0 # Set initial heading
        
        self.dt = 0 # change in time
        self.de = 0 # change in error
        self.last_time = time.time()   
        
        # Initialize error values
        self.error_h = 0 # Error between header and planned path
        self.prev_error_h = 0
        self.sum_error_h = 0
        self.change_error_h = 0
        
        # Initialize header and goal location
        self.header = []; 
        self.hedgeX = 0;
        self.hedgeY = 0;
        self.desX = 0;
        self.desY = 0;
        
        # Define magnitude of path between robot's current location and goal location
        self.desiredPathMag = 0
        
        # Initialize amount robot should rotate and corresponding sign
        self.IMURot = 0
        self.IMURotSign = 0
        
        # Call class motor_pid and inputs initial values to compute new ones as time progresses
        self.pid_motor = motor_pid(self.error_h, self.prev_error_h, self.sum_error_h, self.change_error_h)
        self.robot_drive = PID.drive()
        
    # Define locations the robot shall reach, in addition to its current position (Marvelmind based) and header
    def set_locations(self, head, hedgeX, hedgeY, x, y):
        self.header = head
        self.hedgeX = hedgeX
        self.hedgeY = hedgeY
        self.desX = x
        self.desY = y
        
    # Utilize kinematic equations to adjust the speed of motors ** Work in progress, tuning needed
    def set_velocity(self):
        self.leftVel, self.rightVel = self.pid_motor.change_vel(self.header, self.hedgeX, self.hedgeY, self.desX, self.desY)
        self.robot_drive.set_velocity(self.leftVel, self.rightVel) # Send computed velocity values to the motors - velocity calculated in change_vel function
       
    # Stop robot - set PWM to zero and halt motion
    def stop(self):
        self.comms.send_pwm_goal( 0, 0, 0, 0) # Tells Teensy to send 0 to each port on motor controller
    
    # Pass theta value to the calculateHead function to determine the robot's heading at an instant in time
    def get_heading(self,theta):
        return motor_pid.calculateHead(self,theta)
        
    # Resets PID controller   
    def clear_pid(self):
        self.comms.send_pwm_goal( 0, 0, 0, 0) 

# Responsible for motor-adjusting math
class motor_pid:
    def __init__(self, error, prev_error, sum_error, change_error):
        
        print("PID Initialized")
      
        # Imports error values
        self.error = error
        self.prev_error = prev_error
        self.sum_error = sum_error
        self.change_error = change_error
        
        # Define PID gain values
        self.k_p = 1 # proportional gain
        self.k_d = 0 # derivative gain
        self.k_i = 0 # integral gain
        
        # Wheel characteristics necessary for kinematic equations
        self.R = 0.045 # Wheel radius in meters
        self.L = 0.320 # Distance between wheel centers in meters
        self.vel_Linear = 20 # Define constant linear velocity ** Work in progress, tuning needed
        
        self.dt = 0 # change in time
        self.de = 0 # change in error
        self.last_time = time.time()
        
        # Define locations the robot shall reach and define magnitude of path between robot's current location and goal location
        self.header = []
        self.pathX = 0
        self.pathY = 0
        self.desiredPathMag = 0
        
        # Initialize amount robot should rotate and corresponding sign
        self.IMURot = 0
        self.IMURotSign = 0
        
        # Initialize variable responsible for changing motor values
        self.u = 0
        
        # Initialize PWM value for halting motor motion
        self.pwm = 0
        
        # Establish an integral windup limit to prevent PID controller malfunction
        self.sum_limit = (2000, -2000)
        
    # Define the robot's header
    def calculateHead(self, theta):
        self.header.append(np.cos(theta)) # X-value of vector
        self.header.append(np.sin(theta)) # Y-value of vector
        return self.header
            
    # Using the header, robot's current position, and desired position, use kinematic equations to control left and right motors
    def change_vel(self, header, hedgeX, hedgeY, desX, desY):        
        now = time.time()
        
        headerMag = np.sqrt((header[0])**2 + (header[1])**2) # Get heading magnitude
        
        # Define magnitude of path between robot's current position and the goal position
        desiredPathMag = np.sqrt((desX - hedgeX)**2 + (desY - hedgeY)**2)
        
        # Compute the magnitude of the angle between the robot's header and desired path - uses definition of dot product
        self.IMURot = np.arccos((((header[0])*(desX - hedgeX))+((header[1])*(desY - hedgeY)))/(desiredPathMag*headerMag))
       
        # Compute the direction of the angle between the robot's header and desired path - uses numpy of cross product
        self.IMURotSign = np.cross([header[0], header[1]],[(desX - hedgeX), (desY - hedgeY)]) # will  help determine the direction the robot has to rotate in
        
        # conditional for determining whether the robot has to turn a negative or positive amount
        if self.IMURotSign < 0:
            print("Negative")
            self.IMURot = -1*(self.IMURot) # changes sign of imu rotation amount to account for the cross product results
        else:
            print("Positive")
            self.IMURot = self.IMURot # If positive could turn right 
        
        # P - Proportional error calculation
        self.error = self.IMURot #ensure that error is within [-pi, pi]
        # print("error", self.error) Print values for testing

        # I - Integral error calculation
        self.sum_error += self.error * self.dt# Integral component is sum of errors
        # print("Current sum error: ", self.sum_error) Print values for testing

        # D - Derivative error calculation
        if self.dt == 0:
            self.change_error = (self.error - self.prev_error)/now
        else:
            self.change_error = (self.error - self.prev_error)/self.dt
        self.dt = now - self.last_time # Change in time
        
        # Define previous error
        self.prev_error = self.error
        self.last_time = now     
        
        # Compute PID control variable using P, I, and D terms
        self.u += (self.k_p * self.error) + (self.k_i * self.sum_error * self.dt) + (self.k_d * self.change_error / self.dt)
        # print("u", self.u) Print values for testing
        
        # Clamp limits a error value to a range to prevent script malfunction
        self.sum_error = self.clamp()
        
        # Use kinematic equations to define speed of left and right motors
        self.vel_l = (2*self.vel_Linear - self.u*self.L)/(2*self.R) # Set velocity of left motor based on PID calc
        self.vel_r = (2*self.vel_Linear + self.u*self.L)/(2*self.R) # Set velocity of right motor based on PID calc
        
        # print("left", self.vel_l) Print values for testing
        # print("right", self.vel_r) Print values for testing
        
        return self.vel_l, self.vel_r
    
    # Limit the range of intgral function for integral windup prevention
    def clamp(self):
        upper, lower = self.sum_limit
        value = self.sum_error
        
        if (value > upper):
            return upper
        elif (value < lower):
            return lower
        return value
        
    # Resets PID controller  
    def clear_pid(self):
        self.pid_motor.clear()
        self.comms.send_pwm_goal( 0, 0, 0, 0) 
