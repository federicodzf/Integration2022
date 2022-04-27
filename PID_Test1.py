##################################################################################################################
# PID Resources
        # https://projects.raspberrypi.org/en/projects/robotPID/4
        # Better https://github.com/m-lundberg/simple-pid/blob/master/simple_pid/PID.py
        # PID description and example: https://onion.io/2bt-pid-control-python/
        # Sample PID: https://projects.raspberrypi.org/en/projects/robotPID/0
        # https://github.com/ivmech/ivPID/blob/master/PID.py
        # PID control overview https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID
        
# NOTE TO ENGINEERS: PID is strictly for translational velocity
##################################################################################################################

# Import modules 
from open_motor_serial import open_motor
import time
import matplotlib.pyplot as plt
import numpy as np

# Define port for motor controller
port = "/dev/ttyACM1"
baudRate = 115200

# Initialize values and regulate motor control
class drive:
    # Initializing all values
    def __init__(self):
        # NOTE: Teensy Arduino Must Be Connected
        self.comms = open_motor()
        self.comms.init_serial_port(port, baudRate, 0.5) # Initializes serial port to transfer information from Teensy
        
        # Pulls information from motor
        self.response = []
        
        # Actual vel - initializes a veloicty value for both the left and right motors; left and right motors to be individually controlled
        self.a_vel_left = 0
        self.a_vel_right = 0
        
        # Initialize PWM value for motors
        self.pwm_left = 0
        self.pwm_right = 0

        # Gains based on trial and error - constant throughout PID process
        self.k_p = 0.05 # proportional gain
        self.k_d = 0.01 # derivative gain
        self.k_i = 0.0001 # integral gain
        
        # Look at integral windup in left and right motors
        # Left motor
        self.error_l = 0 # left error
        self.prev_error_l = 0
        self.sum_error_l = 0 # sum of left errors
        self.change_error_l = 0 # change in error
        # Right motor
        self.error_r = 0 # left error
        self.prev_error_r = 0
        self.sum_error_r = 0 # sum of left errors
        self.change_error_r = 0 # change in error
        
        self.dt = 0 # change in time
        self.de = 0 # change in error
        self.last_time = time.time() # Note, time.time() is simply more accurate than other time-grabbing commands
        
        # Call class motor_pid and inputs initial values to compute new ones as time progresses
        self.pid_left = motor_pid(self.error_l, self.prev_error_l, self.sum_error_l, self.change_error_l) # Calls class motor_pid and inputs initial values to compute new ones as time progresses
        self.pid_right = motor_pid(self.error_r, self.prev_error_r, self.sum_error_r, self.change_error_r)
        
        # Initialize structures
        self.actual_v_l = []
        self.actual_v_r = []
        self.set_v = []
        self.x_i = []

    def set_velocity(self, v_motor_l, v_motor_r): # Arugments are the velocity of left and right motor and used in function
    # PWM loop - PWM of motors vary according to the actual velocity of motor and what the desired velocity is
        self.pwm_left = self.pid_left.change_vel(v_motor_l, self.a_vel_left)
        self.pwm_right = self.pid_right.change_vel(v_motor_r, self.a_vel_right)
        
        print("pwm left", self.pwm_left) # Continuously print left motor's PWM values
        print("vel left", (self.a_vel_left*240/1425.1), "(rpm)") # Adjust RPM value to match GoBilda 117 rpm motors and print values
        
        print("pwm right", self.pwm_right) # Continuously print right motor's PWM values
        print("vel right", (self.a_vel_right*240/1425.1), "(rpm)")  # Adjust RPM value to match GoBilda 117 rpm motors and print values

        self.comms.send_pwm_goal( 0, -self.pwm_left, self.pwm_right, 0) # Set speed by calling function in open_motor_serial script
 
        self.get_velocity() # For the instance, call get_velocity function
        
        # Plotting for visualization - to assist with PID tuning
        r = np.size(self.set_v)
        plot_size = 50
        print("r", r) # Uncomment
        
        if r <= plot_size:
            self.actual_v_l = np.append(self.actual_v_l, self.a_vel_left)
            self.actual_v_r = np.append(self.actual_v_r, self.a_vel_right)
            self.set_v = np.append(self.set_v, v_motor_l)
            self.x_i = np.append(self.x_i, r)
            
            # Code commented out for prototyping
            # if r == plot_size:
               # self.pwm_left = 0.1
               # self.pwm_right = 0.1
                    # self.plot(self.x_i, self.set_v, self.actual_v_l, -self.actual_v_r) # Plot causes code to stop

    # Function necessary for plotting - to assist with PID tuning
    def plot(self, x, s_y, a_y_l, a_y_r): # Uses set y-value, gets actual_y_left-value, and actual_y_right-value
        fig, ax = plt.subplots()  # Create a figure containing a single axes
        ax.plot(x, s_y)
        ax.plot(x, a_y_l)
        ax.plot(x, a_y_r)
        plt.show()

    # Stop robot - set PWM to zero and halt motion
    def stop(self):
        self.comms.send_pwm_goal( 0, 0, 0, 0) # Sets motor speeds to zero
        self.pwm_left = 0 
        self.pwm_right = 0 

    # Responsible for getting the velocity of the motors by reading encoders
    def get_velocity(self):
        self.response = self.comms.get_response_json() # Returns json object of result
        print(self.response)
        self.a_vel_left = self.response.get('vel1')
        self.a_vel_right = self.response.get('vel2')
    
    # Resets PID controller
    def clear_pid(self):
        self.pid_left.clear()
        self.pid_right.clear()
        self.pwm_left = 0 
        self.pwm_right = 0 
 
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
        self.k_p = 0.08 # proportional gain
        self.k_d = 0.001 # derivative gain
        self.k_i = 0.0008 # integral gain
        
        # look at integral windup in left motor (used as reference)
        self.error_l = 0 # left error
        self.prev_error_l = 0
        self.sum_error_l = 0 # sum of left errors
        
        self.dt = 0 # change in time
        self.de = 0 # change in error
        self.last_time = time.time()
        
        self.pwm = 0
        
        self.sum_limit = (2000, -2000) # integral windup reset at limit
    
    # Use target and actual velocities to adjust motor speeds
    def change_vel(self, motor_v, actual_motor_v):
        # Get the time
        now = time.time()
        
        # P - Proportional error calculation
        self.error = motor_v - actual_motor_v # error target - actual
        
        # I - Integral error calculation
        self.sum_error += self.error # Integral component is sum of errors

        # D - Derivative error calculation
        self.change_error = self.error - self.prev_error
        self.dt = now - self.last_time # Change in time
        
        # Define previous error
        self.prev_error = self.error
        self.last_time = now     
        
        # Compute PID control variable using P, I, and D terms
        self.pwm += (self.error * self.k_p) + (self.sum_error * self.k_i * self.dt) + (self.change_error * self.k_d / self.dt)
        
        # Clamp limits a error value to a range to prevent script malfunction
        self.sum_error = self.clamp() 
        # print("sum erro check", self.sum_error) Used for testing
        
        return self.pwm
    
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
    def clear(self):
        self.sum_error = 0
        self.change_error = 0
        self.error = 0
        self.pwm = 0
