from open_motor_serial import open_motor # Get data from motors
import time
import matplotlib.pyplot as plt
import numpy as np

# PID is strictly for velocity

# Port /dev/ttyACM0 is Motor Controller
port = "/dev/ttyACM1"
baudRate = 115200

class drive: # Responsible for telling wheel what speed to reach
    def __init__(self): # -- Initializing all values to zero at the start of the code.
        # Teensy Arduino Must Be Connected
        self.comms = open_motor()
        self.comms.init_serial_port(port, baudRate, 0.5) # Initializes serial port to transfer info from Teensy
        
        self.response = [] # What is this doing?
        
        # actual vel - initializes a veloicty value for both the left and right motors; left and right motors to be individually regulated
        self.a_vel_left = 0
        self.a_vel_right = 0
        
        # pwm - initializes a PWM value for motors - set equal to zero at time equals zero denoting no motion.
        self.pwm_left = 0
        self.pwm_right = 0
        
        # PID initialized values
        # https://projects.raspberrypi.org/en/projects/robotPID/4
        # Better https://github.com/m-lundberg/simple-pid/blob/master/simple_pid/PID.py
    # PID description and example: https://onion.io/2bt-pid-control-python/
    # Sample PID: https://projects.raspberrypi.org/en/projects/robotPID/0
    # https://github.com/ivmech/ivPID/blob/master/PID.py
    # PID control overview https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID

    # Gains based on trial and error - constant throughout PID process; hence why they are non-zero
        self.k_p = 0.05 # proportional gain
        self.k_d = 0.01 # derivative gain
        self.k_i = 0.0001 # integral gain
        
        # look at integral windup - integral windup is presence of large error
        # Have to independently analyze the left and right motor since the signals sent are different (hence why drifting occurs)
        # left motor
        self.error_l = 0 # left error
        self.prev_error_l = 0
        self.sum_error_l = 0 # sum of left errors
        self.change_error_l = 0 # change in error
        # right motor
        self.error_r = 0 # left error
        self.prev_error_r = 0
        self.sum_error_r = 0 # sum of left errors
        self.change_error_r = 0 # change in error
        
        self.dt = 0 # change in time
        self.de = 0 # change in error
        self.last_time = time.time() # Note, time.time() is simply more accurate than other time-grabbing commands
        
        self.pid_left = motor_pid(self.error_l, self.prev_error_l, self.sum_error_l, self.change_error_l) # Calls class motor_pid and inputs initial values to compute new ones as time progresses
        self.pid_right = motor_pid(self.error_r, self.prev_error_r, self.sum_error_r, self.change_error_r)
        
        self.actual_v_l = []
        self.actual_v_r = []
        self.set_v = []
        self.x_i = []

    def set_velocity(self, v_motor_l, v_motor_r): # Arugments are the velocity of left and right motor and used in function - clarify how parameters are found
    # pwm loop
        self.pwm_left = self.pid_left.change_vel(v_motor_l, self.a_vel_left) # PWM of motor varies according to the actual velocity of motor and what the desired velocity is (a_vel is actual and v_motor is desired)
        self.pwm_right = self.pid_right.change_vel(v_motor_r, self.a_vel_right)
        
        print("pwm left", self.pwm_left) # Uncomment
        print("vel left", (self.a_vel_left*240/1425.1), "(rpm)") # Uncomment
        
        print("pwm right", self.pwm_right) # Uncomment
        print("vel right", (self.a_vel_right*240/1425.1), "(rpm)") # Uncomment
#         print("pwm right", self.pwm_right)
        # pwm(motor 0, motor 1, motor 2, motor 3)
        self.comms.send_pwm_goal( 0, -self.pwm_left, self.pwm_right, 0) # Make sure the sign of the right pwm is oppposite the left, otherwise error will accumulate and desired pwm cannot be reached 
    # self.comms.send_pwm_goal( 0, 0, 0, 0) # ERROR with setting pwm_goal to zero and also getting velocity
#         if(v_motor_l != 0 and v_motor_r != 0):
        self.get_velocity()
        
        # Plotting
        r = np.size(self.set_v)
        #         print("r", r)
        #         r = 0
        # Change plot_size if you want it to plot over longer time
        plot_size = 50
        
        print("r", r) # Uncomment
        
        if r <= plot_size:
            self.actual_v_l = np.append(self.actual_v_l, self.a_vel_left)
            self.actual_v_r = np.append(self.actual_v_r, self.a_vel_right)
            self.set_v = np.append(self.set_v, v_motor_l)
            self.x_i = np.append(self.x_i, r)
            
#             if r == plot_size:
# #                 self.pwm_left = 0.1
# #                 self.pwm_right = 0.1
#                 self.plot(self.x_i, self.set_v, self.actual_v_l, -self.actual_v_r) # Plot causes code to stop

    def plot(self, x, s_y, a_y_l, a_y_r): # set_y, actual_y_left, actual_y_right
        fig, ax = plt.subplots()  # Create a figure containing a single axes.
        ax.plot(x, s_y)
        ax.plot(x, a_y_l)
        ax.plot(x, a_y_r)
        plt.show()

    # stop robot  - set PWM to zero and halt motion
    def stop(self):
        self.comms.send_pwm_goal( 0, 0, 0, 0)
        self.pwm_left = 0 
        self.pwm_right = 0 

    def get_velocity(self):
        print()
        #print("get vel")
        self.response = self.comms.get_response_json() # Returns json object of result - no need for json
        print(self.response)
        #print(type(self.comms.get_response_json()))
        self.a_vel_left = self.response.get('vel1') # Clarify what this line is accomplishing and the one below it
        self.a_vel_right = self.response.get('vel2')
        #print("response" + self.response)
        #print("0" + self.response[0])
        
        # {} of response is dictionarie
        # print(self.response['pos1'])
        
        #print(self.a_vel_left)
        #print(self.a_vel_right)
        #print(self.response)
        
#   first try pid loop      
#     def pwm(self, motor):
#     # dt
#         now = time.time()
#     # p 
#         #print("set vel")
#         self.error_l = motor - self.a_vel_left # error target - actual
#         print(self.a_vel_left)        
#     # i
#         self.sum_error_l += self.error_l
#     # d
#         # change in time
#         self.de = self.error_l - self.prev_error_l # change in error
#         self.dt = now - self.last_time # change in time
#         
#                 # previous error
#         self.prev_error_l = self.error_l
#         self.last_time = now
        
    def clear_pid(self): # Resets PID controller
        self.pid_left.clear()
        self.pid_right.clear()
        self.pwm_left = 0 
        self.pwm_right = 0 
 
class motor_pid: # Used to get the speed the motor should be going at
    def __init__(self, error, prev_error, sum_error, change_error): #, error, prev_error, change_error):
        print("PID Initialized")
        
        # imports
        self.error = error
        self.prev_error = prev_error
        self.sum_error = sum_error
        self.change_error = change_error
        
        self.k_p = 0.08 # proportional gain -- alter proportional gain to increase the initial acceleration and deceleration rate --- set btwn 0.1-0.07; no right answer ---- current ideal: kp = 0.08; Kd = 0.001; ki = 0.0008
        self.k_d = 0.001 # derivative gain
        self.k_i = 0.0008 # integral gain
        
        # look at integral windup
        
        self.error_l = 0 # left error
        self.prev_error_l = 0
        self.sum_error_l = 0 # sum of left errors
        
        self.dt = 0 # change in time
        self.de = 0 # change in error
        self.last_time = time.time()
        
        self.pwm = 0
        
        self.sum_limit = (2000, -2000) # integral windup reset at limit
    
    def change_vel(self, motor_v, actual_motor_v):
        #("change vl")
        # dt
        now = time.time()
        # p 
        self.error = motor_v - actual_motor_v # error target - actual
#         print("s_m_v", motor_v) # Clarify s = set
#         print("a_m_v", actual_motor_v)
#         print(self.error)
        # i
        self.sum_error += self.error # Integral component is sum of errors
#         print(self.sum_error)
        # d
        # change in time
        self.change_error = self.error - self.prev_error # change in error
        self.dt = now - self.last_time # change in time
        
        # previous error
        self.prev_error = self.error
        self.last_time = now     
        
        
        self.pwm += (self.error * self.k_p) + (self.sum_error * self.k_i * self.dt) + (self.change_error * self.k_d / self.dt)
        #print("pwm", self.pwm)
        self.sum_error = self.clamp() # Clamp limits a value to a range.
#         print("sum erro check", self.sum_error)
        
        return self.pwm
    
    # limit the range of intgral function -- for integral windup, correct?
    def clamp(self):
        upper, lower = self.sum_limit
        value = self.sum_error
        
        if (value > upper):
            return upper
        elif (value < lower):
            return lower
        return value
    
    # clear pid parameters - this is done so the cycle can be repeated
    def clear(self):
        self.sum_error = 0
        self.change_error = 0
        self.error = 0
        self.pwm = 0