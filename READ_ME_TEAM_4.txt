###########################################################

READ ME

ENGINEERING INTEGRATION II SOURCE CODE TEAM 4

CREATION DATE: 04/27/2022
LAST UPDATE: 04/27/2022

###########################################################

----------------------------------------------------------------------------------------------------------------------------------
Section 1: Code Responsible for the Manual Control of the Robot (Relates to motorControl_Servos_pigpio_PIDv2.py and PID_Test1.py):
----------------------------------------------------------------------------------------------------------------------------------

The script "motorControl_Servos_pigpio_PIDv2.py" is responsible for remote-control function of Team 4's "Capture the Flag" robot. The code utilizes the time, pygame, sys, RPi.GPIO, and pigpio modules.

The code interfaces with a translational PID controller script (stored in separated file and appended), "PID_Test1.py", to reduce the deviation of the robot during straight-line motion. Utilizing the PID controller and pygame module, clicking the w,a,s, and d (forward, left, backward, and right motion, respectively) keys on a laptop/wireless keyboard while the cursor is placed on a custom GUI induces motion of two GoBilda 117 rpm motors.

Inducing motion in the flag lifting mechanism is like the process implemented in motor control. Utilizing the pygame module, clicking the space, keypad-enter, up, arrow, and down arrow (close claw, open claw, raise lifting arm, lower lifting arm, respectively) on a laptop/wireless keyboard while the cursor is placed on a custom GUI induces motion of one 35-kg rated servo (base of lifting arm) and 25-kg rated servo (claw). The script utilizes the pigpio library to reduce servo-jitter. Properly using the pigpio library requires entering "sudo pigpiod" into the terminal (because it is classified as a 'daemon'). Alternatively, incorperating the command "from os import system", followed by "system("sudo pigpiod")" reduces this need. Running the script initially (after starting the Raspberry PI) yields an error. This error stems from the Pi not recognizing the pigpio library during the scripts first launch. Stopping the script and restarting solves the error, as the system has now completely loaded the pigpio module. Running the script, a warning will be produced claiming pigpio cannot be initialized. This warning indicates the module has been loaded and is functioning (the system is attempting to overwrite the loaded module with itself - no errors are caused by this).

Code is sent to the PID script ("PID_Test1.py") using the code  

	if run == True:    
            print(speed)
            robot_drive.set_velocity(speed[0], speed[1])

Defined linear velocity values for the left and right motors (different according to whether the robot shall move forward/backward or left/right) are set to speed[0] and speed[1] when a key is pressed. "robot_drive" has access to all functions in the "drive()" class of the "PID_Test1.py" script, bridging the remote control code to the PID controller.

The PID controller is responsible for performing all math necessary for correcting the robot's left and right motors. Corrected motor values are sent to "open_motor_serial.py" to set the motor speeds (through the management provided Teensy microcontroller). This is accomplished using the command:  "self.comms.send_pwm_goal( 0, -self.pwm_left, self.pwm_right, 0)", as found in the PID controller.

NOTE: "open_motor_serial.py" was provided by management.
NOTE: Downloading all modules not standard with the Raspberry PI Bullseye OS can be accomplished using "sudo apt install" in the terminal.

---------------------------------------------------------------------------------------------------
Section 2: Code Responsible for Live Feed Video of Raspberry Pi Camera (Relates to CameraVideo.py):
---------------------------------------------------------------------------------------------------

The script "CameraVideo.py" is responsible for displaying a live-feed view of the management provided Raspberry Pi camera. 

The code requires OpenCV to be downloaded onto the Raspberry Pi (installation tutorial by Robu.in followed). The time module must also be downloaded onto the Raspberry Pi - this feature comes pre-installed with the latest Bullseye OS. Lastly, the Picamera and PiRGBArray modules must be downloaded. Methods for downloading are listed in Furthermore, the "camera" interface must be enabled in the configurations. This is accomplished by following the steps:

	1) From the main Raspberry Pi menu, choose "Preferences"
	2) Choose the "Interfaces" tab
	3) Select the camera module
	4) Restart Pi

Utilizing classes and functions within the Picamera and PiRGBArray (accomplished using the "from" command) allows for integration with OpenCV to establish a feedrate and pixel-image that constantly changes.

---------------------------------------------------------------------------------------------------------------------------------------------
Section 3 - Code Responsible for Combining Remote Control and Ultrasonic Sensor Object Detection (Relates to motors_sensor_remoteControl.py):
---------------------------------------------------------------------------------------------------------------------------------------------

The script "motors_sensor_remoteControl.py" is responsible for interfacing a basic object detection script with the methods discussed in Section 1. The following section will focus on the needs of the ultrasonic sensors to properly function as intended. Ultrasonic sensors will assist with the robot's "scout mode" capabilities; however, using it with the remote-control script allows for easy prototyping.

Relative to the code provided in Section 1, no other modules are required to be imported/installed. 

Currently, the ultrasonic sensors are able to induce movement when the robot comes within 50cm of an object. Establishing connection between the Raspberry Pi's GPIO pins and the ultrasonic sensors (HC-SR04) is accomplished through the command "GPIO.setmode(GPIO.BCM)". Using the "trigger" and "echo" pins, inputs and outputs can be defined for the ultrasonic sensors using "GPIO.setup(GPIO_TRIGGER_#, GPIO.OUT)", where # is an arbitrary value dependent on the number of ultrasonic sensors being used. Getting the ultrasonic sensors to read, filter, and display values is accomplished using the "def distanceUS(echo, trig):" function. The functions are called and used in the main() function of the code.

If the ultrasonic sensors register an object within 50cm, it will cause the robot to stop moving, turn, and proceed in another direction. This is incorperated through the usage of an if-statement.

------------------------------------------------------------------------------------------------
Section 4: Code Responsible for Path Planning:
------------------------------------------------------------------------------------------------

The script "imuMarvelMindContol_v6.py" is responsible for establishing path planning between the robot's current position and desired goal. Usage of the script requires the following modules:
	- import sys
	- import time
	- import MarvelmindHedge
	- import board
	- import adafruit_bno055
	- import PID_Diff
	- import PID_Test1
	- import numpy

The script utilizes the PID controller discussed in Section 1. Furthermore, the script uses an (in progress) differential PID controller. The differential PID controller is stored in the same file as the Section 1 PID. Consequently, it must be called into the main script by appending it.

The Marvelmind Hedgehog indoor GPS is essential for constant usage of the path planning algorithm. The Marvelmind script being imported to the main code is "marvelmind.py".

Successful usage of the code requires a BNO055 Adafruit IMU. Implementation of the IMU into the code is accomplished through the commands:

	i2c = board.I2C()
	sensor = adafruit_bno055.BNO055_I2C(i2c)
	last_val = 0xFFFF

I2C connection is required for proper functioning of the IMU. I2C connection can be established using the following procedure:

	1) From the main Raspberry Pi menu, choose "Preferences"
	2) Choose the "Interfaces" tab
	3) Select the I2C module
	4) Restart Pi

The script functions by first reading the robot's starting location. The robot is instructed to move forward a small amount of time. A vector between the initial and current points is established. The angle between the current-initial vector and a fixed vector <1,0> (<x,y>, respectively) allows for the robot's orientation in a room to be determined. Consequently, the robot's heading can be determined. A path (represented by a vector) between the robot's current position and a goal-node is formed. Using the goal-vector and heading, the angle the robot must turn to be aligned with the desired path can be established.
The angle is computed using the definition of the dot product. Accounting for the sign of the rotation (clockwise or counterclockwise) is accomplished using the numpy cross product feature and crossing the heading with the desired path. Having the sign, it is appended to the magnitude determined using the dot product. The sign allows for the robot to understand whether it must turn left or right. The differential PID controller works identically to that found in Section 1. It is responsible for computing all math necessary and sends the outputs to the "open_motor_serial.py" in order for motor movement. See: https://oroboto.net/2014/10/26/beaglebone-black-pid-control-and-go-to-goal/comment-page-1/ for additional information on the mathematics behind the PID controller. 

** TO BE ADDED (04/27/2022): The differential PID controller must be tuned. The variable controller is producing too small errors such that it cannot efficiently change the rotation speed of the motors. Furthermore, the code shall be revised such that it relies on IMU angles after the initial orientation rather than Marvelmind data to properly determine its direction and error relative to the desired path.

##################################################################################################################################
CONTACT INFORMATION: Should any questions arise, please contact the project engineers:
	Carter Sorensen (Carter.Sorensen@du.edu) - Team Leader
	Federico De Zabala (Fede.DeZabala@du.edu) - Lead Computer Engineer
	Axel Garfio (Axel.Garfio-Pereyra@du.edu) - Lead Mechanical Engineer 

CODE ERROR INFORMATION: Should difficulties occur opening code, please contact the project engineers and see the following GitHub:

	https://github.com/federicodzf/Integration2022

NOTICE: Code and corresponding comments have been written by the engineering design team and collected from open-source resources.
Any code taken has been done so with approval from the developing party.
##################################################################################################################################
