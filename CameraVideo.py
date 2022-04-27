# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32          # Establishes frame rate
camera.rotation = 180          # Camera is positioned upside down initially; prior command rotates the image 180 degrees
rawCapture = PiRGBArray(camera, size=(640, 480)) # Produced 3D red-green-blue array - pixels necessary for image creation

# Allow the camera to power-on and focus
time.sleep(0.1)

# Capture the frames (images) from the camera and continuously update the image
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # Grab the raw NumPy array representing the image, then initialize the timestamp and occupied/unoccupied text
    image = frame.array
    
    # Show the video frame
    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF
    
    # Clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    
    # If the `q` key was pressed, break from the loop and end script
    if key == ord("q"):
        break
