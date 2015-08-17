#   DOLPiManual.py
#
#   This Python program demonstrates the DOLPi polarimetric camera
#   under manual control (potentiometer setting for analyzer at 45 degrees)
#
#   (c) 2015 David Prutchi, Ph.D., licensed under MIT license
#                                  (MIT, opensource.org/licenses/MIT)
#
#   This version uses a voltage-controlled polarization analyzer (VCPA) that
#   consists of a liquid crystal panel(LCP) and a polarizer film.  The polarizer
#   film is placed between the LCP and camera.
#   The VCPA is driven by GPIO pins 22 and 23 (via diodes and a pot) to 3 states:
#   1. 5V for no rotation of polarization
#   2. ~45 degrees of rotation as set by potentiometer
#   3. 0V for 90 degrees of rotation
#
#import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import RPi.GPIO as GPIO
import numpy as np


#IO PINS
#-------
dc0degpin=22   #0 degree VCPA DC bias manual mode
dc45degpin=21  #45 degree VCPA DC bias manual mode
GPIO.setwarnings(False)  #Don't issue warning messages if channels are defined
GPIO.setmode(GPIO.BCM)
GPIO.setup(dc0degpin,GPIO.OUT)
GPIO.setup(dc45degpin,GPIO.OUT)
GPIO.output(dc0degpin,False)
GPIO.output(dc45degpin,False)

#Raspberry Pi Camera Initialization
#----------------------------------
#Initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
#camera.resolution = (1280,720)
camera.framerate=80
rawCapture = PiRGBArray(camera)
camera.led=False

#Auto-Exposure Lock
#------------------
# Wait for the automatic gain control to settle
time.sleep(2)
# Now fix the values
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'
gain = camera.awb_gains
camera.awb_mode = 'off'
camera.awb_gains = gain

#Initialize flags
loop=True  #Initial state of loop flag
first=True #Flag to skip display during first loop
video=True #Use video port?  Video is faster, but image quality is significantly
            #lower than using still-image capture

while loop:
    #grab an image from the camera at 0 degrees
    GPIO.output(dc0degpin,True)
    GPIO.output(dc45degpin,False)
    time.sleep(0.05)
    rawCapture.truncate(0)
    camera.capture(rawCapture, format="bgr",use_video_port=video)
    image0 = rawCapture.array
    # Select one of the two methods of color to grayscale conversion:
    # Blue channel gives better polarization information because wavelengt
    # range is limited
    # True grayscale conversion gives better gray balance for non-polarized light
    R=image0[:,:,1]  #Use blue channel
    #R=cv2.cvtColor(image0,cv2.COLOR_BGR2GRAY) #True grayscale conversion

    #grab an image from the camera at 45 degrees
    GPIO.output(dc0degpin,False)
    GPIO.output(dc45degpin,True)
    time.sleep(0.05)
    rawCapture.truncate(0)
    camera.capture(rawCapture, format="bgr",use_video_port=video)
    image45 = rawCapture.array  #Capture image at 45 degrees rotation
    # Select one of the two methods of color to grayscale conversion:
    # Blue channel gives better polarization information because wavelengt
    # range is limited
    # True grayscale conversion gives better gray balance for non-polarized light
    G=image45[:,:,1]
    #G=cv2.cvtColor(image45,cv2.COLOR_BGR2GRAY)
        
    #grab an image from the camera at 90 degrees
    GPIO.output(dc0degpin,False)
    GPIO.output(dc45degpin,False)
    time.sleep(0.05)
    rawCapture.truncate(0)
    camera.capture(rawCapture, format="bgr",use_video_port=video)
    # Select one of the two methods of color to grayscale conversion:
    # Blue channel gives better polarization information because wavelengt
    # range is limited
    # True grayscale conversion gives better gray balance for non-polarized light
    image90 = rawCapture.array  #Capture image at 90 degree rotation
    B=image90[:,:,1]
    #B=cv2.cvtColor(image90,cv2.COLOR_BGR2GRAY)
    imageDOLPi=cv2.merge([B,G,R])
    cv2.imshow("Image_DOLPi",cv2.resize(imageDOLPi,(320,240),interpolation=cv2.INTER_AREA))  #Display DOLP image
    GPIO.output(dc0degpin,False)
    GPIO.output(dc45degpin,False)
    k = cv2.waitKey(1)  #Check keyboard for input
    if k == ord('x'):   # wait for x key to exit
        loop=False

#   Prepare to leave
cv2.imwrite("image0.jpg",image0)
cv2.imwrite("image90.jpg",image90)
cv2.imwrite("image45.jpg",image45)
cv2.imwrite("RGBpol.jpg",cv2.merge([B,G,R]))
cv2.imwrite("image0g.jpg",R)
cv2.imwrite("image90g.jpg",G)
cv2.imwrite("image45g.jpg",B)
cv2.destroyAllWindows()
quit

