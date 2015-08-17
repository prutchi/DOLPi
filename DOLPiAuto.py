#   DOLPiAuto.py
#
#   This Python program demonstrates the DOLPi polarimetric camera
#
#   (c) 2015 David Prutchi, Ph.D., licensed under MIT license
#                                  (MIT, opensource.org/licenses/MIT)
#
#   This version uses a voltage-controlled polarization analyzer (VCPA) that
#   consists of a liquid crystal panel(LCP) and a polarizer film.  The polarizer
#   film is placed between the LCP and camera.
#   The VCPA is driven by a DAC-controlled AC driver to 3 states:
#   1. Highest DAC output of 5V for no rotation of polarization
#   2. ~45 degrees of rotation
#   3. Low DAC output for 90 degrees of rotation
#
#   An auto-calibration setup is used to determine the optimal voltages to
#   drive the VCPA.  The calibrator consists of a diffuse blue-light
#   LED illuminating the VCPA through a polarizer set at 45 degrees with
#   respect to the VCPA's polarizer.
#   Light transmission is detected by a LDR behind the VCPA and
#   measured by an ADS1015 Analog-to-Digital (ADC) converter IC
#
#Import the necessary packages
#-----------------------------
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import RPi.GPIO as GPIO
import numpy as np
from Adafruit_MCP4725 import MCP4725  #controls MCP4725 DAC
from Adafruit_ADS1x15 import ADS1x15  #controls ADS1015 ADC

#IO PINS
#-------
calLEDpin=23   #Calibration LED connected to pin 24
dc0degpin=22   #0 degree VCPA DC bias manual mode
dc45degpin=21  #45 degree VCPA DC bias manual mode
GPIO.setwarnings(False)  #Don't issue warning messages if channels are defined
GPIO.setmode(GPIO.BCM)
GPIO.setup(calLEDpin,GPIO.OUT)
GPIO.setup(dc0degpin,GPIO.OUT)
GPIO.setup(dc45degpin,GPIO.OUT)
GPIO.output(calLEDpin,False)  #Turn calibration LED off
GPIO.output(dc0degpin,False)
GPIO.output(dc45degpin,False)

#ADS1015 A/D DEFINITIONS
#-----------------------
# Select the A/D chip
#ADS1015 = 0x00  # 12-bit ADC
ADS1115 = 0x01	# 16-bit ADC

# Select the gain
#gain = 6144  # +/- 6.144V
gain = 4096  # +/- 4.096V
# gain = 2048  # +/- 2.048V
# gain = 1024  # +/- 1.024V
# gain = 512   # +/- 0.512V
#gain = 256   # +/- 0.256V

# Select the sample rate
#sps = 8    # 8 samples per second
# sps = 16   # 16 samples per second
# sps = 32   # 32 samples per second
#sps = 64   # 64 samples per second
# sps = 128  # 128 samples per second
# sps = 250  # 250 samples per second
# sps = 475  # 475 samples per second
sps = 860  # 860 samples per second

# Initialize the ADC using the default mode (use default I2C address)
# Set this to ADS1015 or ADS1115 depending on the ADC you are using!
adc = ADS1x15(ic=ADS1115)

#MCP4725 D/A DEFINITIONS
#-----------------------
# Initialize the DAC using the default address
dac = MCP4725(0x62)
# DAC output connected to frontLCD
dac.setVoltage(0)

def cal():
    #DRIVE VOLTAGE CALIBRATION
    #-------------------------
    # A/D converter is connected to a Light Dependent Resistor (LDR) placed
    # behind VCPA.  A LED illuminates the VCPA through a piece of polarizer film.
    # This function finds the drive voltages at which the VCPA should be driven
    # to analyze at 45 degrees
    #
    import matplotlib.pyplot as plt     #Needed only if plotting
    print "Please wait while calibrating..."
    #
    GPIO.output(calLEDpin,True)         #Turn calibration LED on
    #
    # Initialize LCP and flush A/D
    for i in range (1,5):
        dac.setVoltage(4095)            #Turn LCP full ON
        time.sleep(.1)                  #Let it settle
        #dac.setVoltage(0)               #Turn LCP OFF
        flush=adc.readADCSingleEnded(0, gain, sps) #Flush the A/D
    #
    #Initialize the lists to hold the graph
    vol=[]                              #List to hold DAC voltage codes
    light=[]                            #List to hold light transmission
    for volt in range(0,4095,10):       #Iterate through possible VCPA voltage values
        dac.setVoltage(volt)            #Apply voltage to VCPA
        time.sleep(0.05)                #Let it settle
        vol.append(volt)                #Add voltage code to list
        # Read channel 0 in single-ended mode using the settings above
        light.append(adc.readADCSingleEnded(0, gain, sps)) # Measure light level
        #time.sleep(0.15)                #Wait before next step
    #Leave loop with LCPand LED off
    GPIO.output(calLEDpin,False)        #Turn calibration LED off
    dac.setVoltage(0)                   #Turn LCP off
    
    #Comment the following block if a plot of the VCPA's
    #transmission of 45 degree light as a function of DAC voltage is not needed
    plt.plot(vol,light)                 #Plot VCPA transmission as function of voltage
    plt.xlabel('D/A voltage code')
    plt.ylabel('Light transmission A/D counts')
    plt.title('Light transmission through VCPA') 
    plt.show()                          #Show the plot

    light_index_max=light.index(max(light)) #Find index of maximum transmission
    maxVCPAvolt=vol[light_index_max]        #Find DAC voltage code for maximum transmission
    print "max DAC code", maxVCPAvolt
    return  maxVCPAvolt
voltVCPA45=cal() 
voltVCPA90=0
voltVCPA0=3000

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
    dac.setVoltage(0)
    dac.setVoltage(voltVCPA0)
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
    dac.setVoltage(0)
    dac.setVoltage(voltVCPA45)
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
    dac.setVoltage(0)
    dac.setVoltage(voltVCPA90)
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
dac.setVoltage(0)
cv2.destroyAllWindows()
quit

