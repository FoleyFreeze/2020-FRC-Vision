import cv2
from picamera.array import PiRGBArray 
from picamera import PiCamera
import time
import numpy as np
import PySimpleGUI as sg
import configparser
from networktables import NetworkTables
import RPi.GPIO as GPIO
import smbus

AREA_BALL = 210    #150 125 200
CENTER_PIXEL = 159.5
CURVE_MIN = 4
DEFAULT_PARAMETERS_FILENAME = "params.ini"
ROBORIO_SERVER_STATIC_IP = "10.9.10.2"
TARGET_MAX_RATIO = 20
TARGET_MIN_RATIO = 7
font = cv2.FONT_HERSHEY_SIMPLEX
CAMERA_MUX_SELECTION_PIN = 7
CAMERA_MUX_ENABLE1_PIN = 11
CAMERA_MUX_ENABLE2_PIN = 12

INVALID_CAMERA = 0
CAMERA_A = 1
CAMERA_B = 2
CAMERA_C = 3
CAMERA_D = 4

BALL_CAMERA = CAMERA_A # Camera A on the multi-camera adapter
TARGET_CAMERA = CAMERA_C # Camera C on the multi-camera adapter
CAMERA_NAMES = ['Invalid', 'A', 'B', 'C', 'D']

current_camera = INVALID_CAMERA

def on_trackbar():
    pass 

def save_parameters(filename):
    print("Saving " + filename)
    parser = configparser.ConfigParser()
    parser.add_section('Parameters')
    parser.set('Parameters', 'ball', str(cv2.getTrackbarPos("ball", "window")))
    parser.set('Parameters', 'delay', str(cv2.getTrackbarPos("delay", "window")))
    parser.set('Parameters', 'network table', str(cv2.getTrackbarPos("network table", "window")))
    parser.set('Parameters', 'ball low h', str(cv2.getTrackbarPos("ball low h", "window")))
    parser.set('Parameters', 'ball low s', str(cv2.getTrackbarPos("ball low s", "window")))
    parser.set('Parameters', 'ball low v', str(cv2.getTrackbarPos("ball low v", "window")))
    parser.set('Parameters', 'ball high h', str(cv2.getTrackbarPos("ball high h", "window")))
    parser.set('Parameters', 'ball high s', str(cv2.getTrackbarPos("ball high s", "window")))
    parser.set('Parameters', 'ball high v', str(cv2.getTrackbarPos("ball high v", "window")))
    parser.set('Parameters', 'target', str(cv2.getTrackbarPos("target", "window")))
    parser.set('Parameters', 'target low h', str(cv2.getTrackbarPos("target low h", "window")))
    parser.set('Parameters', 'target low s', str(cv2.getTrackbarPos("target low s", "window")))
    parser.set('Parameters', 'target low v', str(cv2.getTrackbarPos("target low v", "window")))
    parser.set('Parameters', 'target high h', str(cv2.getTrackbarPos("target high h", "window")))
    parser.set('Parameters', 'target high s', str(cv2.getTrackbarPos("target high s", "window")))
    parser.set('Parameters', 'target high v', str(cv2.getTrackbarPos("target high v", "window")))
   
    # Add target parameters
    fp=open(filename,'w')
    parser.write(fp)
    fp.close()
    print("Saved " + filename)

def load_parameters(filename):
    print("Loading " + filename)
    parser = configparser.ConfigParser()
    parser.read(filename)
    for sect in parser.sections():
        for k,v in parser.items(sect):
            cv2.setTrackbarPos(k, "window", int(v))
    print("Loaded " + filename)

def check_ball():
    if (cv2.getTrackbarPos("ball", "window") == 1): 
        return True
    else: 
        return False
    #TODO check for network table command here

def check_target():
    if (cv2.getTrackbarPos("target", "window") == 1): 
        return True
    else: 
        return False
    #TODO check for network table command here

def enable_camera(cam):
    set_cam = INVALID_CAMERA
    if (cam == CAMERA_A):
        bus.write_i2c_block_data(0x70, 0x00, [0x04])
        GPIO.output(CAMERA_MUX_SELECTION_PIN, GPIO.LOW)
        GPIO.output(CAMERA_MUX_ENABLE1_PIN, GPIO.LOW)
        GPIO.output(CAMERA_MUX_ENABLE2_PIN, GPIO.HIGH)
        set_cam = CAMERA_A
    elif (cam == CAMERA_B):
        bus.write_i2c_block_data(0x70, 0x00, [0x05])
        GPIO.output(CAMERA_MUX_SELECTION_PIN, GPIO.HIGH)
        GPIO.output(CAMERA_MUX_ENABLE1_PIN, GPIO.LOW)
        GPIO.output(CAMERA_MUX_ENABLE2_PIN, GPIO.HIGH)
        set_cam = CAMERA_B
    elif (cam == CAMERA_C):
        bus.write_i2c_block_data(0x70, 0x00, [0x06])
        GPIO.output(CAMERA_MUX_SELECTION_PIN, GPIO.LOW)
        GPIO.output(CAMERA_MUX_ENABLE1_PIN, GPIO.HIGH)
        GPIO.output(CAMERA_MUX_ENABLE2_PIN, GPIO.LOW)
        set_cam = CAMERA_C
    elif (cam == CAMERA_D):
        bus.write_i2c_block_data(0x70, 0x00, [0x07])
        GPIO.output(CAMERA_MUX_SELECTION_PIN, GPIO.HIGH)
        GPIO.output(CAMERA_MUX_ENABLE1_PIN, GPIO.HIGH)
        GPIO.output(CAMERA_MUX_ENABLE2_PIN, GPIO.LOW)
        set_cam = CAMERA_D
    print("Camera " + CAMERA_NAMES[set_cam] + " enabled")
    return(set_cam)

bus = smbus.SMBus(1) # Used to select a camera on the I2C mux on the camera mux board
GPIO.setmode(GPIO.BOARD) # Used to select a camera on the camera mux board
GPIO.setup(CAMERA_MUX_SELECTION_PIN, GPIO.OUT) # Used to select a camera on the camera mux board
GPIO.setup(CAMERA_MUX_ENABLE1_PIN, GPIO.OUT) # Used to select a camera on the camera mux board
GPIO.setup(CAMERA_MUX_ENABLE2_PIN, GPIO.OUT) # Used to select a camera on the camera mux board
GPIO.setup(15, GPIO.OUT) # No idea why this is required, it only appears in the demo code, with no explanation
GPIO.setup(16, GPIO.OUT) # No idea why this is required, it only appears in the demo code, with no explanation
GPIO.setup(21, GPIO.OUT) # No idea why this is required, it only appears in the demo code, with no explanation
GPIO.setup(22, GPIO.OUT) # No idea why this is required, it only appears in the demo code, with no explanation
GPIO.output(11, GPIO.HIGH) # No idea why this is required, it only appears in the demo code, with no explanation
GPIO.output(12, GPIO.HIGH) # No idea why this is required, it only appears in the demo code, with no explanation
GPIO.output(15, GPIO.HIGH) # No idea why this is required, it only appears in the demo code, with no explanation
GPIO.output(16, GPIO.HIGH) # No idea why this is required, it only appears in the demo code, with no explanation
GPIO.output(21, GPIO.HIGH) # No idea why this is required, it only appears in the demo code, with no explanation
GPIO.output(22, GPIO.HIGH) # No idea why this is required, it only appears in the demo code, with no explanation

time.sleep(2)
# Start with target camera enabled
current_camera = enable_camera(TARGET_CAMERA)

cam = PiCamera ()
cam.resolution = (320, 240)
cam.exposure_mode = "snow"
cam.awb_mode = "off"
cam.awb_gains = (1.3, 1.8)
cam.shutter_speed = 8000
cam.saturation = -50
time.sleep(2)
rawcapture = PiRGBArray (cam, size=(320,240))

ball_id = 0

lower_hue= 0
lower_saturation = 0
lower_value = 0
higher_hue = 0
higher_saturation= 0
higher_value = 0
# Setup window for saving parameters
layout = [[sg.Text('use as default')],            
                 [sg.Submit(), sg.Cancel()]] 
sg.theme('DarkBlue1')
cv2.namedWindow("window")
cv2.createTrackbar("mode", "window" , 0, 1, on_trackbar)
cv2.createTrackbar("ball", "window", 0, 1, on_trackbar)
cv2.createTrackbar("delay", "window", 0,1, on_trackbar)
cv2.createTrackbar("network table", "window", 0,1, on_trackbar)
cv2.createTrackbar("ball low h", "window" , 0, 180, on_trackbar)
cv2.createTrackbar("ball low s", "window" , 0, 255, on_trackbar)
cv2.createTrackbar("ball low v", "window" , 0, 255, on_trackbar)
cv2.createTrackbar("ball high h", "window" , 0, 180, on_trackbar)
cv2.createTrackbar("ball high s", "window" , 0, 255, on_trackbar)
cv2.createTrackbar("ball high v", "window" , 0, 255, on_trackbar)
cv2.createTrackbar("target", "window", 0, 1, on_trackbar)
cv2.createTrackbar("target low h", "window", 0, 180, on_trackbar)
cv2.createTrackbar("target low s", "window", 0, 255, on_trackbar)
cv2.createTrackbar("target low v", "window", 0, 255, on_trackbar)
cv2.createTrackbar("target high h", "window" , 0, 180, on_trackbar)
cv2.createTrackbar("target high s", "window" , 0, 255, on_trackbar)
cv2.createTrackbar("target high v", "window" , 0, 255, on_trackbar)

load_parameters(DEFAULT_PARAMETERS_FILENAME)

if(cv2.getTrackbarPos("delay", "window") == 1):
    time.sleep (20)
NetworkTables.initialize(server=ROBORIO_SERVER_STATIC_IP)
NetworkTables.setUpdateRate(0.040)
vis_nt = NetworkTables.getTable("Vision")

for frame in cam.capture_continuous(rawcapture, format="bgr", use_video_port=True):
    
    start = time.process_time()

    # Read image array (NumPy format)
    image = frame.array

    # Convert to hsv for further processing
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    if (check_ball() == True):

        lower_hue = cv2.getTrackbarPos("ball low h", "window")
        lower_saturation = cv2.getTrackbarPos("ball low s", "window")
        lower_value = cv2.getTrackbarPos("ball low v", "window")
        higher_hue = cv2.getTrackbarPos("ball high h", "window")
        higher_saturation = cv2.getTrackbarPos("ball high s", "window")
        higher_value = cv2.getTrackbarPos("ball high v", "window")

        lower_mask = np.array([lower_hue, lower_saturation, lower_value])
        higher_mask = np.array([higher_hue, higher_saturation, higher_value])
        mask = cv2.inRange(hsv, lower_mask, higher_mask)
       
    elif (check_target() == True):

        lower_hue = cv2.getTrackbarPos("target low h", "window")
        lower_saturation = cv2.getTrackbarPos("target low s", "window")
        lower_value = cv2.getTrackbarPos("target low v", "window")
        higher_hue = cv2.getTrackbarPos("target high h", "window")
        higher_saturation = cv2.getTrackbarPos("target high s", "window")
        higher_value = cv2.getTrackbarPos("target high v", "window")

        lower_mask = np.array([lower_hue, lower_saturation, lower_value])
        higher_mask = np.array([higher_hue, higher_saturation, higher_value])
        mask = cv2.inRange(hsv, lower_mask, higher_mask)
        
    # Ball functions
    if (check_ball() == True):
        if(current_camera != BALL_CAMERA):
            current_camera = enable_camera(BALL_CAMERA)
        contours,hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            perimeter = cv2.arcLength(c, True)
            approxCurve = cv2.approxPolyDP(c, perimeter * 0.02, True)
            if  (len (approxCurve) > CURVE_MIN): 
                (x,y), radius = cv2.minEnclosingCircle(c)
                area_ball = cv2.contourArea(c)
                center = (int (x), int (y))
                radius = int(radius)
                if(area_ball > AREA_BALL):
                    cv2.circle(image, center, radius, (0,0,255), 2)
                    distance = 0.00272*y*y - 1.3546*y + 180
                    fov = (-0.568*y + 653.5)/2
                    difference = x - CENTER_PIXEL
                    angle = difference*(60/fov)
                    ball_data = "%d,%.2f,%.2f" % (ball_id, distance, angle)
                    ball_id = ball_id + 1
                    if (cv2.getTrackbarPos("network table", "window") == 1):
                        vis_nt.putString("Ball", ball_data)
                    if (cv2.getTrackbarPos("mode", "window") == 1):
                        print (ball_data+ ",x = " + "{:3.1f}".format(x) + ",y = " + "{:3.1f}".format(y))
                        #dt = (time.process_time()-start)*1000 #execution time in ms
                        #print("Ball_dt: %2.2f" % dt)
                    break
                    
    # Target functions
    if (check_target() == True):
        if(current_camera != TARGET_CAMERA):
            current_camera = enable_camera(TARGET_CAMERA)
        contours,hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            perimeter = cv2.arcLength(c, True)
            approxCurve = cv2.approxPolyDP(c, perimeter * 0.01, True)
            print(len(approxCurve))
            if  (len (approxCurve) >= 8):
                area_target = cv2.contourArea(c)
                x,y,w,h = cv2.boundingRect(c)
                area_rect = w*h
                target_ratio = (area_target/area_rect)*100
                print ("tar_a=" + "{:3.2f}".format(area_target) + ",tar_rect=" + "{:3.2f}".format(area_rect) +",ratio = " + "{:3.2f}".format(target_ratio) )
                if (target_ratio < TARGET_MAX_RATIO and target_ratio > TARGET_MIN_RATIO):
                    cv2.putText(image, str (area_rect), (round(x+w/2),y), font, 1,(255,255,255),2,cv2.LINE_AA)
                    cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
                    cv2.drawContours(image, contours, -1, (0, 255, 0), 3)        
                    break

    # Display images
    debug = cv2.getTrackbarPos("mode", "window")
    if (debug == 1):
        cv2.imshow("Image", image)
        cv2.imshow("mask", mask)
    else: 	
        if (cv2.getWindowProperty("Image", cv2.WND_PROP_VISIBLE) == 1):
            cv2.destroyWindow("Image")
        if (cv2.getWindowProperty("mask", cv2.WND_PROP_VISIBLE) == 1):
            cv2.destroyWindow("mask")

    # Clear stream for image array 
    rawcapture.truncate(0)

    # Wait for 1ms to see if the escape key was pressed, and if so, exit
    key = cv2.waitKey(1)
    
    if key == 27: # Press esc, close program
        break
    elif key == ord('s'): # Press 's', save parameters
        fname = sg.popup_get_file('Save Parameters')
        if not fname:
            sg.popup("Warning", "No filename supplied, using " + DEFAULT_PARAMETERS_FILENAME)
            fname = DEFAULT_PARAMETERS_FILENAME
        else:
            sg.popup('The filename you chose was', fname)
        save_parameters(fname) # fname has the filename

cv2.destroyAllWindows()