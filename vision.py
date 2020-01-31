import cv2
from picamera.array import PiRGBArray 
from picamera import PiCamera
import time
import numpy as np
import PySimpleGUI as sg
import configparser

layout = [[sg.Text('use as default')],            
                 [sg.Submit(), sg.Cancel()]]      

DEFAULT_PARAMETERS_FILENAME = "params.ini"
CENTER_PIXEL = 159.5
CURVE_MIN = 4
AREA_BALL = 210    #150 125 200

cam = PiCamera ()
cam.resolution = (320, 240)

rawcapture = PiRGBArray (cam, size=(320,240))

def on_trackbar():
    pass 

def save_parameters(filename):
    print(filename)
    parser = configparser.ConfigParser()
    parser.add_section('Parameters')
    parser.set('Parameters', 'ball low h', str(cv2.getTrackbarPos("ball low h", "window")))
    parser.set('Parameters', 'ball low s', str(cv2.getTrackbarPos("ball low s", "window")))
    parser.set('Parameters', 'ball low v', str(cv2.getTrackbarPos("ball low v", "window")))
    parser.set('Parameters', 'ball high h', str(cv2.getTrackbarPos("ball high h", "window")))
    parser.set('Parameters', 'ball high s', str(cv2.getTrackbarPos("ball high s", "window")))
    parser.set('Parameters', 'ball high v', str(cv2.getTrackbarPos("ball high v", "window")))
    fp=open(filename,'w')
    parser.write(fp)
    fp.close()

def load_parameters(filename):
    print(filename)
    parser = configparser.ConfigParser()
    parser.read(filename)
    for sect in parser.sections():
        print('Section:', sect)
        for k,v in parser.items(sect):
            cv2.setTrackbarPos(k, "window", int(v))
            print(' {} = {}'.format(k,v))
       
time.sleep (1)
cv2.namedWindow("window")
cv2.createTrackbar("mode", "window" , 0, 1, on_trackbar)
cv2.createTrackbar("ball", "window", 0, 1, on_trackbar)
cv2.createTrackbar("ball low h", "window" , 0, 180, on_trackbar)
cv2.createTrackbar("ball low s", "window" , 0, 255, on_trackbar)
cv2.createTrackbar("ball low v", "window" , 0, 255, on_trackbar)
cv2.createTrackbar("ball high h", "window" , 0, 180, on_trackbar)
cv2.createTrackbar("ball high s", "window" , 0, 255, on_trackbar)
cv2.createTrackbar("ball high v", "window" , 0, 255, on_trackbar)

load_parameters(DEFAULT_PARAMETERS_FILENAME)

# Setup window for saving parameters
sg.theme('DarkBlue1')

for frame in cam.capture_continuous(rawcapture, format="bgr", use_video_port=True):
    
    # read image array (NumPy format)
    image = frame.array

    #convert to hsv for further processing
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    track_ball = cv2.getTrackbarPos("ball", "window")
    if (track_ball == 1):

        ball_lower_hue = cv2.getTrackbarPos("ball low h", "window")
        ball_lower_saturation = cv2.getTrackbarPos("ball low s", "window")
        ball_lower_value =cv2.getTrackbarPos("ball low v", "window")
        ball_higher_hue =cv2.getTrackbarPos("ball high h", "window")
        ball_higher_saturation =cv2.getTrackbarPos("ball high s", "window")
        ball_higher_value =cv2.getTrackbarPos("ball high v", "window")
       
        ball_lower_mask = np.array([ball_lower_hue, ball_lower_saturation, ball_lower_value])
        ball_higher_mask = np.array([ball_higher_hue, ball_higher_saturation, ball_higher_value])
        mask = cv2.inRange(hsv, ball_lower_mask, ball_higher_mask)
        
        contours,hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            perimeter = cv2.arcLength(c, True)
            approxCurve = cv2.approxPolyDP(c, perimeter * 0.02, True)
            if  (len (approxCurve) > CURVE_MIN): # TODO approxCurve is a list, need list of points, not just one value
                (x,y), radius = cv2.minEnclosingCircle(c)
                area = cv2.contourArea(c)
                center = (int (x), int (y))
                radius = int(radius)
                if(area > AREA_BALL):
                    cv2.circle(image, center, radius, (0,0,255), 2)
                    distance = 0.00272*y*y - 1.3546*y + 180
                    fov = (-0.568*y + 653.5)/2
                    difference = x - CENTER_PIXEL
                    angle = difference*(60/fov)
                    if (cv2.getTrackbarPos("mode", "window") == 1):
                        print ("x = " + "{:3.1f}".format(x) + " y = " + "{:3.1f}".format(y) + " Dist = " + "{:2.2f}".format(distance) + " Angle = " + "{:2.2f}".format(angle))

    # display image
    debug = cv2.getTrackbarPos("mode", "window")
    if (debug == 1):
        cv2.imshow("Image", image)
        cv2.imshow("Color", hsv)
        if (track_ball == 1):
            cv2.imshow("mask", mask)
    else: 	
        cv2.destroyWindow("Color")
        cv2.destroyWindow("Image")
        if (cv2.getWindowProperty("mask", cv2.WND_PROP_VISIBLE) == 1):
            cv2.destroyWindow("mask")

    # wait for 1ms to see if the escape key was pressed, and if so, exit
    key = cv2.waitKey(1)
    
    # clear stream for image array 
    rawcapture.truncate(0)

    if key == 27: #press esc, close program
        break
    elif key == ord('s'): #press 's', save parameters
        fname = sg.popup_get_file('Save Parameters')
        if not fname:
            sg.popup("Warning", "No filename supplied, using " + DEFAULT_PARAMETERS_FILENAME)
            fname = DEFAULT_PARAMETERS_FILENAME
        else:
            sg.popup('The filename you chose was', fname)
        save_parameters(fname) # fname has the filename

    
cv2.destroyAllWindows()