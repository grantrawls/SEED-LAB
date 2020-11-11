#Grant Rawls
#Demo 1
#10/16/2020
# Some code taken from the following link:
#https://www.philipzucker.com/aruco-in-opencv/

# This program will detect an Aruco marker, and then print the distance away that it appears from the camera
# Before use, you will need to calibrate the camera with the CALIBRATION_CONSTANT
#Steps:
#Set up Aruco Marker 1 meter (or other known distance) from the camera
#Adjust Camera Calibration accordingly


import numpy as np
import cv2
import cv2.aruco as aruco
import math
import smbus
import time
import board
import busio
from smbus2 import SMBus
#import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

#Globals:
CALIBRATION_CONSTANT = 2.26    #used to tune the distance of the marker
THETA = 48.8
camera_matrix = np.array([[2.6822003708394282e+03, 0., 1.5588865381021240e+03], [0., 2.6741978758743703e+03, 1.2303469240154550e+03], [0., 0., 1.]])
dist_co = np.array([2.0426196677407879e-01, -3.3902097431574091e-01, -4.1813964792274307e-03, -1.0425257413809015e-02, 8.2004709580884308e-02])
WITHIN_FOOT = False
MARKER_DETECTED = False
#Arduino Address
address = 0x04
#LCD stuff not used in this demo
#sizing for LCD screen
#lcd_columns = 16
#lcd_rows = 2
## Initialise I2C bus.
#i2c = busio.I2C(board.SCL, board.SDA)
## Initialise the LCD class
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
#lcd.clear()
## Set LCD color to purple
#lcd.color = [50, 0, 50]


bus = smbus.SMBus(1)

#Function to get the distance of the aruco marker
def get_distance(corners):
    rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_co)
    return (tvec[0][0][2] * 100/CALIBRATION_CONSTANT)

def debug(string):
    bool == True # Make this false if you want to turn off debug mode
    if bool:
        print(string)
        
#Function to get the center of the aruco image in x,y pixel coordinates
def get_center(corners): 
    return( abs(((corners[0][0][3][0] - corners[0][0][0][0])/2) + corners[0][0][0][0]),
            abs(((corners[0][0][3][1] - corners[0][0][0][1])/2) + corners[0][0][0][1]) )
        
#function to write data to arduino via i2c
def writeNumber(value):
    bus.write_byte(address, value)
    return -1

#Main program
def main():
    cap = cv2.VideoCapture(0)   #activate camera
    #infinite loop
    while(True):
    # Aruco Detection
        ret, frame = cap.read()
        h, w, c = frame.shape    #setup camera window
        ref = w/2    #center of frame
        
        #camera calibration stuff
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        if corners: # if a marker is detected
                
                #Angle Detection
                frame = aruco.drawDetectedMarkers(frame, corners, ids)
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, None) 
                aruco.drawAxis(frame, camera_matrix, None, rvec, tvec, length=0.05)
                center = get_center(corners)
                #negative angle for right side of screen
                if(center[0] > ref):
                    angle = (center[0] - ref) * ((THETA/2)/(ref))
                    loc = "Angle is -" + str(angle)
                #positive angle for left side of screen
                if(center[0] < ref):
                    angle = (ref - center[0]) * ((THETA/2)/(ref))
                    loc = "Angle is +" + str(angle)
