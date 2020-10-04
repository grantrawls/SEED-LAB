#Communication betweeen Pi, Arduino and LCD

import smbus
import time
import board
import busio
from smbus2 import SMBus
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import numpy as np
import cv2
import cv2.aruco as aruco
import math

#sizing for LCD screen
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)
# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

lcd.clear()
# Set LCD color to purple
lcd.color = [50, 0, 50]
bus = smbus.SMBus(1)

# Arduino address
address = 0x04

def writeNumber(value):
    bus.write_byte(address, value)
    return -1

def readNumber():
    number = bus.read_byte(address)
    return number

def get_center(corners): # Get the center of the aruco image in x,y pixel coordinates
    return( abs(((corners[0][0][3][0] - corners[0][0][0][0])/2) + corners[0][0][0][0]),
            abs(((corners[0][0][3][1] - corners[0][0][0][1])/2) + corners[0][0][0][1]) )

def debug(string):
    bool == True # Make this false if you want to turn off debug mode
    if bool:
        print(string)

def main():
    cap = cv2.VideoCapture(0)
    camera_matrix = np.array([[1, 0, 0], [0, 1, 0], [0,0,1]])
    dist_co = np.array([[1, 1, 1, 1]])
    while(True):

    # Aruco Detection
        ret, frame = cap.read()
        h, w, c = frame.shape
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if corners:
            try:
                center = get_center(corners)
                if(center[0] > w/2 and center[1] < h/2): loc, pos = "Sector 1: Postion 0", 0
                if(center[0] < w/2 and center[1] < h/2): loc, pos = "Sector 2: Postion 1", 1
                if(center[0] < w/2 and center[1] > h/2): loc, pos = "Sector 3: Postion 0", 2
                if(center[0] > w/2 and center[1] > h/2): loc, pos = "Sector 4: Postion 0", 3
                cv2.putText(frame, loc, org = (0, 400), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color = (0, 0, 0))
                debug("The center is: " + str(center))
                debug("The x value of center is: " + str(center[0]))
                debug("The y value of center is: " + str(center[1]))
            except: pass
                
        rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_co)

        # Status Handling
        if(not corners):status = "No marker found"
        else:status = "Found a marker!"
        cv2.putText(frame, status, org = (0, 300), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color = (0, 0, 0))
        frame = aruco.drawDetectedMarkers(frame, corners)
        cv2.imshow('frame',frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        writeNumber(pos)
        print ("RPI: Hi Arduino, I sent you ", pos)
        time.sleep(1) 

        number = readNumber()
        print ("Arduino: Hey RPI, the received number is ", number)

        lin1 = "sent: " + str(var)
        lin2 = "\ngot:  " + str(number)
        lcd.message = lin1 + lin2
        
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    print("Operation Complete")
    
