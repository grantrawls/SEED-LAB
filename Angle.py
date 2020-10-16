#Grant Rawls
#Demo 1
#10/16/2020
# Some code taken from the following link:
#https://www.philipzucker.com/aruco-in-opencv/

# This program will detect an Aruco marker, and then print the angle
# that it is from the center of the screen

import numpy as np
import cv2
import cv2.aruco as aruco
import math
THETA = 48.8

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
        ref = w/2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if corners:
            #try:
                center = get_center(corners)
                if(center[0] > ref):
                    angle = (center[0] - ref) * ((THETA/2)/(ref))
                    loc = "Angle is +" + str(angle)
                if(center[0] < ref):
                    angle = (ref - center[0]) * ((THETA/2)/(ref))
                    loc = "Angle is -" + str(angle)
                cv2.putText(frame, loc, org = (0, 400), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color = (0, 255, 255))
                debug("The center is: " + str(center))
                debug("The x value of center is: " + str(center[0]))
                debug("The y value of center is: " + str(center[1]))
            #except: pass
        rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_co)
    
        # Status Handling
        if(not corners):status = "No marker found"
        else:status = "Found a marker!"
        cv2.putText(frame, status, org = (0, 300), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color = (0, 255, 255))
        frame = aruco.drawDetectedMarkers(frame, corners)
        cv2.imshow('frame',frame)
    
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()
    print("Operation Complete")

