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
#Globals:
CALIBRATION_CONSTANT = 1.33
camera_matrix = np.array([[2.6822003708394282e+03, 0., 1.5588865381021240e+03], [0., 2.6741978758743703e+03, 1.2303469240154550e+03], [0., 0., 1.]])
dist_co = np.array([2.0426196677407879e-01, -3.3902097431574091e-01, -4.1813964792274307e-03, -1.0425257413809015e-02, 8.2004709580884308e-02])
WITHIN_FOOT = False
MARKER_DETECTED = False

def get_distance(corners):
    rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_co)
    return (tvec[0][0][2] * 100/CALIBRATION_CONSTANT)

def debug(string):
    bool == True # Make this false if you want to turn off debug mode
    if bool:
        print(string)

def main():
    cap = cv2.VideoCapture(0)
    while(True):
    # Aruco Detection
        ret, frame = cap.read()
        h, w, c = frame.shape
        ref = w/2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if corners: # if a marker is detected
                MARKER_DETECTED = True
                distance = get_distance(corners)
                cv2.putText(frame, str(distance), org = (0, 400), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color = (0, 255, 255))
                status = "Found a marker!"
                if(distance > 30 and distance < 31):
                    WITHIN_FOOT = True

        if(not corners):
            status = "No marker found"
            MARKER_DETECTED = False

        
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

