import cv2
from cv2 import aruco
import numpy as np
import time
import os

# if pi run modprobe to be able to acces the picam
if True:
    os.system('sudo modprobe bcm2835-v4l2')
    time.sleep(1)
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
font = cv2.FONT_HERSHEY_PLAIN
camera = cv2.VideoCapture(0)
camera_matrix = np.loadtxt('camera_matrix.txt')
distortion = np.loadtxt('distortion.txt')
while True:
    key = cv2.waitKey(1)
    if key == 113:
        break

    ret, img = camera.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # documentation for detectorparameters see
    # https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html
    parameters = aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
    parameters.cornerRefinementWinSize = 20
    parameters.cornerRefinementMaxIterations = 60
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if corners:
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.185, camera_matrix, distortion)
        cv2.putText(gray, str(tvecs[0][0]), (10, 30), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(gray, str(rvecs[0][0]), (10, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(gray, "press q to quit", (10, 450), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow("gray", gray)
    else:
        cv2.putText(gray, "No marker detected", (10, 30), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(gray, "press q to quit", (10, 450), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow("gray", gray)

camera.release()
cv2.destroyAllWindows()