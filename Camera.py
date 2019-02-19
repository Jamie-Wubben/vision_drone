import _thread
import os
import time

import cv2
import cv2.aruco as aruco
import numpy as np

from Logging import Logger as log, PositionLogger as pos


class Camera:
    logger = log.logger
    positionLog = pos.logger

    def __init__(self, working_mode):
        self.logger.info("Init camera")

        # if pi run modprobe to be able to acces the picam
        if working_mode == "pi":
            os.system('sudo modprobe bcm2835-v4l2')
            time.sleep(0.1)

        # use calibration matrix to be able to estimate pose
        try:
            self.cameraDistortion = np.loadtxt('distortion.txt')
            self.cameraMatrix = np.loadtxt('camera_matrix.txt')
        except FileNotFoundError:
            self.logger.error("Calibration file not found error.")
            # TODO close program if file not found

        self.markerLength = 0.185  # length of real marker in meters

        self.running = True
        self.ret = None
        self.frame = None
        self.out = None
        self.cap = None

        # self.marker_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    """
    process command, takes command and starts the right method
    some methods needs to be start in a thread
    returns -1 if exception accours
    returns 0 if command was not recognized
    returns 1 if everything went write
    """
    def process_command(self, command):
        if command == 'start_camera':
            try:
                _thread.start_new_thread(self.start_camera, ())
            except Exception as e:
                self.logger.warn("unable to start thread start_camera")
                self.logger.exception(e)
                return -1
        elif command == 'stop_camera':
            self.stop_camera()
        elif command == 'find_target':
            try:
                _thread.start_new_thread(self.find_target, ())
            except Exception as e:
                self.logger.warn("unable to start thread find_target")
                self.logger.exception(e)
                return -1
        elif command == 'exit':
            self.close()
        else:
            return 0

        return 1

    def close(self):
        self.logger.info("Closing...")
        self.stop_camera()

    def start_camera(self):
        self.logger.info("Starting camera ...")
        # for recording
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        dir_name = "./FlightMovies"
        counter = 0
        for f in os.listdir(dir_name):
            if os.path.isfile(dir_name + "/" + f) and f.find("video") is not -1:
                counter += 1
        filename = dir_name + "/video" + str(counter) + ".avi"
        self.out = cv2.VideoWriter(filename, fourcc, 20.0, (640, 480))
        self.logger.info("Videofile is stored in " + filename)
        self.logger.info("Start camera and record.")

        self.running = True
        self.cap = cv2.VideoCapture("FlightMovies/realFlight.avi")
        # if runned for second time program stays in wile loop because other messages are not received anymore
        while True:
            self.ret, self.frame = self.cap.read()
            # record and show the camerafeeds
            if self.ret:
                self.out.write(self.frame)
                #cv2.imshow('frame', self.frame)
                if cv2.waitKey(1) & (0xFF == ord('q') or not self.running):
                    break
            else:
                break

    def stop_camera(self):
        self.running = False
        self.logger.info("Stop camera.")
        self.cap.release()
        cv2.destroyAllWindows()
        self.out.release()

    def find_target(self):
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        font = cv2.FONT_HERSHEY_PLAIN
        camera_matrix = np.loadtxt('camera_matrix.txt')
        distortion = np.loadtxt('distortion.txt')

        # self.marker_socket.connect(("localhost", 5764))

        while True:
            key = cv2.waitKey(1)
            if key == 113:
                break

            if not self.ret:
                break

            # check to find target
            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            # documentation for detectorparameters see
            # https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html
            parameters = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            if corners:
                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.185, camera_matrix, distortion)
                cv2.putText(gray, str(tvecs[0][0]), (10, 30), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(gray, str(rvecs[0][0]), (10, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(gray, "press q to quit", (10, 450), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow("gray", gray)
                self.positionLog.info(str(tvecs[0][0][0])+","+str(tvecs[0][0][1])+","+str(tvecs[0][0][2]))
            else:
                cv2.putText(gray, "No marker detected", (10, 30), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(gray, "press q to quit", (10, 450), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow("gray", gray)

        self.cap.release()
        cv2.destroyAllWindows()

        # target found start descending

        # outside virtual boarder 2 move in xy plane
        # self.marker_socket.sendall("1100,1500,1500,1500\n".encode())
        # print("send messages move")
        # inside virtual boarder 1 start descending again

        # altitude is below 3 m make virtual boarder 1 and 2 smaller

        # move in xy plane

        # descend againg

        # uav landed
        # self.marker_socket.sendall("landed\n".encode())
