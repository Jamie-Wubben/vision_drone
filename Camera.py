import _thread
import os
import time
import socket
import cv2
import cv2.aruco as aruco
import numpy as np

from Logging import Logger as log, PositionLogger as pos
from PositionProcessor import PositionProcessor


class Camera:
    logger = log.logger
    positionLog = pos.logger

    def __init__(self, working_mode):
        self.logger.info("Init camera")
        self.working_mode = working_mode
        # if pi run modprobe to be able to acces the picam
        if working_mode == "pi":
            os.system('sudo modprobe bcm2835-v4l2')
            time.sleep(0.1)

        self.logger.info("Camera inited correctly")
        # use calibration matrix to be able to estimate pose
        try:
            self.cameraDistortion = np.loadtxt('distortion.txt')
            self.cameraMatrix = np.loadtxt('camera_matrix.txt')
        except FileNotFoundError:
            self.logger.error("Calibration file not found error.")
            # TODO close program if file not found

        self.markerLength = 0.185  # length of real marker in meters
        self.running = True
        self.ret = None  # Boolean: true if information from camera
        self.frame = None  # Information from camera
        self.out = None  # Output stream, which saves the video
        self.cap = None  # Camera

        self.find_target_running = False

        # socket to give Ardusim information about the marker
        self.marker_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.message = "loiter\n"
        self.lastMessage = self.message

        self.positionProcessor = PositionProcessor()

    """
    process command, takes command and starts the right method
    some methods needs to be start in a thread
    returns -1 if exception accours
    returns 0 if command was not recognized
    returns 1 if everything went right
    """

    def process_command(self, command):
        if command == 'start_camera':
            try:
                self.logger.info("start new thread to start camera")
                _thread.start_new_thread(self.start_camera, ())
            except Exception as e:
                self.logger.warn("unable to start thread start_camera")
                self.logger.exception(e)
                return -1
        elif command == 'stop_camera':
            self.logger.info("stopping the camera")
            self.stop_camera()
        elif command == 'find_target':
            try:
                self.logger.info("start new thread to find target")
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
        # stops the camera but also kills the python application
        self.logger.info("Closing...")
        self.stop_camera()

    def start_camera(self):
        self.logger.info("Starting camera ...")
        # looks how many files there exist with name video in ./FlightMovies/
        # new video will have a higher number so that the old videos wont be overwritten
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
        # self.cap = cv2.VideoCapture("FlightMovies/realFlight1.avi")

        self.cap = cv2.VideoCapture(0)
        self.logger.info("recording the video feed")
        while self.running:
            self.ret, self.frame = self.cap.read()
            # record and show the camerafeeds
            if self.ret:
                #self.logger.info("camera running....")
                self.out.write(self.frame)
                # cv2.imshow('frame', self.frame)
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    self.logger.info("stop the video feed")
                    break
            else:
                self.logger.warn("stop the video feed: self.ret == False")
                break

    def stop_camera(self):
        self.running = False
        self.logger.info("Stop camera.")
        self.cap.release()
        cv2.destroyAllWindows()
        self.out.release()

    def find_target(self):
        self.logger.info("starting find_target")
        self.find_target_running = True
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        font = cv2.FONT_HERSHEY_PLAIN

        self.marker_socket.connect(("localhost", 5764))
        self.logger.info("made connection with ardusim")
        for x in range(0, 10):
            self.logger.info("send message: " + str(x))
            self.marker_socket.sendall("ping\n".encode())
            time.sleep(1)


        # start by sending loiter, later the message will change so that the drone will move
        """
        self.marker_socket.sendall(self.message.encode())
        noCameraCounter = 0
        noMarkerCounter = 0

        while self.find_target_running:
            key = cv2.waitKey(1)
            if key == 113:
                self.logger.info("stop the camera feed")
                break
            if not self.ret:
                # because start_camera and find_target are started in different threads there is a possibility
                # that they are started at (almost) the same moment. When this happens there would self.ret would be
                # False. In this way the camera has time to "warm up".
                noCameraCounter += 1
                self.logger.warn("no camera: counter = " + str(noCameraCounter))
                if noCameraCounter >= 10:
                    self.logger.error("no camera after 10 tries")
                    self.marker_socket.sendall("error\n".encode())
                    self.logger.error("error, uav land")
                    self.find_target_running = False
                    break
                time.sleep(1)
            else:
                # check to find target
                gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

                # documentation for detectorparameters see
                # https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html
                parameters = aruco.DetectorParameters_create()
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
                if corners:
                    noMarkerCounter = 0
                    rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.185, self.cameraMatrix,
                                                                               self.cameraDistortion)
                    # Rodrigues: calculated rotation matrix from rotation vector
                    rmat = cv2.Rodrigues(rvecs[0][0])[0]
                    # concat rmat and tvecs to make a projection matrix
                    P = np.c_[rmat, tvecs[0][0]]
                    # decompose projectionMatrix and retrive the angles
                    # https://shimat.github.io/opencvsharp_2410/html/b268a47c-b24b-aa64-a273-c1b1927b7ec0.htm
                    angles = -cv2.decomposeProjectionMatrix(P)[6] + 180

                    # Don't show the camera processing on the drone/pi
                    if self.working_mode != "pi":
                        cv2.putText(gray, str(tvecs[0][0]), (10, 60), font, 1, (0, 0, 0), 2, cv2.LINE_AA)
                        cv2.putText(gray, str(angles).replace('[', '').replace(']', ''), (10, 90), font, 1, (0, 0, 0), 2
                                    , cv2.LINE_AA)
                        cv2.putText(gray, "press q to quit", (10, 450), font, 1, (0, 0, 0), 2, cv2.LINE_AA)
                        aruco.drawAxis(gray, self.cameraMatrix, self.cameraDistortion, rvecs, tvecs, 0.1)
                        cv2.imshow("frame", gray)

                    self.message = self.positionProcessor.process(tvecs[0][0][0], tvecs[0][0][1], tvecs[0][0][2],
                                                                  angles[0])

                    # TODO find nicer way to todo this
                    self.positionLog.info(
                        str(tvecs[0][0][0]).replace('.', ',') + ";"
                        + str(tvecs[0][0][1]).replace('.', ',') + ";"
                        + str(tvecs[0][0][2]).replace('.', ',') + ";"
                        + str(angles[0]).replace('.', ',').replace('[', '').replace(']', '') + ";"
                        + str(angles[1]).replace('.', ',').replace('[', '').replace(']', '') + ";"
                        + str(angles[2]).replace('.', ',').replace('[', '').replace(']', '')
                    )
                else:
                    cv2.putText(gray, "No marker detected", (10, 30), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(gray, "press q to quit", (10, 450), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.imshow("frame", gray)
                    noMarkerCounter += 1
                    if noMarkerCounter > 50:
                        self.message = "loiter\n"

                if self.message is not self.lastMessage:
                    self.logger.info("send message to ardusim " + self.message)
                    self.lastMessage = self.message
                    self.marker_socket.sendall(self.message.encode())
                    # TODO stop this method when command land is send
                """