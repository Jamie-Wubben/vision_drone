import _thread
import os
import time
import socket
import cv2
import cv2.aruco as aruco
import numpy as np

from Logging import Logger as log, PositionLogger as pos


class Camera:
    logger = log.logger
    positionLog = pos.logger

    def __init__(self, working_mode):
        self.logger.info("Init camera")
        self.working_mode = working_mode
        # if pi run modprobe to be able to access the picam
        if working_mode == "pi":
            os.system('sudo modprobe bcm2835-v4l2')
            time.sleep(0.1)

        self.logger.info("Camera inited correctly")

        self.ret = None  # Boolean: true if information from camera
        self.frame = None  # Information from camera
        self.out = None  # Output stream, which saves the video
        self.cap = None  # Camera
        self.cameraRunning = False
        self.find_target_running = False
        # socket to give Ardusim information about the marker
        self.marker_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.message = "loiter\n"
        self.lastMessage = self.message
        self.lastMarkerTime = timeMillis()

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
        self.cameraRunning = True

        self.cap = cv2.VideoCapture("FlightMovies/goodFlight3.avi")
        # self.cap = cv2.VideoCapture(0)

        self.logger.info("recording the video feed")
        while self.cameraRunning:
            self.ret, self.frame = self.cap.read()
            # record and show the camerafeeds
            if self.ret:
                self.out.write(self.frame)
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    self.logger.info("stop the video feed")
                    break
            else:
                self.logger.warn("stop the video feed: self.ret == False")
                break

    def stop_camera(self):
        self.cameraRunning = False
        self.logger.info("Stop camera.")
        self.cap.release()
        cv2.destroyAllWindows()
        self.out.release()

    def find_target(self):
        # /////////////INIT ////////////////
        self.find_target_running = self.find_target_init()

        self.marker_socket.connect(("localhost", 5764))
        self.logger.info("made connection with ardusim")
        # start by sending loiter, later the message will change so that the drone will move
        # will send error if the init function failed such that the UAV would land directly
        self.marker_socket.sendall(self.message.encode())

        expectedMarkerID = min(self.markerLength.keys())  # ID of marker that is currently used

        # /////////////SEARCH////////////////
        noCameraTime = timeMillis()
        wrongMarkerTime = timeMillis()
        while self.find_target_running:
            # /////////////CAMERA NOT AVAILABLE////////////////
            # if there is no camera send loiter
            # if there is not camera for 10 consecutive seconds send error
            if not self.ret:
                self.logger.warn("no camera ")
                self.marker_socket.sendall("loiter\n".encode())
                if timeMillis() > noCameraTime + 10 * 1000:
                    self.logger.error("no camera after 10 seconds")
                    self.marker_socket.sendall("error\n".encode())
                    self.logger.error("error, uav land")
                    self.find_target_running = False
                time.sleep(0.5)
            # /////////////CAMERA AVAILABLE////////////////
            # if camera available reset the noCameraTime
            # and check to find markers
            else:
                noCameraTime = timeMillis()
                gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
                # documentation for detectorparameters see
                # https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
                # /////////////NO MARKERS DETECTED////////////////
                if not corners:
                    # within 1 s => dont send anything the UAV will repeat it last action
                    # in between 1 and 5 seconds => recover
                    # more then 5 seconds => loiter
                    if self.lastMarkerTime + 1000 < timeMillis() <= self.lastMarkerTime + 6000:
                        self.message = "recover\n"
                    elif self.lastMarkerTime + 6000 < timeMillis():
                        self.message = "loiter\n"
                # /////////////MARKERS DETECTED////////////////
                else:
                    self.lastMarkerTime = timeMillis()
                    # check for expected marker
                    if expectedMarkerID not in ids:
                        # If the expected marker is not detected but for more then 2 seconds some other marker is
                        # detected change the expectedMarkerID and use that one
                        self.message = "loiter\n"
                        if wrongMarkerTime + 6000 < timeMillis():
                            self.logger.info("changed marker due to not finding expected marker")
                            expectedMarkerID = int(ids[0])
                            self.logger.info("new expectedMarkerID = " + str(int(ids[0])))
                    else:
                        wrongMarkerTime = timeMillis()
                        # get index in the array, [0][0] because it will return a tuple of arrays (one for each axis),
                        # and you want to get the first element
                        index = np.where(ids == expectedMarkerID)[0][0]
                        # pose estimation
                        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners,
                                                                                   self.markerLength.get(
                                                                                       expectedMarkerID),
                                                                                   self.cameraMatrix,
                                                                                   self.cameraDistortion)
                        # Rodrigues: calculated rotation matrix from rotation vector
                        rmat = cv2.Rodrigues(rvecs[index][0])[0]
                        # concat rmat and tvecs to make a projection matrix
                        P = np.c_[rmat, tvecs[index][0]]
                        # decompose projectionMatrix and retrive the angles
                        # https://shimat.github.io/opencvsharp_2410/html/b268a47c-b24b-aa64-a273-c1b1927b7ec0.htm
                        angles = -cv2.decomposeProjectionMatrix(P)[6] + 180
                        # Logging the pose estimation
                        self.positionLog.info(str(expectedMarkerID) + ";" + str(tvecs[index][0][0]) + ";"
                                              + str(tvecs[index][0][1]) + ";" + str(tvecs[index][0][2]) + ";" + str(
                            angles[0][0]) + ";" + str(angles[1][0]) + ";" + str(angles[2][0]))
                        # /////////////MULTIPLE MARKER UPDATE////////////////
                        # if multiple markers register the marker and check for an update
                        if len(ids) > 1:
                            idsSet = set(ids.flatten())
                            idsSet.discard(expectedMarkerID)
                            intersection = [value for value in idsSet
                                            if value in set(self.ids_activated.keys())]
                            for i in intersection:
                                if self.ids_activated[i] is -1:
                                    # register marker
                                    self.ids_activated[i] = tvecs[index][0][2] / 2
                                    self.logger.info("marker " + str(i) + " activated")
                                else:
                                    # check current altitude to activation level array
                                    # update marker
                                    if tvecs[index][0][2] < self.ids_activated[i]:
                                        self.logger.info("changed markerID to " + str(i))
                                        expectedMarkerID = i
                        # /////////////DECISION TO MOVE THE UAV////////////////
                        self.message = process(tvecs[index][0][0], tvecs[index][0][1], tvecs[index][0][2], angles[2])

            # /////////////SEND DECISION TO ARDUSIM////////////////
            # Only send new messages to avoid sending a lot of messages on the network
            if self.message is not self.lastMessage:
                self.logger.info("send message to ardusim " + self.message)
                self.lastMessage = self.message
                self.marker_socket.sendall(self.message.encode())
                # TODO stop this method when command land is send

    def find_target_init(self):
        # use calibration matrix to be able to estimate pose
        try:
            self.cameraDistortion = np.loadtxt('distortion.txt')
            self.cameraMatrix = np.loadtxt('camera_matrix.txt')
        except FileNotFoundError:
            self.logger.error("Calibration file not found error.")
            self.message = "error\n"
            return False

        # length of real marker in meters
        self.markerLength = {
            0: 0.545,
            1: 0.185,
            2: 0.185,
            3: 0.185
        }
        activated_ids_set = set(self.markerLength.keys())
        activated_ids_set.remove(min(self.markerLength.keys()))
        self.ids_activated = dict.fromkeys(activated_ids_set, -1)
        self.logger.info("starting find_target")
        self.find_target_running = True
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
        self.parameters.cornerRefinementWinSize = 20
        self.parameters.cornerRefinementMaxIterations = 60
        return True


def process(x, y, z, alfa):
    alfa = str(alfa).replace("[", "")
    alfa = alfa.replace("]", "")
    x_angle = np.rad2deg(np.arctan(x / z))
    y_angle = np.rad2deg(np.arctan(y / z))

    if z < 0:
        return "loiter\n"
    elif 0 < z < 13:
        angle = 10
    else:
        angle = 20

    if z < 0.3:
        return "land\n"
    elif (abs(x_angle) > angle) or (abs(y_angle) > angle):
        # if the drone is outside virtual border move it towards the center
        return "move," + str(x) + "," + str(y) + "," + str(alfa) + "\n"
    else:
        return "descend\n"


def timeMillis():
    return int(round(time.time() * 1000))
