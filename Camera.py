import cv2
import cv2.aruco as aruco
import time, os
import numpy as np
import yaml
import Logger as log
import _thread


class Camera:
    logger = log.logger

    def __init__(self, working_mode):
        self.logger.info("Init camera")

        # if pi run modprobe to be able to acces the picam
        if working_mode == "pi":
            os.system('sudo modprobe bcm2835-v4l2')
            time.sleep(0.1)

        # use calibration matrix to be able to estimate pose
        global calibration
        try:
            with open("Calibration/calibration.yaml", 'r') as stream:
                calibration = yaml.load(stream)
            self.logger.info("Calibration file found and loaded")
        except FileNotFoundError or yaml.YAMLError:
            self.logger.error("Calibration file not found error.")
            # TODO close program if file not found
        self.cameraDistortion = np.array(calibration.get('dist_coeff')[0])  # TODO fix [0]
        self.cameraMatrix = np.array(calibration.get('camera_matrix'))
        self.markerLength = 0.25  # length of real marker in meters

        self.running = True
        self.ret = None
        self.frame = None
        self.out = None
        self.cap = None

    def print_time(self, threadName, delay):
        count = 0
        while count < 5:
            time.sleep(delay)
            count += 1
            print("%s: %s" % (threadName, time.ctime(time.time())))

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
            found = self.find_target()
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
        self.cap = cv2.VideoCapture(0)
        print("cap is opened " + str(self.cap.isOpened()))
        # if runned for second time program stays in wile loop because other messages are not received anymore
        while True:
            self.ret, self.frame = self.cap.read()
            # record and show the camerafeeds
            if self.ret:
                self.out.write(self.frame)
                # cv2.imshow('frame', self.frame)
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
        self.logger.info("Searching for target")
        while self.running:
            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

            corners, ids, rejectedImgPoints = aruco.detectMarkers(image=gray, dictionary=aruco_dict,
                                                                  cameraMatrix=self.cameraMatrix,
                                                                  distCoeff=self.cameraDistortion)
            # gray = aruco.drawDetectedMarkers(gray, corners)
            # if cv2.waitKey(20) & 0xFF == ord('q'):
            # break

            if corners:
                self.logger.info("Target found.")
                ret = aruco.estimatePoseSingleMarkers(corners, self.markerLength,
                                                      self.cameraMatrix,
                                                      self.cameraDistortion)
                # TODO ugly
                rotation_vector, translation_vector = ret[0][0, 0, :], ret[1][0, 0, :]

                print(translation_vector)

        return True
