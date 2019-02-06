import cv2, time, os
import cv2.aruco as aruco
import threading
import numpy as np
import yaml
import pickle


class Camera(threading.Thread):
    def __init__(self, working_mode):
        global calibration
        threading.Thread.__init__(self)
        # if pi run modprobe to be able to acces the picam
        if working_mode == "pi":
            os.system('sudo modprobe bcm2835-v4l2')
            self.cap = cv2.VideoCapture(0)
            time.sleep(0.1)
        else:
            self.cap = cv2.VideoCapture(0)

        # for recording
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        dir_name = "./FlightMovies"
        counter = 0
        for f in os.listdir(dir_name):
            if os.path.isfile(f):
                counter += 1
        self.out = cv2.VideoWriter(dir_name + "/video" + str(counter) + ".avi", fourcc, 20.0, (640, 480))

        # use calibration matrix to be able to estimate pose
        try:
            with open("Calibration/calibration.yaml", 'r') as stream:
                calibration = yaml.load(stream)
        except FileNotFoundError or yaml.YAMLError:
            print("Not able to open calibration.yaml please calibrate the camera first.")
            # TODO close program if file not found
        self.cameraDistortion = np.array(calibration.get('dist_coeff')[0])  # TODO fix [0]
        self.cameraMatrix = np.array(calibration.get('camera_matrix'))
        self.markerLength = 0.25  # length of real marker in meters

        self.running = True
        self.ret = None
        self.frame = None

    def run(self):
        self.start_camera()

    def close(self):
        self.stop_camera()
        self.cap.release()
        self.out.release()
        cv2.destroyAllWindows()

    def start_camera(self):
        while self.running:
            self.ret, self.frame = self.cap.read()
            # record and show the camerafeeds
            if self.ret:
                self.out.write(self.frame)
                cv2.imshow('frame', self.frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                break

    def stop_camera(self):
        self.running = False

    def find_target(self):
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
                ret = aruco.estimatePoseSingleMarkers(corners, self.markerLength,
                                                      self.cameraMatrix,
                                                      self.cameraDistortion)
                # TODO ugly
                rotation_vector, translation_vector = ret[0][0, 0, :], ret[1][0, 0, :]

                print(translation_vector)

        return True
