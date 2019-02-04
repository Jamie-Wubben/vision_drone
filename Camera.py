import cv2, time, os
import cv2.aruco as aruco
import threading


class Camera(threading.Thread):
    def __init__(self, working_mode):
        threading.Thread.__init__(self)
        if working_mode == "pi":
            os.system('sudo modprobe bcm2835-v4l2')
            self.cap = cv2.VideoCapture(0)
            time.sleep(0.1)
        else:
            self.cap = cv2.VideoCapture(0)
        self.running = True
        self.ret = None
        self.frame = None
        self.markerLength = 0.25  # length of real marker in meters

    def run(self):
        self.start_camera()

    def close(self):
        self.stop_camera()
        self.cap.release()
        cv2.destroyAllWindows()

    def start_camera(self):
        while self.running:
            self.ret, self.frame = self.cap.read()
            cv2.imshow('frame', self.frame)
            if cv2.waitKey(20) & 0xFF == ord('q'):
                break

    def stop_camera(self):
        self.running = False

    def find_target(self):
        target_found = False
        while self.running or not target_found:
            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            parameters = aruco.DetectorParameters_create()

            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            gray = aruco.drawDetectedMarkers(gray, corners)
            cv2.imshow('detected', gray)
            if cv2.waitKey(20) & 0xFF == ord('q'):
                break

            if corners:
                print("corners")
                print(corners)
                print("ids")
                print(ids)
                camera_matrix = [[555.25002546, 0., 255.24349188], [0., 557.92461715, 273.10908953], [0., 0., 1.]]
                dist_coeffs = [[2.24271864e+00, -1.75524841e+01, -2.02023682e-02, -5.08466811e-02, 3.51847199e+01]]
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, self.markerLength, camera_matrix, dist_coeffs)
                print("rvec")
                print(rvec)
                print("tvec")
                print(tvec)
                target_found = True
