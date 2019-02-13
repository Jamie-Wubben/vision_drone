# information based on https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/sandbox/ludovic/aruco_calibration_rotation.html
import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox
import os
import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
# squaresX, squaresY, squareLength, markerLength, dictionary
board = aruco.CharucoBoard_create(4, 4, 0.05, 0.04, aruco_dict)
font = cv2.FONT_HERSHEY_PLAIN


def makeMarker(path):
    id = 1
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    img = aruco.drawMarker(aruco_dict, id, 600)
    fig = plt.figure()
    plt.imshow(img, cmap='gray')
    plt.axis("off")
    plt.savefig(path + "/marker" + str(id) + ".pdf")
    plt.show()


def makeBoard(path):
    """
    makes a CHARUCO chessboard that fits on a A4 paper shows it to the user and saves it in a directory
    :param path: path to save the chessboard
    """
    imboard = board.draw((2000, 2000))
    cv2.imwrite(path + "/chessboard.jpg", imboard)
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    plt.imshow(imboard, cmap='gray', interpolation="nearest")
    ax.axis("off")
    plt.show()


def makePictures(path):
    """
    Activates the camera and let the user take pictures with the C-key
    The camera can be turned off by pressing the Q-key
    Each picture is saved in the directory given by the user
    :param path: path to save the pictures
    """
    camera = cv2.VideoCapture(0)
    ret, img = camera.read()
    count = 0
    for f in os.listdir(path):
        if f.find("picture") is not -1:
            count += 1

    while True:
        name = path + "/picture" + str(count) + ".jpg"
        ret, img = camera.read()
        cv2.putText(img, "Press c to take picture.", (10, 30), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(img, "Press q to quit.", (10, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow("img", img)
        key = cv2.waitKey(1)
        if key == 99:
            cv2.imwrite(name, img)
            count += 1
        elif key == 113:
            break

    camera.release()
    cv2.destroyAllWindows()


def read_chessboards(images):
    """Charuco base pose estimation."""
    allCorners = []
    allIds = []
    decimator = 0
    # SUB PIXEL CORNER DETECTION CRITERION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

    for im in images:
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

        if len(corners) > 0:
            # SUB PIXEL DETECTION
            for corner in corners:
                cv2.cornerSubPix(gray, corner, winSize=(3, 3), zeroZone=(-1, -1), criteria=criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3 and decimator % 1 == 0:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        decimator += 1
    imsize = gray.shape
    return allCorners, allIds, imsize


def calibrate_camera(allCorners, allIds, imsize):
    """ Calibrates the camera using the dected corners."""
    cameraMatrixInit = np.array([[1000., 0., imsize[0] / 2.], [0., 1000., imsize[1] / 2.], [0., 0., 1.]])
    distCoeffsInit = np.zeros((5, 1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    # flags = (cv2.CALIB_RATIONAL_MODEL)
    (ret, camera_matrix, distortion, rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
        charucoCorners=allCorners,
        charucoIds=allIds,
        board=board,
        imageSize=imsize,
        cameraMatrix=cameraMatrixInit,
        distCoeffs=distCoeffsInit,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

    return ret, camera_matrix, distortion, rotation_vectors, translation_vectors


def checkResults(images):
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


if __name__ == "__main__":
    # choose path
    root = tk.Tk()
    root.withdraw()
    messagebox.showinfo("Calibration", "The purpose of this program is to calibrate the camera\n"
                                       "There are several steps, each one will be explained later\n")

    answer = messagebox.askyesno("Raspberry pi", "Is this program running on a Raspberry pi")
    # if pi run modprobe to be able to acces the picam
    if answer:
        os.system('sudo modprobe bcm2835-v4l2')

    messagebox.showinfo("make board", "The first step is to make a calibration board\nPlease give a path to save the "
                                      "board")
    pathEmpty = True
    path = None
    while pathEmpty:
        path = filedialog.askdirectory()
        pathEmpty = not os.path.exists(path)

    makeBoard(path)

    messagebox.showinfo("print board", "The board is saved in: " + path + "/chessboard.jpg \nPlease print this board "
                                       "\nNormally the squares have are 0.05m long and the markers 0.04m.\n"
                                       "Check this and make sure the dimensions are the same.")

    messagebox.showinfo("making pictures", "Now we will take some pictures of the chessboard.\n"
                                           "Make sure that there is an webcam attached to your device.\n"
                                           "Pictures will be save in the same directory as the chessboard.\n")
    makePictures(path)
    # read pictures from given path
    images = np.array([path + "/" + f for f in os.listdir(path) if f.find("picture") is not -1])

    messagebox.showinfo("calibration", "Now the camera will be calibrated.\n"
                                       "Calibration files will be stored in: " + path + "/camera_matrix.txt and "
                                                                                        "distortion.txt."
                                       "This may take a will so please be patient.")
    allCorners, allIds, imsize = read_chessboards(images)
    ret, camera_matrix, distortion, rvecs, tvecs = calibrate_camera(allCorners, allIds, imsize)
    np.savetxt('camera_matrix.txt', camera_matrix)
    np.savetxt('distortion.txt', distortion)

    makeMarker(path)
    messagebox.showinfo("check results", "The calibration is complied.\n You can check the results by printing out "
                                         "marker1.pdf Make sure this marker has a length of 0.185m and hold it before "
                                         "the camera.")
    checkResults(images)
