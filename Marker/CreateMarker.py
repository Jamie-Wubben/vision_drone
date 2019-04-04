import cv2.aruco as aruco
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-i", "--id", help="the id of the marker default 1")
args = parser.parse_args()

id = 1 if not args.id else args.id

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
img = aruco.drawMarker(aruco_dict, id, 600)
fig = plt.figure()
plt.imshow(img, cmap = 'gray')
plt.axis("off")
plt.savefig("marker" + str(id) + ".pdf")
plt.show()
