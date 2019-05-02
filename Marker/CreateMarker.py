import cv2.aruco as aruco
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-i", "--id", help="the id of the marker default 1")
args = parser.parse_args()

id = 1 if not args.id else args.id

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
img = aruco.drawMarker(aruco_dict, id, 800)

# good for A4 paper
plt.figure(figsize=(9.5, 9.5), dpi=100)
plt.imshow(img, cmap='gray', aspect="equal")
plt.axis("off")
plt.savefig("marker" + str(id) + ".pdf", bbox_inches="tight")
plt.show()