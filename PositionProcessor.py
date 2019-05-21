import time
import numpy as np


class PositionProcessor:

    def __init__(self):
        self.angle = 10

    """
    This method takes the x,y,z,alfa coordinates from the camera
    returns a string with either: land, descend, (x,y) or empty
    """
    def process(self, x, y, z, alfa):
        alfa = str(alfa).replace("[", "")
        alfa = alfa.replace("]", "")
        x_angle = np.rad2deg(np.arctan(x / z))
        y_angle = np.rad2deg(np.arctan(y / z))

        if z > 13:
            self.angle = 20
        else:
            self.angle = 10

        if z < 0.3:
            return "land\n"
        elif (abs(x_angle) > self.angle) or (abs(y_angle) > self.angle):
            # if the drone is outside virtual border move it towards the center
            return "move," + str(x) + "," + str(y) + "," + str(alfa) + "\n"
        else:
            return "descend\n"
