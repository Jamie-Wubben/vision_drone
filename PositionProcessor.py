import time
import numpy as np


class PositionProcessor:

    def __init__(self):
        self.lastTime = timeMillis()
        self.angle = 10
        self.lastMessage = ""

    """
    This method takes the x,y,z coordinates from the camera
    checks if the messages was send within 65 mseconds of the last messages
    returns a string with either: land, descend, (x,y) or empty
    """
    def process(self, x, y, z):
        # TODO change constant to something more intelligent
        # only do something is the marker is detected continuously
        if self.lastTime + 65 > timeMillis():
            x_angle = np.rad2deg(np.arctan(x / z))
            y_angle = np.rad2deg(np.arctan(y / z))

            if z < 0.5:
                if self.lastMessage is not "land":
                    self.lastMessage = "land"
                    return "land\n"
            elif (abs(x_angle) > self.angle) or (abs(y_angle) > self.angle):
                # if the drone is outside virtual border move it towards the center
                self.lastMessage = str(x) + "," + str(y)
                return str(x) + "," + str(y) + "\n"
            else:
                if self.lastMessage is not "descend":
                    self.lastMessage = "descend"
                    return "descend\n"
            return ""
        self.lastTime = timeMillis()
        return ""


def timeMillis():
    return int(round(time.time() * 1000))
