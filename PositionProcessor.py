import time
import numpy as np


class PositionProcessor:

    def __init__(self):
        self.lastTime = timeMillis()
        self.angle = 10

    """
    This method takes the x,y,z coordinates from the camera
    checks if the messages was send within 65 mseconds of the last messages
    returns a string with either: land, descend, (x,y) or empty
    """
    def process(self, x, y, z,alfa):
        # TODO change constant to something more intelligent
        # only do something is the marker is detected continuously
        if self.lastTime + 65 > timeMillis():
            self.lastTime = timeMillis()
            x_angle = np.rad2deg(np.arctan(x / z))
            y_angle = np.rad2deg(np.arctan(y / z))

            if z < 0.5:
                return "land\n"
            elif (abs(x_angle) > self.angle) or (abs(y_angle) > self.angle):
                # if the drone is outside virtual border move it towards the center
                return "move," + str(x) + "," + str(y) + "," + str(alfa) + "\n"
            else:
                return "descend\n"
        else:
            self.lastTime = timeMillis()
            return "loiter\n"


def timeMillis():
    return int(round(time.time() * 1000))
