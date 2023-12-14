import numpy as np
from core.se3 import *
from core.so3 import *


class Aruco:
    def __init__(self, SE3, aruco_id):
        self.SE3 = SE3
        self.id = aruco_id
        self.layer = self.get_layer()
        self.angle = None
        self.solvable = True

    def get_layer(self):
        return int(np.clip((self.SE3.translation[2] - 11) // 50, -1, 8))

    def __repr__(self):
        return "Layer: " + str(self.layer) + "\nSE3: " + self.SE3.__repr__() + "\nID: " + str(self.id)
