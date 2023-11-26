import cv2
from core.so3 import *

"""
[450 -10 550   0   0   0]
Robot:  [  4.49998826e+02  -1.00424940e+01   5.50018150e+02  -4.90701886e-03
  -2.20060224e-03  -6.01824994e-03] [-185161   18656   63938  108305    9532 -107155]
Camera:  [[[ 0.01389302  0.07296657  0.68056817]]] [[[-2.13977189  2.15591813  0.09443021]]]
"""


# X, Y = cv2.calibrateRobotWorldHandEye()

# t2c_rot = SO3.from_euler_angles([-2.13977189, 2.15591813, 0.09443021], "xyz")

# t2c_rot = np.asmatrix(t2c_rot.rot)
# b2r_rot = np.asmatrix(np.eye(3))

pos_r = [np.deg2rad(i.flatten()) for i in np.load("posr.npz", allow_pickle=True)["arr_0"]]
pos_t = [i.flatten() for i in np.load("post.npz", allow_pickle=True)["arr_0"]]
cam_r = [i.flatten() for i in np.load("cam_r_vec.npz", allow_pickle=True)["arr_0"]]
cam_t = [1000 * i.flatten() for i in np.load("cam_t_vec.npz", allow_pickle=True)["arr_0"]]

X = cv2.calibrateRobotWorldHandEye(
    pos_r,
    pos_t,
    cam_r,
    cam_t,
)
print(X)
pass
# print(Y)
