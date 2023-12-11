import cv2
from core.so3 import *
import glob


pos_r = [
    (SO3.rz(np.deg2rad(i.flatten())[0]) * SO3.ry(np.deg2rad(i.flatten())[1]) * SO3.rx(np.deg2rad(i.flatten())[2])).rot
    for i in np.load("data/posr.npz", allow_pickle=True)["arr_0"]
]  # DKT rotace
pos_t = [i.flatten() for i in np.load("data/post.npz", allow_pickle=True)["arr_0"]]
# DKT tranclace
cam_t = [i.flatten() for i in np.load("data/cam_t_vec.npz", allow_pickle=True)["arr_0"]]  # Camrea2Cube translace
cam_r = [i.flatten() for i in np.load("data/cam_r_vec.npz", allow_pickle=True)["arr_0"]]  # Camrea2Cube rotace


for i in range(len(cam_r)):
    cam_r[i] = SO3.exp(cam_r[i]).rot

X, Y, U, V = cv2.calibrateRobotWorldHandEye(pos_r, pos_t, cam_r, cam_t)
# np.savez("Cam_R.npz", U)
# np.savez("Cam_T.npz", V)

# print(X)  # Gripper2Cube translation
# print(Y)  # Gripper2Cube Rotation
print(U)  # base2Camrea translation
print(V)  # base2Camera Rotation
pass
