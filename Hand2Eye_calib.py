import cv2
from core.so3 import *
import glob



DIST_MATRIX = np.array([[  2.31616457e+03  , 0.00000000e+00 ,  6.31242571e+02],
 	[  0.00000000e+00  , 2.34163983e+03  , 2.40442592e+02],
 	[  0.00000000e+00 ,  0.00000000e+00  , 1.00000000e+00]])
CAM_MATRIX = np.array([  1.31600325e-01 , -2.13500794e+00 , -3.13652860e-02,   1.69360473e-03,
    9.34997291e+00])

pos_r = [
    (SO3.rz(np.deg2rad(i.flatten())[0]) * SO3.ry(np.deg2rad(i.flatten())[1]) * SO3.rx(np.deg2rad(i.flatten())[2])).rot
    for i in np.load("data/posr.npz", allow_pickle=True)["arr_0"]
]
pos_t = [i.flatten() for i in np.load("data/post.npz", allow_pickle=True)["arr_0"]]

cam_t = [i.flatten() for i in np.load("data/cam_t_vec.npz", allow_pickle=True)["arr_0"]]
cam_r = [i.flatten() for i in np.load("data/cam_r_vec.npz", allow_pickle=True)["arr_0"]]

images = glob.glob("./images/hand2eye/*.png")

# for filename in images:
#     coord = filename.split("/")[3]
#     coord = coord.split("_")
#     image = cv2.imread(filename) 
#     aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
#     params = cv2.aruco.DetectorParameters_create()
#     rgb_cv_image = np.array(image.getData(), dtype="uint8").reshape((image.getRows(), image.getCols(), 3))
#     gray_cv_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_BGR2GRAY)
#     ret, thresh = cv2.threshold(gray_cv_image, 80, 255, 0)

#     corners, ids, rejected_im_points = cv2.aruco.detectMarkers(thresh, aruco_dict,parameters = params)
#     rgb_cv_image = cv2.aruco.drawDetectedMarkers(rgb_cv_image, corners = corners, ids=ids, borderColor=(0, 255, 255))
#     cam_r, cam_t= cv2.aruco.estimatePoseSingleMarkers(corners, 0.039, CAM_MATRIX, DIST_MATRIX)
    




for i in range(len(cam_r)):
    cam_r[i] = SO3.exp(cam_r[i]).rot

X, Y, U, V = cv2.calibrateRobotWorldHandEye(
    pos_r,
    pos_t,
    cam_r,
    cam_t
    
)
# print(X[0]) 
# print(X[1]) 
np.savez("Cam_R.npz", U)
np.savez("Cam_T.npz", V)
print(U) 
print(V) 
pass
# print(Y)
