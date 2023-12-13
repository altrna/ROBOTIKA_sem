import sys
import numpy as np
sys.path.append("/bluebot/data/Pyrocon")
from CRS_commander import Commander

from robCRSgripper import robCRSgripper
from robCRSdkt import robCRSdkt
from robCRSikt import robCRSikt
from robotCRS import robCRS97
from core.se3 import *
import PyCapture2
import cv2
import time



def get_single_cube_pose(camera, camera_matrix, distortion_matrix):
    

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    params = cv2.aruco.DetectorParameters_create()
    image = camera.retrieveBuffer()
    image = image.convert(PyCapture2.PIXEL_FORMAT.RGB8)


    rgb_cv_image = np.array(image.getData(), dtype="uint8").reshape((image.getRows(), image.getCols(), 3))
     
    gray_cv_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray_cv_image, 80, 255, 0)

    corners, ids, rejected_im_points = cv2.aruco.detectMarkers(thresh, aruco_dict,parameters = params)
    #rgb_cv_image = cv2.aruco.drawDetectedMarkers(rgb_cv_image, corners = corners, ids=ids, borderColor=(0, 255, 255))

    r_vec, t_vec= cv2.aruco.estimatePoseSingleMarkers(corners, 39, camera_matrix, distortion_matrix)
    cv2.imshow('frame',rgb_cv_image)
    return r_vec, t_vec, rgb_cv_image

if __name__ == "__main__":
    cam_matrix = np.array([[  2.10813831e+03, 0.00000000e+00,   6.49912597e+02],
 [  0.00000000e+00,   2.11337088e+03,   3.45497271e+02],
 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])

    cam_matrix = np.array([[  2.31616457e+03 ,  0.00000000e+00  , 6.31242571e+02],
 	[  0.00000000e+00  , 2.34163983e+03 ,  2.40442592e+02],
 	[  0.00000000e+00 ,  0.00000000e+00  , 1.00000000e+00]]) #new

    dist_matrix = np.array([ -6.60263309e-02, -4.23421181e-01,  -1.32225502e-02,   1.26508906e-03, 2.59213104e+00])
    dist_matrix = np.array([  1.31600325e-01 , -2.13500794e+00 , -3.13652860e-02,   1.69360473e-03,
    9.34997291e+00])# new matrix images/charuco2
    bus = PyCapture2.BusManager()
    camera = PyCapture2.Camera()

    # Select first camera on the bus
    camera.connect(bus.getCameraFromIndex(0))

    # Start capture
    camera.startCapture()
    r_vec, t_vec, img4 = get_single_cube_pose(camera, cam_matrix, dist_matrix)
    #T_RC = SE3(translation=[520, -75, 1250], rotation=SO3.ry(np.pi))
    #t_pose = T_RC.act(t_vec)
    robot = robCRS97()
    cmd = Commander(robot)
    cmd.open_comm("/dev/ttyUSB0")
    cmd.init()
    robCRSgripper(cmd, 0.025)
    prev_irc = None
    positions = []
    rotations = []
    cam_r = []
    cam_t = []
    random_configs = [np.random.randint(-25, 25, (3,)) for i in range(10)]
    for x in range(350, 651, 50):
        for y in range (-190, 101, 50):
            for z in range(250, 601, 50):
                cmd.wait_ready()
                this_config = np.random.randint(0, len(random_configs))
                pos = np.array([x, y, z, *random_configs[this_config]])
                print(pos)
                irc = None
                try:
                    irc = cmd.find_closest_ikt(pos)
                except:
                    print(len(robCRSikt(robot, pos)))
                #ikt_pos = robCRSikt(robot, pos)[0]
                if irc is not None:
                    cmd.coordmv(irc)
                else:
                     continue
                cmd.wait_ready()
                r_vec, t_vec, img44 = get_single_cube_pose(camera, cam_matrix, dist_matrix)
                t, pos = cmd.axis_get_pos()
                pos_in_deg = cmd.irctoangles(pos)
                dkt_pos = robCRSdkt(robot, pos_in_deg)
                if t_vec is not None and r_vec is not None:
                    positions.append(dkt_pos[:3])
                    rotations.append(dkt_pos[-3:])
                    cv2.imwrite(f"images/hand2eye/{dkt_pos}.png", img44)
                    cam_r.append(r_vec)
                    cam_t.append(t_vec)
                    
                print("Robot: ", dkt_pos,pos)
                print("Camera: ", t_vec, r_vec)
                print("-------------------------------")
                prev_irc = irc
    np.savez("post.npz", positions)
    np.savez("posr.npz", rotations)
    np.savez("cam_r_vec.npz", cam_r)
    np.savez("cam_t_vec.npz", cam_t)
    #desired_position_angles = robCRSikt(robot, t_pose)
    #desired_position_irc = cmd.anglestoirc(p_deg)
    #cmd.coordmv(desired_position_irc)

  
