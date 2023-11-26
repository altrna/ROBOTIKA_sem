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




def get_single_cube_pose(camera, camera_matrix, distortion_matrix):
	

	aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
	params = cv2.aruco.DetectorParameters_create()
	image = camera.retrieveBuffer()
	image = image.convert(PyCapture2.PIXEL_FORMAT.RGB8)


	rgb_cv_image = np.array(image.getData(), dtype="uint8").reshape((image.getRows(), image.getCols(), 3))
	gray_cv_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(gray_cv_image, 80, 255, 0)

	corners, ids, rejected_im_points = cv2.aruco.detectMarkers(thresh, aruco_dict,parameters = params)
	rgb_cv_image = cv2.aruco.drawDetectedMarkers(rgb_cv_image, corners = corners, ids=ids, borderColor=(0, 255, 255))

	r_vec, t_vec= cv2.aruco.estimatePoseSingleMarkers(corners, 0.039, camera_matrix, distortion_matrix)
	cv2.imshow('frame',rgb_cv_image)

	return r_vec, t_vec

if __name__ == "__main__":
   
	cam_matrix = np.array([[  2.22780761e+03,   0.00000000e+00 ,  6.79803570e+02], [  0.00000000e+00, 2.23516942e+03, 4.69847972e+02], [  0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

	dist_matrix = np.array([-0.07772019 , 0.27231058 , 0.00259939 , 0.00127269 ,-0.94383607])
	bus = PyCapture2.BusManager()
	camera = PyCapture2.Camera()

	# Select first camera on the bus
	camera.connect(bus.getCameraFromIndex(0))

	# Start capture
	camera.startCapture()
	r_vec, t_vec = get_single_cube_pose(camera, cam_matrix, dist_matrix)
	#T_RC = SE3(translation=[520, -75, 1250], rotation=SO3.ry(np.pi))
	#t_pose = T_RC.act(t_vec)
	robot = robCRS97()
	cmd = Commander(robot)
	cmd.open_comm("/dev/ttyUSB0")
	cmd.init()
	for x in range(300,500, 100):
		for y in range (-300, 0, 100):
			pos = np.array([x, y, 400, 0,0,0])
			ikt_pos = robCRSikt(rob, pos)[0]
			cmd.coordmv(cmd.anglestoirc(ikt_pos)
			r_vec, t_vec = get_single_cube_pose(camera, cam_matrix, dist_matrix)
			t, pos = cmd.axis_get_pos()
			pos_in_deg = cmd.irctoangles(pos)
			dkt_pos = robCRSdkt(robot, pos_in_deg)
			print("Robot: ", dkt_pos,pos)
			print("Camera: ", t_vec, r_vec)
			input()
		
		
	
	#desired_position_angles = robCRSikt(robot, t_pose)
	#desired_position_irc = cmd.anglestoirc(p_deg)
	#cmd.coordmv(desired_position_irc)

  
