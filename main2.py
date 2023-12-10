import sys
import numpy as np
sys.path.append("/bluebot/data/Pyrocon")
from CRS_commander import Commander
from robCRSgripper import robCRSgripper
from robCRSdkt import robCRSdkt
from robCRSikt import robCRSikt
from robotCRS import robCRS97
from core.se3 import *
from core.so3 import *
import PyCapture2
import cv2


class Aruco:
	def __init__(self, SE3, aruco_id):
		self.SE3 = SE3
		self.id = aruco_id
		self.layer = self.get_layer()	

	def get_layer(self):
		return np.clip((self.SE3.translation[2]-10)//50, -1, 5)

	def is_grabbable(self, all_arucos):
		...
		self.grabbable = ...
	def __repr__(self):
		return "Layer: " + str(self.layer) + "\nSE3: " + self.SE3.__repr__() + "\nID: " + str(self.id)

def get_arucos_pose(camera, camera_matrix, distortion_matrix, base2cam):
	aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
	params = cv2.aruco.DetectorParameters_create()
	image = camera.retrieveBuffer()
	image = image.convert(PyCapture2.PIXEL_FORMAT.RGB8)


	rgb_cv_image = np.array(image.getData(), dtype="uint8").reshape((image.getRows(), image.getCols(), 3))
	gray_cv_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(gray_cv_image, 80, 255, 0)

	corners, ids, rejected_im_points = cv2.aruco.detectMarkers(thresh, aruco_dict,parameters = params)
	r_vec, t_vec = cv2.aruco.estimatePoseSingleMarkers(corners, 39, camera_matrix, distortion_matrix)
	l = np.shape(r_vec)[0]
	arucos = []
	for i in range(l):
		cam2aruco = SE3(t_vec[i].flatten(), SO3.exp(r_vec[i].flatten()))
		base2aruco = base2cam*cam2aruco
		arucos.append(Aruco(base2aruco, ids[i]))
	return arucos

def move_to_pos(cmd, des_pos):
	try:
		irc = cmd.find_closest_ikt(des_pos)
		cmd.wait_ready()
		cmd.coordmv(irc)
		cmd.wait_ready()
		return True
	except:
		irc = None
		print("ERROR: No IKT!")
		return False

def release(cmd, val=-1):
	robCRSgripper(cmd, val)

def pick_up(cmd, cube_se3):
	release(cmd, val=-1.5)
	cube_se3.translation[0:2] += [20,-8]
	cube_se3.translation[2] = 100
	yaw = np.arctan2(cube_se3.rotation.rot[1,0], cube_se3.rotation.rot[0,0])
	des_pos = [*cube_se3.translation, np.rad2deg(yaw), 90, 0]
	print(des_pos)
	cont = move_to_pos(cmd, des_pos)
	if not cont:
		return
	des_pos[2] = 50
	print(des_pos)
	cont = move_to_pos(cmd, des_pos)
	if not cont:
		return
	robCRSgripper(cmd, 0.1)
	cmd.wait_gripper_ready()
	des_pos[2] = 200
	cont = move_to_pos(cmd, des_pos)
	if not cont:
		return
	print(des_pos)

if __name__ == "__main__":
	robot = robCRS97()
	cmd = Commander(robot)
	cmd.open_comm("/dev/ttyUSB0", speed=19200)
	cmd.init(hard_home=False)
	#cmd.hard_home()
	cmd.soft_home()	
	robCRSgripper(cmd, 0)
	bus = PyCapture2.BusManager()
	camera = PyCapture2.Camera()

	camera.connect(bus.getCameraFromIndex(0))

	camera.startCapture()
	cam_matrix = np.array([[  2.10813831e+03, 0.00000000e+00,   6.49912597e+02],
 [  0.00000000e+00,   2.11337088e+03,   3.45497271e+02],
 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
	"""new matrix = [[  2.31616457e+03   0.00000000e+00   6.31242571e+02]
 	[  0.00000000e+00   2.34163983e+03   2.40442592e+02]
 	[  0.00000000e+00   0.00000000e+00   1.00000000e+00]]"""

	dist_matrix = np.array([ -6.60263309e-02, -4.23421181e-01,  -1.32225502e-02,   1.26508906e-03, 2.59213104e+00])
	dist_matrix = np.array([  1.31600325e-01 , -2.13500794e+00 , -3.13652860e-02,   1.69360473e-03,
    9.34997291e+00])# new matrix images/charuco2
	aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
	camr = np.load("Cam_R.npz")["arr_0"]
	camt = np.load("Cam_T.npz")["arr_0"].flatten()
	print(camr, camt)
	base2cam = SE3(camt, SO3(camr))
	cp = get_arucos_pose(camera, cam_matrix, dist_matrix, base2cam)
	for i in range(len(cp)):
		#print(cp[i])
		print(cp[i])
		#pick_up(cmd, cp[i])
		#move_to_pos(cmd, [0, -400, 150, 0, 90, 0])
		release(cmd)
	
	
	


