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
		return int(np.clip((self.SE3.translation[2] - 10) // 50, -1, 5))
	
	def grab_angle(self, angle):
		self.angle = angle

	def __repr__(self):
		return "Layer: " + str(self.layer) + "\nSE3: " + self.SE3.__repr__() + "\nID: " + str(self.id)





def get_distance_from_other_arucos_in_layer(all_arucos_in_layer):
	def get_distance_matrix(a):
		return np.sqrt(np.sum(np.square(a[:, np.newaxis, :] - a[np.newaxis, :, :]), axis=-1))
	all_centres = np.array([x.SE3.translation for x in all_arucos_in_layer])
	dist_mat = get_distance_matrix(all_centres)
	print(dist_mat)
	return dist_mat


def get_arucos_pose(camera, camera_matrix, distortion_matrix, base2cam):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    params = cv2.aruco.DetectorParameters_create()
    image = camera.retrieveBuffer()
    image = image.convert(PyCapture2.PIXEL_FORMAT.RGB8)

    rgb_cv_image = np.array(image.getData(), dtype="uint8").reshape((image.getRows(), image.getCols(), 3))
    gray_cv_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray_cv_image, 80, 255, 0)

    corners, ids, rejected_im_points = cv2.aruco.detectMarkers(thresh, aruco_dict, parameters=params)
    r_vec, t_vec = cv2.aruco.estimatePoseSingleMarkers(corners, 39, camera_matrix, distortion_matrix)
    l = np.shape(r_vec)[0]
    arucos = []
    for i in range(l):
        cam2aruco = SE3(t_vec[i].flatten(), SO3.exp(r_vec[i].flatten()))
        base2aruco = base2cam * cam2aruco
        arucos.append(Aruco(base2aruco, ids[i][0]))
    return arucos


def move_to_pos(cmd, des_pos):
    try:
        irc = cmd.find_closest_ikt(des_pos)
    except:
        irc = None
    if irc is not None:
        cmd.wait_ready()
        cmd.coordmv(irc)
        cmd.wait_ready()
    else:
        print("ERROR: No IKT!")
        cmd.init(hard_home=False)


def release(cmd):
	robCRSgripper(cmd, -1)
	cmd.wait_gripper_ready()

def grip(cmd):
	robCRSgripper(cmd, 0.1)
	cmd.wait_gripper_ready()

def pick_up(cmd, cube_se3, layer, max_layer):
	release(cmd)
	cube_se3.translation[0:2] += [20, -8]
	cube_se3.translation[2] = 50 * (layer + 2)
	yaw = np.arctan2(cube_se3.rotation.rot[1, 0], cube_se3.rotation.rot[0, 0])
	des_pos = [*cube_se3.translation, np.rad2deg(yaw), 90, 0]
	print(des_pos)
	move_to_pos(cmd, des_pos)
	des_pos[2] = 50 * (layer + 1)
	#print(des_pos)
	move_to_pos(cmd, des_pos)
	grip(cmd)
	des_pos[2] = 50 * (max_layer + 2)
	move_to_pos(cmd, des_pos)
	#print(des_pos)

def sort_layers(all_arucos):
	layer_dict = dict()
	for i in range(len(all_arucos)):
		cl = all_arucos[i].layer
		if cl not in layer_dict:
			layer_dict[cl] = [all_arucos[i]]
		else:
			layer_dict[cl].append(all_arucos[i])
	return layer_dict

def main_init(cmd, camera, cam_matrix, dist_matrix, base2cam):
	irc = cmd.anglestoirc([0,0,0,0,0,0])
	cmd.wait_ready()
	cmd.coordmv(irc)
	cmd.wait_ready()
	sort_dict = {}
	boxes = []
	arucos = get_arucos_pose(camera, cam_matrix, dist_matrix, base2cam)
	for aruco in arucos:
		if aruco.layer==-1:
			boxes.append(aruco)
	for aruco in arucos:
		if aruco.layer>=0 and aruco.id not in sort_dict.keys():
			if boxes != []:
				sort_dict[aruco.id] = boxes[0]
				boxes = boxes[1:]
			else:
				print("Not enough boxes for cubes")
				return {}
	return sort_dict

def get_layers_in_order(all_arucos):
	layer_dict = sort_layers(all_arucos)
	boxes = layer_dict[-1]
	kwargs = list(layer_dict.keys())
	kwargs.sort(reverse=True)
	kwargs.pop(-1)
	return layer_dict, boxes, kwargs


def get_cube_from_layer(all_arucos_in_layer):
	layer_distance_matrix = get_distance_from_other_arucos_in_layer(all_arucos_in_layer)
	idx_mask_sum = np.sum(layer_distance_matrix <= 60, axis=0)
	for i in range(len(idx_mask_sum)):
		if idx_mask_sum[i] > 1:
			continue
		closest_aruco = all_arucos_in_layer[np.argsort(layer_distance_matrix[i])[1]]
		current_aruco = all_arucos_in_layer[i]
		transformation = (current_aruco.SE3.inv() * closest_aruco.SE3).transformation[:2]
		print(np.arctan(transformation[1], transformation[0])
		
if __name__ == "__main__":
	robot = robCRS97()
	cmd = Commander(robot)
	cmd.open_comm("/dev/ttyUSB0", speed=19200)
	cmd.init(hard_home=False)
	# cmd.hard_home()
	cmd.soft_home()
	robCRSgripper(cmd, -1)
	bus = PyCapture2.BusManager()
	camera = PyCapture2.Camera()

	camera.connect(bus.getCameraFromIndex(0))

	camera.startCapture()
	cam_matrix = np.array(
		[
		    [2.10813831e03, 0.00000000e00, 6.49912597e02],
		    [0.00000000e00, 2.11337088e03, 3.45497271e02],
		    [0.00000000e00, 0.00000000e00, 1.00000000e00],
		]
	)

	dist_matrix = np.array([-6.60263309e-02, -4.23421181e-01, -1.32225502e-02, 1.26508906e-03, 2.59213104e00])
	aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
	camr = np.load("Cam_R.npz")["arr_0"]
	camt = np.load("Cam_T.npz")["arr_0"].flatten()
	base2cam = SE3(camt, SO3(camr))

	#sort_dict = main_init(cmd, camera, cam_matrix, dist_matrix, base2cam)

	cp = get_arucos_pose(camera, cam_matrix, dist_matrix, base2cam)

	get_cube_from_layer(cp)

	layer_order, boxrs, sorted_indices = get_layers_in_order(cp)
	max_layer = sorted_indices[0]


	for i in sorted_indices:

		for j in layer_order[i]:

			#pick_up(cmd, j.SE3, i, max_layer)
			#des_pos = [*sort_dict[j.id].SE3.translation, 0, 90, 0]
		#	des_pos[2] = 250
			#move_to_pos(cmd, des_pos)
			release(cmd)
			pass
