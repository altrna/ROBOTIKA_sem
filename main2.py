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
	def __init__(self, SE3, aruco_id, real = True):
		self.SE3 = SE3
		self.id = aruco_id
		self.layer = self.get_layer()
		self.real = real
		

	def get_layer(self):
		return int(np.clip((self.SE3.translation[2] - 11) // 50, -1, 5))
	
	def grab_angle(self, angle):
		self.angle = angle

	def __repr__(self):
		return "Layer: " + str(self.layer) + "\nSE3: " + self.SE3.__repr__() + "\nID: " + str(self.id)





def get_distance_from_other_arucos_in_layer(all_arucos_in_layer):
	def get_distance_matrix(a):
		return np.sqrt(np.sum(np.square(a[:, np.newaxis, :] - a[np.newaxis, :, :]), axis=-1))
	all_centres = np.array([x.SE3.translation[:2] for x in all_arucos_in_layer])
	dist_mat = get_distance_matrix(all_centres)
	#print(dist_mat)
	return dist_mat


def get_arucos_pose(cmd, camera, camera_matrix, distortion_matrix, base2cam):
	irc = cmd.anglestoirc([0,0,0,0,0,0])
	cmd.wait_ready()
	cmd.coordmv(irc)
	cmd.wait_ready()
	aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
	params = cv2.aruco.DetectorParameters_create()
	image = camera.retrieveBuffer()
	image = image.convert(PyCapture2.PIXEL_FORMAT.RGB8)

	rgb_cv_image = np.array(image.getData(), dtype="uint8").reshape((image.getRows(), image.getCols(), 3))
	gray_cv_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(gray_cv_image, 80, 255, 0)

	corners, ids, rejected_im_points = cv2.aruco.detectMarkers(thresh, aruco_dict, parameters=params)
	r_vec, t_vec = cv2.aruco.estimatePoseSingleMarkers(corners, 39.5, camera_matrix, distortion_matrix)
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
        return True
    else:
        print("ERROR: No IKT!")
        cmd.init(hard_home=False)
        return False


def release(cmd):
	robCRSgripper(cmd, -1)
	cmd.wait_gripper_ready()

def grip(cmd):
	robCRSgripper(cmd, 0.1)
	cmd.wait_gripper_ready()

def pick_up(cmd, cube, layer, max_layer):
	release(cmd)
	cube.SE3.translation[0:2] += [20, -12]
	cube.SE3.translation[2] = 50 * (layer + 2.5)
	yaw = cube.angle
	des_pos = [*cube.SE3.translation, yaw, 90, 0]
	move_to_pos(cmd, des_pos)
	des_pos[2] = 50 * (layer + 1)
	move_to_pos(cmd, des_pos)
	grip(cmd)
	des_pos[2] = 50 * (layer + 2.5)
	move_to_pos(cmd, des_pos)


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
	sort_dict = {}
	boxes = []
	arucos = get_arucos_pose(cmd, camera, cam_matrix, dist_matrix, base2cam)
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
				#TODO box sort
				return {}
	return sort_dict

def get_layers_in_order(all_arucos):
	layer_dict = sort_layers(all_arucos)
	kwargs = list(layer_dict.keys())
	kwargs.sort(reverse=True)
	if -1 in kwargs:
		kwargs.pop(-1)
	return layer_dict, kwargs


def get_cube_angle_from_layer(all_arucos_in_layer):
	if len(all_arucos_in_layer) == 1:
		angle = np.rad2deg(np.arctan2(all_arucos_in_layer[0].SE3.rotation.rot[1, 0], all_arucos_in_layer[0].SE3.rotation.rot[0, 0]))
		all_arucos_in_layer[0].grab_angle(angle)
		return
	layer_distance_matrix = get_distance_from_other_arucos_in_layer(all_arucos_in_layer)
	idx_mask_sum = np.sum(layer_distance_matrix <= 65, axis=0)
	for i in range(len(idx_mask_sum)):
		if idx_mask_sum[i] > 2:
			all_arucos_in_layer[i].grab_angle(None)
			#TODO nemozna reseni
			continue
		closest_aruco = all_arucos_in_layer[np.argsort(layer_distance_matrix[i])[1]]
		current_aruco = all_arucos_in_layer[i]
		translation = (current_aruco.SE3.inverse() * closest_aruco.SE3).translation[:2]
		x_vec = np.array([1,0])
		t_0 = translation/(np.linalg.norm(translation)+1e-10)
		dot_product=np.dot(t_0, x_vec)
		angle = np.rad2deg(np.arctan2(current_aruco.SE3.rotation.rot[1, 0], current_aruco.SE3.rotation.rot[0, 0]))
		angle += 90 if abs(dot_product)<0.2 else 0
		all_arucos_in_layer[i].grab_angle(angle)
		print(all_arucos_in_layer[i].angle)

def find_ikt_no_angle(des_pos):
	x = np.linspace(-80, 80, 40)
	y = np.linspace(0, 100, 20)
	z = np.linspace(-80, 80, 40)
	z,y,x = np.meshgrid(z, y, x)
	all_angles=np.array([z.flatten(),y.flatten(), x.flatten()]).T
	for i in range(np.shape(all_angles)[0]):
		cur_pos = [*des_pos, *all_angles[i]]
		try:
			irc = cmd.find_closest_ikt(cur_pos)
			return irc
		except:
			continue
	raise Exception("NO IKT FOUND")
	

if __name__ == "__main__":

	robot = robCRS97()
	cmd = Commander(robot)
	cmd.open_comm("/dev/ttyUSB0", speed=19200)
	cmd.init(hard_home=False)
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
	sort_dict = main_init(cmd, camera, cam_matrix, dist_matrix, base2cam)

	all_sorted = False
	while not all_sorted:

		cp = get_arucos_pose(cmd, camera, cam_matrix, dist_matrix, base2cam)
		layer_dict, layer_order = get_layers_in_order(cp)
		max_layer = layer_order[0]
		get_cube_angle_from_layer(layer_dict[max_layer])
		'''
		for c in cp:
			for l in range(c.layer):
				curr_fake = Aruco(c.SE3, c.id, real = False)
				layer_dict[l].append(curr_fake)
		'''
		print("Pick up start")

		for layer in layer_order:
			get_cube_angle_from_layer(layer_dict[layer])
			print(layer_dict[layer])
			for cube in layer_dict[layer]:
				if cube.real and cube.angle is not None:
					pick_up(cmd, cube, layer, max_layer)
					des_pos = [*sort_dict[cube.id].SE3.translation, 0, 90, 0]
					des_pos[2] = 150
					ikt_found = move_to_pos(cmd, des_pos)
					if not ikt_found:
						des_irc = find_ikt_no_angle(des_pos[:3])
						cmd.wait_ready()
						cmd.coordmv(des_irc)
						cmd.wait_ready()
					release(cmd)
					pass
			break
		
