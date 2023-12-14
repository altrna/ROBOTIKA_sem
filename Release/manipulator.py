import sys
import numpy as np
from core.se3 import *
from core.so3 import *
from core.aruco import *
from core.box import *
import PyCapture2
import cv2

sys.path.append("/bluebot/data/Pyrocon")
from CRS_commander import Commander
from robCRSgripper import robCRSgripper
from robCRSdkt import robCRSdkt
from robCRSikt import robCRSikt
from robotCRS import robCRS97


class Manipulator:
    def __init__(self, homing=True):
        # robot init
        self.robot = robCRS97()
        self.cmd = Commander(self.robot)
        self.cmd.open_comm("/dev/ttyUSB0", speed=19200)

        self.cmd.init(hard_home=False)
        self.cmd.soft_home()
        if homing:
            self.cmd.hard_home()
            self.cmd.soft_home()

        # set camera
        self.bus = PyCapture2.BusManager()
        self.camera = PyCapture2.Camera()
        self.camera.connect(self.bus.getCameraFromIndex(0))
        self.camera.startCapture()

        # set aruco parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # load camera matrix and distortion matrix
        self.camera_matrix = np.load("data/camera_matrix.npz")["arr_0"]
        self.distortion_matrix = np.load("data/distortion_matrix.npz")["arr_0"]

        # load base to camera transformation
        camr = np.load("data/base2cam_R.npz")["arr_0"]
        camt = np.load("data/base2cam_T.npz")["arr_0"]
        self.base2cam = SE3(camt, SO3(camr))

    def home(self) -> None:
        self.cmd.soft_home()

    def get_arucos_pose(self):
        """
        Returns list of objects Aruco containing SE3 and ID

        Returns:
            list[Aruco]: List of all visible arucos
        """
        # Move EOF to capture image
        _, irc = self.cmd.axis_get_pos()
        angles = self.cmd.irctoangles(irc)
        angles[0] = 25
        irc = self.cmd.anglestoirc(angles)
        self.cmd.coordmv(irc)
        self.cmd.wait_ready()

        image = self.camera.retrieveBuffer()
        image = image.convert(PyCapture2.PIXEL_FORMAT.RGB8)

        rgb_cv_image = np.array(image.getData(), dtype="uint8").reshape((image.getRows(), image.getCols(), 3))
        gray_cv_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray_cv_image, 80, 255, 0)

        corners, ids, rejected_im_points = cv2.aruco.detectMarkers(
            thresh, self.aruco_dict, parameters=self.aruco_params
        )
        r_vec, t_vec = cv2.aruco.estimatePoseSingleMarkers(corners, 39.5, self.camera_matrix, self.distortion_matrix)

        if r_vec is None:
            return []
        l = np.shape(r_vec)[0]
        arucos = []
        for i in range(l):
            cam2aruco = SE3(t_vec[i].flatten(), SO3.exp(r_vec[i].flatten()))
            base2aruco = self.base2cam * cam2aruco
            arucos.append(Aruco(base2aruco, ids[i][0]))
        return arucos

    def move_to_pos(self, des_pos):
        """Move effector to desired position

        Args:
            des_pos (list[float]): list of shape (6,) containing cartesian coords and angles Rz, Ry, Rx

        Returns:
            bool: True if motion was executed successfully
        """
        try:
            #print("Desired position", des_pos)
            des_pos = np.array(des_pos)
            des_pos[0:2] += np.array([0, 0])

            irc = self.cmd.find_closest_ikt(des_pos)
        except:
            irc = None
        if irc is not None:
            self.cmd.coordmv(irc)
            self.cmd.wait_ready()
            return True
        else:
            print("ERROR: No IKT!")
            return False

    def release(self):
        """
        Release gripper
        """
        robCRSgripper(self.cmd, -1)
        self.cmd.wait_gripper_ready()

    def grip(self) -> None:
        """
        Grip
        """
        robCRSgripper(self.cmd, 0.1)
        self.cmd.wait_gripper_ready()

    def pick_up(self, cube, max_layer):
        """
        Pick up cube

        Args:
            cube (Aruco): Target cube
        """
        print(f"Picking cube at {cube.SE3.translation[0:2]} in layer {cube.layer}, Z = {cube.SE3.translation[2]}")
        cube.SE3.translation[2] = 50 * (max_layer+ 2.5)
        yaw = cube.angle
        des_pos = [*cube.SE3.translation, yaw, 90, 0]
        if not self.move_to_pos(des_pos):
            return False
        self.release()

        des_pos[2] = 50 * (cube.layer + 1)
        if not self.move_to_pos(des_pos):
            return False

        self.grip()

        des_pos[2] = 50 * (max_layer + 2.5)
        if not self.move_to_pos(des_pos):
            return False
        return True

    def place_in_box(self, box):
        """
        Place cube in box

        Args:
            box (Box): Target box
        """
        print("Place in box")
        des_pos = [*box.SE3.translation, 0, 90, 0]
        ikt_found = self.move_to_pos(des_pos)
        if not ikt_found:
            des_irc = self.find_ikt_no_angle(des_pos[:3])
            self.cmd.coordmv(des_irc)
            self.cmd.wait_ready()
        self.release()

    def target_init(self):
        """Assign cube ID to boxes

        Returns:
            sort_dict (dict): Dictionary containing IDs as keys and corresponding box
            boxes (list[Box]): List of boxes
        """
        # assign cube IDs to SE3 of the box
        sort_dict = {}
        boxes = []
        arucos = self.get_arucos_pose()
        for aruco in arucos:
            if aruco.layer == -1:
                boxes.append(Box(aruco.SE3, aruco.id))
        if boxes == []:
            return None, None
        for aruco in arucos:
            if aruco.layer >= 0 and aruco.id not in sort_dict.keys():
                boxes.sort(key=lambda obj: obj.assigned_amount)
                boxes[0].append(aruco.id)
                sort_dict[aruco.id] = boxes[0]  # box with least assigned cubes
        return sort_dict, boxes

    def find_ikt_no_angle(self, des_pos):
        """
        Find config for position regardless of angle

        Args:
            des_pos (list[float]): Desired position

        Raises:
            Exception: Position is unreachable
        """
        print("Move to regardless of angle")
        z = np.linspace(-70, 70, 40)
        y = np.linspace(20, 90, 20)
        x = np.linspace(-70, 70, 40)
        z, y, x = np.meshgrid(z, y, x)
        all_angles = np.array([z.flatten(), y.flatten(), x.flatten()]).T
        for i in range(np.shape(all_angles)[0]):
            cur_pos = [*des_pos, *all_angles[i]]
            try:
                irc = self.cmd.find_closest_ikt(cur_pos)
                return irc
            except:
                continue
        raise Exception("NO IKT FOUND")


if __name__ == "__main__":
    pass
