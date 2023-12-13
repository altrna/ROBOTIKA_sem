import sys
import numpy as np

from core.se3 import *
from core.so3 import *
from core.box import *
from core.aruco import *
from manipulator import *
from aruco_utilities import *


def main():
    # TODO když nemá řešení break cyklu
    # def has_solution ...
    robot = Manipulator(homing=False)
    boxes_dict, boxes = robot.target_init()

    all_sorted = False
    while not all_sorted:
        arucos = robot.get_arucos_pose()

        if len(arucos) == 0:
            print("No visible aruco")
            all_sorted = True
            break

        for cube in arucos:
            if cube.layer > -1:
                if not is_sorted(cube, boxes_dict):
                    break
        else:
            all_sorted = True
            break

        layer_dict, layer_order = get_layers_in_order(arucos)
        get_cube_angle_from_layer(layer_dict[layer_order[0]])

        print("Pick up start")
        layer = layer_order[0]
        print(layer_dict[layer])
        for cube in layer_dict[layer]:
            if cube.angle is not None and not is_sorted(cube, boxes_dict):
                robot.pick_up(cube)
                if cube.id in boxes_dict:
                    robot.place_in_box(boxes_dict[cube])
                else:
                    # cube ID not assigned to any box
                    boxes.sort(key=lambda obj: obj.assigned_amount)
                    boxes[0].append(cube.id)
                    boxes_dict[cube.id] = boxes[0]
                    robot.place_in_box(boxes_dict[cube])
            else:
                continue


def is_sorted(cube: Aruco, boxes_dict: dict):
    """
    Args:
        cube (Aruco): Cube
        boxes_dict (dict): Boxes dictionary

    Returns:
        bool: True if cube is located in certain radius from box
    """
    max_radius = 100
    if cube.id not in boxes_dict:
        return False

    box = boxes_dict[cube.id]
    if np.linalg.norm(box.SE3.translation[0:2] - cube.SE3.translation[0:2]) < max_radius:
        return True
    else:
        return False


if __name__ == "__main__":
    main()
