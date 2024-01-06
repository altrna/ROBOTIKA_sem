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
    if boxes is None:
        print("INFO: No visible boxes, terminating program")
        return 
    all_sorted = False
    while not all_sorted:
        arucos = robot.get_arucos_pose()
        print(arucos)

        if len(arucos) == 0:
            print("No visible aruco")
            all_sorted = True
            break

        not_solved_arucos = []
        for cube in arucos:
            if cube.layer > -1:
                if not is_sorted(cube, boxes_dict):
                    not_solved_arucos.append(cube)
        if not_solved_arucos == []:
            print("All sorted")
            all_sorted = True
            break
        arucos = not_solved_arucos        
        layer_dict, layer_order = get_layers_in_order(arucos)
        max_layer = layer_order[0]
        for l in range(len(layer_order)):
            cube_order = get_cube_angle_from_layer(layer_dict[layer_order[l]])
            if all([cube_order[i].angle is None for i in range(len(cube_order))]):
                if l< len(layer_order)-1:
                    layer_dict[layer_order[l+1]].extend(cube_order)
                print(f"INFO: Unsolvable layer: layer {layer_order[l]}.")
            else:
                break
        else:
            print("INFO: Unsolvable task.")
            print("INFO: Terminating program.")
            break

        print("Pick up start")
        for cube in cube_order:
            if cube.angle is not None: #and not is_sorted(cube, boxes_dict):
                if robot.pick_up(cube, max_layer):
                    if cube.id in boxes_dict:
                        robot.place_in_box(boxes_dict[cube.id])
                    else:
                        # cube ID not assigned to any box
                        boxes.sort(key=lambda obj: obj.assigned_amount)
                        boxes[0].append(cube.id)
                        boxes_dict[cube.id] = boxes[0]
                        robot.place_in_box(boxes_dict[cube.id])
                    
                else: # No IKT
                    print("INFO: Solvable, but no IKT.")
                    robot.home()
                    print("INFO: Terminating program.")
                    return
            else:
                continue
    robot.home()

def is_sorted(cube: Aruco, boxes_dict):
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
