import numpy as np
from core.so3 import *
from core.se3 import *
from core.aruco import *


def get_distance_from_other_arucos(all_arucos_in_layer: list[Aruco]) -> np.ndarray:
    """
    Retruns matrix of distances from each cube in layer

    Args:
        all_arucos_in_layer (list[Aruco]): List of arucos in layer

    Returns:
        np.ndarray: matrix of L2 norms
    """
    all_centres = np.array([x.SE3.translation[:2] for x in all_arucos_in_layer])
    dist_mat = np.sqrt(np.sum(np.square(all_centres[:, np.newaxis, :] - all_centres[np.newaxis, :, :]), axis=-1))
    return dist_mat


def sort_layers(arucos: list[Aruco]) -> dict:
    """
    Returns dictionary containing layers and corresponding arucos in layer

    Args:
        arucos (list[Aruco]): List of all visible arucos

    Returns:
        dict: Dictionary containing layers as keys and corresponding arucos in layer
    """
    layer_dict = dict()
    for i in range(len(arucos)):
        current_layer = arucos[i].layer
        if current_layer not in layer_dict:
            layer_dict[current_layer] = [arucos[i]]
        else:
            layer_dict[current_layer].append(arucos[i])
    return layer_dict


def get_layers_in_order(all_arucos: list[Aruco]) -> (dict, list):
    """
    Returns dictionary containing layers and corresponding arucos in layer and sorted list of layers

    Args:
        arucos (list[Aruco]): List of all visible arucos

    Returns:
        dict: Dictionary containing layers as keys and corresponding arucos in layer without layer -1 (box)
        kwargs: Sorted list of layers
    """
    layer_dict = sort_layers(all_arucos)
    kwargs = list(layer_dict.keys())
    kwargs.sort(reverse=True)
    if -1 in kwargs:
        kwargs.pop(-1)
    return layer_dict, kwargs


def get_cube_angle_from_layer(all_arucos_in_layer: list[Aruco]) -> None:
    """Assign a grab angle for each visible aruco certain layer

    Args:
        all_arucos_in_layer (list[Aruco]): List of all arucos in layer
    """

    if len(all_arucos_in_layer) == 1:
        angle = np.rad2deg(
            np.arctan2(all_arucos_in_layer[0].SE3.rotation.rot[1, 0], all_arucos_in_layer[0].SE3.rotation.rot[0, 0])
        )
        all_arucos_in_layer[0].grab_angle(angle)
        return

    layer_distance_matrix = get_distance_from_other_arucos(all_arucos_in_layer)
    idx_mask_sum = np.sum(layer_distance_matrix <= 65, axis=0)
    for i in range(len(idx_mask_sum)):
        if idx_mask_sum[i] > 2:
            # Assign grab angle in another iteration
            all_arucos_in_layer[i].grab_angle(None)
            # TODO nemozna reseni
            continue
        closest_aruco = all_arucos_in_layer[np.argsort(layer_distance_matrix[i])[1]]
        current_aruco = all_arucos_in_layer[i]
        # translation from current cube to the closest cube
        translation = (current_aruco.SE3.inverse() * closest_aruco.SE3).translation[:2]
        x_vec = np.array([1, 0])  # grab vector
        t_0 = translation / np.linalg.norm(translation)
        dot_product = np.dot(t_0, x_vec)

        # cube Rz angle
        angle = np.rad2deg(np.arctan2(current_aruco.SE3.rotation.rot[1, 0], current_aruco.SE3.rotation.rot[0, 0]))

        angle += 90 if abs(dot_product) < 0.35 else 0  # 0.35 ~ 70 deg
        all_arucos_in_layer[i].angle = angle

        # print(all_arucos_in_layer[i].angle)
