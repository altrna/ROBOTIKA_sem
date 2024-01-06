import numpy as np
from core.so3 import *
from core.se3 import *
from core.aruco import *


def get_distance_from_other_arucos(all_arucos_in_layer):
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


def sort_layers(arucos):
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


def get_layers_in_order(all_arucos):
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


def get_cube_angle_from_layer(all_arucos_in_layer):
    """Assign a grab angle for each visible aruco certain layer

    Args:
        all_arucos_in_layer (list[Aruco]): List of all arucos in layer
    Returns:
        List[Aruco]: List of cubes sorted by distances from other cubes
    """

    if len(all_arucos_in_layer) == 1:
        angle = np.rad2deg(
            np.arctan2(all_arucos_in_layer[0].SE3.rotation.rot[1, 0], all_arucos_in_layer[0].SE3.rotation.rot[0, 0])
        )
        all_arucos_in_layer[0].angle = angle
        return all_arucos_in_layer

    layer_distance_matrix = get_distance_from_other_arucos(all_arucos_in_layer)
    idx_mask_sum = np.sum(layer_distance_matrix <= 70, axis=0)
    print(layer_distance_matrix)
    for i in range(len(idx_mask_sum)):  
        current_aruco = all_arucos_in_layer[i]
        angle = np.rad2deg(np.arctan2(current_aruco.SE3.rotation.rot[1, 0], current_aruco.SE3.rotation.rot[0, 0]))
        orientation = []
        for j in range(0, len(idx_mask_sum)):
            if layer_distance_matrix[i][j] <=70:
                if i == j:
                    continue
                closest_aruco = all_arucos_in_layer[j]
                translation = (current_aruco.SE3.inverse() * closest_aruco.SE3).translation[:2]
                x_vec = np.array([1, 0])  # grab vector
                t_0 = translation / np.linalg.norm(translation)
                dot_product = np.dot(t_0, x_vec)
                orientation.append(abs(dot_product) < 0.68)
                if abs(abs(dot_product) - 0.705) < 0.15:
                    angle = None
                    break
        else:
            if all(orientation):
                angle += 90
            elif all([not i for i in orientation]):
                pass
            else:
                angle = None
        all_arucos_in_layer[i].angle = angle

    layer_distance_vector = np.sum(layer_distance_matrix , axis = 0)
    cube_order_distance = np.argsort(layer_distance_vector)
    cube_order_distance = np.flip(cube_order_distance, axis = 0)
    sorted_cubes = [all_arucos_in_layer[i] for i in cube_order_distance]
    return sorted_cubes
 




