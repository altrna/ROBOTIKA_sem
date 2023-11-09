#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#     Homework functions implemented by Aleš Trna

from __future__ import annotations
import numpy as np
from numpy.typing import ArrayLike

def rodrigues_formula(angle, v):
    skew_symetric_matrix = np.array([[0,-v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    t = SO3()
    t.rot = np.eye(3) + np.sin(angle)*skew_symetric_matrix + (1-np.cos(angle))*skew_symetric_matrix @ skew_symetric_matrix
    return t

class SO3:
    """This class represents an SO3 rotations internally represented by rotation
    matrix."""

    def __init__(self, rotation_matrix: ArrayLike | None = None) -> None:
        """Creates a rotation transformation from rot_vector."""
        super().__init__()
        self.rot: np.ndarray = (
            np.asarray(rotation_matrix) if rotation_matrix is not None else np.eye(3)
        )

    @staticmethod
    def exp(rot_vector: ArrayLike) -> SO3:
        """Compute SO3 transformation from a given rotation vector, i.e. exponential
        representation of the rotation."""
        v = np.asarray(rot_vector)
        print("shape exp: ", np.shape(v))
        assert v.shape == (3,)
        angle = np.linalg.norm(v)
        v_ = v/angle if angle != 0 else v
        skew_symetric_matrix = np.array([[0,-v_[2], v_[1]], [v_[2], 0, -v_[0]], [-v_[1], v_[0], 0]])
        t = SO3()
        t.rot = np.eye(3) + np.sin(angle)*skew_symetric_matrix + (1-np.cos(angle))*skew_symetric_matrix @ skew_symetric_matrix
        return t

    def log(self) -> np.ndarray:
        """Compute rotation vector from this SO3"""
        v = np.zeros(3)
        angle_arg = 0.5 * (np.trace(self.rot)-1)       
        if angle_arg >= -1 and angle_arg <= 1:
            angle = np.arccos(angle_arg)
            vector_constant = 2*np.sin(angle)
            if vector_constant != 0:
                mat = 1/vector_constant*(self.rot - np.transpose(self.rot)) if np.sin(angle) != 0 else np.zeros((3,3)) 
                v = np.array([mat[2,1], mat[0,2], mat[1,0]])*angle
        return v

    def __mul__(self, other: SO3) -> SO3:
        """Compose two rotations, i.e., self * other"""
        new_rot = self.rot @ other.rot
        return SO3(new_rot)

    def inverse(self) -> SO3:
        """Return inverse of the transformation."""
        return SO3(self.rot.T)

    def act(self, vector: ArrayLike) -> np.ndarray:
        """Rotate given vector by this transformation."""
        v = np.asarray(vector)
        assert v.shape == (3,)
        return self.rot @ v

    def __eq__(self, other: SO3) -> bool:
        """Returns true if two transformations are almost equal."""
        return np.allclose(self.rot, other.rot)

    @staticmethod
    def rx(angle: float) -> SO3:
        """Return rotation matrix around x axis."""
        sine = np.sin(angle)
        cosine = np.cos(angle)
        return SO3(np.array([[1, 0, 0], [0, cosine, -sine], [0, sine, cosine]]))

    @staticmethod
    def ry(angle: float) -> SO3:
        """Return rotation matrix around y axis."""
        sine = np.sin(angle)
        cosine = np.cos(angle)
        return SO3(np.array([[cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]]))

    @staticmethod
    def rz(angle: float) -> SO3:
        """Return rotation matrix around z axis."""
        sine = np.sin(angle)
        cosine = np.cos(angle)
        return SO3(np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]]))

    @staticmethod
    def from_quaternion(q: ArrayLike) -> SO3:
        """Compute rotation from quaternion in a form [qx, qy, qz, qw].""" 
        new_SO3_obj = SO3(SO3().exp(2*np.arccos(q[3])*(q[0:3]/np.linalg.norm(q[0:3]))).rot)
        return new_SO3_obj


    def to_quaternion(self) -> np.ndarray:
        """Compute quaternion from self."""
        # todo: HW1opt: implement to quaternion
        #raise NotImplementedError("To quaternion needs to be implemented")
        angle, axis = self.to_angle_axis()
        qw = np.array([np.cos(angle/2)])
        qxyz = axis*np.sin(angle/2)
        print(qxyz, qw)
        return np.concatenate((qxyz, qw), axis=0)

    @staticmethod
    def from_angle_axis(angle: float, axis: ArrayLike) -> SO3:
        """Compute rotation from angle axis representation."""
        # todo: HW1opt: implement from angle axis
        return rodrigues_formula(angle,axis)
        

    def to_angle_axis(self) -> tuple[float, np.ndarray]:
        """Compute angle axis representation from self."""
        v = np.zeros(3)
        angle_arg = 0.5 * (np.trace(self.rot)-1)       
        if angle_arg >= -1 and angle_arg <= 1:
            angle = np.arccos(angle_arg)
            vector_constant = 2*np.sin(angle)
            if vector_constant != 0:
                mat = 1/vector_constant*(self.rot - np.transpose(self.rot)) if np.sin(angle) != 0 else np.zeros((3,3)) 
                v = np.array([mat[2,1], mat[0,2], mat[1,0]])
        return (angle, v)
        

    @staticmethod
    def from_euler_angles(angles: ArrayLike, seq: list[str]) -> SO3:
        """Compute rotation from euler angles defined by a given sequence.
        angles: is a three-dimensional array of angles
        seq: is a list of axis around which angles rotate, e.g. 'xyz', 'xzx', etc.
        """
        new_SO3_obj = SO3()
        for x in range(len(seq)):
            if seq[x] == "x":
                new_SO3_obj.rot = new_SO3_obj.rot @ SO3.rx(angles[x]).rot
            elif seq[x] == "y":
                new_SO3_obj.rot = new_SO3_obj.rot @ SO3.ry(angles[x]).rot
            elif seq[x] == "z":
                new_SO3_obj.rot = new_SO3_obj.rot @ SO3.rz(angles[x]).rot
        return new_SO3_obj

    def __hash__(self):
        return id(self)
