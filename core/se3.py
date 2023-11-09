#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#     Homework functions implemented by Aleš Trna

from __future__ import annotations
import numpy as np
from numpy.typing import ArrayLike

import so3


class SE3:
    """Transformation in 2D that is composed of rotation and translation."""

    def __init__(
        self, translation: ArrayLike | None = None, rotation: SO3 | None = None
    ) -> None:
        """Crete an SE3 transformation. Identity is the default."""
        super().__init__()
        self.translation = (
            np.asarray(translation) if translation is not None else np.zeros(3)
        )
        self.rotation = rotation if rotation is not None else SO3()
        assert self.translation.shape == (3,)

    def __mul__(self, other: SE3) -> SE3:
        """Compose two transformation, i.e., self * other"""
        composition = SE3(self.rotation.rot @ other.translation + self.translation, SO3(self.rotation.rot @ other.rotation.rot))
        return composition

    def inverse(self) -> SE3:
        """Compute inverse of the transformation"""
        inverse_translation = (self.rotation.inverse().rot @ self.translation)*(-1)
        inverse = SE3(inverse_translation, self.rotation.inverse())
        return inverse

    def act(self, vector: ArrayLike) -> np.ndarray:
        """Rotate given 3D vector by this transformation."""
        v = np.asarray(vector)
        assert v.shape == (3,)
        v = self.rotation.rot @ v + self.translation
        return v

    def set_from(self, other: SE3):
        """Copy the properties into current instance."""
        self.translation = other.translation
        self.rotation = other.rotation

    def __eq__(self, other: SE3) -> bool:
        """Returns true if two transformations are almost equal."""
        return (
            np.allclose(self.translation, other.translation)
            and self.rotation == other.rotation
        )

    def __hash__(self):
        return id(self)

    def __repr__(self):
        return f"(translation={self.translation}, log_rotation={self.rotation.log()})"
