import numpy as np

cam_matrix = np.array(
    [
        [2.10813831e03, 0.00000000e00, 6.49912597e02],
        [0.00000000e00, 2.11337088e03, 3.45497271e02],
        [0.00000000e00, 0.00000000e00, 1.00000000e00],
    ]
)

dist_matrix = np.array([-6.60263309e-02, -4.23421181e-01, -1.32225502e-02, 1.26508906e-03, 2.59213104e00])

np.savez("camera_matrix.npz", cam_matrix)
np.savez("distortion_matrix.npz", dist_matrix)
