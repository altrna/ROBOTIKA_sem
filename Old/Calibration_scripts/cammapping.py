from core.so3 import SO3
from core.se3 import SE3
import numpy as np
import matplotlib.pyplot as plt
import cv2

n = 8
uv_c = np.random.uniform(-100, 100, size=(n,2))
T_RC = SE3(translation=[520, -75, 1250], rotation=SO3.ry(np.pi))
T_CR = T_RC.inverse()
fig : plt.Figure = plt.figure()
ax_cam : plt.Axes = fig.add_subplot(121)
ax_cam.plot(uv_c.T, ms=10, color="black")
K = np.array([[-1/240, 0], [0, 1/240], [0, 0]])
