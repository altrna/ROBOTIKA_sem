import numpy as np


x = np.linspace(-100, 100, 40)
y = np.linspace(0, 100, 20)
z = np.linspace(-100, 100, 40)

x,y,z = np.meshgrid(x, y, z)
XYZ=np.array([x.flatten(),y.flatten(), z.flatten()]).T
print(XYZ)
