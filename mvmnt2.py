import numpy as np
from core.so3 import *
f = [i.flatten() for i in np.load("posr.npz", allow_pickle=True)["arr_0"]]
for i in f:
    print(np.linalg.det(SO3.from_euler_angles(i, 'xyz').rot))
    #print(i, "\n", SO3.from_euler_angles(i, 'xyz').rot)
