import cv2
from mp_handler import *
import numpy as np
from typing import Tuple

from stereo_camera import StereoCameraSystem
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from time import time
from graph_plot import plot_3d
from new_functions import *
import threading

landmarks = landmark_translate(
    True, ["W1", "E1", "S1", "H1", "H2", "S2", "E2", "W2"])
          # 0     1     2     3     4     5     6     7

right = PoseGetter(2, "right", landmarks, [640, 480])
left = PoseGetter(4, "left", landmarks, [640, 480])

font_size = 0.8

TestCamSys = StereoCameraSystem("right_camera", "left_camera", "cali_imgs/right_imgs",
                                "cali_imgs/left_imgs", "cali_imgs/sync_imgs", [7, 9])
TestCamSys.calibrate(True, "test_mtx.npz")

wcs = []
times = []
t0 = time.time()
ms = []

print(left.is_open())
print(right.is_open())

while left.is_open() and right.is_open():
    try:
        left_points, left_img = left.run_cycle()
        right_points, right_img = right.run_cycle()
        world_coord = TestCamSys.triangulate(right_points, left_points)

        goal_angle = cal_angle(world_coord)
        print(goal_angle)
        
        right.show_vid(None)
        left.show_vid(None)

    except Exception as e:
        print(e)

    if cv2.waitKey(5) == ord('q'):
        break

plot_3d(wcs, times)

ms = np.array(ms)
ax = plt.plot(ms[:, 0])
ax = plt.plot(ms[:, 1])
ax = plt.plot(ms[:, 2])
ax = plt.plot(ms[:, 3])
plt.show()
