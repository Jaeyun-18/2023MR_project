import cv2
from mp_handler import *
import numpy as np

# test algorithm for left-front-right 90 degree camera arrangement


def test_algorithm1(side_points: np.ndarray, front_side_points: np.ndarray):
    diff_vector = np.array([[1, -1, 0, 0], [0, 1, -1, 0], [0, 0, 1, -1]])
    side_vectors = diff_vector @ side_points
    front_vectors = diff_vector @ front_side_points

    side_vectors = (
        front_vectors[:, 1:] / side_vectors[:, 1:]) * side_vectors
    return np.hstack(front_vectors, side_vectors[:, 0:1])


left_landmarks = landmark_translate(True, ["H1", "S1", "E1", "W1"])
front_landmarks = landmark_translate(
    True, ["H1", "S1", "E1", "W1", "H2", "S2", "E2", "W2"])

# left = PoseGetter(0, "left", left_landmarks)
front = PoseGetter(0, "center", front_landmarks)

font_size = 0.8


while front.is_open():
    # left_points, left_img = left.run_cycle(True)
    front_points, front_img = front.run_cycle()

    try:
        # t1 = test_algorithm1(left_points, front_points)
        # L0 = t1[:, 0]
        # L1 = t1[:, 1]
        # L2 = t1[:, 2]
        # CL0 = np.cross(L0, L1)
        # CL1 = np.cross(L1, L2)

        # print(CL0)
        # print(CL1)

        print("================")

        """angle0 = np.arcsin(
            (np.linalg.norm(CL0) / (np.linalg.norm(L0) * np.linalg.norm(L1)))) / np.pi * 180
        angle1 = np.arcsin(
            (np.linalg.norm(CL1) / (np.linalg.norm(L1) * np.linalg.norm(L2)))) / np.pi * 180

        print(angle0)
        print(angle1)"""

        front.show_vid({"S1": 90, "E1": 100})

    except Exception as e:
        # print(e)
        pass

    if cv2.waitKey(5) & 0xFF == 27:
        break
