import cv2
from mp_handler import *
import numpy as np
from typing import Tuple

# test algorithm for left-front-right 90 degree camera arrangement


def test_algorithm1(side_points: np.ndarray, front_side_points: np.ndarray) -> np.ndarray:
    diff_matrix = np.array([[1, -1, 0, 0], [0, 1, -1, 0], [0, 0, 1, -1]])
    side_vectors = diff_matrix @ side_points
    front_vectors = diff_matrix @ front_side_points

    side_vectors = (
        front_vectors[:, 1:] / side_vectors[:, 1:]) * side_vectors

    return np.hstack((front_vectors, side_vectors[:, 0:1]))


# test algorithm for left-right 90 degree(45 each) camera arrangement


def test_algorithm2(left_points: np.ndarray, right_points: np.ndarray) -> np.ndarray:
    diff_matrix = np.array([[1, -1, 0, 0], [0, 1, -1, 0], [0, 0, 1, -1]])
    left_vectors = diff_matrix @ left_points
    right_vectors = diff_matrix @ right_points
    right_vectors = (
        left_vectors[:, 1:] / right_vectors[:, 1:]) * right_vectors
    dr = right_vectors[:, 0:1]
    dl = left_vectors[:, 0:1]
    x_vectors = (dl - dr) / np.sqrt(2)
    z_vectors = (dl + dr) / np.sqrt(2)
    y_vectors = right_vectors[:, 1:2]
    return np.hstack((x_vectors, y_vectors, z_vectors))


def vectors_to_angles(final_vector: np.ndarray) -> Tuple[float, float]:
    L0 = final_vector[:, 0]
    L1 = final_vector[:, 1]
    L2 = final_vector[:, 2]
    CL0 = np.cross(L0, L1)
    CL1 = np.cross(L1, L2)
    angle0 = np.arcsin(
        (np.linalg.norm(CL0) / (np.linalg.norm(L0) * np.linalg.norm(L1)))) / np.pi * 180
    angle1 = np.arcsin(
        (np.linalg.norm(CL1) / (np.linalg.norm(L1) * np.linalg.norm(L2)))) / np.pi * 180
    return angle0, angle1


left_landmarks = landmark_translate(True, ["H1", "S1", "E1", "W1"])
front_landmarks = landmark_translate(
    True, ["H1", "S1", "E1", "W1", "H2", "S2", "E2", "W2"])

left = PoseGetter(4, "left", left_landmarks, [640, 480])
front = PoseGetter(6, "center", front_landmarks, [640, 480])

font_size = 0.8


while front.is_open():
    try:
        left_points, left_img = left.run_cycle()
        front_points, front_img = front.run_cycle()

        # f1 = test_algorithm1(left_points, front_points[:4])
        # f2 = test_algorithm2(left_points, front_points[:4])

        # angle0, angle1 = vectors_to_angles(f1)
        # angle0, angle1 = vectors_to_angles(f2)

        # print("f1", angle0, angle1)
        # print("f2", angle0, angle1)

        # front.show_vid({"S1": angle0, "E1": angle1})
        # left.show_vid({"S1": angle0, "E1": angle1})
        left.show_vid(None)
        front.show_vid(None)
        print(front_points[6])

    except Exception as e:
        print(e)
        pass

    if cv2.waitKey(5) & 0xFF == 27:
        break
