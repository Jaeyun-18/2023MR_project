import cv2
from mp_handler import *
import numpy as np

left_landmarks = landmark_translate(True, ["H1", "S1", "E1", "W1"])
front_landmarks = landmark_translate(
    True, ["H1", "S1", "E1", "W1", "H2", "S2", "E2", "W2"])

# left = PoseGetter(0, "left", left_landmarks)
front = PoseGetter(0, "center", front_landmarks)

font_size = 0.8


while front.is_open():
    # left_points, left_img = left.run_cycle(True)
    front_points, front_img = front.run_cycle()
    front.show_vid({"S1": 90, "E1": 100})

    try:
        diff_vector = np.array([[1, -1, 0, 0], [0, 1, -1, 0], [0, 0, 1, -1]])
        """left_vectors = diff_vector @ left_points
        front_left_vectors = diff_vector @ front_points[:4]
        left_vectors = (
            (front_left_vectors[:, 1] / left_vectors[:, 1]) * np.eye(3)) @ left_vectors

        total_left_vectors = np.hstack((front_left_vectors,
                                        left_vectors[:, 0].reshape(-1, 1)))"""
        print("================")

        """L0 = total_left_vectors[:, 0]
        L1 = total_left_vectors[:, 1]
        L2 = total_left_vectors[:, 2]"""

        """CL0 = np.cross(L0, L1)
        CL1 = np.cross(L1, L2)"""

        # print(CL0)
        # print(CL1)

        """angle0 = np.arcsin(
            (np.linalg.norm(CL0) / (np.linalg.norm(L0) * np.linalg.norm(L1)))) / np.pi * 180
        angle1 = np.arcsin(
            (np.linalg.norm(CL1) / (np.linalg.norm(L1) * np.linalg.norm(L2)))) / np.pi * 180

        print(angle0)
        print(angle1)"""

        # front.show_vid()

    except Exception as e:
        # print(e)
        pass

    if cv2.waitKey(5) & 0xFF == 27:
        break
