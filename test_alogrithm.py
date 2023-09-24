import cv2
from mp_handler import *
import numpy as np

left_landmarks = landmark_translate(True, ["H1", "S1", "E1", "W1"])
front_landmarks = landmark_translate(
    True, ["H1", "S1", "E1", "W1", "H2", "S2", "E2", "W2"])

left = PoseGetter(0, "left", left_landmarks)
front = PoseGetter(
    4, "center", front_landmarks)


while left.is_open():
    left_points, left_img = left.run_cycle(True)
    front_points, front_img = front.run_cycle(True)

    cv2.imshow('center image', cv2.flip(left_img, 1))
    cv2.imshow('left image', cv2.flip(front_img, 1))

    try:
        diff_vector = np.array([[1, -1, 0, 0], [0, 1, -1, 0], [0, 0, 1, -1]])
        left_vectors = diff_vector @ left_points
        front_left_vectors = diff_vector @ front_points[:4]
        left_vectors = (
            (front_left_vectors[:, 1] / left_vectors[:, 1]) * np.eye(3)) @ left_vectors

        total_left_vectors = np.hstack((front_left_vectors,
                                        left_vectors[:, 0].reshape(-1, 1)))
        print("================")

        L0 = total_left_vectors[:, 0]
        L1 = total_left_vectors[:, 1]
        L2 = total_left_vectors[:, 2]

        CL0 = np.cross(L0, L1)
        CL1 = np.cross(L1, L2)

        # print(CL0)
        # print(CL1)

        print(np.arcsin((np.linalg.norm(CL0) /
              (np.linalg.norm(L0) * np.linalg.norm(L1)))) / np.pi * 180)
        print(np.arcsin((np.linalg.norm(CL1) /
              (np.linalg.norm(L1) * np.linalg.norm(L2)))) / np.pi * 180)

    except Exception as e:
        # print(e)
        pass

    if cv2.waitKey(5) & 0xFF == 27:
        break
