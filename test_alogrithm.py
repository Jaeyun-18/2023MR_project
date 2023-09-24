import cv2
from mp_handler import *
import numpy as np

left_landmarks = landmark_translate(True, ["H1", "S1", "E1", "W1"])
front_landmarks = landmark_translate(
    True, ["H1", "S1", "E1", "W1", "H2", "S2", "E2", "W2"])

left = PoseGetter(4, "left", left_landmarks)
front = PoseGetter(
    0, "center", front_landmarks)


while left.is_open():
    left_points, left_img = left.run_cycle(True)
    front_points, front_img = front.run_cycle(True)

    cv2.imshow('center image', cv2.flip(left_img, 1))
    cv2.imshow('left image', cv2.flip(front_img, 1))
    diff_vector = np.array([[1, -1, 0, 0], [0, 1, -1, 0], [0, 0, 1, -1]])
    try:
        left_vectors = diff_vector @ left_points
        front_left_vectors = diff_vector @ front_points[:4]
        left_vectors = front_left_vectors[:, 1] / left_vectors[:, 1]

        print("================")
    except:
        pass
    if cv2.waitKey(5) & 0xFF == 27:
        break
