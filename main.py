import cv2
from mp_handler import *
from functions import *
import numpy as np

front_landmarks = landmark_translate(
    True, ["S1", "S2", "E1", "E2", "W1", "W2", "H1", "H2"])
left_landmarks = landmark_translate(True, ["S1", "S2", "E1", "W1", "H1"])
#right_landmarks = landmark_translate(True, ["S1", "S2", "E2", "W2", "H2"])

left = PoseGetter(1, "left", left_landmarks, [200,200])
front = PoseGetter(0, "front", front_landmarks, [200,200])
#right = PoseGetter(2, "right", right_landmarks)
shoulder_length = float(input('INPUT SHOULDER_LENGTH : '))

while front.is_open():
    try:
        left_points, left_img = left.run_cycle()
        front_points, front_img = front.run_cycle()
        #right_points, right_img = right.run_cycle()

        # marking
        S1_F = front_points[0]
        S2_F = front_points[1]
        E1_F = front_points[2]
        E2_F = front_points[3]
        W1_F = front_points[4]
        W2_F = front_points[5]
        H1_F = front_points[6]
        H2_F = front_points[7]

        S1_L = left_points[0]
        S2_L = left_points[1]
        E1_L = left_points[2]
        W1_L = left_points[3]
        H1_L = left_points[4]

        #S1_R = left_points[0]
        #S2_R = left_points[1]
        #E2_R = left_points[2]
        #W2_R = left_points[3]
        #H2_R = left_points[4]

        Shoudler_angle_1, Shoulder_angle_2 = calculate_Left_Shoulder_angle(S1_F, S2_F, E1_F, S1_L, E1_L, S2_L)
        shoulder_length = abs(S1_F[0]-S2_F[0])
        Elbow_angle_1, Elbow_angle_2 = calculate_Left_Elbow_angle(S1_F,S1_L,E1_F,W1_F,H1_F,E1_L,W1_L,H1_L,S2_F,shoulder_length)
        
        left.show_vid(None)
        front.show_vid(None)
        #right.show_vid(None)
    
    except:
        pass
    if cv2.waitKey(5) & 0xFF == 27:
        break
