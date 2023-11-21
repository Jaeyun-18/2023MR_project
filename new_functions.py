import numpy as np

# right motor ID(up to down) - even(0,2,4,6)

# left motor ID(up to down) - odd(1,3,5,7)

def rad2six(radian_angle):
    six_angle = (radian_angle / np.pi) * 180
    
    return six_angle
    

# def cal_LS_0N2(right_shoulder_coor, left_shoulder_coor, left_elbow_coor, left_hip_coor):  # leftNright:0 upNdown:2
#     Shoulder_vec = left_shoulder_coor - right_shoulder_coor
#     Upper_arm_vec = left_elbow_coor - left_shoulder_coor
#     Body_vec = left_shoulder_coor - left_hip_coor
    
#     vector_z = np.cross(Shoulder_vec, Body_vec)

#     uz = np.dot(vector_z, Upper_arm_vec) / np.linalg.norm(vector_z)
#     ub = np.dot(Body_vec, Upper_arm_vec) / np.linalg.norm(Body_vec)
#     us = np.dot(Shoulder_vec, Upper_arm_vec) / np.linalg.norm(Shoulder_vec)
    
#     Theta_L = np.arctan2(uz, us)
#     Phi_L = np.arccos(ub / np.linalg.norm(Body_vec))

#     Theta_L = rad2six(90 - Theta_L)  # frontNback angle
#     Phi_L = rad2six(Phi_L)  # upNdown angle
    
#     return motor0_angle, Phi_L


def cal_LS_02(left_shoulder_coor, left_elbow_coor, left_hip_coor, right_shoulder_coor):  # leftNright:0 upNdown:2
    
    Shoulder_vec = left_shoulder_coor - right_shoulder_coor
    Body_vec = left_shoulder_coor - left_hip_coor
    Front_vec = np.cross(Shoulder_vec, Body_vec)
    
    upper_arm_vec = left_elbow_coor - left_shoulder_coor
    
    vector_z = np.cross(Shoulder_vec, Front_vec)*(-1)
    
    Phi_L = np.arccos(np.dot(upper_arm_vec, vector_z)/(np.linalg.norm(upper_arm_vec)*np.linalg.norm(vector_z))) # upNdown angle

    if Phi_L > np.pi/2:
        proj_z_vec = vector_z / \
            np.linalg.norm(vector_z)*np.linalg.norm(upper_arm_vec) * \
            np.sin(Phi_L - 90.0)
        upper_arm_projection_vec = upper_arm_vec + proj_z_vec
    else:
        proj_z_vec = vector_z / \
            np.linalg.norm(vector_z)*np.linalg.norm(upper_arm_vec) * \
            np.cos(Phi_L)
        upper_arm_projection_vec = upper_arm_vec - proj_z_vec

    Theta_L = np.arccos(np.dot(upper_arm_projection_vec, Shoulder_vec)/(
        np.linalg.norm(upper_arm_projection_vec)*np.linalg.norm(Shoulder_vec))) # frontNback angle
    
    check_angle = np.arccos(np.dot(upper_arm_projection_vec, Front_vec)/(np.linalg.norm(upper_arm_projection_vec)*np.linalg.norm(Front_vec)))
    if check_angle > np.pi/2:
        Theta_L *= (-1)

    Theta_L = rad2six(Theta_L)
    Phi_L = rad2six(Phi_L)
    
    return Theta_L, Phi_L

def cal_RS_13(left_shoulder_coor, right_hip_coor, right_shoulder_coor, right_elbow_coor):  # leftNright:1 upNdown:3
    
    Shoulder_vec = right_shoulder_coor - left_shoulder_coor #left to right shoulder vector(x-axis)
    Body_vec = right_hip_coor - right_shoulder_coor #right hip to right shoulder body vector
    Front_vec = np.cross(Body_vec, Shoulder_vec) #back to front(y-axis)
    
    upper_arm_vector = (right_elbow_coor - right_shoulder_coor)
    
    vector_z = np.cross(Shoulder_vec, Front_vec)
    
    Phi_R = np.arccos(np.dot(upper_arm_vector, vector_z) / \
        (np.linalg.norm(upper_arm_vector)*np.linalg.norm(vector_z))) #upNdown angle
    
    if Phi_R > np.pi/2:
        proj_z_vec = vector_z / \
            np.linalg.norm(vector_z)*np.linalg.norm(upper_arm_vector) * \
            np.sin(Phi_R - 90.0)
        upper_arm_projection_vec = upper_arm_vector + proj_z_vec
    else:
        proj_z_vec = vector_z / \
            np.linalg.norm(vector_z)*np.linalg.norm(upper_arm_vector) * \
            np.cos(Phi_R)
        upper_arm_projection_vec = upper_arm_vector - proj_z_vec
    
    Theta_R = np.arccos(np.dot(upper_arm_projection_vec, Shoulder_vec)/(
        np.linalg.norm(upper_arm_projection_vec)*np.linalg.norm(Shoulder_vec))) #frontNback angle

    check_angle = np.arccos(np.dot(upper_arm_projection_vec, Front_vec)/(np.linalg.norm(upper_arm_projection_vec)*np.linalg.norm(Front_vec)))
    if check_angle > np.pi/2:
        Theta_R *= (-1)

    Theta_R = rad2six(Theta_R)
    Phi_R = rad2six(Phi_R)

    return Theta_R, Phi_R


def cal_LE_46(left_shoulder_coor,  left_elbow_coor, left_wrist_coor, right_shoulder_coor):  # upNdown 4, leftNright 6

    upper_arm_vector = (left_elbow_coor - left_shoulder_coor) / \
        np.linalg.norm(left_elbow_coor-left_shoulder_coor)
    lower_arm_vector = (left_wrist_coor - left_elbow_coor) / \
        np.linalg.norm(left_wrist_coor - left_elbow_coor)
    Shoulder_vec = left_shoulder_coor - right_shoulder_coor

    Theta_L = rad2six(np.arccos(np.dot(upper_arm_vector, lower_arm_vector))) # upNdown angle
        
    Shoulder_cross_vec = np.cross(upper_arm_vector, Shoulder_vec) / \
        (np.linalg.norm(Shoulder_vec)*np.linalg.norm(upper_arm_vector))
    Elbow_cross_vec = np.cross(upper_arm_vector, lower_arm_vector) / \
        (np.linalg.norm(upper_arm_vector)*np.linalg.norm(lower_arm_vector))
        
    Phi_L = rad2six((np.arccos(np.dot(Shoulder_cross_vec, Elbow_cross_vec) / \
        (np.linalg.norm(Shoulder_cross_vec)*np.linalg.norm(Elbow_cross_vec))))) + (np.pi/2) # leftNright angle

    return Theta_L, Phi_L


def cal_RE_57(left_shoulder_coor, right_hip_coor, right_shoulder_coor, right_elbow_coor, right_wrist_coor):  # upNdown 5, leftNright 7

    upper_arm_vector = (right_elbow_coor - left_shoulder_coor) / \
        np.linalg.norm(right_elbow_coor-left_shoulder_coor)
    lower_arm_vector = (right_wrist_coor - right_elbow_coor) / \
        np.linalg.norm(right_wrist_coor - right_elbow_coor)

    vector_z = np.cross(upper_arm_vector, lower_arm_vector) #down to up(z-axis)

    Shoulder_vec = right_shoulder_coor - left_shoulder_coor #left to right shoulder vector(x-axis)
    Body_vec = right_shoulder_coor - right_hip_coor #right hip to right shoulder body vector
    Front_vec = np.cross(Body_vec, Shoulder_vec) #back to front(y-axis)

    Shoulder_cross_vec = np.cross(Shoulder_vec,upper_arm_vector) / \
        (np.linalg.norm(Shoulder_vec)*np.linalg.norm(upper_arm_vector))
    Elbow_cross_vec = np.cross(lower_arm_vector, upper_arm_vector) / \
        (np.linalg.norm(lower_arm_vector)*np.linalg.norm(upper_arm_vector))
        
    Phi_R = rad2six((np.arccos(np.dot(Shoulder_cross_vec, Elbow_cross_vec) / \
        (np.linalg.norm(Shoulder_cross_vec)*np.linalg.norm(Elbow_cross_vec))))) + (np.pi/2) # leftNright angle
    
    return Theta_R, Phi_R

    return Theta_R, Phi_R
