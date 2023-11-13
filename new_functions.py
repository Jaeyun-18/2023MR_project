import numpy as np

#right motor ID(up to down) - even(0,2,4,6)

#left motor ID(up to down) - odd(1,3,5,7)


def cal_LS_0N2(right_shoulder_coor, left_shoulder_coor, left_elbow_coor, left_hip_coor): #leftNright:0 upNdown:2
    Shoulder_vec = left_shoulder_coor - right_shoulder_coor
    Upper_arm_vec = left_elbow_coor - left_shoulder_coor
    Body_vec = left_shoulder_coor - left_hip_coor

    front_vec = np.cross(Shoulder_vec, Body_vec)
    z_vec = np.cross(Shoulder_vec, front_vec)
    motor2_angle = np.arccos(np.dot(Upper_arm_vec,z_vec)/(np.linalg.norm(Upper_arm_vec)*np.linalg.norm(z_vec)))
    re_z_vec = np.cos(motor2_angle)*z_vec/np.linalg.norm(z_vec)
    Shoulder_projection_vec = Shoulder_vec - re_z_vec
    motor0_angle = np.arccos(np.dot(Shoulder_projection_vec,front_vec)/(np.linalg.norm(Shoulder_projection_vec)*np.linalg.norm(front_vec)))
    
    motor0_angle = (motor0_angle/np.pi)*180.0
    motor2_angle = (motor2_angle/np.pi)*180.0

    return motor0_angle, motor2_angle

def cal_LE_46(left_shoulder_coor, left_elbow_coor, left_wrist_coor): #upNdown 4, leftNright 6
    upper_arm_vector = (left_elbow_coor - left_shoulder_coor)/np.linalg.norm(left_elbow_coor-left_shoulder_coor)
    lower_arm_vector = (left_wrist_coor - left_elbow_coor)/np.linalg.norm(left_wrist_coor - left_elbow_coor)
    vector_x = upper_arm_vector
    vector_z = np.cross(lower_arm_vector, vector_x)
    vector_y = np.cross(vector_z,vector_x)
    
    theta = (np.arccos((np.dot(vector_x, lower_arm_vector)/(np.norm(vector_x)*np.norm(lower_arm_vector))))/np.pi)*180 #upNdown angle
    
    length = np.norm(lower_arm_vector)*np.cos(theta)
    
    diff_vector = length*(upper_arm_vector/np.norm(upper_arm_vector))
    
    proj_vector = lower_arm_vector - diff_vector
    
    phi = (np.arccos(np.dot(proj_vector,vector_z)/(np.norm(proj_vector)*np.norm(vector_z)))/np.pi)*180 #leftNright angle
    
    return theta, phi

def cal_RS_1N3(right_shoulder_coor, left_shoulder_coor, right_elbow_coor, right_hip_coor): #leftNright:1 upNdown:3
    Shoulder_vec = right_shoulder_coor - left_shoulder_coor
    Upper_arm_vec = right_elbow_coor - right_shoulder_coor
    Body_vec = right_hip_coor - right_shoulder_coor

    front_vec = np.cross(Shoulder_vec, Body_vec)
    z_vec = np.cross(Shoulder_vec, front_vec)
    motor3_anlge = np.arccos(np.dot(Upper_arm_vec, z_vec)/(np.linalg.norm(Upper_arm_vec)*np.linalg.norm(z_vec)))
    re_z_vec = np.cos(motor3_anlge)*z_vec/np.linalg.norm(z_vec)
    Shoulder_projection_vec = Shoulder_vec - re_z_vec
    motor1_angle = np.arccos(np.dot(Shoulder_projection_vec,front_vec)/(np.linalg.norm(Shoulder_projection_vec)*np.linalg.norm(front_vec)))
    
    motor1_angle = (motor1_angle/np.pi)*180.0
    motor3_angle = (motor3_anlge/np.pi)*180.0
    
    return motor1_angle, motor3_anlge

def cal_RE_57(right_shoulder_coor, right_elbow_coor, right_wrist_coor): #upNdown 5, leftNright 7
    upper_arm_vector = (right_elbow_coor - right_shoulder_coor)/np.linalg.norm(right_elbow_coor-right_shoulder_coor)
    lower_arm_vector = (right_wrist_coor - right_elbow_coor)/np.linalg.norm(right_wrist_coor - right_elbow_coor)
    vector_y = upper_arm_vector
    vector_z = np.cross(lower_arm_vector, vector_x)
    vector_x = np.cross(vector_y,vector_z)
    
    theta = (np.arccos((np.dot(vector_y, lower_arm_vector)/(np.norm(vector_y)*np.norm(lower_arm_vector))))/np.pi)*180 #upNdown angle
    
    length = np.norm(lower_arm_vector)*np.cos(theta)
    
    diff_vector = length*(upper_arm_vector/np.norm(upper_arm_vector))
    
    proj_vector = lower_arm_vector - diff_vector
    
    phi = (np.arccos(np.dot(proj_vector,vector_z)/(np.norm(proj_vector)*np.norm(vector_z)))/np.pi)*180 #leftNright angle
    
    return theta, phi