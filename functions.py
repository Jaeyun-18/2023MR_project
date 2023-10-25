import numpy as np

def calculate_Shoulder_angle(S1_F, S2_F, E1_F, S1_L, E1_L):
    #Shoulder vector
    vec_x = S1_F[0]-S2_F[0]
    Shoulder_vec = np.array([vec_x, 0.0, 0.0])

    #Upper_arm_vector
    Shoulder_x = S1_F[0]
    Shoulder_y = S1_F[1]
    Shoulder_z = S1_L[0]
    Elbow_x = E1_F[0]
    Elbow_y = E1_F[1]
    Elbow_z = E1_L[0]
    Upper_arm_vec = np.array([Elbow_x-Shoulder_x, Elbow_y-Shoulder_y, Elbow_z-Shoulder_z])

    #첫번째 각도
    Shoulder_angle_1_semi = np.arccos(np.dot(Upper_arm_vec,Shoulder_vec)/(np.linalg.norm(Upper_arm_vec)*np.linalg.norm(Shoulder_vec)))
    Shoulder_angle_1 = (Shoulder_angle_1_semi/np.pi)*180.0-90.0

    #두번째 각도 / 정면으로 뻗은걸 0도로 기준
    base_vec = np.array([0.0, -1.0])
    arm_vec = np.array([Elbow_z, Elbow_y])
    Shoulder_angle_2_semi = np.arccos(np.dot(base_vec,arm_vec)/np.linalg.norm(arm_vec))

    Sign = 1.0

    if Elbow_z > Shoulder_z:
        Sign = +1.0  
    else:   
        Sign = -1.0

    Shoulder_angle_2 = Sign*(Shoulder_angle_2_semi/np.pi)*180.0

    return Shoulder_angle_1, Shoulder_angle_2

def calculate_Elbow_angle(S1_F,S1_L,E1_F,W1_F,H1_F,E1_L,W1_L,H1_L,S2_F,shoulder_length): #아랫팔 벡터용 함수
    p = shoulder_length/(S1_F[0]-S2_F[0])
    #FRONT cam
    E1_F_x = p*(E1_F[0]-S1_F[0]) #왼쪽 팔꿈치 x자표
    E1_F_y = p*(E1_F[1]-S1_F[1]) #왼쪽 팔꿈치 y좌표
    W1_F_x = p*(W1_F[0]-S1_F[0]) #왼쪽 팔목 x좌표
    W1_F_y = p*(W1_F[1]-S1_F[1]) #왼쪽 팔목 y좌표

    #LEFT cam
    q = (S1_F[1]-H1_F[1])/(S1_L[1]-H1_L[1])
    E1_L_z = q*(E1_F[0]-S1_L[0]) #왼쪽 팔꿈치 z좌표
    W1_L_z = q*(W1_L[0]-S1_L[0]) #왼쪽 팔목 z좌표

    X_Sh = p*(S2_F[0]-S1_F[0])
    Y_Sh = 0.0
    Z_Sh = 0.0
    Shoulder_vec = np.array([X_Sh,Y_Sh,Z_Sh])

    X_Up = E1_F_x
    Y_Up = E1_F_y
    Z_Up = E1_L_z
    Upper_arm_vec = np.array([X_Up,Y_Up,Z_Up])

    X_Low = W1_F_x - E1_F_x
    Y_Low = W1_F_y - E1_F_y
    Z_Low = W1_L_z - E1_L_z
    Lower_arm_vec = np.array([X_Low,Y_Low,Z_Low])

    Elbow_angle_1_semi = np.arccos(np.dot(Upper_arm_vec,Lower_arm_vec)/(np.linalg.norm(Upper_arm_vec)*np.linalg.norm(Lower_arm_vec)))
    Elbow_angle_1 = 180.0-(Elbow_angle_1_semi/np.pi)*180.0

    Upper_cross_vec = np.cross(Shoulder_vec,Upper_arm_vec)/(np.linalg.norm(Shoulder_vec)*np.linalg.norm(Upper_arm_vec))
    Lower_cross_vec = np.cross(Upper_arm_vec,Lower_arm_vec)/(np.linalg.norm(Upper_arm_vec)*np.linalg.norm(Lower_arm_vec))

    Elbow_angle_2_semi = np.arccos(np.dot(Upper_cross_vec,Lower_cross_vec)/(np.linalg.norm(Upper_cross_vec)*np.linalg.norm(Lower_cross_vec)))
    Elbow_angle_2 = (Elbow_angle_2_semi/np.pi)*180.0

    return Elbow_angle_1, Elbow_angle_2