import numpy as np

def calculate_S1_side(S1_L, E1_L): #좌측면, 원
    S1_L = np.array(S1_L) #left shoulder / sidecame
    E1_L = np.array(E1_L) #left elbow / sidecame

    radian = np.arctan(np.abs(S1_L[1]-E1_L[1]) / np.abs(S1_L[0]-E1_L[0]))
    angle_L = np.abs(radian)*180.0/np.pi
    
    if E1_L[0] > S1_L[0]:
        return angle_L - 90.0
    else:   
        return 90.0 - angle_L
    
def calculate_S1_front(H1_F, S1_F, E1_F, basic_length): #정면, 좌측 반원
    H1_F = np.array(H1_F) # First
    S1_F = np.array(S1_F) # Mid
    E1_F = np.array(E1_F) # End

    x_arm_1 = np.abs(E1_F[0]-S1_F[0]) # x축에 내린 윗팔의 정사영 길이
    angle_front = 90.0 - np.abs(np.arccos(x_arm_1/basic_length)*180.0/np.pi)
    
    y_S1 = S1_F[1]
    UP = 0 #위, 아래 판단 bool 변수

    if E1_F[1] < y_S1:
        UP = 1
    elif E1_F[1] >= y_S1:
        UP = 0
        
    return angle_front, UP

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