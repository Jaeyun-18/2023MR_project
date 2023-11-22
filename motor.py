import os
import math

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address for Dynamixel MX
TORQUE_ENABLE_AX       = 24               # Control table address is different in Dynamixel model
GOAL_POSITION_AX       = 30
PRESENT_POSITION       = 36

# Protocol version
PROTOCOL_VERSION           = 1.0               #  See which protocol version is used in the Dynamixel

# Default setting 


BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque 
DXL_MOVING_STATUS_THRESHOLD = 10               # Dynamixel MX moving status threshold


index = 0
def angle(value):
    return (value+150)/300*1024

DXL_ID= [0,2,4,6,1,3,5,7] 
goal_angle=[
    [0],
    [0],
    [0],
    [0],
    [80],
    [0],
    [0],
    [0]
]

dxl_goal_position=[]
for i in range(len(DXL_ID)):
    dxl_goal_position.append([])
    for j in goal_angle[i]:
        dxl_goal_position[-1].append(math.floor(angle(j)))
 
# Initialize PortHandler instance
# Set the port paths
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel#1 Torque
for ID_number in DXL_ID:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID_number, TORQUE_ENABLE_AX, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % ID_number)

interval=25
scale=3
while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    # inputKey=getch()
    # if inputKey == chr(0x1b):
    #     break
    # elif inputKey=='A':
    #     dxl_goal_position[3][0]-=scale*interval
    for ID_number in range(4):
        if(dxl_goal_position[ID_number][0]>1023):
            dxl_goal_position[ID_number][0]=1023
        elif(dxl_goal_position[ID_number][0]<0):
            dxl_goal_position[ID_number][0]=0


    while 1:
        print(goal_angle)
        # Write Dynamixel#1 goal position
        for ID_number in DXL_ID:
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID_number, GOAL_POSITION_AX, dxl_goal_position[DXL_ID.index(ID_number)][index])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0: 
                print("%s" % packetHandler.getRxPacketError(dxl_error))

        memo=[500 for i in range(len(DXL_ID))]
        memo[DXL_ID.index(6)]=2048
        while 1:
            # Read Dynamixel#1 present position
            for ID_number in DXL_ID:
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, ID_number, PRESENT_POSITION)
                dxl_torque_enable, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, ID_number, TORQUE_ENABLE)

                if dxl_present_position>4096:
                    dxl_present_position=memo[DXL_ID.index(ID_number)]
                else:
                    memo[DXL_ID.index(ID_number)]=dxl_present_position
                
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                
                
                print("[ID:%03d] GoalPos:%.02f  PresPos:%.02f" % (ID_number, (dxl_goal_position[DXL_ID.index(ID_number)][index]), dxl_present_position),end='|  ')
                print("torque: %d" % (dxl_torque_enable,))
            print(' ')
            result=1

            for ID_number in DXL_ID: 
                result=result*(abs(dxl_goal_position[DXL_ID.index(ID_number)][index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD)

            if not result:
                break
        # Change goal position
        if index == len(dxl_goal_position[0])-1:
            index = 0
            break
        else:
            index +=1  


# Disable Dynamixel#1 Torque
for ID_number in DXL_ID:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID_number, TORQUE_ENABLE_AX, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))


# Close port
portHandler.closePort()