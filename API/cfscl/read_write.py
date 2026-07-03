#!/usr/bin/env python
#
# *********     Gen Write Example      *********
#
#
# Available CF Servo model on this example : CF35-12
# This example is tested with a CF Servo(CF35-12), and an URT
#

import sys
import os

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

sys.path.append("..")
from cfservo_sdk import *                      # Uses CF Servo SDK library

# Default setting
CFS_ID                      = 1                 # CF Servo ID : 1
BAUDRATE                    = 1000000           # CF Servo default baudrate : 1000000
DEVICENAME                  = '/dev/ttyACM0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1" Linux: "/dev/ttyACM0" Mac: "/dev/tty.usbserial-*"
CFS_MINIMUM_POSITION_VALUE  = 0           # CF Servo will rotate between this value
CFS_MAXIMUM_POSITION_VALUE  = 4095
CFS_MOVING_SPEED            = 2400        # CF Servo moving speed
CFS_MOVING_ACC              = 50          # CF Servo moving acc
CFS_MOVING_TORQUE           = 500         # CF Servo moving torque

index = 0
cfs_goal_position = [CFS_MINIMUM_POSITION_VALUE, CFS_MAXIMUM_POSITION_VALUE]         # Goal position

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = cfs(portHandler)
    
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

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    # Write CF Servo goal position/moving speed/moving acc
    cfs_comm_result, cfs_error = packetHandler.WritePosEx(CFS_ID, cfs_goal_position[index], CFS_MOVING_SPEED, CFS_MOVING_ACC, CFS_MOVING_TORQUE)
    if cfs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(cfs_comm_result))
    elif cfs_error != 0:
        print("%s" % packetHandler.getRxPacketError(cfs_error))

    while 1:
        # Read CF Servo present position
        cfs_present_position, cfs_present_speed, cfs_comm_result, cfs_error = packetHandler.ReadPosSpeed(CFS_ID)
        if cfs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(cfs_comm_result))
        else:
            print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (CFS_ID, cfs_goal_position[index], cfs_present_position, cfs_present_speed))
        if cfs_error != 0:
            print(packetHandler.getRxPacketError(cfs_error))

        # Read CF Servo moving status
        moving, cfs_comm_result, cfs_error = packetHandler.ReadMoving(CFS_ID)
        if cfs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(cfs_comm_result))

        if moving==0:
            break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

# Close port
portHandler.closePort()
