#!/usr/bin/env python
#
# *********     Sync Read and Sync Write Example      *********
#
#
# Available CF Servo model on this example : CF35-12
# This example is tested with a CF Servo(CF35-12), and an URT
#

import sys
import os
import time

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
from cfservo_sdk import *                   # Uses CF Servo SDK library

# Control table address

# Default setting
BAUDRATE                    = 1000000           # CF Servo default baudrate : 1000000
DEVICENAME                  = '/dev/ttyACM0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1" Linux: "/dev/ttyACM0" Mac: "/dev/tty.usbserial-*"

CFS_MINIMUM_POSITION_VALUE  = 0             # CF Servo will rotate between this value
CFS_MAXIMUM_POSITION_VALUE  = 4095              
CFS_MOVING_SPEED            = 2400          # CF Servo moving speed
CFS_MOVING_ACC              = 50            # CF Servo moving acc
CFS_MOVING_TORQUE           = 500           # CF Servo moving torque

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

groupSyncRead = GroupSyncRead(packetHandler, CFS_PRESENT_POSITION_L, 11)

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    for cfs_id in range(1, 11):
        # Add CF Servo#1~10 goal position\moving speed\moving accc value to the Syncwrite parameter storage
        cfs_addparam_result = packetHandler.SyncWritePosEx(cfs_id, cfs_goal_position[index], CFS_MOVING_SPEED, CFS_MOVING_ACC, CFS_MOVING_TORQUE)
        if cfs_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % cfs_id)

    # Syncwrite goal position
    cfs_comm_result = packetHandler.groupSyncWrite.txPacket()
    if cfs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(cfs_comm_result))

    # Clear syncwrite parameter storage
    packetHandler.groupSyncWrite.clearParam()
    time.sleep(0.002) #wait for servo status moving=1
    while 1:
        # Add parameter storage for CF Servo#1~10 present position value
        for cfs_id in range(1, 11):
            cfs_addparam_result = groupSyncRead.addParam(cfs_id)
            if cfs_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % cfs_id)

        cfs_comm_result = groupSyncRead.txRxPacket()
        if cfs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(cfs_comm_result))

        cfs_last_moving = 0;
        for cfs_id in range(1, 11):
            # Check if groupsyncread data of CF Servo#1~10 is available
            cfs_data_result, cfs_error = groupSyncRead.isAvailable(cfs_id, CFS_PRESENT_POSITION_L, 11)
            if cfs_data_result == True:
                # Get CF Servo#cfs_id present position moving value
                cfs_present_position = groupSyncRead.getData(cfs_id, CFS_PRESENT_POSITION_L, 2)
                cfs_present_speed = groupSyncRead.getData(cfs_id, CFS_PRESENT_SPEED_L, 2)
                cfs_present_moving = groupSyncRead.getData(cfs_id, CFS_MOVING, 1)
                # print(cfs_present_moving)
                print("[ID:%03d] PresPos:%d PresSpd:%d" % (cfs_id, cfs_present_position, packetHandler.sts_tohost(cfs_present_speed, 15)))
                if cfs_present_moving==1:
                    cfs_last_moving = 1;
            else:
                print("[ID:%03d] groupSyncRead getdata failed" % cfs_id)
                continue
            if cfs_error:
                print(packetHandler.getRxPacketError(cfs_error))            
        print("---")

        # Clear syncread parameter storage
        groupSyncRead.clearParam()
        if cfs_last_moving==0:
            break
    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

# Close port
portHandler.closePort()
