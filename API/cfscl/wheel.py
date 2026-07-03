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
from cfservo_sdk import *                 # Uses CF Servo SDK library

# Default setting
CFS_ID                      = 1                 # CF Servo ID : 1
BAUDRATE                    = 1000000           # CF Servo default baudrate : 1000000
DEVICENAME                  = '/dev/ttyACM0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1" Linux: "/dev/ttyACM0" Mac: "/dev/tty.usbserial-*"
CFS_MOVING_SPEED0           = 2400        # CF Servo moving speed
CFS_MOVING_SPEED1           = -2400       # CF Servo moving speed
CFS_MOVING_ACC              = 50          # CF Servo moving acc
CFS_MOVING_TORQUE           = 500         # CF Servo moving torque

index = 0
cfs_move_speed = [CFS_MOVING_SPEED0, 0, CFS_MOVING_SPEED1, 0]

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

cfs_comm_result, cfs_error = packetHandler.WheelMode(CFS_ID)
if cfs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(cfs_comm_result))
elif cfs_error != 0:
    print("%s" % packetHandler.getRxPacketError(cfs_error))   
while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    # Write CF Servo goal position/moving speed/moving acc
    cfs_comm_result, cfs_error = packetHandler.WriteSpec(CFS_ID, cfs_move_speed[index], CFS_MOVING_ACC, CFS_MOVING_TORQUE)
    if cfs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(cfs_comm_result))
    if cfs_error != 0:
        print("%s" % packetHandler.getRxPacketError(cfs_error))

    # Change move speed
    index += 1
    if index == 4:
        index = 0

# Close port
portHandler.closePort()
