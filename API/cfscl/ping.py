#!/usr/bin/env python
#
# *********     Ping Example      *********
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
from cfservo_sdk import *                   # Uses CF Servo SDK library

# Default setting
CFS_ID                  = 1                 # CF Servo ID : 1
BAUDRATE                = 1000000           # CF Servo default baudrate : 1000000
DEVICENAME              = '/dev/ttyACM0'    # Check which port is being used on your controller
                                            # ex) Windows: "COM1" Linux: "/dev/ttyACM0" Mac: "/dev/tty.usbserial-*"

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

# Try to ping the CF Servo
# Get CF Servo model number
cfs_model_number, cfs_comm_result, cfs_error = packetHandler.ping(CFS_ID)
if cfs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(cfs_comm_result))
else:
    print("[ID:%03d] ping Succeeded. CF Servo model number : %d" % (CFS_ID, cfs_model_number))
if cfs_error != 0:
    print("%s" % packetHandler.getRxPacketError(cfs_error))

# Close port
portHandler.closePort()

