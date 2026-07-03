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
from cfservo_sdk import *

# Default setting
PREV_ID = 1                 # CFServo ID : 1
NEW_ID = 2                  # Change the Servo ID
BAUDRATE = 1000000          # CFServo default baudrate:1000000
DEVICENAME = '/dev/ttyACM0' # Check which port is being used on your controller
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
    print("Succeeded to set the baudrate")
else:
    print("Failed to set the baudrate")
    portHandler.closePort()
    quit()


cfs_comm_result, cfs_error = packetHandler.unLockEprom(PREV_ID)
if cfs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(cfs_comm_result))
elif cfs_error != 0:
    print("%s" % packetHandler.getRxPacketError(cfs_error))
    getch()
    quit()

cfs_comm_result, cfs_error = packetHandler.write1ByteTxRx(PREV_ID, CFS_ID, NEW_ID)
if cfs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(cfs_comm_result))
else:
    packetHandler.LockEprom(PREV_ID)
    print("Succeeded to change the Servo ID")
if cfs_error != 0:
    print("%s" % packetHandler.getRxPacketError(cfs_error))
    getch()
    quit()