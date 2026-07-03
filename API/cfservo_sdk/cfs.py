#!/usr/bin/env python

from .stservo_def import *
from .protocol_packet_handler import *
from .group_sync_read import *
from .group_sync_write import *

# Baud Rate Definition
CFS_1M = 0
CFS_0_5M = 1
CFS_250K = 2
CFS_128K = 3
CFS_115200 = 4
CFS_76800 = 5
CFS_57600 = 6
CFS_38400 = 7
CFS_19200 = 8
CFS_14400 = 9
CFS_9600 = 10
CFS_4800 = 11

# Memory Table Definition
#-------EPROM(Read-Only)--------
CFS_MODEL_L = 3
CFS_MODEL_H = 4

#-------EPROM(Read/Write)--------
CFS_ID = 5
CFS_BAUD_RATE = 6
CFS_MIN_ANGLE_LIMIT_L = 9
CFS_MIN_ANGLE_LIMIT_H = 10
CFS_MAX_ANGLE_LIMIT_L = 11
CFS_MAX_ANGLE_LIMIT_H = 12
CFS_POSITION_LOOP_P = 21
CFS_POSITION_LOOP_D = 22
CFS_POSITION_LOOP_I = 23
CFS_CW_DEAD = 26
CFS_CCW_DEAD = 27
CFS_PROTECTION_CURRENT = 28
CFS_OFS_L = 31
CFS_OFS_H = 32
CFS_MODE = 33

#-------SRAM(Read/Write)--------
CFS_TORQUE_ENABLE = 40
CFS_ACC = 41
CFS_GOAL_POSITION_L = 42
CFS_GOAL_POSITION_H = 43
CFS_GOAL_TORQUE_L = 44
CFS_GOAL_TORQUE_H = 45
CFS_GOAL_SPEED_L = 46
CFS_GOAL_SPEED_H = 47
CFS_LOCK = 55

#-------SRAM(Read-Only)--------
CFS_PRESENT_POSITION_L = 56
CFS_PRESENT_POSITION_H = 57
CFS_PRESENT_SPEED_L = 58
CFS_PRESENT_SPEED_H = 59
CFS_PRESENT_LOAD_L = 60
CFS_PRESENT_LOAD_H = 61
CFS_PRESENT_VOLTAGE = 62
CFS_PRESENT_TEMPERATURE = 63
CFS_MOVING = 66
CFS_PRESENT_CURRENT_L = 69
CFS_PRESENT_CURRENT_H = 70

class cfs(protocol_packet_handler):
    def __init__(self, portHandler):
        protocol_packet_handler.__init__(self, portHandler, 0)
        self.groupSyncWrite = GroupSyncWrite(self, CFS_ACC, 7)

    def WritePosEx(self, sts_id, position, speed, acc, torque):
        txpacket = [acc, self.sts_lobyte(position), self.sts_hibyte(position), self.sts_lobyte(torque), self.sts_hibyte(torque), self.sts_lobyte(speed), self.sts_hibyte(speed)]
        return self.writeTxRx(sts_id, CFS_ACC, len(txpacket), txpacket)

    def ReadPos(self, sts_id):
        sts_present_position, sts_comm_result, sts_error = self.read2ByteTxRx(sts_id, CFS_PRESENT_POSITION_L)
        return self.sts_tohost(sts_present_position, 15), sts_comm_result, sts_error

    def ReadSpeed(self, sts_id):
        sts_present_speed, sts_comm_result, sts_error = self.read2ByteTxRx(sts_id, CFS_PRESENT_SPEED_L)
        return self.sts_tohost(sts_present_speed, 15), sts_comm_result, sts_error

    def ReadPosSpeed(self, sts_id):
        sts_present_position_speed, sts_comm_result, sts_error = self.read4ByteTxRx(sts_id, CFS_PRESENT_POSITION_L)
        sts_present_position = self.sts_loword(sts_present_position_speed)
        sts_present_speed = self.sts_hiword(sts_present_position_speed)
        return self.sts_tohost(sts_present_position, 15), self.sts_tohost(sts_present_speed, 15), sts_comm_result, sts_error

    def ReadMoving(self, sts_id):
        moving, sts_comm_result, sts_error = self.read1ByteTxRx(sts_id, CFS_MOVING)
        return moving, sts_comm_result, sts_error

    def SyncWritePosEx(self, sts_id, position, speed, acc, torque):
        txpacket = [acc, self.sts_lobyte(position), self.sts_hibyte(position), self.sts_lobyte(torque), self.sts_hibyte(torque), self.sts_lobyte(speed), self.sts_hibyte(speed)]
        return self.groupSyncWrite.addParam(sts_id, txpacket)

    def RegWritePosEx(self, sts_id, position, speed, acc, torque):
        txpacket = [acc, self.sts_lobyte(position), self.sts_hibyte(position), self.sts_lobyte(torque), self.sts_hibyte(torque), self.sts_lobyte(speed), self.sts_hibyte(speed)]
        return self.regWriteTxRx(sts_id, CFS_ACC, len(txpacket), txpacket)

    def RegAction(self):
        return self.action(BROADCAST_ID)

    def WheelMode(self, sts_id):
        return self.write1ByteTxRx(sts_id, CFS_MODE, 1)

    def WriteSpec(self, sts_id, speed, acc, torque):
        speed = self.sts_toscs(speed, 15)
        txpacket = [acc, 0, 0, self.sts_lobyte(torque), self.sts_hibyte(torque), self.sts_lobyte(speed), self.sts_hibyte(speed)]
        return self.writeTxRx(sts_id, CFS_ACC, len(txpacket), txpacket)

    def LockEprom(self, sts_id):
        return self.write1ByteTxRx(sts_id, CFS_LOCK, 1)

    def unLockEprom(self, sts_id):
        return self.write1ByteTxRx(sts_id, CFS_LOCK, 0)

