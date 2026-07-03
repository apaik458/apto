#!/usr/bin/env python
import atexit
from cfservo_sdk import *
import cv2
import numpy as np
import pyrealsense2 as rs


# Data Byte Length
LEN_PRESENT_POSITION = 2
LEN_GOAL_POSITION    = 2
LEN_PRESENT_SPEED    = 2
LEN_GOAL_SPEED       = 2


def cleanup_handler():
    """Cleanup function to ensure motors are disconnected properly."""
    open_clients = list(AptoSDK.OPEN_CLIENTS)
    for open_client in open_clients:
        if open_client.port_handler.is_using:
            print('Forcing client to close.')
        open_client.port_handler.is_using = False
        open_client.disconnect()
    print('Shutting down')

def signed_to_unsigned(value: int, size: int) -> int:
    """Converts the given value to its unsigned representation"""
    if value < 0:
        bit_size = 8 * size
        max_value = (1 << bit_size) - 1
        value = max_value + value
    return value

def unsigned_to_signed(value: int, size: int) -> int:
    """Converts the given value from its unsigned representation"""
    bit_size = 8 * size
    if (value & (1 << (bit_size - 1))) != 0:
        value = -((1 << bit_size) - value)
    return value

## Conversions (0 -> 2048 -> 4096 = -pi -> 0 -> pi), (positive motor rotation = clockwise)
def pos_scale_atv(angle):
    """Converts from joint angles to motor values"""
    return (angle / np.pi + 1) * 2048

def pos_scale_vta(value):
    """Converts from motor values to joint angles"""
    return value * (2 * np.pi / 4096) - np.pi

## Conversions (0 -> 3400 = 0 -> 1.659pi), (50 steps per second = 0.732 RPM)
def vel_scale_stv(angle):
    """Converts from joint speeds to motor values"""
    return (angle / (1.659*np.pi)) * 3400

def vel_scale_vts(value):
    """Converts from motor values to joint speeds"""
    return (value / 3400) * (1.659*np.pi)

## Safety clips all joints so nothing unsafe can happen
def angle_safety_clip(joints):
    min = np.array([0.0,
                    -np.pi/4, -np.pi/10, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2,
                    -np.pi/4, -np.pi/10, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2])
    max = np.array([np.pi/2,
                    np.pi/2, np.pi/2, np.pi/2, np.pi/3, np.pi/2, np.pi/2, 0.0,
                    np.pi/2, np.pi/2, np.pi/2, np.pi/3, np.pi/2, np.pi/2, 0.0])
    return np.clip(joints, min, max)

## Adds an offset to the joints to center them properly
def angle_offset(joints):
    value_offset = np.array([62,
                              0,   0,  27, 52,  0, -48, 0,
                             52, -17, -38,  0, 27,   0, 0])
    angle_offset = pos_scale_vta(value_offset) + np.ones(value_offset.shape) * np.pi
    return joints + angle_offset


class AptoSDK:
    """
    Client for communicating with Waveshare motors
    """

    # The currently open clients.
    OPEN_CLIENTS = set()
    
    def __init__(self,
                 port: str = '/dev/ttyACM0',
                 baudrate = 1000000):
        """Initialises a new client"""
        self.motor_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]
        self.port_name = port
        self.baudrate = baudrate

        self.port_handler = PortHandler(port)
        self.packet_handler = cfs(self.port_handler)

        self._pos_reader = AptoPosReader(
            self,
            self.motor_ids
        )

        self._vel_reader = AptoVelReader(
            self,
            self.motor_ids
        )

        self.camera = Camera()

        self._sync_writers = {}
        self.OPEN_CLIENTS.add(self)

        # Parameters
        self.kP = 32               # default PID = 32, 0, 32
        self.kI = 0
        self.kD = 32
        self.curr_lim = 500        # 500 * 0.0065A = 3.25A
        self.moving_speed  = 683   # 683 * (0.732 / 50) = 10RPM
        self.moving_acc    = 25    # 25 * 100 = 2500 steps/s^2
        self.moving_torque = 500   # 500 * 0.0065A = 3.25A

        self.prev_pos = self.pos = self.curr_pos = np.zeros(7)
        self.prev_vel = self.vel = self.curr_vel = np.zeros(7)

    def is_connected(self):
        return self.port_handler.is_open

    def connect(self):
        """Connects to the Waveshare motors"""
        # Typo in port_handler.py, clearPort()?
        # assert not self.is_connected, 'Client is already connected.'

        if self.port_handler.openPort():
            print('Succeeded to open port: %s', self.port_name)
        else:
            raise OSError(
                ('Failed to open port at {} (Check that the device is powered '
                 'on and connected to your computer).').format(self.port_name))

        if self.port_handler.setBaudRate(self.baudrate):
            print('Succeeded to set baudrate to %d', self.baudrate)
        else:
            raise OSError(
                ('Failed to set the baudrate to {} (Ensure that the device was '
                 'configured for this baudrate).').format(self.baudrate))
        
        # Enable motor torques
        self.set_torque_enabled(self.motor_ids, False)

        # Set the default parameters
        self.sync_write(self.motor_ids, np.ones(len(self.motor_ids)) * self.kP, CFS_POSITION_LOOP_P, 1)       # Pgain stiffness
        self.sync_write(self.motor_ids, np.ones(len(self.motor_ids)) * self.kI, CFS_POSITION_LOOP_I, 1)       # Igain
        self.sync_write(self.motor_ids, np.ones(len(self.motor_ids)) * self.kD, CFS_POSITION_LOOP_D, 1)       # Dgain damping
        self.sync_write(self.motor_ids, np.ones(len(self.motor_ids)) * self.curr_lim, CFS_PROTECTION_CURRENT, 2) # Protection current limit
        
        # Set operating mode
        self.sync_write(self.motor_ids, np.zeros(len(self.motor_ids)), CFS_MODE, 1) # 0 = position control, 1 = velocity control
    
    def disconnect(self):
        """Disconnects from the Waveshare device"""
        if not self.is_connected:
            return
        
        if self.port_handler.is_using:
            print('Port handler in use; cannot disconnect.')
            return
        
        # Stop camera streaming
        self.camera.pipeline.stop()

        # Ensure motors are disabled at the end.
        self.set_torque_enabled(self.motor_ids, False, retries=0)
        self.port_handler.closePort()
        if self in self.OPEN_CLIENTS:
            self.OPEN_CLIENTS.remove(self)

    def set_torque_enabled(self,
                           motor_ids,
                           enabled,
                           retries = -1,
                           retry_interval = 0.25):
        """Sets whether torque is enabled for the motors"""
        remaining_ids = list(motor_ids)
        while remaining_ids:
            remaining_ids = self.write_byte(remaining_ids, int(enabled), CFS_TORQUE_ENABLE)
            if remaining_ids:
                print('Could not set torque %s for IDs: %s',
                    'enabled' if enabled else 'disabled',
                    str(remaining_ids))
            if retries == 0:
                break
            time.sleep(retry_interval)
            retries -= 1

    def read_pos(self):
        """Returns the current positions"""
        positions = self._pos_reader.read().round(3)
        for i, _ in enumerate(positions):
            if i in [1, 2, 3, 4, 5, 6]:
                positions[i] *= -1

        return positions
    
    def write_desired_pos(self, motor_ids, positions):
        """Writes the given desired positions"""
        assert len(motor_ids) == len(positions)
        
        # Clip joint angles within limits
        positions = angle_safety_clip(positions)

        # Switch position direction for certain joints
        for i, _ in enumerate(positions):
            if i in [1, 2, 3, 4, 5, 6]:
                positions[i] *= -1

        # Add joint angle offsets
        positions = angle_offset(positions)

        # Convert joint angles to motor values
        positions = pos_scale_atv(positions)

        # Add servo goal position\moving speed\moving acc values to the Syncwrite parameter storage
        for id in motor_ids:
            cfs_addparam_result = self.packet_handler.SyncWritePosEx(id, int(positions[id]), self.moving_speed, self.moving_acc, self.moving_torque)
            if cfs_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % id)

        # Syncwrite goal position
        cfs_comm_result = self.packet_handler.groupSyncWrite.txPacket()
        if cfs_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(cfs_comm_result))

        # Clear syncwrite parameter storage
        self.packet_handler.groupSyncWrite.clearParam()
    
    def set_pose(self, pose):
        """Set a goal pose for the joints (radians)"""
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)

        self.write_desired_pos(self.motor_ids, self.curr_pos)
    
    def read_vel(self):
        """Returns the current velocities"""
        return self._vel_reader.read().round(3)

    def write_desired_vel(self, motor_ids, velocities):
        """Writes the given desired velocities"""
        assert len(motor_ids) == len(velocities)
        
        velocities = vel_scale_stv(velocities)
        # handle negative velocities
        velocities = [int(-v)|0b1000000000000000 if v<0 else v for v in velocities]
        self.sync_write(motor_ids, velocities, CFS_GOAL_SPEED_L, LEN_GOAL_SPEED)

    def set_vel(self, vel):
        """Set target velocities for the joints (rad/s)"""
        self.prev_vel = self.curr_vel
        self.curr_vel = np.array(vel)
        self.write_desired_vel(self.motor_ids, self.curr_vel)

    def write_byte(self, motor_ids, value, address):
        """Writes a value to the motors"""
        self.check_connected()
        errored_ids = []
        for motor_id in motor_ids:
            sts_comm_result, sts_error = self.packet_handler.write1ByteTxRx(motor_id, address, value)
            success = self.handle_packet_result(
                sts_comm_result, sts_error, motor_id, context='write_byte')
            if not success:
                errored_ids.append(motor_id)
        return errored_ids

    def sync_write(self, motor_ids, values, address, size):
        """Writes values to a group of motors"""
        self.check_connected()
        key = (address, size)
        if key not in self._sync_writers:
            self._sync_writers[key] = GroupSyncWrite(self.packet_handler, address, size)
        sync_writer = self._sync_writers[key]

        errored_ids = []
        for motor_id, desired_pos in zip(motor_ids, values):
            value = signed_to_unsigned(int(desired_pos), size=size)
            value = value.to_bytes(size, byteorder='little')
            success = sync_writer.addParam(motor_id, value)
            if not success:
                errored_ids.append(motor_id)

        if errored_ids:
            print('Sync write failed for: %s', str(errored_ids))

        comm_result = sync_writer.txPacket()
        self.handle_packet_result(comm_result, context='sync_write')

        sync_writer.clearParam()

    def check_connected(self):
        """Ensures the robot is connected"""
        if not self.is_connected:
            self.connect()
            raise OSError('Must call connect() first.')

    def handle_packet_result(self,
                             comm_result,
                             sts_error = None,
                             sts_id = None,
                             context = None):
        """Handles the result from a communication request"""
        error_message = None
        if comm_result != COMM_SUCCESS:
            error_message = self.packet_handler.getTxRxResult(comm_result)
        elif sts_error is not None:
            error_message = self.packet_handler.getRxPacketError(sts_error)
        if error_message:
            if sts_id is not None:
                error_message = '[Motor ID: {}] {}'.format(sts_id, error_message)
            if context is not None:
                error_message = '> {}: {}'.format(context, error_message)
            print(error_message)
            return False
        return True
    
    def convert_to_unsigned(self, value: int, size: int) -> int:
        """Converts the given value to its unsigned representation"""
        if value < 0:
            max_value = (1 << (8 * size)) - 1
            value = max_value + value
        return value

class AptoReader:
    """
    Reads data from Waveshare motors
    """

    def __init__(self, client, motor_ids, address, size):
        """Initialises a new reader"""
        self.client = client
        self.motor_ids = motor_ids
        self.address = address
        self.size = size
        self._initialise_data()

        self.operation = GroupSyncRead(client.packet_handler, address, size)

        for motor_id in motor_ids:
            success = self.operation.addParam(motor_id)
            if not success:
                raise OSError('[Motor ID: {}] Could not add parameter to bulk read.'.format(motor_id))

    def read(self, retries = 1):
        """Reads data from the motors"""
        self.client.check_connected()
        success = False
        while not success and retries >= 0:
            comm_result = self.operation.txRxPacket()
            success = self.client.handle_packet_result(comm_result, context='read')
            retries -= 1

        # If we failed, send a copy of the previous data.
        if not success:
            return self._get_data()

        errored_ids = []
        for i, motor_id in enumerate(self.motor_ids):
            # Check if the data is available.
            available = self.operation.isAvailable(motor_id, self.address, self.size)
            if not available:
                errored_ids.append(motor_id)
                continue

            self._update_data(i, motor_id)

        if errored_ids:
            print('Bulk read data is unavailable for: %s', str(errored_ids))

        return self._get_data()

    def _initialise_data(self):
        """Initialises the cached data"""
        self._data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index, motor_id):
        """Updates the data index for the given motor ID"""
        self._data[index] = self.operation.getData(motor_id, self.address, self.size)

    def _get_data(self):
        """Returns a copy of the data"""
        return self._data.copy()

class AptoPosReader(AptoReader):
    """Reads positions"""

    def __init__(self,
                 client,
                 motor_ids):
        super().__init__(
            client,
            motor_ids,
            address=CFS_PRESENT_POSITION_L,
            size=LEN_PRESENT_POSITION,
        )

    def _initialise_data(self):
        """Initialises the cached data"""
        self._pos_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index, motor_id):
        """Updates the data index for the given motor ID"""
        pos = self.operation.getData(motor_id, CFS_PRESENT_POSITION_L, LEN_PRESENT_POSITION)
        pos = unsigned_to_signed(pos, size=4)
        self._pos_data[index] = pos_scale_vta(pos)
    
    def _get_data(self):
        """Returns a copy of the data"""
        return self._pos_data.copy()

class AptoVelReader(AptoReader):
    """Reads velocities"""

    def __init__(self,
                 client,
                 motor_ids):
        super().__init__(
            client,
            motor_ids,
            address=CFS_PRESENT_SPEED_L,
            size=LEN_PRESENT_SPEED,
        )

    def _initialise_data(self):
        """Initialises the cached data"""
        self._vel_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index, motor_id):
        """Updates the data index for the given motor ID"""
        vel = self.operation.getData(motor_id, CFS_PRESENT_SPEED_L, LEN_PRESENT_SPEED)
        
        if vel&0b1000000000000000:
            vel &= 0b0111111111111111
            vel *= -1
        # vel = unsigned_to_signed(vel, size=4)
        self._vel_data[index] = vel_scale_vts(vel)
    
    def _get_data(self):
        """Returns a copy of the data"""
        return self._vel_data.copy()

class Camera:
    """
    Gets image data from camera
    """

    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in self.device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(self.config)
    
    def get_frame(self):
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            break

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        images = color_image

        return images

# Register global cleanup function.
atexit.register(cleanup_handler)