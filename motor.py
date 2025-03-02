# motor.py
# Motor state and communication functions

import can
import platform
from constants import *

class MotorState:
    def __init__(self):
        # Input values
        self.p_in = 0.0
        self.v_in = 0.0
        self.kp_in = 0.0
        self.kd_in = 0.50
        self.t_in = 0.0

        # Measured values
        self.p_out = 0.0
        self.v_out = 0.0
        self.t_out = 0.0

def float_to_uint(x, x_min, x_max, bits):
    """Convert float to unsigned integer for CAN transmission"""
    span = x_max - x_min
    offset = x_min
    if bits == 12:
        return int((x - offset) * 4095.0 / span)
    elif bits == 16:
        return int((x - offset) * 65535.0 / span)
    return 0

def uint_to_float(x_int, x_min, x_max, bits):
    """Convert unsigned integer to float from CAN reception"""
    span = x_max - x_min
    offset = x_min
    if bits == 12:
        return (float(x_int) * span / 4095.0) + offset
    elif bits == 16:
        return (float(x_int) * span / 65535.0) + offset
    return 0.0

def enter_mode(bus):
    """Enter MIT mode"""
    if bus is None:
        return False
    msg = can.Message(
        arbitration_id=CONTROLLER_ID,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
        is_extended_id=False
    )
    try:
        bus.send(msg)
        return True
    except:
        return False

def exit_mode(bus):
    """Exit MIT mode"""
    if bus is None:
        return False
    msg = can.Message(
        arbitration_id=CONTROLLER_ID,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD],
        is_extended_id=False
    )
    try:
        bus.send(msg)
        return True
    except:
        return False

def zero_position(bus):
    """Set current position as zero reference"""
    if bus is None:
        return False
    msg = can.Message(
        arbitration_id=CONTROLLER_ID,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE],
        is_extended_id=False
    )
    try:
        bus.send(msg)
        return True
    except:
        return False

def pack_cmd(bus, motor_state):
    """Pack motor control command and send via CAN"""
    if bus is None:
        return False

    # Constrain values
    p_des = max(min(motor_state.p_in, P_MAX), P_MIN)
    v_des = max(min(motor_state.v_in, V_MAX), V_MIN)
    kp = max(min(motor_state.kp_in, KP_MAX), KP_MIN)
    kd = max(min(motor_state.kd_in, KD_MAX), KD_MIN)
    t_ff = max(min(motor_state.t_in, T_MAX), T_MIN)

    # Convert to integers
    p_int = float_to_uint(p_des, P_MIN, P_MAX, 16)
    v_int = float_to_uint(v_des, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12)

    # Pack into buffer
    buf = bytearray(8)
    buf[0] = p_int >> 8
    buf[1] = p_int & 0xFF
    buf[2] = v_int >> 4
    buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)
    buf[4] = kp_int & 0xFF
    buf[5] = kd_int >> 4
    buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)
    buf[7] = t_int & 0xFF

    msg = can.Message(
        arbitration_id=CONTROLLER_ID,
        data=buf,
        is_extended_id=False
    )
    try:
        bus.send(msg)
        return True
    except:
        return False

def unpack_reply(msg, motor_state):
    """Unpack motor status reply from CAN message"""
    if msg is None:
        return False

    try:
        buf = msg.data
        id_received = buf[0]
        p_int = (buf[1] << 8) | buf[2]
        v_int = (buf[3] << 4) | (buf[4] >> 4)
        i_int = ((buf[4] & 0xF) << 8) | buf[5]

        motor_state.p_out = uint_to_float(p_int, P_MIN, P_MAX, 16)
        motor_state.v_out = uint_to_float(v_int, V_MIN, V_MAX, 12)
        motor_state.t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12)
        return True
    except:
        return False

def detect_can_interface():
    """Detect the appropriate CAN interface based on the operating system"""
    system = platform.system()

    if system == 'Windows':
        # For Windows, use SLCAN interface with a COM port
        print("Detected Windows OS")
        interface = 'slcan'
        channel = 'COM5'  # Default COM port

    elif system == 'Linux':
        # For Linux, use socketcan interface
        print("Detected Linux OS")
        interface = 'slcan'
        channel = '/dev/ttyACM0'  # Default CAN interface
        print("Make sure CAN interface is up: sudo ip link set can0 up type can bitrate 1000000")
    else:
        # Default fallback
        print(f"Unsupported OS: {system}, using default configuration")
        interface = 'slcan'
        channel = 'COM5'

    return interface, channel
