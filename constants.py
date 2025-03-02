# constants.py
# Constants for AK80-64 motor

# Position limits (radians)
P_MIN = -12.5
P_MAX = 12.5

# Velocity limits (rad/s)
V_MIN = -8.0
V_MAX = 8.0

# Position gain limits
KP_MIN = 0.0
KP_MAX = 500.0

# Velocity gain limits
KD_MIN = 0.0
KD_MAX = 5.0

# Torque limits (N·m)
T_MIN = -144.0
T_MAX = 144.0

# CAN controller ID
CONTROLLER_ID = 0x17  # 23 in decimal
