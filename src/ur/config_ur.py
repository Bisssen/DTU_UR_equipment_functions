from math import pi

###             CONSTANTS             ####
# Used to determine the RealTime version from message length
MESSAGE_SIZE_TO_VERSION = {'3.0': 1044, '3.2': 1060}

MESSAGE_SIZE = 'message_size'
TIME = 'time'
Q_B = 'q_b'
Q_S = 'q_s'
Q_E = 'q_e'
Q_W1 = 'q_w1'
Q_W2 = 'q_w2'
Q_W3 = 'q_w3'
B = 'b'
S = 's'
E = 'e'
W1 = 'w1'
W2 = 'w2'
W3 = 'w3'
V_B = 'v_b'
V_S = 'v_s'
V_E = 'v_e'
V_W1 = 'v_w1'
V_W2 = 'v_w2'
V_W3 = 'v_w3'
X_ACTUAL = 'x_actual'
Y_ACTUAL = 'y_actual'
Z_ACTUAL = 'z_actual'
RX_ACTUAL = 'rx_actual'
RY_ACTUAL = 'ry_actual'
RZ_ACTUAL = 'rz_actual'
V_X = 'v_x'
V_Y = 'v_y'
V_Z = 'v_z'
V_RX = 'v_rx'
V_RY = 'v_ry'
V_RZ = 'v_rz'
F_X = 'f_x'
F_Y = 'f_y'
F_Z = 'f_z'
F_RX = 'f_rx'
F_RY = 'f_ry'
F_RZ = 'f_rz'
X = 'x'
Y = 'y'
Z = 'z'
RX = 'rx'
RY = 'ry'
RZ = 'rz'
ROBOT_MODE = 'robot_mode'
STATUS = 'status'

# Mapping of data message to variables
DATA_MAP = {
    MESSAGE_SIZE: 0, TIME: 1,
    Q_B: 2, Q_S: 3, Q_E: 4, Q_W1: 5, Q_W2: 6, Q_W3: 7,
    B: 32, S: 33, E: 34, W1: 35, W2: 36, W3: 37,
    V_B: 38, V_S: 39, V_E: 40, V_W1: 41, V_W2: 42, V_W3: 43,
    X_ACTUAL: 56, Y_ACTUAL: 57, Z_ACTUAL: 58, RX_ACTUAL: 59, RY_ACTUAL: 60, RZ_ACTUAL: 61,
    V_X: 62, V_Y: 63, V_Z: 64, V_RX: 65, V_RY: 66, V_RZ: 67,
    F_X: 68, F_Y: 69, F_Z: 70, F_RX: 71, F_RY: 72, F_RZ: 73,
    X: 74, Y: 75, Z: 76, RX: 77, RY: 78, RZ: 79,
    ROBOT_MODE: 95, STATUS: 132
}



###             VARIABLES             ###
# Socket connection parameters
IP = '192.168.2.100'
PORT = 30003

# The threshold for determining end of movement for older versions of RealTime
VELOCITY_MEAN_THRESHOLD = 0.001

# Task to base transform
TRANSFORM = {'p0i':[500.08, -630.98, 15.56],
             'pxi':[500.98, -531.09, 15.06], 
             'pyi':[400.15, -629.81, 14.90]}

# Home pose for use with home function
HOME_POSE = [0.15, 0.12, 0.35, 
             pi, 0, 0]

# Default task space orientation of the end effector
# (Does not have to be the same as home pose)
DEFAULT_ORIENTATION = [pi, 0, 0]
