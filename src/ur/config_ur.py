from math import pi

###             CONSTANTS             ####
# Used to determine the RealTime version from message length
MESSAGE_SIZE_TO_VERSION = {'3.0': 1044, '3.2': 1060}

# Mapping of data message to variables
DATA_MAP = {'message_size': 0, 'time': 1,
            'q_b': 2,'q_s': 3, 'q_e': 4, 'q_w1': 5, 'q_w2': 6, 'q_w3': 7,
            'b': 32, 's': 33, 'e': 34, 'w1': 35, 'w2': 36, 'w3': 37,
            'v_b': 38, 'v_s': 39, 'v_e': 40, 'v_w1': 41, 'v_w2': 42, 'v_w3': 43,
            'x_actual': 56, 'y_actual': 57, 'z_actual': 58, 'rx_actual': 59, 'ry_actual': 60, 'rz_actual': 61,
            'v_x': 62, 'v_y': 63, 'v_z': 64, 'v_rx': 65, 'v_ry': 66, 'v_rz': 67,
            'f_x': 68, 'f_y': 69, 'f_z': 70, 'f_rx': 71, 'f_ry': 72, 'f_rz': 73,
            'x': 74, 'y': 75, 'z': 76, 'rx': 77, 'ry': 78, 'rz': 79,
            'robot_mode': 95, 'status': 132}


###             VARIABLES             ###
# Socket connection parameters
IP = '192.38.66.254'
PORT = 30003

# The threshold for determining end of movement for older versions of RealTime
VELOCITY_MEAN_THRESHOLD = 0.001

# Task to base transform
TRANSFORM = {'p0i': [-403.50, 242.49, 27.22],
             'pxi': [-405.45, 143.12, 26.68], 
             'pyi': [-303.82, 240.87, 25.87]}

# Denavit Hartenberg parameters (can be found on UR website)
DH_PARAMETERS = {'a': [0, -0.612, -0.5723, 0, 0, 0],
                 'd': [0.1273, 0, 0, 0.163941, 0.1157, 0.0922], 
                 'alpha': [pi/2, 0, 0, pi/2, -pi/2, 0]}

# Home pose for use with home function
HOME_POSE = [0.15, 0.12, 0.20, 
             pi, 0, 0]

# Default task space orientation of the end effector
# (Does not have to be the same as home pose)
DEFAULT_TASK_ORIENTATION = [pi, 0, 0]
