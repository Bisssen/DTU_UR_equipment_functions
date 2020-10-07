from math import pi

CONTROLLER_VERSION = 1.8
VELOCITY_MEAN_THRESHOLD = 0.001

IP = '192.38.66.226'
PORT = 30003

DATA_MAP = {'message_size': 0, 'time': 4,
            'q_b': 12,'q_s': 20, 'q_e': 28, 'q_w1': 36, 'q_w2': 44, 'q_w3': 52,
            'b': 252, 's': 260, 'e': 268, 'w1': 276, 'w2': 284, 'w3': 292,
            'v_b': 300, 'v_s': 308, 'v_e': 316, 'v_w1': 324, 'v_w2': 332, 'v_w3': 340,
            'x_actual': 444, 'y_actual': 452, 'z_actual': 460, 'rx_actual': 468, 'ry_actual': 476, 'rz_actual': 484,
            'v_x': 492, 'v_y': 500, 'v_z': 508, 'v_rx': 516, 'v_ry': 524, 'v_rz': 532,
            'f_x': 540, 'f_y': 548, 'f_z': 556, 'f_rx': 564, 'f_ry': 572, 'f_rz': 580,
            'x': 588, 'y': 596, 'z': 604, 'rx': 612, 'ry': 620, 'rz': 628,
            'robot_mode': 756, 'status': 1052}

# UR5 transform
TRANSFORM = {'p0i':[-403.50, 242.49, 27.22],
             'pxi':[-405.45, 143.12, 26.68], 
             'pyi':[-303.82, 240.87, 25.87]}

'''
# UR3 transform
TRANSFORM = {'p0i':[-119.38, 449.1, 10.66],
             'pxi':[-124.4, 232.3, 8.09], 
             'pyi':[31.39, 446.77, 10.76]}
'''

HOME = {'position':[0.15, 0.12, 0.20],
        'angle':[pi, 0, 0]}
