import time
from math import pi
import numpy as np
import socket
import sys

from . import config_ur
from .communication_ur import communication_thread


class UR:
    def __init__(self):
        # Whether the program is run in python 2 or not
        self.python_2 = (sys.version_info.major == 2)

        #
        self.rotation_matrice = np.array([[0, 0, 0],
                                          [0, 0, 0],
                                          [0, 0, 0]])

        #
        self.origen_task = 0

        #
        self.transform_init(config_ur.TRANSFORM['p0i'],
                            config_ur.TRANSFORM['pxi'],
                            config_ur.TRANSFORM['pyi'])

        # The default position of the gripper
        self.home_position = config_ur.HOME['position']
        self.home_angle = config_ur.HOME['angle']

        # Dictionary containing all the ur data which have been reading
        self.ur_data = {}

        # Connecting socket directly to robot
        self.socket_ur_send = socket.socket(socket.AF_INET,
                                            socket.SOCK_STREAM)
        self.socket_ur_send.connect((config_ur.IP,
                                     config_ur.PORT))

        # Starting communication script
        self.communication_thread = communication_thread(self.python_2)

    def transform_init(self, p0i, pxi, pyi):
        p0 = np.array(p0i)
        px = np.array(pxi)
        py = np.array(pyi)
        p0 = p0/1000.
        px = px/1000.
        py = py/1000.
        vx = px-p0
        vy = py-p0
        vx = vx/np.linalg.norm(vx)
        vy = vy/np.linalg.norm(vy)
        vz = np.cross(vx, vy)
        self.rotation_matrice = np.array([vx, vy, vz])
        self.rotation_matrice = np.transpose(self.rotation_matrice)
        self.origen_task = p0

    def transform(self, x, y, z):
        b = np.array([x, y, z])
        t = self.rotation_matrice.dot(np.transpose(b)) + self.origen_task
        return t

    def inverse_transform(self, x, y, z):
        b = np.array([x, y, z])
        it = np.transpose(self.rotation_matrice).dot(np.transpose((b - self.origen_task)))
        return it

    def set_tcp(self, x=0, y=0, z=0, rx=0, ry=0, rz=0):
        self.socket_ur_send.send(('set_tcp(p[' + str(x) + ',' + str(y) +
                                  ',' + str(z) + ',' + str(rx) + ',' +
                                  str(ry) + ',' + str(rz) + '])\n').encode())
        time.sleep(0.1)

    def get_position(self, world=True):
        self.read()
        if config_ur.CONTROLLER_VERSION >= 3.0:
            x = self.ur_data['x_actual']
            y = self.ur_data['y_actual']
            z = self.ur_data['z_actual']
        else:
            x = self.ur_data['x']
            y = self.ur_data['y']
            z = self.ur_data['z']
        if world:
            return self.inverse_transform(x, y, z)
        else:
            return (x, y, z)

    def move(self, x, y, z, rx=pi, ry=0, rz=0, acc=1, speed=0.1,
             transform=True):
        if transform:
            x, y, z = self.transform(x, y, z)
        self.socket_ur_send.send(('movel(p[' + str(x) + ',' + str(y) +
                                  ',' + str(z) + ',' + str(rx) + ',' +
                                  str(ry) + ',' + str(rz) + '],' + str(acc) +
                                  ',' + str(speed) + ')\n').encode())
        self.wait()

    def move_relative(self, x=0, y=0, z=0, rx=0, ry=0, rz=0, acc=1, speed=0.1):
        x_current, y_current, z_current = self.get_position(world=False)
        if config_ur.CONTROLLER_VERSION >= 3.0:
            rx_current = self.ur_data['rx_actual']
            ry_current = self.ur_data['ry_actual']
            rz_current = self.ur_data['rz_actual']
        else:
            rx_current = self.ur_data['rx']
            ry_current = self.ur_data['ry']
            rz_current = self.ur_data['rz']

        self.move(x_current + x, y_current + y, z_current + z,
                  rx_current + rx, ry_current + ry, rz_current + rz,
                  acc, speed, transform=False)

    def move_tool(self, x=0, y=0, z=0, rx=0, ry=0, rz=0, acc=1, speed=0.1):
        send_string = 'movel(pose_trans(get_forward_kin(),' +\
                      'p[' + str(x) + ',' + str(y) + ',' + str(z) +\
                      ',' + str(rx) + ',' + str(ry) + ',' + str(rz) + ']' +\
                      '),' + str(acc) + ',' + str(speed) + ')\n'
        self.socket_ur_send.send(send_string.encode())
        data = self.wait()

        return data

    def speed(self, x=0, y=0, z=0, rx=0, ry=0, rz=0, acc=0.5, time=1):
        self.socket_ur_send.send(('speedl([' + str(x) + ',' + str(y) + ',' +
                                  str(z) + ',' + str(rx) + ',' + str(ry) +
                                  ',' + str(rz) + '],' + str(acc) + ',' +
                                  str(time) + ')\n').encode())
        self.wait()

    def set_home(self, pos, angle):
        self.home_position = pos
        self.home_angle = angle

    def home(self):
        self.move(*(self.home_position), *(self.home_angle))

    def read(self):
        data = self.communication_thread.data
        # Removing last entry: empty due to fenceposting in sending process
        data_split = data.split(';')[:-1]
        for item in data_split:
            data_point, data_value = item.split(':')
            self.ur_data[data_point] = float(data_value)

    def wait(self):
        # Hold-off to let the robot start movement before using data
        time.sleep(0.1)
        controller_time = 0

        # OBS
        data = []

        if config_ur.CONTROLLER_VERSION < 3.2:
            v_b_signal = list(np.ones(20))
            v_s_signal = list(np.ones(20))
            v_e_signal = list(np.ones(20))
            v_w1_signal = list(np.ones(20))
            v_w2_signal = list(np.ones(20))
            v_w3_signal = list(np.ones(20))

        while True:
            self.read()

            if controller_time != self.ur_data['time']:
                controller_time = self.ur_data['time']
            else:
                if int(str(config_ur.SOCKETS['port send'])[-1]) >= 3:
                    time.sleep(1/1000)
                else:
                    time.sleep(1/20)
                continue

            if config_ur.CONTROLLER_VERSION >= 3.2:
                if self.ur_data['status'] == 1:
                    break
            else:
                # OBS
                data.append([self.ur_data['v_b'], self.ur_data['v_s'], self.ur_data['v_e'], 
                             self.ur_data['v_w1'], self.ur_data['v_w2'], self.ur_data['v_w3'],
                             self.ur_data['robot_mode']])

                v_b_signal, v_b_mean = self.moving_average(v_b_signal, self.ur_data['v_b'])
                v_s_signal, v_s_mean = self.moving_average(v_s_signal, self.ur_data['v_s'])
                v_e_signal, v_e_mean = self.moving_average(v_e_signal, self.ur_data['v_e'])
                v_w1_signal, v_w1_mean = self.moving_average(v_w1_signal, self.ur_data['v_w1'])
                v_w2_signal, v_w2_mean = self.moving_average(v_w2_signal, self.ur_data['v_w2'])
                v_w3_signal, v_w3_mean = self.moving_average(v_w3_signal, self.ur_data['v_w3'])
                
                if(abs(v_b_mean) < config_ur.VELOCITY_MEAN_THRESHOLD and
                   abs(v_s_mean) < config_ur.VELOCITY_MEAN_THRESHOLD and
                   abs(v_e_mean) < config_ur.VELOCITY_MEAN_THRESHOLD and
                   abs(v_w1_mean) < config_ur.VELOCITY_MEAN_THRESHOLD and
                   abs(v_w2_mean) < config_ur.VELOCITY_MEAN_THRESHOLD and
                   abs(v_w3_mean) < config_ur.VELOCITY_MEAN_THRESHOLD):
                    time.sleep(0.05) # Give time for velocities to reach 0
                    break

        return data

    def moving_average(self, signal, new_point):
        if new_point > 1e5:
            new_point = 0
        new_signal = signal[1:] + [new_point]
        average = sum(new_signal)/len(new_signal)

        return new_signal, average

    def send_line(self, _str):
        if type(_str) is str:
            self.socket_ur_send.send(_str.encode())
        elif type(_str) is bytes:
            self.socket_ur_send.send(_str)
        else:
            print("Input to send_line must be of type str or type bytes")

    def shutdown(self):
        self.communication_thread.shutdown()
