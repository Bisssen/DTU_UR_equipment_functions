import time
from math import pi
import numpy as np
import socket
import sys

from . import config_ur
from .communication_ur import communication_thread


class UR:
    def __init__(self, ip=None, port=None):
        # Whether the program is run in python 2 or not
        self.python_2 = (sys.version_info.major == 2)

        # Transformation to task
        self.task_transform = None
        if TRANSFORM in config_ur.__dict__:
            self.set_task_transform(config_ur.TRANSFORM['p0i'],
                                    config_ur.TRANSFORM['pxi'],
                                    config_ur.TRANSFORM['pyi'])
        else:
            print('UR: "TRANSFORM" has not been set: task2base and base2task transforms are not available.')

        # The default pose of the end effector
        self.home_pose = None
        if HOME_POSE in config_ur.__dict__:
            self.set_home(pose=config_ur.HOME_POSE)
        else:
            print('UR: "HOME_POSE" has not been set: home functionality is not available.')

        # The denavit hartenberg parameters to find forward kinematics
        self.dh = None
        if DH_PARAMETERS in config_ur.__dict__:
            self.set_dh_parameters(config_ur.DH_PARAMETERS['a'],
                                   config_ur.DH_PARAMETERS['d'],
                                   config_ur.DH_PARAMETERS['alpha'])
        else:
            print('UR: "DH_PARAMETERS" have not been set: forward kinematics are not available.')

        # The denavit hartenberg parameters to find forward kinematics
        self.default_orientation = None
        if DEFAULT_ORIENTATION in config_ur.__dict__:
            self.set_default_orientation(config_ur.DEFAULT_ORIENTATION)
        else:
            print('UR: "DEFAULT_ORIENTATION" have not been set: default orientation in home function not available.')

        # Dictionary containing all the ur data which have been reading
        self.ur_data = {}

        # Connecting socket directly to robot
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # If no ip is provided, then use default
        if ip is None:
            self.ip = config_ur.IP
        else:
            self.ip = ip

        # If no port is provided, then use default
        if port is None:
            self.port = config_ur.PORT
        else:
            self.port = port

        # Connect to the UR arm
        self.socket.connect((self.ip, self.port))
        # Starting communication script
        self.communication_thread = communication_thread(self.ip, self.port)

        # Make sure that the communication thread have started receiving data
        while len(self.ur_data) == 0:
            self.read()
        
        print('UR: UR has been initiated.')

    def set_task_transform(self, p0i, pxi, pyi):
        p0 = np.array(p0i)
        px = np.array(pxi)
        py = np.array(pyi)
        p0 = p0 / 1000.
        px = px / 1000.
        py = py / 1000.
        vx = px - p0
        vy = py - p0
        vx = vx / np.linalg.norm(vx)
        vy = vy / np.linalg.norm(vy)
        vz = np.cross(vx, vy)
        vy = np.cross(vz, vx)
        
        self.task_transform = np.identity(4)
        self.task_transform[:3,:3] = np.transpose( np.array([vx, vy, vz]) )
        self.task_transform[:3] = p0

    def transform_base2task(self, x, y, z):
        if self.task_transform:
            return self.task_transform.dot( [x, y, z, 1] )[:3]
        else:
            print('UR: Task transform has not been set.')
            return None

    def transform_task2base(self, x, y, z):
        if self.task_transform:
            return np.linalg.inv(self.task_transform).dot( [x, y, z, 1] )[:3]
        else:
            print('UR: Task transform has not been set.')
            return None

    def set_tcp(self, x=0, y=0, z=0, rx=0, ry=0, rz=0):
        self.socket.send((f'set_tcp(p[{x},{y},{z},{rx},{ry},{rz}])\n').encode())
        time.sleep(0.1)

    def set_dh_parameters(self, a, d, alpha):
        self.dh = DH(a=a, d=d, alpha=alpha)

    def set_default_orientation(self, orientation):
        self.default_orientation = orientation

    def set_home(self, pose):
        self.home_pose = pose

    def get_position(self, world=True):
        x, y, z, _, _, _ = self.get_pose()
        if world:
            return self.transform_base2task(x, y, z)
        else:
            return (x, y, z)

    def get_pose(self):
        self.read()
        # The older version have the position values in a different place
        if (self.communication_thread.message_size >=
                config_ur.MESSAGE_SIZE_TO_VERSION['3.0']):
            x = self.ur_data['x_actual']
            y = self.ur_data['y_actual']
            z = self.ur_data['z_actual']
            rx = self.ur_data['rx_actual']
            ry = self.ur_data['ry_actual']
            rz_current = self.ur_data['rz_actual']
        else:
            x = self.ur_data['x']
            y = self.ur_data['y']
            z = self.ur_data['z']
            rx = self.ur_data['rx']
            ry = self.ur_data['ry']
            rz = self.ur_data['rz']
        return [x, y, z, rx, ry, rz]

    def get_joints(self):
        self.read()
        b = self.ur_data['b']
        s = self.ur_data['s']
        e = self.ur_data['e']
        w1 = self.ur_data['w1']
        w2 = self.ur_data['w2']
        w3 = self.ur_data['w3']
        return [b, s, e, w1, w2, w3]

    def move(self, x=None, y=None, z=None, rx=None, ry=None, rz=None, 
                   b=None, s=None, e=None, w1=None, w2=None, w3=None, 
                   pose=None, mode='linear', transform=True, relative=False,
                   acc=0.5, speed=0.1, wait=False):
        if mode[0] not in ['l', 'j']:
            print('UR: "mode" must be either \'l\', \'linear\', \'j\' or \'joint\'')
            return

        if pose:
            if len(pose) != 6:
                print('UR: "pose" must consist of exactly 6 values.')
                return
        else:
            if mode[0] == 'l':
                if None in [x, y, z, rx, ry, rz]:
                    print('UR: "x", "y", "z", "rx", "ry" and "rz" must all be defined when not using "pose".')
                    return
                if None in [rx, ry, rz]:
                    if self.default_orientation:
                        rx, ry, rz = self.default_orientation
                    else:
                        print('UR: Default orientation has not been set.')
                        return
                pose = [x, y, z, rx, ry, rz]
            elif mode[0] == 'j':
                if None in [b, s, e, w1, w2, w3]:
                    print('UR: "b", "s", "e", "w1", "w2" and "w3" must all be defined when not using "pose".')
                    return
                pose = [b, s, e, w1, w2, w3]

        if transform and mode[0] == 'l':
            pose[:3] = self.transform_task2base(*pose[:3])

        if relative:
            if mode[0] == 'l':
                current_pose = np.asarray(self.get_pose())
            if mode[0] == 'j':
                current_pose = np.asarray(self.get_joints())
            pose = (np.asarray(pose) + current_pose).tolist()

        self.socket.send((f'move{mode[0]}({pose},{acc},{speed})\n').encode())
        if wait:
            self.wait()

    def move_tool(self, x=0, y=0, z=0, rx=0, ry=0, rz=0, acc=1, speed=0.1,
                  wait=False):
        self.socket.send((f'movel(pose_trans(get_forward_kin(),p[{x},{y},{z},{rx},{ry},{rz}]),{acc},{speed})\n').encode())
        if wait:
            self.wait()

    def home(self):
        if self.home_pose:
            self.move(pose=self.home_pose)
        else:
            print('UR: Home pose has not been set.')

    def speed(self, x=0, y=0, z=0, rx=0, ry=0, rz=0, 
                    b=0, s=0, e=0, w1=0, w2=0, w3=0, 
                    pose=None, mode='linear', transform=True,
                    acc=0.5, time=1, wait=False):
        if pose:
            if len(pose) != 6:
                print('UR: "pose" must consist of exactly 6 values.')
                return
        else:
            if mode[0] == 'l':
                pose = [x, y, z, rx, ry, rz]
            elif mode[0] == 'j':
                pose = [b, s, e, w1, w2, w3]

        if transform and mode[0] == 'l':
            t = self.task_transform[:3,3]
            v_task = np.array([x, y, z, 1])
            v_base = T.dot(v_task)
            pose[:3] = v_base[:3] - t

        self.socket.send((f'speed{mode[0]}({pose},{acc},{time})\n').encode())
        if wait:
            self.wait()

    def speed_tool(self, x=0, y=0, z=0, acc=0.5, time=1):
        T = self.get_forward_kinematics()
        t = T[:3,3]
        v_tool = np.array([x, y, z, 1])
        v_base = T.dot(v_tool)
        v_speed = v_base[:3] - t
        self.speed(x=v_speed[0], y=v_speed[1], z=v_speed[2], acc=acc, time=time)

        # TODO: Test built-in pose_trans for speed_tool
        # send_string = f'movel(pose_trans(get_forward_kin(),p[{x},{y},{z},{rx},{ry},{rz}]),{acc},{speed})\n'

    def stop(self, acc=5, mode='linear'):
        self.socket.send((f'stop{mode[0]}({acc})\n').encode())

    def get_forward_kinematics(self):
        if self.dh:
            joints = self.get_joints()
            return self.dh.calculate_forward_kinematics(joints)
        else:
            print('UR: DH parameters have not been set.')
            return None

    def read(self):
        data = self.communication_thread.data
        # Removing last entry: empty due to fenceposting in sending process
        data_split = data.split(';')[:-1]
        for item in data_split:
            data_point, data_value = item.split(':')
            self.ur_data[data_point] = float(data_value)

    def moving_average(self, signal, new_point):
        if new_point > 1e5:
            new_point = 0
        new_signal = signal[1:] + [new_point]
        average = sum(new_signal)/len(new_signal)

        return new_signal, average

    def wait(self):
        # Hold-off to let the robot start movement before using data
        time.sleep(0.1)
        controller_time = 0

        if (self.communication_thread.message_size <
                config_ur.MESSAGE_SIZE_TO_VERSION['3.2']):
            velocity_series = [[1] * 20] * 6

        while True:
            self.read()

            # Test if new data have arrived
            if controller_time != self.ur_data['time']:
                controller_time = self.ur_data['time']
            else:
                # If not sleep the rate that is equal to
                # when the next new data should arive
                if int(str(self.port)[-1]) >= 3:
                    time.sleep(1/1000)
                else:
                    time.sleep(1/20)
                continue

            # If newer software then read the status directly
            if (self.communication_thread.message_size >=
                    config_ur.MESSAGE_SIZE_TO_VERSION['3.2']):
                if self.ur_data['status'] == 1:
                    break
            # Otherwise check if the arm is still moving
            else:
                current_velocities = [self.ur_data['v_b'],
                                      self.ur_data['v_s'],
                                      self.ur_data['v_e'],
                                      self.ur_data['v_w1'],
                                      self.ur_data['v_w2'],
                                      self.ur_data['v_w3']]
                total_mean_velocity = 0
                for i, velocity in enumerate(velocity_series):
                    velocity_series[i], velocity_mean = self.moving_average(velocity, current_velocities[i])
                    total_mean_velocity += abs(velocity_mean)
                
                if total_mean_velocity < config_ur.VELOCITY_MEAN_THRESHOLD * 6:
                    break

    def send_line(self, _str):
        if type(_str) is str:
            self.socket.send(_str.encode())
        elif type(_str) is bytes:
            self.socket.send(_str)
        else:
            print('UR: Input to send_line must be of type str or type bytes')

    def shutdown(self):
        self.communication_thread.shutdown()


class DH:
    def __init__(self, a, d, alpha):
        self.a = a
        self.d = d
        self.alpha = alpha
    
    def calculate_forward_kinematics(self, joints):
        # Calculates forward kinematics for the robot based on joint values
        T = np.identity(4)
        for i in range(len(self.a)):
            M_i = np.array([[cos(joints[i]), -sin(joints[i]) * cos(self.alpha[i]),  sin(self.alpha[i]) * sin(joints[i]), self.a[i] * cos(joints[i])],
                            [sin(joints[i]),  cos(self.alpha[i]) * cos(joints[i]), -sin(self.alpha[i]) * cos(joints[i]), self.a[i] * sin(joints[i])],
                            [0,               sin(self.alpha[i]),                   cos(self.alpha[i]),                  self.d[i]                 ],
                            [0,               0,                                    0,                                   1                         ]])
            T = np.matmul(T, M_i)
        return T
