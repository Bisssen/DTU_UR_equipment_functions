import time
from math import pi
import numpy as np
import socket

import ur_config
from ur_communication import communication_thread


class Robot:
    def __init__(self):
        self.rotation_matrix = np.array([[0,0,0],[0,0,0],[0,0,0]])
        self.origin_task = 0
        self.transformInit(ur_config.TRANSFORM['p0i'],
                           ur_config.TRANSFORM['pxi'],
                           ur_config.TRANSFORM['pyi'])

        self.home_pos = ur_config.HOME['position']
        self.home_angle = ur_config.HOME['angle']

        self.robot_data = {}

        # Connecting socket directly to robot
        self.socket_robot_send = socket.socket(socket.AF_INET,
                                               socket.SOCK_STREAM)
        self.socket_robot_send.connect((ur_config.IP,
                                        ur_config.PORT))

        # Starting communication script
        self.communication_thread = communication_thread()
        time.sleep(2)

    def send_line(self, str_):
        self.socket_robot_send.send(str_)

    def transformInit(self, p0i, pxi, pyi):
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
        vz = np.cross(vx,vy)
        self.rotation_matrix = np.array([vx,vy,vz])
        self.rotation_matrix = np.transpose(self.rotation_matrix)
        self.origin_task = p0

    def transform(self, x, y, z):
        b = np.array([x,y,z])
        t = self.rotation_matrix.dot(np.transpose(b)) + self.origin_task
        return t

    def inverseTransform(self, x, y, z):
        b = np.array([x,y,z])
        it = np.transpose(self.rotation_matrix).dot(np.transpose((b - self.origin_task)))
        return it

    def set_tcp(self, x=0, y=0, z=0, rx=0, ry=0, rz=0):
        self.socket_robot_send.send(('set_tcp(p[' + str(x) + ',' + str(y) +\
                                     ',' + str(z) + ',' + str(rx) + ',' +\
                                     str(ry) + ',' + str(rz) + '])\n').encode())
        time.sleep(0.1)

    def getPosition(self, world=True):
        self.read()
        x = self.robot_data['x']
        y = self.robot_data['y']
        z = self.robot_data['z']
        if world:
            return self.inverseTransform(x, y, z)
        else:
            return (x, y, z)

    def move(self, x, y, z, rx=pi, ry=0, rz=0, acc=1, speed=0.1, transform=True):
        if transform:
            x,y,z = self.transform(x, y, z)
        self.socket_robot_send.send(('movel(p[' + str(x) + ',' + str(y) +
                                     ',' + str(z) + ',' + str(rx) + ',' +
                                     str(ry) + ',' + str(rz) + '],' + str(acc) +
                                     ',' + str(speed) + ')\n').encode())
        self.wait()

    def moveRelative(self, x=0, y=0, z=0, rx=0, ry=0, rz=0, acc=1, speed=0.1):
        self.read()
        self.move(self.robot_data['x'] + x, self.robot_data['y'] + y,
                  self.robot_data['z'] + z, self.robot_data['rx'] + rx,
                  self.robot_data['ry'] + ry,self.robot_data['rz'] + rz,
                  acc, speed, transform=False)

    def moveTool(self, x=0, y=0, z=0, rx=0, ry=0, rz=0, acc=1, speed=0.1):
        send_string = 'movel(pose_trans(get_forward_kin(),' +\
                      'p['+str(x)+','+str(y)+','+str(z)+','+str(rx)+','+str(ry)+','+str(rz)+']' +\
                      '),'+str(acc)+','+str(speed)+')\n'
        self.socket_robot_send.send(send_string.encode())
        self.wait()
        # To move 10cm along Z-axis and turn 90 degrees around Z-axis
        #global pose_wrt_tool = p[0,0,0.1,0,0 , 1.57]
        #global pose_wrt_base = pose_trans(get_forward_kin(), pose_wrt_tool)
        #movel( pose_wrt_base , a=1.2, v=0.25)
        # get_forward_kin() returns current pose.

    def speed(self, x=0, y=0, z=0, rx=0, ry=0, rz=0, acc=0.5, time=1):
        self.socket_robot_send.send(('speedl([' + str(x) + ',' + str(y) + ',' +\
                                     str(z) + ',' + str(rx) + ',' + str(ry) +\
                                     ',' + str(rz) + '],' + str(acc) + ',' +\
                                     str(time) + ')\n').encode())
        self.wait()

    def setHome(self, pos, angle):
        self.home_pos = pos
        self.home_angle = angle

    def home(self):
        self.move(self.home_pos[0], self.home_pos[1], self.home_pos[2],\
		  self.home_angle[0], self.home_angle[1], self.home_angle[2])

    def read(self):
        data = self.communication_thread.data
        data_split = data.split(';')[:-1] # Removing last entry: empty due to fenceposting in sending process
        for item in data_split:
            data_point, data_value = item.split(':')
            self.robot_data[data_point] = float(data_value)

    def wait(self):
        time.sleep(0.1) # Give the robot status time to change to 2 = 'active'
        while True:
            time.sleep(0.1)
            self.read()
            if self.robot_data['status'] == 1:
                break

    def sendLine(self, _str):
        if type(_str) is str:
            self.socket_robot_send.send(_str.encode())
        elif type(_str) is bytes:
            self.socket_robot_send.send(_str)
        else:
            print("Input to sendLine must be of type str or type bytes")

    def shutdown(self):
        self.communication_thread.shutdown()
