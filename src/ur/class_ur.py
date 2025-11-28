import time
from math import pi, cos, sin
import numpy as np
import socket
import sys

from . import config_ur
from .communication_ur import communication_thread

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..ros2.node import URNode


class UR:
    def __init__(self, node: 'URNode', ip=None, port=None):
        self.node = node
        # Whether the program is run in python 2 or not
        self.python_2 = (sys.version_info.major == 2)

        self.non_blocking_start_time: float | None = None

        # Timer that keeps track of when the robot just started moving
        self.moving_timer: None|float = None

        self.stopping_timer: None|float = None
        self.stopping_time = 1.0

        # Default speed used for path generation
        self._default_path_speed: float = 0.1
        self._default_path_acceleration: float = 0.5

        # Counter that gives the path functions unique names
        self.path_counter = 0

        # Transformation to task
        self.task_transform = None
        if 'TRANSFORM' in config_ur.__dict__:
            self.set_task_transform(config_ur.TRANSFORM['p0i'],
                                    config_ur.TRANSFORM['pxi'],
                                    config_ur.TRANSFORM['pyi'])
        else:
            self.node.get_logger().info('UR: "TRANSFORM" has not been set: task2base and base2task transforms are not available.')

        # The default pose of the end effector
        self.home_pose = None
        if 'HOME_POSE' in config_ur.__dict__:
            self.set_home(pose=config_ur.HOME_POSE)
        else:
            self.node.get_logger().info('UR: "HOME_POSE" has not been set: home functionality is not available.')

        # The denavit hartenberg parameters to find forward kinematics
        self.default_orientation = None
        if 'DEFAULT_ORIENTATION' in config_ur.__dict__:
            self.set_default_orientation(config_ur.DEFAULT_ORIENTATION)
        else:
            self.node.get_logger().info('UR: "DEFAULT_ORIENTATION" has not been set: default orientation in home function not available.')

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

        self.node.get_logger().info(f'UR: Connecting to UR on ip: {self.ip}.')

        # Connect to the UR arm
        self.socket.connect((self.ip, self.port))
        # Starting communication script
        # THIS IS NOT a thread anymore
        self.communication_thread = communication_thread(self.ip, self.port)

        # Make sure that the communication thread have started receiving data
        while len(self.ur_data) == 0:
            self.read()
        
        self.node.get_logger().info('UR: UR is ready.')

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
        self.task_transform[:3,3] = p0

        self.rot_mat = np.array([vx, vy, vz])
        self.rot_mat = np.transpose(self.rot_mat)
        self.origin = p0

    def transform_task2base(self, x, y, z):
        if self.task_transform is not None:
            return self.task_transform.dot( [x, y, z, 1] )[:3]
        else:
            self.node.get_logger().info('UR: Task transform has not been set.')
            return None

    def transform_base2task(self, x, y, z):
        if self.task_transform is not None:
            return np.linalg.inv(self.task_transform).dot( [x, y, z, 1] )[:3]
        else:
            self.node.get_logger().info('UR: Task transform has not been set.')
            return None

    def set_tcp(self, x=0, y=0, z=0, rx=0, ry=0, rz=0):
        self.socket.send((f'set_tcp(p[{x},{y},{z},{rx},{ry},{rz}])\n').encode())
        time.sleep(0.1)

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

    def get_pose(self, read=True):
        if read:
            self.read()
        # The older version have the position values in a different place
        if (self.communication_thread.message_size >=
                config_ur.MESSAGE_SIZE_TO_VERSION['3.0']):
            x = self.ur_data[config_ur.X_ACTUAL]
            y = self.ur_data[config_ur.Y_ACTUAL]
            z = self.ur_data[config_ur.Z_ACTUAL]
            rx = self.ur_data[config_ur.RX_ACTUAL]
            ry = self.ur_data[config_ur.RY_ACTUAL]
            rz = self.ur_data[config_ur.RZ_ACTUAL]
        else:
            x = self.ur_data[config_ur.X]
            y = self.ur_data[config_ur.Y]
            z = self.ur_data[config_ur.Z]
            rx = self.ur_data[config_ur.RX]
            ry = self.ur_data[config_ur.RY]
            rz = self.ur_data[config_ur.RZ]
        return [x, y, z, rx, ry, rz]

    def get_pose_velocity(self, read=True) -> list[float]:
        if read:
            self.read()
        return [self.ur_data[config_ur.V_X],
                self.ur_data[config_ur.V_Y],
                self.ur_data[config_ur.V_Z],
                self.ur_data[config_ur.V_RX],
                self.ur_data[config_ur.V_RY],
                self.ur_data[config_ur.V_RZ]]

    def get_joints(self, read=True) -> list[float]:
        if read:
            self.read()
        b = self.ur_data[config_ur.B]
        s = self.ur_data[config_ur.S]
        e = self.ur_data[config_ur.E]
        w1 = self.ur_data[config_ur.W1]
        w2 = self.ur_data[config_ur.W2]
        w3 = self.ur_data[config_ur.W3]
        return [b, s, e, w1, w2, w3]
    
    def get_joints_velocity(self, read=True) -> list[float]:
        if read:
            self.read()
        return [self.ur_data[config_ur.V_B],
                self.ur_data[config_ur.V_S],
                self.ur_data[config_ur.V_E],
                self.ur_data[config_ur.V_W1],
                self.ur_data[config_ur.V_W2],
                self.ur_data[config_ur.V_W3]]

    def move(self, x=None, y=None, z=None, rx=None, ry=None, rz=None, 
                   b=None, s=None, e=None, w1=None, w2=None, w3=None, 
                   pose=None, mode='linear', transform=True, relative=False,
                   acc=0.5, speed=0.1, wait=False):
        pose = self.generate_move(x, y, z, rx, ry, rz,
                                  b, s, e, w1, w2, w3,
                                  pose, mode, transform, relative)

        print(f'move{mode[0]}(p{pose},{acc},{speed})\n')
        if mode[0] == 'j':
            self.socket.send((f'move{mode[0]}({pose},{acc},{speed})\n').encode())
        else:
            self.socket.send((f'move{mode[0]}(p{pose},{acc},{speed})\n').encode())
        
        self.moving_timer = time.time()
        if wait:
            self.wait()
    
    def generate_move(self, x=None, y=None, z=None, rx=None, ry=None, rz=None, 
                      b=None, s=None, e=None, w1=None, w2=None, w3=None, 
                      pose=None, mode='linear', transform=True, relative=False):
        if mode[0] not in ['l', 'j']:
            self.node.get_logger().error('UR: "mode" must be either \'l\', \'linear\', \'j\' or \'joint\'')
            return

        if pose:
            if len(pose) != 6:
                self.node.get_logger().error('UR: "pose" must consist of exactly 6 values.')
                return
        else:
            if relative:
                if mode[0] == 'l':
                    pose = [x, y, z, rx, ry, rz]
                elif mode[0] == 'j':
                    pose = [b, s, e, w1, w2, w3]
                pose = [0 if v is None else v for v in pose]
            else:
                if mode[0] == 'l':
                    if None in [rx, ry, rz]:
                        if self.default_orientation:
                            rx, ry, rz = self.default_orientation
                        else:
                            self.node.get_logger().error('UR: Default orientation has not been set.')
                            return
                    if None in [x, y, z, rx, ry, rz]:
                        self.node.get_logger().error('UR: "x", "y", "z" must all be defined when not using "pose".')
                        self.node.get_logger().error('    "rx", "ry" and "rz" must either be defined or default orientation be used.')
                        return
                    pose = [x, y, z, rx, ry, rz]
                elif mode[0] == 'j':
                    if None in [b, s, e, w1, w2, w3]:
                        self.node.get_logger().error('UR: "b", "s", "e", "w1", "w2" and "w3" must all be defined when not using "pose".')
                        return
                    pose = [b, s, e, w1, w2, w3]

        if relative:
            if mode[0] == 'l':
                current_pose = np.asarray(self.get_pose())
            elif mode[0] == 'j':
                current_pose = np.asarray(self.get_joints())

            if transform and mode[0] == 'l':
                current_pos = np.asarray(self.transform_base2task(*current_pose[:3]))
                pose[:3] = (current_pos + np.asarray(pose[:3])).tolist()
                pose[:3] = self.transform_task2base(*pose[:3])
                pose[3:] = current_pose[3:].tolist()
            else:
                pose = (np.asarray(pose) + current_pose).tolist()
        else:
            if transform and mode[0] == 'l':
                pose[:3] = self.transform_task2base(*pose[:3])

        return pose

    # Poses must contain the 6 positions and
    # a mode in the form linear, l or joint, j
    def path(self, poses, transform=True, relative=False,
             acc=0.5, speed=None, wait=False, r=0.05) -> None:
        if speed is None:
            speed = self._default_path_speed
        if acc is None:
            acc = self._default_path_acceleration
        
        if self.check_if_at_end_point(poses):
            return

        data = self.get_path_data(poses, transform, relative)
        if len(data) == 0:
            return
        # Send the actual commands that needs to be sent to move the path
        # Start of the function
        send_string = f'def follow_path_{self.path_counter}():\n'
        self.path_counter += 1

        second_last_point = self.get_second_last_point(poses, r)
        for i, pose in enumerate(data):
            # Skip the last points once we are close enough
            if second_last_point - 1 <= i:
                r = 0.0
                break
            send_string += self.generate_move_string(pose, acc, speed, r)

        # Add the final position
        send_string += self.generate_move_string(data[-1], acc, speed, r)
        send_string += 'end\n'

        self.send_line(send_string)

        self.moving_timer = time.time()
        if wait:
            self.wait()
    
    def get_path_data(
            self,
            poses: list[float | str],
            transform: bool,
            relative: bool) -> list[float | str]:
        '''
        Generates the right pose data, based on the transform and relative flag
        '''
        # List containing all the valid data
        data = []
        # Convert the poses to the correct data
        for pose in poses:
            if pose[7] is None:
                pose[7] = False
            if pose[6][0] == 'l':
                data.append([self.generate_move(x=pose[0], y=pose[1], z=pose[2],
                                                rx=pose[3], ry=pose[4], rz=pose[5],
                                                mode=pose[6], transform=transform,
                                                relative=relative), 'l', pose[7]])
            elif pose[6][0] == 'j':
                data.append([self.generate_move(b=pose[0], s=pose[1], e=pose[2],
                                                w1=pose[3], w2=pose[4], w3=pose[5],
                                                mode=pose[6], transform=transform,
                                                relative=relative), 'j', pose[7]])
            else:
                self.node.get_logger().error('UR: "mode" must be either \'l\', \'linear\', \'j\' or \'joint\'')
                return data
        return data
    
    def generate_move_string(
            self,
            pose: list[float | str],
            acc: float,
            speed: float,
            r: float) -> str:
        if pose[1] == 'j':
            send_string = f'    move{pose[1]}({pose[0]},{acc},{speed},r={r})\n'
        else:
            send_string = f'    move{pose[1]}(p{pose[0]},{acc},{speed},r={r})\n'
        return send_string
    
    def set_payload_weight(self, weight: float) -> None:
        command = 'def set_payload():\n' +\
                  f'    set_payload_mass({weight})\n' +\
                  'end\n'
        self.send_line(command)


    def move_tool(self, x=0, y=0, z=0, rx=0, ry=0, rz=0, acc=1, speed=0.1,
                  wait=False):
        self.socket.send((f'movel(pose_trans(get_forward_kin(),p[{x},{y},{z},{rx},{ry},{rz}]),{acc},{speed})\n').encode())
        self.moving_timer = time.time()
        if wait:
            self.wait()

    def home(self, acc=0.5, speed=0.1, wait=False):
        if self.home_pose:
            self.move(pose=self.home_pose, acc=acc, speed=speed, wait=wait)
        else:
            self.node.get_logger().error('UR: Home pose has not been set.')

    def speed(self, x=0, y=0, z=0, rx=0, ry=0, rz=0, 
                    b=0, s=0, e=0, w1=0, w2=0, w3=0, 
                    pose=None, mode='linear', transform=True,
                    acc=0.5, duration=1, wait=False):
        if pose:
            if len(pose) != 6:
                self.node.get_logger().error('UR: "pose" must consist of exactly 6 values.')
                return
        else:
            if mode[0] == 'l':
                pose = [x, y, z, rx, ry, rz]
            elif mode[0] == 'j':
                pose = [b, s, e, w1, w2, w3]

        if transform and mode[0] == 'l':
            t = self.task_transform[:3,3]
            v_task = np.array([x, y, z, 1])
            v_base = self.task_transform.dot(v_task)
            pose[:3] = v_base[:3] - t

        self.socket.send((f'speed{mode[0]}({pose},{acc},{duration})\n').encode())
        self.moving_timer = time.time()
        if wait:
            self.wait()

    def speed_tool(self, x=0, y=0, z=0, acc=0.5, time=1):
        T = self.get_forward_kinematics()
        t = T[:3,3]
        v_tool = np.array([x, y, z, 1])
        v_base = T.dot(v_tool)
        v_speed = v_base[:3] - t
        self.speed(x=v_speed[0], y=v_speed[1], z=v_speed[2], acc=acc, time=time)
        self.moving_timer = time.time()

    def stop(self, acc=5, mode='linear', wait=False):
        self.socket.send((f'stop{mode[0]}({acc})\n').encode())
        self.moving_timer = time.time()
        if wait:
            self.wait()

    def get_forward_kinematics(self):
        '''
        I don't think this works?
        '''
        return pose_to_transmat(self.get_pose())

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
        ## NB THE WAIT function does not work with the current ros implementation
        # Hold-off to let the robot start movement before using data
        time.sleep(0.1)
        controller_time = 0

        if (self.communication_thread.message_size <
                config_ur.MESSAGE_SIZE_TO_VERSION['3.2']):
            velocity_series = [[1] * 20] * 6

        while True:
            self.read()

            # Test if new data have arrived
            if controller_time != self.ur_data[config_ur.TIME]:
                controller_time = self.ur_data[config_ur.TIME]
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
                current_velocities = [self.ur_data[config_ur.V_B],
                                      self.ur_data[config_ur.V_S],
                                      self.ur_data[config_ur.V_E],
                                      self.ur_data[config_ur.V_W1],
                                      self.ur_data[config_ur.V_W2],
                                      self.ur_data[config_ur.V_W3]]
                total_mean_velocity = 0
                for i, velocity in enumerate(velocity_series):
                    velocity_series[i], velocity_mean = self.moving_average(velocity, current_velocities[i])
                    total_mean_velocity += abs(velocity_mean)
                
                if total_mean_velocity < config_ur.VELOCITY_MEAN_THRESHOLD * 6:
                    break

    def wait_non_blocking(self) -> bool:
        '''
        This is not updated to work with pre V3.2 software.
        Returns True if the robot is still moving
        and otherwise returns False

        This is NOT meant to be a tool for checking if the robot is still moving
        It mimics the wait function in a non blocking manner
        This means that after the start time is initialized, it will assume the robot
        is moving for 0.1 seconds 

        I guess this is not needed any more, but keep it until the new one is tested
        '''
        
        # initialize the start time
        if self.non_blocking_start_time is None:
            self.non_blocking_start_time = time.time()
        
        # Wait a bit before checking the moving flag, to ensure it is updated
        # There will be some delay from sending move command -> robot moves -> flag gets updated -> flag gets received
        # This is especially true for the pre 3.2 speed check, but it is not currently implemented
        if self.non_blocking_start_time + 0.1 > time.time():
            return True
        
        # TODO could also implement a timeout but whatever

        # Make sure the status flag is up to date
        self.read()

        if not self.ur_data['status'] == 1:
            # Robot is still moving
            return True
    

        self.non_blocking_start_time = None
        return False

    def is_moving(self) -> bool:
        # If the moving timer is not, set then something is wrong
        # but rely on the data to check if the robot is moving
        if self.moving_timer is None:
            return self.check_ur_if_moving()
        
        # Wait until the robot have been moving a little bit before checking the 
        # ur data to ensure the data is updated
        if self.moving_timer + 1.0 > time.time():
            return True
        
        return self.check_ur_if_moving()


    def check_ur_if_moving(self) -> bool:
        self.read()
        # If newer software then read the status directly
        if (self.communication_thread.message_size >=
                config_ur.MESSAGE_SIZE_TO_VERSION['3.2']):
            if self.ur_data['status'] == 1:
                return False
        # Otherwise check if the arm is still moving
        # This does not work very well. It is possible to tune it if
        # and use it if the arm only moves fast, but it is very difficult
        # to get it right if the arm is moving slowly
        # Use check if joint is reached instead
        else:
            current_velocities = [self.ur_data[config_ur.V_B],
                                  self.ur_data[config_ur.V_S],
                                  self.ur_data[config_ur.V_E],
                                  self.ur_data[config_ur.V_W1],
                                  self.ur_data[config_ur.V_W2],
                                  self.ur_data[config_ur.V_W3]]
            total_mean_velocity = 0
            for vel in current_velocities:
                total_mean_velocity += abs(vel)
            total_mean_velocity *= 1/6

            if total_mean_velocity > config_ur.VELOCITY_MEAN_THRESHOLD:
                self.stopping_timer = None
                return True

            if self.stopping_timer is None:
                self.stopping_timer = time.time()

            if self.stopping_timer + self.stopping_time > time.time():
                return True

            return False
        
        return True

    def check_if_pos_is_reached(self, pos: list[float], tolerance: float = 0.1) -> bool:
        current_pos = self.get_pose()

        pos_is_reached = True
        for current_point, desired_point in zip(current_pos, pos):
            if abs(current_point - desired_point) > tolerance:
                pos_is_reached = False
                break

        return pos_is_reached

    def check_if_joints_is_reached(self, joints: list[float], tolerance: float = 0.1) -> bool:
        current_joints = self.get_joints()

        joints_is_reached = True
        for current_joints, desired_joints in zip(current_joints, joints):
            if abs(current_joints - desired_joints) > tolerance:
                joints_is_reached = False
                break

        return joints_is_reached

    def send_line(self, _str):
        self.stopping_timer = None
        if type(_str) is str:
            self.socket.send(_str.encode())
        elif type(_str) is bytes:
            self.socket.send(_str)
        else:
            self.node.get_logger().error('UR: Input to send_line must be of type str or type bytes')

    def shutdown(self):
        self.communication_thread.shutdown()

    def get_second_last_point(self, joints_list: list[list[float]], r: float)-> int:
        '''
        Calculates and returns the index of the last point in joints_list
        that can be reached without skipping the last point with the given
        r value
        '''
        end_point = fwdkin(joints_list[-1])[0]
        for i, joints in enumerate(joints_list):
            current_point = fwdkin(joints)[0]

            if distance_3d_squared(end_point, current_point) < r**2:
                return i
        
        return len(joints_list) - 1
    
    def check_if_at_end_point(self, joints_list: list[list[float]]) -> bool:
        '''
        Returns true if the first and last position in joints_list is the same
        '''
        for end_joint, start_joint in zip(joints_list[-1], joints_list[0]):
            if not(end_joint == start_joint):
                return False
        return True

    def set_default_path_speed(self, speed: float) -> None:
        self._default_path_speed = speed

    def set_default_path_acceleration(self, acceleration: float) -> None:
        self._default_path_acceleration = acceleration

# Both DH and fwdkin works to get forward kinematics
# DH is a bit more of a general implementation where
# You provide the robot specifications
# while fwdkin is hardcoded
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

    def get_forward_kinematics(self, joints: list[float]) -> tuple[list[float]]:
        T = self.calculate_forward_kinematics(joints)
        return T[0:3, 3], rotation_matrix_to_rodrigues(T[0:3, 0:3])

def rodrigues_vec_to_rotation_mat(rodrigues_vec):
    theta = np.linalg.norm(rodrigues_vec)
    if theta < sys.float_info.epsilon:              
        rotation_mat = np.eye(3, dtype=float)
    else:
        r = rodrigues_vec / theta
        r0, r1, r2 = r
        I = np.eye(3, dtype=float)
        r_rT = np.array([[r0*r0, r0*r1, r0*r2],
                         [r1*r0, r1*r1, r1*r2],
                         [r2*r0, r2*r1, r2*r2]])
        r_cross = np.array([[0, -r2, r1],
                            [r2, 0, -r0],
                            [-r1, r0, 0]])
        rotation_mat = cos(theta) * I + (1 - cos(theta)) * r_rT + sin(theta) * r_cross
    return rotation_mat


def pose_to_transmat(pose):
    M = np.identity(4) # initialize
    M[:3,:3] = rodrigues_vec_to_rotation_mat(pose[3:]) # rotation part
    M[:3,3] = np.transpose(pose[:3]) # translation part
    return M


def distance_3d_squared(point1: list[float], point2: list[float]) -> float:
    return (point2[0] - point1[0])**2 +\
           (point2[1] - point1[1])**2 +\
           (point2[2] - point1[2])**2


def fwdkin(v, degrees=False):
    '''
    Return the forwards kinematics of a UR10
    '''
    if degrees:
        v1 = v[0]/180 * np.pi
        v2 = v[1]/180 * np.pi
        v3 = v[2]/180 * np.pi
        v4 = v[3]/180 * np.pi
        v5 = v[4]/180 * np.pi
        v6 = v[5]/180 * np.pi
    else:
        v1 = v[0]
        v2 = v[1]
        v3 = v[2]
        v4 = v[3]
        v5 = v[4]
        v6 = v[5]
    ## UR5
    a = [0.00000, -0.42500, -0.39243,  0.00000,  0.00000,  0.0000]
    d = [0.08920,  0.00000,  0.00000,  0.10900,  0.09300,  0.0820]
    ## UR10
    a = [0.00000, -0.612, -0.5723,  0.00000,  0.00000,  0.0000]
    d = [0.1273,  0.00000,  0.00000,  0.163941,  0.1157,  0.0922]
    ## Can be found here: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
    # for different robot arms
    T12 = [[np.cos(v1), 0, np.sin(v1), 0],
           [np.sin(v1), 0, -np.cos(v1), 0],
           [0, 1, 0, d[0]], [0, 0, 0, 1]]
    T23 = [[np.cos(v2), -np.sin(v2), 0, a[1] * np.cos(v2)],
           [np.sin(v2), np.cos(v2), 0, a[1] * np.sin(v2)],
           [0, 0, 1, 0], [0, 0, 0, 1]]
    T34 = [[np.cos(v3), -np.sin(v3), 0, a[2] * np.cos(v3)],
           [np.sin(v3), np.cos(v3), 0, a[2] * np.sin(v3)],
           [0, 0, 1, 0], [0, 0, 0, 1]]
    T45 = [[np.cos(v4), 0, np.sin(v4), 0],
           [np.sin(v4), 0, -np.cos(v4), 0],
           [0, 1, 0, d[3]], [0, 0, 0, 1]]
    T56 = [[np.cos(v5), 0, -np.sin(v5), 0],
           [np.sin(v5), 0, np.cos(v5), 0],
           [0, -1, 0, d[4]], [0, 0, 0, 1]]
    T67 = [[np.cos(v6), -np.sin(v6), 0, 0],
           [np.sin(v6), np.cos(v6), 0, 0],
           [0, 0, 1, d[5]], [0, 0, 0, 1]]
    T = np.matmul(T12, T23)
    T = np.matmul(T, T34)
    T = np.matmul(T, T45)
    T = np.matmul(T, T56)
    T = np.matmul(T, T67)

    Tnew = T[0:3, 0:3]
    rvec = rotation_matrix_to_rodrigues(Tnew)

    return T[0:3, 3], rvec


def rotation_matrix_to_rodrigues(R: np.ndarray) -> np.ndarray:
    """
    Convert a 3x3 rotation matrix to a Rodrigues rotation vector.
    
    Parameters:
        R (np.ndarray): 3x3 rotation matrix.
    
    Returns:
        np.ndarray: 3x1 Rodrigues rotation vector.

    From ChatGTP but works
    """
    # Ensure R is a numpy array
    R = np.asarray(R)
    
    # Compute angle
    theta = np.arccos((np.trace(R) - 1) / 2.0)
    
    if np.isclose(theta, 0):
        return np.zeros(3)  # No rotation
    
    # Compute axis
    rx = (R[2,1] - R[1,2]) / (2*np.sin(theta))
    ry = (R[0,2] - R[2,0]) / (2*np.sin(theta))
    rz = (R[1,0] - R[0,1]) / (2*np.sin(theta))
    
    axis = np.array([rx, ry, rz])
    
    # Rodrigues vector = axis * angle
    return axis * theta
