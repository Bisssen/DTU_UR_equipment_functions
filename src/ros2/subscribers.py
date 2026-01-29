from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from ..utils import quaternion_to_euler
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode


class Ros2Subscribers():
    def __init__(self, node: 'URNode') -> None:
        self.node = node

        # Change to service
        self.payload_setter_subscriber =\
        self.node.create_subscription(
            Float32,
            'set_payload',
            self.payload_setter_callback,
            10
        )

        self.pose_command_subscriber =\
            self.node.create_subscription(
                Pose,
                'set_ur_pose',
                self.pose_command_callback,
                10
            )
        
        self.joints_command_subscriber =\
            self.node.create_subscription(
                JointState,
                'set_ur_joints',
                self.joints_command_callback,
                10
            )

    def pose_command_callback(self, msg: Pose) -> None:
        '''
        Send the robot to a specific position.
        '''
        wrist_angles = quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
            )
        
        self.node.ur.move(
            msg.position.x,
            msg.position.y,
            msg.position.z,
            wrist_angles[0],
            wrist_angles[1],
            wrist_angles[2],
            mode='linear',
            transform=False,
            wait=False
        )
    
    def joints_command_callback(self, msg: JointState) -> None:
        '''
        Send the ur to a specific joints state
        '''
        # Format joints list similar to actions.py execute_callback
        # The path function expects: [joint1, joint2, ..., joint6, 'j', time]
        joints_list = list(msg.position)
        joints_list.append('j')
        joints_list.append(0.05)  # Time parameter (same as in actions.py)

        self.node.get_logger().info(f'Received joint command to position: {list(msg.position)} from position {self.node.ur.get_joints()}')

        # Execute the path with the formatted joints list
        self.node.ur.path(
            [joints_list],  # Wrap in list as path expects a list of waypoints
            False,
            False,
            None,  # Acceleration (None means use default value)
            None,  # Speed (None means use default value)
            False
        )

    def payload_setter_callback(self, msg: Float32) -> None:
        payload = msg.data
        self.node.ur.set_payload_weight(payload)
