from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode
from ..utils import euler_to_quaternion


class Ros2Publishers():
    def __init__(self, node: 'URNode') -> None:
        self.node = node
        self.ur_pose_publisher = self.node.create_publisher(Pose, 'ur_pose', 10)

        self.joint_positions_publisher = self.node.create_publisher(JointState, 'ur_joint_positions', 10)

        self.ur_moving_publisher = self.node.create_publisher(Bool, 'ur_moving', 10)

    def publish_ur_pose(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> None:
        pose = Pose()

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        q = euler_to_quaternion(roll, pitch, yaw)

        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        self.ur_pose_publisher.publish(pose)

    def publish_joint_positions(self, b: float, s: float, e: float,
                                w1: float, w2: float, w3: float) -> None:        
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = ['b', 's', 'e', 'w1', 'w2', 'w3']
        msg.position = [b, s, e, w1, w2, w3]
        self.joint_positions_publisher.publish(msg)

    def publish_robot_is_moving(self, is_moving):
        msg = Bool()
        msg.data = is_moving
        self.ur_moving_publisher.publish(msg)

# TODO make more publishers
# I guess I can make more now, but at some point I will need to know what they should look like
