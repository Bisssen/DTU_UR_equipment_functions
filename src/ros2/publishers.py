
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode

from geometry_msgs.msg import Pose
from ..utils import euler_to_quaternion


class Ros2Publishers():
    def __init__(self, node: 'URNode'):
        self.node = node


        self.ur_pose_publisher = self.node.create_publisher(Pose, 'ur_pose', 10)


    def publish_ur_pose(self, x, y, z, roll, pitch, yaw):
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


