from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode

from geometry_msgs.msg import Pose
from ..utils import quaternion_to_euler


class Ros2Subscribers():
    def __init__(self, node: 'URNode') -> None:
        self.node = node


        self.pose_command_subscriber = self.node.create