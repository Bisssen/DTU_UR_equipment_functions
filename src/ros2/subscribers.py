from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode

from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from ..utils import quaternion_to_euler


class Ros2Subscribers():
    def __init__(self, node: 'URNode') -> None:
        self.node = node


        self.payload_setter_subscriber =\
        self.node.create_subscription(
            Float32,
            'set_payload',
            self.payload_setter_callback,
            10
        )
    #     self.pose_command_subscriber =\
    #         self.node.create_subscription(
    #             Pose,
    #             'set_ur_pose',
    #             self.pose_command_callback,
    #             10
    #         )

    # def pose_command_callback(self, msg: Pose) -> None:

    #     msg.position.x
    #     msg.position.y
    #     msg.position.z
    #     msg.orientation.x
    #     msg.orientation.y
    #     msg.orientation.z
    #     msg.orientation.w

    def payload_setter_callback(self, msg: Float32) -> None:
        payload = msg.data
        self.node.ur.set_payload_weight(payload)


# TODO test trajectories. See if there kinda is a cap on the size
#... how should it look
