from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode
from ..utils import euler_to_quaternion


class Ros2Publishers():
    def __init__(self, node: 'URNode') -> None:
        self.node = node
        self.ur_pose_publisher = self.node.create_publisher(JointState, 'ur_pose', 10)
        # self.ur_pose_velocity_publisher = self.node.create_publisher(Pose, 'ur_pose_velocity', 10)

        self.ur_joints_publisher = self.node.create_publisher(JointState, 'ur_joints', 10)
        # self.ur_joints_velocity_publisher = self.node.create_publisher(JointState, 'ur_joints_velocity', 10)

        self.is_ur_moving_publisher = self.node.create_publisher(Bool, 'is_ur_moving', 10)

    def publish_ur_pose(self,
                        pose: list[float],
                        velocity_list: list[float]) -> None:
        if not self.validate_list_size(pose, 6):
            return
        self.ur_pose_publisher.publish(
            self.convert_to_ros2_jointstate(
                pose,
                velocity_list,
                False
            )
        )
    
    # def publish_ur_pose_velocity(self, pose_velocity: list[float]) -> None:
    #     if not self.validate_list_size(pose_velocity, 6):
    #         return
    #     self.ur_pose_velocity_publisher.publish(self.convert_to_ros2_pose(pose_velocity))
    
    # def convert_to_ros2_pose(self, pose_list: list[float]) -> Pose:
    #     pose = Pose()

    #     pose.position.x = pose_list[0]
    #     pose.position.y = pose_list[1]
    #     pose.position.z = pose_list[2]

    #     q = euler_to_quaternion(pose_list[3], pose_list[4], pose_list[5])

    #     pose.orientation.x = q[0]
    #     pose.orientation.y = q[1]
    #     pose.orientation.z = q[2]
    #     pose.orientation.w = q[3]
    #     return pose

    def publish_ur_joints(self,
                          joints_list: list[float],
                          velocity_list: list[float]) -> None:
        if not self.validate_list_size(joints_list, 6):
            return
        self.ur_joints_publisher.publish(
            self.convert_to_ros2_jointstate(
                joints_list,
                velocity_list
            )
        )

    # def publish_ur_joints_velocity(self, joints_velocity_list: list[float]) -> None:
    #     if not self.validate_list_size(joints_velocity_list, 6):
    #         return
    #     self.ur_joints_velocity_publisher.publish(
    #         self.convert_to_ros2_jointstate(
    #             joints_velocity_list
    #         )
    #     )
    
    def convert_to_ros2_jointstate(self,
                                   jointstate_list: list[float],
                                   velocity_list: list[float],
                                   joint=True) -> JointState:
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        if joint:
            msg.name = ['b', 's', 'e', 'w1', 'w2', 'w3']
        else:
            msg.name = ['x', 'y', 'z', 'rx', 'ry', 'rz']
        msg.position = jointstate_list
        msg.velocity = velocity_list
        return msg

    def publish_is_ur_moving(self, is_moving):
        msg = Bool()
        msg.data = is_moving
        self.is_ur_moving_publisher.publish(msg)


    def validate_list_size(self, _list:list[float], size=6) -> bool:
        if not len(_list) == size:
            self.node.get_logger().error(
                f'Trying to publish list: {_list} '
                f'of size {len(_list)} '
                f'but it must be of size {size}.'
            )
            return False
        return True
# TODO make more publishers
# I guess I can make more now, but at some point I will need to know what they should look like
