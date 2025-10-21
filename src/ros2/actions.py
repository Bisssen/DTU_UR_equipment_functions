# trajectory_server.py
import rclpy
from rclpy.action import ActionServer

from control_msgs.action import FollowJointTrajectory
from control_msgs.action import FollowJointTrajectory

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode

class Ros2Actions():
    def __init__(self, node: 'URNode'):
        self.node = node
        self._action_server = ActionServer(
            self.node,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory

        # 🔧 PLACE YOUR CUSTOM CODE HERE
        poses = []
        for point in trajectory.points:
            tmp_list = list(point.positions)
            tmp_list.append('j')
            tmp_list.append(None)
            poses.append(tmp_list)
            # Simulate execution delay
            # await self._simulate_execution(point)

        print(self.node.ur.path(poses, False, False))

        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        # Populate result if needed
        return result
