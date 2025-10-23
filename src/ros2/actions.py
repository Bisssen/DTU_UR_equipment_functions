from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory

import asyncio
from concurrent.futures import ThreadPoolExecutor

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode

class Ros2Actions():
    def __init__(self, node: 'URNode') -> None:
        self.node = node
        self.executor = ThreadPoolExecutor()

        self.follow_trajectory_action = ActionServer(
            self.node,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle) -> None:
        trajectory = goal_handle.request.trajectory

        poses = []
        for point in trajectory.points:
            tmp_list = list(point.positions)
            tmp_list.append('j')
            tmp_list.append(None)
            poses.append(tmp_list)

        await asyncio.get_event_loop().run_in_executor(
            self.executor,
            self.node.ur.path,
            poses,
            False,
            False,
            0.5,  # Accelertaion
            0.1,  # Speed
            True
        )

        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        # Populate result if needed
        return result
