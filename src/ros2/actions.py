from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory

import threading

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode

class Ros2Actions():
    def __init__(self, node: 'URNode') -> None:
        self.node = node

        self.follow_trajectory_action = ActionServer(
            self.node,
            FollowJointTrajectory,
            '/ur10/follow_joint_trajectory',
            self.execute_callback
        )

    def execute_callback(self, goal_handle) -> None:
        trajectory = goal_handle.request.trajectory

        print('Received movement commeand')
        poses = []
        for i, point in enumerate(trajectory.points):
            tmp_list = list(point.positions)
            tmp_list.append('j')
            tmp_list.append(0.05)
            poses.append(tmp_list)
    
            if i == len(trajectory.points) - 1:
                print('----')
                print(point.positions)
                print('----')

        # Run the blocking function in a separate thread

        self.node.ur.path(
            poses,
            False,
            False,
            0.5,  # Accelertaion
            0.5,  # Speed
            False
        )

        # Keep the main loop running, but block the code until
        # The UR executes its movement.
        self.node.ros2_timers.timer_main_loop_blocking()

        print('done moving')

        goal_handle.succeed()
    
        return FollowJointTrajectory.Result()

