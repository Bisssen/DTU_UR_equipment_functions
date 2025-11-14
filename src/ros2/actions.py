from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory

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

        final_pose = list(trajectory.points[-1])
        self.node.get_logger().info(f'Received movement command to point: {final_pose}')

        poses = []        
        for point in trajectory.points:
            tmp_list = list(point.positions)
            tmp_list.append('j')
            tmp_list.append(0.05)
            poses.append(tmp_list)
        
        max_pose_size = 100
        while len(poses) > max_pose_size:
            small_poses = poses[:max_pose_size]
            poses = poses[max_pose_size:]

            self.node.ur.path(
                small_poses,
                False,
                False,
                0.5,  # Accelertaion
                0.5,  # Speed
                False
            )

            self.node.ros2_timers.timer_main_loop_blocking(small_poses[-1][:6])

        # Run the final part of the trajectory
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
        self.node.ros2_timers.timer_main_loop_blocking(final_pose)

        self.node.get_logger().info(f'Done with movement')

        goal_handle.succeed()
    
        return FollowJointTrajectory.Result()

