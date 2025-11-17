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

        final_joints = list(trajectory.points[-1])
        self.node.get_logger().info(f'Received movement command to joint position: {final_joints}')

        joints = []        
        for point in trajectory.points:
            tmp_list = list(point.positions)
            tmp_list.append('j')
            tmp_list.append(0.05)
            joints.append(tmp_list)
        
        max_pose_size = 100
        while len(joints) > max_pose_size:
            small_joints = joints[:max_pose_size]
            joints = joints[max_pose_size:]

            self.node.ur.path(
                small_joints,
                False,
                False,
                0.5,  # Accelertaion
                0.5,  # Speed
                False
            )

            self.node.ros2_timers.timer_main_loop_blocking(small_joints[-1][:6])

        # Run the final part of the trajectory
        self.node.ur.path(
            joints,
            False,
            False,
            0.5,  # Accelertaion
            0.5,  # Speed
            False
        )

        # Keep the main loop running, but block the code until
        # The UR executes its movement.
        self.node.ros2_timers.timer_main_loop_blocking(final_joints)

        self.node.get_logger().info(f'Done with movement')

        goal_handle.succeed()
    
        return FollowJointTrajectory.Result()

