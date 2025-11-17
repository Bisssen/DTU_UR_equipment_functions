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

        joints_list = []        
        for point in trajectory.points:
            tmp_list = list(point.positions)
            tmp_list.append('j')
            tmp_list.append(0.05)
            joints_list.append(tmp_list)

        final_joints = list(joints_list[-1][:6])
        self.node.get_logger().info(f'Received movement command to joint position: {final_joints}')

        max_pose_size = 10000000
        while len(joints_list) > max_pose_size:
            small_joints = joints_list[:max_pose_size]
            joints_list = joints_list[max_pose_size:]

            self.follow_joints_list(small_joints)

        
        self.follow_joints_list(joints_list)

        self.node.get_logger().info(f'Done with movement')

        goal_handle.succeed()
    
        return FollowJointTrajectory.Result()


    def follow_joints_list(self, joints_list: list[float]) -> None:
        # Run the final part of the trajectory
        self.node.ur.path(
            joints_list,
            False,
            False,
            0.5,  # Accelertaion
            0.5,  # Speed
            False
        )

        # Keep the main loop running, but block the code until
        # The UR executes its movement.
        self.node.ros2_timers.timer_main_loop_blocking(joints_list[-1][:6])