from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from control_msgs.action import FollowJointTrajectory
from rclpy.callback_groups import ReentrantCallbackGroup
import time

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode

class Ros2Actions():
    def __init__(self, node: 'URNode') -> None:
        self.node = node

        self.wait_for_stopping_timer = None

        self.follow_trajectory_action = ActionServer(
            self.node,
            FollowJointTrajectory,
            '/ur10/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback
        )

    def execute_callback(self, goal_handle: ServerGoalHandle) -> any:
        trajectory = goal_handle.request.trajectory

        joints_list = []        
        for point in trajectory.points:
            tmp_list = list(point.positions)
            tmp_list.append('j')
            tmp_list.append(0.05)
            joints_list.append(tmp_list)

        final_joints = list(joints_list[-1][:6])
        self.node.get_logger().info(f'Received movement command to joint position: {final_joints} from position {self.node.ur.get_joints()}')

        self.follow_joints_list(goal_handle, joints_list)

        self.node.get_logger().info(f'Done with movement')
    
        return FollowJointTrajectory.Result()

    def follow_joints_list(self, goal_handle: ServerGoalHandle, joints_list: list[float]) -> None:
        # Run the final part of the trajectory
        self.node.ur.path(
            joints_list,
            False,
            False,
            None,  # Accelertaion  (None means use default value)
            None,  # Speed  (None means use default value)
            False
        )

        # Keep the main loop running, but block the code until
        # The UR executes its movement.
        # self.node.ros2_timers.timer_main_loop_blocking(joints_list[-1][:6])
        while not self.node.ur.check_if_joints_is_reached(joints_list[-1][:6]):
            self.node.ros2_timers.timer_main_loop()
            ## Check cancelation
            if self.check_if_canceled(goal_handle):
                return

            self.publish_feedback(goal_handle)

        # Wait some extra time after the early detection to ensure the arm is stopped
        self.wait_for_stopping_timer = time.time()
        while self.wait_for_stopping_timer + 1 > time.time():
            self.node.ros2_timers.timer_main_loop()
            
        goal_handle.succeed()

    def check_if_canceled(self, goal_handle: ServerGoalHandle) -> bool:
        if goal_handle.is_cancel_requested:
            self.node.get_logger().info('Movement command canceled')
            goal_handle.canceled()
            return True

        return False

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> None:
        # Stop the arm
        self.node.ur.speed(0, 0, 0, 0, 0, 0, transform=False, acc=1.0, wait=False)
        return CancelResponse.ACCEPT  # Allow cancellation
    
    def publish_feedback(self, goal_handle: ServerGoalHandle) -> None:
        feedback_msg = FollowJointTrajectory.Feedback()
        feedback_msg.actual.positions = self.node.ur.get_pose()
        feedback_msg.actual.velocities = self.node.ur.get_pose_velocity(read=False)

        goal_handle.publish_feedback(feedback_msg)