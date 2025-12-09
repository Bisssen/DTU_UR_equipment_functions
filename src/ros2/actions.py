from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from control_msgs.action import FollowJointTrajectory
from rclpy.callback_groups import ReentrantCallbackGroup
from ur_message_types.action import Move
import time

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode


class Ros2Actions():
    def __init__(self, node: 'URNode') -> None:
        self.pose_move_to_action = MoveToPoseAction(node)
        self.joint_move_to_action = MoveToJointAction(node)
        self.pose_tracjectory_action = PoseTrajectoryAction(node)
        self.joint_tracjectory_action = JointTracjectoryAction(node)

class BaseAction():
    def __init__(self, node: 'URNode') -> None:
        self.node = node

        self.wait_for_stopping_timer = None

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
    
    def wait_extra(self, extra_time=1) -> None:
        # Wait some extra time after the early detection to ensure the arm is stopped
        self.wait_for_stopping_timer = time.time()
        while self.wait_for_stopping_timer + extra_time > time.time():
            self.node.ros2_timers.timer_main_loop()


class MoveToPoseAction(BaseAction):
    def __init__(self, node: 'URNode') -> None:
        super().__init__(node)

        self.move_to_pose_action = ActionServer(
            self.node,
            Move,
            '/ur10/move_to_pose',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback
        )
    
    def execute_callback(self, goal_handle: ServerGoalHandle) -> any:
        position = goal_handle.request.desired_position
        speed = goal_handle.request.speed
        acceleration = goal_handle.request.acceleration
        extra_wait = goal_handle.request.extra_wait

        self.node.get_logger().info(f'Received movement command to joint position: {position} from position {self.node.ur.get_pose()}')

        self.node.ur.move(pose=position,
                          mode='l',
                          transform=False,
                          speed=speed,
                          acc=acceleration,
                          wait=False)
        
        success = True
        while not self.node.ur.check_if_pos_is_reached(position):
            self.node.ros2_timers.timer_main_loop()
            ## Check cancelation
            if self.check_if_canceled(goal_handle):
                success = False
                break

            self.publish_feedback(goal_handle)

        if success:
            # Wait a little extra to ensure the arm have stopped moving
            self.wait_extra(extra_wait)
                
            goal_handle.succeed()

        self.node.get_logger().info(f'Done with movement')
    
        return Move.Result()

    def publish_feedback(self, goal_handle: ServerGoalHandle) -> None:
        feedback_msg = Move.Feedback()
        feedback_msg.current_position = self.node.ur.get_pose()

        goal_handle.publish_feedback(feedback_msg)

class MoveToJointAction(BaseAction):
    def __init__(self, node: 'URNode') -> None:
        super().__init__(node)

        self.move_to_pose_action = ActionServer(
            self.node,
            Move,
            '/ur10/move_to_joint',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback
        )
    
    def execute_callback(self, goal_handle: ServerGoalHandle) -> any:
        position = goal_handle.request.desired_position
        speed = goal_handle.request.speed
        acceleration = goal_handle.request.acceleration
        extra_wait = goal_handle.request.extra_wait

        self.node.get_logger().info(f'Received movement command to joint position: {position} from position {self.node.ur.get_pose()}')

        self.node.ur.move(pose=position,
                          mode='j',
                          transform=False,
                          speed=speed,
                          acc=acceleration,
                          wait=False)
        
        success = True
        while not self.node.ur.check_if_joints_is_reached(position):
            self.node.ros2_timers.timer_main_loop()
            ## Check cancelation
            if self.check_if_canceled(goal_handle):
                success = False
                break

            self.publish_feedback(goal_handle)

        if success:
            # Wait a little extra to ensure the arm have stopped moving
            self.wait_extra(extra_wait)
                
            goal_handle.succeed()

        self.node.get_logger().info(f'Done with movement')
    
        return Move.Result()

    def publish_feedback(self, goal_handle: ServerGoalHandle) -> None:
        feedback_msg = Move.Feedback()
        feedback_msg.current_position = self.node.ur.get_joints()

        goal_handle.publish_feedback(feedback_msg)

class PoseTrajectoryAction(BaseAction):
    def __init__(self, node: 'URNode') -> None:
        super().__init__(node)

        self.follow_pose_trajectory_action = ActionServer(
            self.node,
            FollowJointTrajectory,
            '/ur10/follow_pose_trajectory',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback
        )

    def execute_callback(self, goal_handle: ServerGoalHandle) -> any:
        pose_list, final_pose = get_list_from_trajectory(goal_handle, 'l')

        self.node.get_logger().info(f'Received movement command to position: {final_pose} from position {self.node.ur.get_pose()}')

        self.follow_pose_list(goal_handle, pose_list)

        self.node.get_logger().info(f'Done with movement')
    
        return FollowJointTrajectory.Result()

    def follow_pose_list(self, goal_handle: ServerGoalHandle, pose_list: list[float]) -> None:
        # Run the final part of the trajectory
        self.node.ur.path(
            pose_list,
            False,
            False,
            None,  # Acceleration  (None means use default value)
            None,  # Speed  (None means use default value)
            False
        )

        # Keep the main loop running, but block the code until
        # The UR executes its movement.
        # self.node.ros2_timers.timer_main_loop_blocking(joints_list[-1][:6])
        while not self.node.ur.check_if_pos_is_reached(pose_list[-1][:6]):
            self.node.ros2_timers.timer_main_loop()
            ## Check cancelation
            if self.check_if_canceled(goal_handle):
                return

            self.publish_feedback(goal_handle)

        # Wait a little extra to ensure the arm have stopped moving
        self.wait_extra(1)
            
        goal_handle.succeed()
    
    def publish_feedback(self, goal_handle: ServerGoalHandle) -> None:
        feedback_msg = FollowJointTrajectory.Feedback()
        feedback_msg.actual.positions = self.node.ur.get_pose()
        feedback_msg.actual.velocities = self.node.ur.get_pose_velocity(read=False)

        goal_handle.publish_feedback(feedback_msg)



class JointTracjectoryAction(BaseAction):
    def __init__(self, node: 'URNode') -> None:
        super().__init__(node)

        self.follow_joint_trajectory_action = ActionServer(
            self.node,
            FollowJointTrajectory,
            '/ur10/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback
        )

    def execute_callback(self, goal_handle: ServerGoalHandle) -> any:
        joints_list, final_joints = get_list_from_trajectory(goal_handle, 'j')

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
            None,  # Acceleration  (None means use default value)
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

        # Wait a little extra to ensure the arm have stopped moving
        self.wait_extra(1)
            
        goal_handle.succeed()
    
    def publish_feedback(self, goal_handle: ServerGoalHandle) -> None:
        feedback_msg = FollowJointTrajectory.Feedback()
        feedback_msg.actual.positions = self.node.ur.get_joints()
        feedback_msg.actual.velocities = self.node.ur.get_joints_velocity(read=False)

        goal_handle.publish_feedback(feedback_msg)


def get_list_from_trajectory(
        goal_handle: ServerGoalHandle,
        mode: str,
        r_value=0.05) -> tuple[list[list[float|'str']], list[float|'str']]:
    trajectory = goal_handle.request.trajectory

    _list = []
    for point in trajectory.points:
        tmp_list = list(point.positions)
        tmp_list.append(mode)
        tmp_list.append(r_value)
        _list.append(tmp_list)

    final = list(_list[-1][:6])
    return _list, final
