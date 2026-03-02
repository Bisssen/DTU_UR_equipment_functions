from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64, Bool
from rclpy.callback_groups import ReentrantCallbackGroup
from terrain_hopper_teleop.msg import ArmCmd

from ..utils import quaternion_to_euler
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode


class Ros2Subscribers():
    def __init__(self, node: 'URNode') -> None:
        self.node = node

        # Collision safety state
        self.collision_active = False

        # Last tick_id received from ArmCmd (updated even when collision rejects the command)
        self.last_received_tick: int | None = None

        # Latest distance from collision monitor
        self.latest_distance: float = 0.0

        # Separate callback group for collision safety (ensures not blocked by other ops)
        self.collision_callback_group = ReentrantCallbackGroup()

        # Change to service
        self.payload_setter_subscriber =\
        self.node.create_subscription(
            Float32,
            'set_payload',
            self.payload_setter_callback,
            10
        )

        self.pose_command_subscriber =\
            self.node.create_subscription(
                Pose,
                'set_ur_pose',
                self.pose_command_callback,
                10
            )

        self.joints_command_subscriber =\
            self.node.create_subscription(
                JointState,
                'set_ur_joints',
                self.joints_command_callback,
                10
            )

        self.arm_cmd_subscriber =\
            self.node.create_subscription(
                ArmCmd,
                '/real/arm/control',
                self.arm_cmd_callback,
                10
            )

        self.distance_subscriber =\
            self.node.create_subscription(
                Float64,
                '/collision_monitor/distance',
                self.distance_callback,
                10
            )

        # Get collision safety parameters
        collision_topic = self.node.get_parameter('collision_topic').value
        recovery_topic = self.node.get_parameter('recovery_topic').value
        self.collision_safety_enabled = self.node.get_parameter('collision_safety_enabled').value

        if self.collision_safety_enabled:
            # Collision subscriber (uses separate callback group)
            self.collision_subscriber = self.node.create_subscription(
                Bool,
                collision_topic,
                self.collision_callback,
                10,
                callback_group=self.collision_callback_group
            )

            # Recovery subscriber (uses same separate callback group)
            self.recovery_subscriber = self.node.create_subscription(
                Bool,
                recovery_topic,
                self.recovery_callback,
                10,
                callback_group=self.collision_callback_group
            )

            self.node.get_logger().info(f'Collision safety enabled - subscribing to {collision_topic}')
        else:
            self.collision_subscriber = None
            self.recovery_subscriber = None
            self.node.get_logger().info('Collision safety disabled')

    def pose_command_callback(self, msg: Pose) -> None:
        '''
        Send the robot to a specific position.
        '''
        if self.collision_active:
            self.node.get_logger().warn('Pose command rejected - collision active')
            return

        wrist_angles = quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
            )

        self.node.ur.move(
            msg.position.x,
            msg.position.y,
            msg.position.z,
            wrist_angles[0],
            wrist_angles[1],
            wrist_angles[2],
            mode='linear',
            transform=False,
            wait=False
        )
    
    def joints_command_callback(self, msg: JointState) -> None:
        '''
        Send the ur to a specific joints state
        '''
        if self.collision_active:
            self.node.get_logger().warn('Joints command rejected - collision active')
            return

        # Format joints list similar to actions.py execute_callback
        # The path function expects: [joint1, joint2, ..., joint6, 'j', time]
        joints_list = list(msg.position)
        joints_list.append('j')
        joints_list.append(0.05)  # Time parameter (same as in actions.py)

        self.node.get_logger().info(f'Received joint command to position: {list(msg.position)} from position {self.node.ur.get_joints()}')

        # If buffering is enabled, append to buffer; otherwise execute immediately
        if self.node.trajectory_buffer is not None:
            self.node.trajectory_buffer.append(joints_list)
        else:
            self.node.ur.path(
                [joints_list],
                False,
                False,
                None,
                None,
                False
            )

    def arm_cmd_callback(self, msg: ArmCmd) -> None:
        '''
        Receive a joint configuration with an associated tick_id and append
        it to the trajectory buffer for buffered execution.
        '''
        self.last_received_tick = msg.tick_id

        if self.collision_active:
            self.node.get_logger().warn('ArmCmd rejected - collision active')
            return

        if self.node.trajectory_buffer is None:
            self.node.get_logger().warn(
                'ArmCmd received but buffered_trajectory_enabled is False - dropping command'
            )
            return

        joints_list = list(msg.joints.position)
        joints_list.append('j')
        joints_list.append(None)  # legacy placeholder, not used by path()
        self.node.trajectory_buffer.append(joints_list, tick_id=msg.tick_id)

    def distance_callback(self, msg: Float64) -> None:
        '''Update latest distance measurement from collision monitor.'''
        self.latest_distance = float(msg.data)

    def payload_setter_callback(self, msg: Float32) -> None:
        payload = msg.data
        self.node.ur.set_payload_weight(payload)

    def collision_callback(self, msg: Bool) -> None:
        '''
        Handle collision detection signal from collision_monitor_node.
        When collision is detected, stop the robot and block new commands.
        '''
        if msg.data and not self.collision_active:
            self.node.get_logger().warn('COLLISION DETECTED - Stopping robot')
            self.collision_active = True
            if self.node.trajectory_buffer is not None:
                self.node.trajectory_buffer.clear()
            self.node.ur.stop(acc=5.0)  # Emergency stop
            self.node.ros2_publishers.publish_collision_status(self.collision_active)
            # Publish the joint state after stopping
            collision_joints = self.node.ur.get_joints()
            self.node.ros2_publishers.publish_collision_joints(collision_joints)

    def recovery_callback(self, msg: Bool) -> None:
        '''
        Handle recovery signal to resume normal operation.
        '''
        if msg.data and self.collision_active:
            self.node.get_logger().info('RECOVERY - Resuming normal operation')
            self.collision_active = False
            if self.node.trajectory_buffer is not None:
                self.node.trajectory_buffer.resume()
            self.node.ros2_publishers.publish_collision_status(self.collision_active)
