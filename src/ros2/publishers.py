from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from terrain_hopper_teleop.msg import ArmFeedback


from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode


class Ros2Publishers():
    # Threshold for detecting joint jumps caused by corrupted socket data
    # Even at max UR10 velocity (~3.14 rad/s) at 200Hz, delta = 0.016 rad
    # Use 0.05 rad as threshold - any larger jump in a single sample is corrupted data
    MAX_JOINT_DELTA = 0.05  # radians per sample

    def __init__(self, node: 'URNode') -> None:
        self.node = node
        self.ur_pose_publisher = self.node.create_publisher(JointState, 'ur_pose', 10)
        # self.ur_pose_velocity_publisher = self.node.create_publisher(Pose, 'ur_pose_velocity', 10)

        self.ur_joints_publisher = self.node.create_publisher(JointState, '/ur10/joint_states_ur10', 10)
        # self.ur_joints_velocity_publisher = self.node.create_publisher(JointState, 'ur_joints_velocity', 10)

        self.is_ur_moving_publisher = self.node.create_publisher(Bool, 'is_ur_moving', 10)

        # Collision safety status publisher
        self.collision_status_pub = self.node.create_publisher(Bool, '/collision_safety/active', 10)

        # Collision joints publisher - publishes joint state when collision is detected
        self.collision_joints_pub = self.node.create_publisher(JointState, '/collision_safety/joints', 10)

        # ArmFeedback publisher
        self.arm_feedback_pub = self.node.create_publisher(ArmFeedback, '/real/arm/feedback', 10)

        # Store last valid joint positions for jump detection
        self.last_valid_joints: list[float] | None = None

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

    def publish_ur_joints(self,
                          joints_list: list[float],
                          velocity_list: list[float]) -> None:
        if not self.validate_list_size(joints_list, 6):
            return

        # Validate that joint data is not all zeros (indicates socket read failure)
        if not self.validate_joint_data(joints_list):
            return

        self.ur_joints_publisher.publish(
            self.convert_to_ros2_jointstate(
                joints_list,
                velocity_list
            )
        )

    def convert_to_ros2_jointstate(self,
                                   jointstate_list: list[float],
                                   velocity_list: list[float],
                                   joint=True) -> JointState:
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        if joint:
            msg.name = ['shoulder_pan_joint',
                        'shoulder_lift_joint',
                        'elbow_joint',
                        'wrist_1_joint',
                        'wrist_2_joint',
                        'wrist_3_joint']

        else:
            msg.name = ['x', 'y', 'z', 'rx', 'ry', 'rz']
        msg.position = jointstate_list
        msg.velocity = velocity_list
        return msg

    def publish_is_ur_moving(self, is_moving):
        msg = Bool()
        msg.data = is_moving
        self.is_ur_moving_publisher.publish(msg)

    def publish_collision_status(self, collision_active: bool) -> None:
        '''
        Publish current collision safety status.
        '''
        msg = Bool()
        msg.data = collision_active
        self.collision_status_pub.publish(msg)

    def publish_collision_joints(self, joints: list[float]) -> None:
        '''
        Publish joint state when collision is detected.
        Uses last valid joints as fallback if current read is invalid.
        '''
        if not self.validate_list_size(joints, 6):
            # Fall back to last valid joints if available
            if self.last_valid_joints is not None:
                joints = self.last_valid_joints
                self.node.get_logger().warn(
                    'Using last valid joints for collision state (invalid size)'
                )
            else:
                self.node.get_logger().error(
                    'Cannot publish collision joints: invalid size and no fallback'
                )
                return

        # Validate that joint data is not all zeros or invalid
        elif not self.validate_joint_data(joints):
            # Fall back to last valid joints if available
            if self.last_valid_joints is not None:
                joints = self.last_valid_joints
                self.node.get_logger().warn(
                    'Using last valid joints for collision state (invalid data)'
                )
            else:
                self.node.get_logger().error(
                    'Cannot publish collision joints: invalid data and no fallback'
                )
                return

        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = ['shoulder_pan_joint',
                    'shoulder_lift_joint',
                    'elbow_joint',
                    'wrist_1_joint',
                    'wrist_2_joint',
                    'wrist_3_joint']
        msg.position = joints
        self.collision_joints_pub.publish(msg)

    def publish_arm_feedback(
        self,
        joints: list[float],
        last_tick_applied: int,
        collision: bool,
        distance: float,
    ) -> None:
        '''
        Publish ArmFeedback with current joint state and the tick of the
        waypoint the arm is currently moving toward.
        '''
        if not self.validate_list_size(joints, 6):
            return
        if not self.validate_joint_data(joints):
            return

        msg = ArmFeedback()
        msg.joints.header.stamp = self.node.get_clock().now().to_msg()
        msg.joints.name = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
        ]
        msg.joints.position = joints
        msg.last_tick_applied = last_tick_applied
        msg.collision = collision
        msg.distance = float(distance)
        self.arm_feedback_pub.publish(msg)

    def validate_list_size(self, _list: list[float], size=6) -> bool:
        if not len(_list) == size:
            self.node.get_logger().error(
                f'Trying to publish list: {_list} '
                f'of size {len(_list)} '
                f'but it must be of size {size}.'
            )
            return False
        return True

    def validate_joint_data(self, joints_list: list[float]) -> bool:
        """
        Validates that joint data is not all zeros or invalid.
        All zeros typically indicates a socket communication failure.
        Also detects sudden jumps caused by corrupted socket data.
        Returns True if data is valid, False otherwise.
        """
        # Check if all joints are exactly zero (socket read failure indicator)
        if all(abs(joint) < 1e-6 for joint in joints_list):
            self.node.get_logger().debug(
                'Skipping joint state publish: all joints are zero (possible socket read failure)',
                throttle_duration_sec=1.0  # Only log once per second
            )
            return False

        # Check for NaN or infinity values
        if any(not (-10 < joint < 10) for joint in joints_list):
            self.node.get_logger().debug(
                f'Skipping joint state publish: invalid joint values detected: {joints_list}',
                throttle_duration_sec=1.0
            )
            return False

        # Check for sudden jumps (corrupted socket data)
        if self.last_valid_joints is not None:
            max_delta = max(abs(joints_list[i] - self.last_valid_joints[i]) for i in range(6))
            if max_delta > self.MAX_JOINT_DELTA:
                self.node.get_logger().debug(
                    f'Skipping joint state publish: detected jump of {max_delta:.4f} rad (corrupted data)',
                    throttle_duration_sec=1.0
                )
                return False

        # Data is valid, update last known good joints
        self.last_valid_joints = joints_list.copy()
        return True
