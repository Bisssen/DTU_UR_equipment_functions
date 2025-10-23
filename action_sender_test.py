# trajectory_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from builtin_interfaces.msg import Time, Duration
import time
import numpy as np

class TrajectoryClient(Node):

    def __init__(self):
        super().__init__('trajectory_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, 'follow_joint_trajectory')

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()

        trajectory = JointTrajectory()
        trajectory.header = Header()
        trajectory.header.stamp = Time(sec=0, nanosec=0)
        trajectory.header.frame_id = 'world'
        trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        points_list =\
            [[np.deg2rad(-156), np.deg2rad(-67), np.deg2rad(-25), np.deg2rad(-119), np.deg2rad(89), np.deg2rad(-112)],
             [np.deg2rad(-156), np.deg2rad(-48), np.deg2rad(-46), np.deg2rad(-119), np.deg2rad(89), np.deg2rad(-112)],
             [np.deg2rad(-179), np.deg2rad(-48), np.deg2rad(-46), np.deg2rad(-136), np.deg2rad(89), np.deg2rad(-112)]]#,
            #  [-1.58, -2.84, 1.56, 1.83, 0.51, 1.58],
            #  [-1.68, -2.84, 1.56, 1.83, 0.51, 1.58],
            #  [-1.78, -2.84, 1.56, 1.83, 0.51, 1.58],
            #  [-1.88, -2.84, 1.56, 1.83, 0.51, 1.58]]

        for i, point_list in enumerate(points_list):
            point = JointTrajectoryPoint()
            point.positions = point_list

            point.velocities = [0.0] * 6
            point.accelerations = [0.0] * 6
            point.effort = []
            point.time_from_start =\
                Duration(sec=1 + i, nanosec=0)

            trajectory.points.append(point)
        goal_msg.trajectory = trajectory

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result received: {result}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = TrajectoryClient()
    while True:
        client.send_goal()
        time.sleep(1)
        
        rclpy.spin(client)
        break

if __name__ == '__main__':
    main()