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
import random

class TrajectoryClient(Node):

    def __init__(self):
        super().__init__('trajectory_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/ur10/follow_joint_trajectory')

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
        randomy = 1.0 * (random.random() * 2 -1)

        points_list =\
            [\
             [np.deg2rad(-179 + randomy) , np.deg2rad(-48 + randomy), np.deg2rad(-46 + randomy), np.deg2rad(-136 + randomy), np.deg2rad(89 + randomy), np.deg2rad(-112 + randomy)]]#,
            #  [-1.58, -2.84, 1.56, 1.83, 0.51, 1.58],
            #  [-1.68, -2.84, 1.56, 1.83, 0.51, 1.58],
            #  [-1.78, -2.84, 1.56, 1.83, 0.51, 1.58],
            #  [-1.88, -2.84, 1.56, 1.83, 0.51, 1.58]]
        '''
        [np.deg2rad(-156), np.deg2rad(-67), np.deg2rad(-25), np.deg2rad(-119), np.deg2rad(89), np.deg2rad(-112)],
        [np.deg2rad(-156), np.deg2rad(-48), np.deg2rad(-46), np.deg2rad(-119), np.deg2rad(89), np.deg2rad(-112)],
        '''

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
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        # time.sleep(1)
        
        # cancel_future = self.goal_handle.cancel_goal_async()
        # cancel_future.add_done_callback(self.cancel_done)

        # print('ccc')

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

        rclpy.shutdown()

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result received: {result}')
        self.send_goal()
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    client = TrajectoryClient()
    # while True:
    client.send_goal()
    # time.sleep(1)
    #cancel_goal()
    rclpy.spin(client)
        # try:
        #     rclpy.spin(client)
        
        # except KeyboardInterrupt:
        #     future = client._action_client._cancel_goal_async(client.goal_handle)
        #     rclpy.spin_until_future_complete(client, future)
        # finally:
        #     client.destroy_node()
        #     rclpy.shutdown()

        # break

if __name__ == '__main__':
    main()