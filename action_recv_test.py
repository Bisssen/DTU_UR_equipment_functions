# trajectory_server.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

class TrajectoryServer(Node):

    def __init__(self):
        super().__init__('trajectory_server')
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received trajectory goal')

        trajectory = goal_handle.request.trajectory

        # 🔧 PLACE YOUR CUSTOM CODE HERE
        for point in trajectory.points:
            self.get_logger().info(f'Executing point: {point.positions}')
            # Simulate execution delay
            # await self._simulate_execution(point)

        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        # Populate result if needed
        return result

    # async def _simulate_execution(self, point: JointTrajectoryPoint):
    #     # Simulate time delay for execution
    #     import asyncio
    #     await asyncio.sleep(point.time_from_start.sec + point.time_from_start.nanosec / 1e9)

def main(args=None):
    rclpy.init(args=args)
    server = TrajectoryServer()
    rclpy.spin(server)

if __name__ == '__main__':
    main()