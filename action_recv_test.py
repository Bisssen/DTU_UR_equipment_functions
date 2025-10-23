# trajectory_server.py
import rclpy
from rclpy.node import Node
import threading
import time

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse

class TrajectoryServer(Node):

    def __init__(self):
        super().__init__('trajectory_server')
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
        )

        self._goal_handle = None
        self._goal_lock = threading.Lock()

        self.timetime = self.create_timer(0.5, self.timt)

    def timt(self):
        print('hi')


    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        trajectory = goal_handle.request.trajectory

        poses = []
        
        feedback_msg = FollowJointTrajectory.Feedback()

        for point in trajectory.points:
        
            # If goal is flagged as no longer active (ie. another goal was accepted),
            # then stop executing
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return FollowJointTrajectory.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return FollowJointTrajectory.Result()


            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for demonstration purposes
                    
            tmp_list = list(point.positions)
            tmp_list.append('j')
            tmp_list.append(None)
            poses.append(tmp_list)
            time.sleep(1)
            print(poses)
        


        with self._goal_lock:
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return FollowJointTrajectory.Result()

            goal_handle.succeed()

        return FollowJointTrajectory.Result()



from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    server = TrajectoryServer()

    # Use MultiThreadedExecutor to allow concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(server)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        server.destroy_node()
        rclpy.shutdown()