from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode

class Ros2Timers():
    def __init__(self, node: 'URNode') -> None:
        self.node = node

        self.timer_frequency = 1/200  # 20Hz
        
        # Timers that replaces the threads that used to run the different socket connections
        ### NB This one MUST be quite fast, as it needs to keep the socket empty, for the data reading to work
        # self.ur_connection_timer = self.node.create_timer(self.timer_frequency / 10, self.node.ur.communication_thread.receive)

        self.node.get_logger().info("UR: Starting communication timer...")

        # Timer that publishes the pose of the UR
        # self.pose_publish_timer = self.node.create_timer(self.timer_frequency, self.publish_ur_data)

        self.main_loop_timer = self.node.create_timer(self.timer_frequency, self.timer_main_loop)
    

    def publish_ur_data(self) -> None:
        pose = self.node.ur.get_pose()
        pose_velocity = self.node.ur.get_pose_velocity(read=False)
        joints = self.node.ur.get_joints(read=False)
        joints_velocity = self.node.ur.get_joints_velocity(read=False)

        self.node.ros2_publishers.publish_ur_pose(pose)
        self.node.ros2_publishers.publish_ur_pose_velocity(pose_velocity)

        self.node.ros2_publishers.publish_ur_joints(joints)
        self.node.ros2_publishers.publish_ur_joints_velocity(joints_velocity)

        self.node.ros2_publishers.publish_is_ur_moving(
            self.node.ur.is_moving()
        )
    
    def timer_main_loop(self) -> None:
        # Make sure to read data from the UR
        self.node.ur.communication_thread.receive()

        # Publish UR data
        self.publish_ur_data()

    
    def timer_main_loop_blocking(self) -> None:
        '''
        Will run the timer_main_loop while blocking if the robot is still moving.
        This is used to keep the main loop running while the program is blocking
        during actions
        '''
        while self.node.ur.is_moving():
            self.timer_main_loop()