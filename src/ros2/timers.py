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
        joints = self.node.ur.get_joints(read=False)

        self.node.ros2_publishers.publish_ur_pose(
            pose[0],
            pose[1],
            pose[2],
            pose[3],
            pose[4],
            pose[5]
        )

        self.node.ros2_publishers.publish_joint_positions(
            joints[0],
            joints[1],
            joints[2],
            joints[3],
            joints[4],
            joints[5]
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