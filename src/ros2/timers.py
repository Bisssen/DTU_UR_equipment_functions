from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode

class Ros2Timers():
    def __init__(self, node: 'URNode'):
        self.node = node

        self.timer_frequency = 1/20  # 20Hz
        
        # Timers that replaces the threads that used to run the different socket connections
        ### NB This one MUST be quite fast, as it needs to keep the socket empty, for the data reading to work
        self.ur_connection_timer = self.node.create_timer(self.timer_frequency / 10, self.node.ur.communication_thread.receive)

        self.node.get_logger().info("UR: Starting communication timer...")

        # Timer that publishes the pose of the UR
        self.pose_publish_timer = self.node.create_timer(self.timer_frequency, self.node.ur.publish_ur_pose)