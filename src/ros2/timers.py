from .node import URNode

class ros2_timer():
    def __init__(self, node: URNode):
        self.node = node

        self.timer_frequency = 1/20  # 20Hz
        
        # Timers that replaces the threads that used to run the different socket connections
        self.ur_connection_timer = self.node.create_timer(self.timer_frequency, self.node.ur.communication_thread.receive)

        self.node.get_logger().info("UR: Starting communication timer...")

        