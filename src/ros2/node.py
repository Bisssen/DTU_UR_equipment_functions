from rclpy.node import Node
from ..ur.class_ur import UR

class URNode(Node):
    def __init__(self):
        super().__init__('ur_node')
        self.get_logger().info('ur_node has started!')

        self.ur = UR()  # TODO add ros2 way of setting ip