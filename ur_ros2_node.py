import rclpy
from rclpy.node import Node
from src.ur.class_ur import UR

class URNode(Node):
    def __init__(self):
        super().__init__('ur_node')
        self.get_logger().info('URNode has started!')

def main(args=None):
    rclpy.init(args=args)
    node = URNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
