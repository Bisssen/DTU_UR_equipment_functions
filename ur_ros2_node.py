import rclpy
from src.ros2.node import URNode



def main(args=None):
    rclpy.init(args=args)
    node = URNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
