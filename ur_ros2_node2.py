import rclpy
from ur_code.src.ros2.node import URNode


def main(args=None):
    print('Hi')
    rclpy.init(args=args)
    node = URNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
