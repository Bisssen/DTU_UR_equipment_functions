import rclpy
from ur_code.src.ros2.node import URNode


def main(args=None):
    rclpy.init(args=args)
    node = URNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



# import rclpy
# from ur_code.src.ros2.node import URNode
# from rclpy.executors import MultiThreadedExecutor


# def main(args=None):
#     rclpy.init(args=args)
#     node = URNode()
#     executor = MultiThreadedExecutor()
#     executor.add_node(node)

#     try:
#         executor.spin()
#     finally:
#         executor.shutdown()
#         node.destroy_node()
#         rclpy.shutdown()
