from rclpy.node import Node
from ..ur.class_ur import UR
from .timers import Ros2Timers
from .publishers import Ros2Publishers
import ipaddress

class URNode(Node):
    def __init__(self):
        super().__init__('ur_node')
        self.get_logger().info('ur_node has started!')

        self.ros2_publishers = Ros2Publishers(self)


        self.ur = UR(self, ip=self.get_ip())  # TODO add ros2 way of setting ip

        self.ros2_timers = Ros2Timers(self)
    
    def get_ip(self) -> str | None:
        custom_ip: str | None
        custom_ip = self.declare_parameter('ip', 'None').get_parameter_value().string_value

        if custom_ip == 'None':
            custom_ip = None
        elif not is_valid_ip(custom_ip):
            self.get_logger().warning(f'Provided ip of {custom_ip} is not valid. Using default instead')
            custom_ip = None


def is_valid_ip(ip_str: str) -> bool:
    try:
        ipaddress.ip_address(ip_str)
        return True
    except ValueError:
        return False
