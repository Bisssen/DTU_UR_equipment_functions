from rclpy.node import Node
from ..ur.class_ur import UR
from .timers import Ros2Timers
from .actions import Ros2Actions
from .publishers import Ros2Publishers
from .subscribers import Ros2Subscribers
import ipaddress

# TODO
# Update readme

class URNode(Node):
    def __init__(self) -> None:
        super().__init__('ur_node')
        self.get_logger().info('ur_node has started!')

        self.ros2_publishers = Ros2Publishers(self)


        self.ur = UR(self, ip=self.get_ip())

        self.ros2_timers = Ros2Timers(self)

        self.ros2_subscribers = Ros2Subscribers(self)

        self.ros2_actions = Ros2Actions(self)
    
    def get_ip(self) -> str | None:
        custom_ip: str | None
        custom_ip = self.declare_parameter('ip', 'None').get_parameter_value().string_value

        if custom_ip == 'None':
            custom_ip = None
        elif not is_valid_ip(custom_ip):
            self.get_logger().warning(f'Provided ip of {custom_ip} is not valid. Using default instead')
            custom_ip = None

        return custom_ip

def is_valid_ip(ip_str: str) -> bool:
    try:
        ipaddress.ip_address(ip_str)
        return True
    except ValueError:
        return False
