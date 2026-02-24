from rclpy.node import Node
from ..ur.class_ur import UR
from .timers import Ros2Timers
from .actions import Ros2Actions
from .publishers import Ros2Publishers
from .services import Ros2Services
from .subscribers import Ros2Subscribers
from .trajectory_buffer import TrajectoryBuffer
import ipaddress


class URNode(Node):
    def __init__(self) -> None:
        super().__init__('ur_node')
        self.get_logger().info('ur_node has started!')

        # Collision safety parameters
        self.declare_parameter('collision_topic', '/collision_monitor/collision')
        self.declare_parameter('recovery_topic', '/collision_safety/recovery')
        self.declare_parameter('collision_safety_enabled', False)

        # Buffered trajectory parameters
        self.declare_parameter('buffered_trajectory_enabled', False)
        self.declare_parameter('buffer_flush_interval', 0.1)
        self.declare_parameter('buffer_min_batch_size', 2)
        self.declare_parameter('buffer_max_batch_size', 50)
        self.declare_parameter('buffer_max_age', 0.2)
        self.declare_parameter('buffer_blend_radius', 0.05)
        self.declare_parameter('buffer_check_tolerance', 0.01)

        self.ros2_publishers = Ros2Publishers(self)

        self.ur = UR(self, ip=self.get_ip())

        # Create trajectory buffer if enabled (must be after self.ur, before timers/subscribers)
        if self.get_parameter('buffered_trajectory_enabled').value:
            self.trajectory_buffer = TrajectoryBuffer(
                ur=self.ur,
                flush_interval=self.get_parameter('buffer_flush_interval').value,
                min_batch_size=self.get_parameter('buffer_min_batch_size').value,
                max_batch_size=self.get_parameter('buffer_max_batch_size').value,
                max_buffer_age=self.get_parameter('buffer_max_age').value,
                blend_radius=self.get_parameter('buffer_blend_radius').value,
                check_tolerance=self.get_parameter('buffer_check_tolerance').value,
                logger=self.get_logger(),
            )
            self.get_logger().info('Buffered trajectory execution enabled')
        else:
            self.trajectory_buffer = None

        self.ros2_timers = Ros2Timers(self)

        self.ros2_subscribers = Ros2Subscribers(self)

        self.ros2_services = Ros2Services(self)

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
