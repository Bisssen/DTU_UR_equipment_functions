import rclpy
from arduino_command_messages.srv import SetPayload
from moveit_msgs.srv import GetPositionFK

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode


class Ros2Services():
    def __init__(self, node: 'URNode') -> None:
        self.node = node

        # Change to service
        self.payload_setter_subscriber =\
        self.node.create_service(
            SetPayload,
            'set_payload',
            self.payload_setter_callback
        )

        self.cli = self.node.create_client(GetPositionFK, '/ur10/compute_fk')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for /ur10/compute_ik service...')

        self.req = GetPositionFK.Request()

    def payload_setter_callback(
            self,
            request: SetPayload.Request,
            response: SetPayload.Response) -> SetPayload.Response:
        payload = request.payload
        self.node.ur.set_payload_weight(payload)
        response.success = True
        return response
    
    def get_fk(self, joints):
        # Fill request fields
        self.req.robot_state.joint_state.name =['shoulder_pan_joint',
                        'shoulder_lift_joint',
                        'elbow_joint',
                        'wrist_1_joint',
                        'wrist_2_joint',
                        'wrist_3_joint']
        print(joints[:6])

    
        self.req.robot_state.joint_state.position = joints[:6]

        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

