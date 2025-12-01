from ur_message_types.srv import SetFloat

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode


class Ros2Services():
    def __init__(self, node: 'URNode') -> None:
        self.node = node

        # Change to service
        self.payload_setter_subscriber =\
        self.node.create_service(
            SetFloat,
            '/ur10/set_payload',
            self.payload_setter_callback
        )

        # It is actually velocity but whatever
        self.speed_setter_subscriber =\
        self.node.create_service(
            SetFloat,
            '/ur10/set_speed',
            self.speed_setter_callback
        )

        self.acceleration_setter_subscriber =\
        self.node.create_service(
            SetFloat,
            '/ur10/set_acceleration',
            self.acceleration_setter_callback
        )

    def payload_setter_callback(
            self,
            request: SetFloat.Request,
            response: SetFloat.Response) -> SetFloat.Response:
        payload = request.data
        self.node.ur.set_payload_weight(payload)
        response.success = True
        return response

    def speed_setter_callback(
            self,
            request: SetFloat.Request,
            response: SetFloat.Response) -> SetFloat.Response:
        speed = request.data
        self.node.ur.set_default_path_speed(speed)
        response.success = True
        return response

    def acceleration_setter_callback(
            self,
            request: SetFloat.Request,
            response: SetFloat.Response) -> SetFloat.Response:
        acceleration = request.data
        self.node.ur.set_default_path_acceleration(acceleration)
        response.success = True
        return response