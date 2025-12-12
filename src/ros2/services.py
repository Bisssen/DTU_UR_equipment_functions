from ur_message_types.srv import SetFloat
import time

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode


class Ros2Services():
    def __init__(self, node: 'URNode') -> None:
        self.node = node

        # Change to service
        self.payload_setter_service =\
        self.node.create_service(
            SetFloat,
            '/ur10/set_payload',
            self.payload_setter_callback
        )

        # It is actually velocity but whatever
        self.speed_setter_service =\
        self.node.create_service(
            SetFloat,
            '/ur10/set_speed',
            self.speed_setter_callback
        )

        self.acceleration_setter_service =\
        self.node.create_service(
            SetFloat,
            '/ur10/set_acceleration',
            self.acceleration_setter_callback
        )

        self.sleep_time = time.time()

    def payload_setter_callback(
            self,
            request: SetFloat.Request,
            response: SetFloat.Response) -> SetFloat.Response:
        payload = request.data
        self.node.ur.set_payload_weight(payload)
        self.sleep_time = time.time()
        while self.sleep_time + 1 > time.time():
            self.node.ros2_timers.timer_main_loop()

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