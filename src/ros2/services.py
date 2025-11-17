from arduino_command_messages.srv.ur_srv import SetPayload

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

    def payload_setter_callback(
            self,
            request: SetPayload.Request,
            response: SetPayload.Response) -> SetPayload.Response:
        payload = request.payload
        self.node.ur.set_payload_weight(payload)
        response.success = True
        return response


# TODO test trajectories. See if there kinda is a cap on the size
#... how should it look
