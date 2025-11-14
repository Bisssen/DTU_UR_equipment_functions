from TODO.srv import TODO

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .node import URNode


class Ros2Services():
    def __init__(self, node: 'URNode') -> None:
        self.node = node

        # Change to service
        self.payload_setter_subscriber =\
        self.node.create_service(
            TODO,
            'set_payload',
            self.payload_setter_callback
        )

    def payload_setter_callback(
            self,
            request: TODO.Request,
            response: TODO.Response) -> TODO.Response:
        payload = request.data
        self.node.ur.set_payload_weight(payload)
        response.data = True
        return response


# TODO test trajectories. See if there kinda is a cap on the size
#... how should it look
