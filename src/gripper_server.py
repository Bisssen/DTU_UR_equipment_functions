import sys
import time

from .server.class_server import Server
from .server.server_data_helping_functions import check_request, convert_to_bool,\
                                                  test_if_within_function_limits
from .gripper.class_gripper import Gripper


class GripperServer:
    def __init__(self, server_port=4443, usb_port=None):
        # Whether the program is run in python 2 or not
        self.python_2 = (sys.version_info.major == 2)

        # The response, which gets sent to the user
        self.response = None

        # The server itself
        self.server = Server(server_port, 'Gripper server')

        # The connection to the ur arm
        self.gripper = Gripper(usb_port)

        # Start the main loop of the UR server
        self.main_loop()

    # The main loop of the server
    def main_loop(self):
        left_over_data = ''
        # If the user CTRL c'es then exit the program
        try:
            while True:
                # Slow the loop down a bit to not use all resources
                time.sleep(0.05)
                # Check if the user have sent data
                request = self.server.main_loop(self.response)
                # Erase any response there might be after sending it
                self.response = None
                
                # If there is no request received then continue
                if request is None and len(left_over_data) == 0:
                    continue

               # If using python 3 then decode the request
                if not self.python_2 and request is not None:
                    request = request.decode()

                left_over_data, request,\
                    request_complete = check_request(request,
                                                     left_over_data)

                # Check if an entire message have been received
                if not request_complete:
                    continue
                
                # React on the request sent
                self.handle_request(request)
        except KeyboardInterrupt:
            self.gripper.shutdown()
            print('KeyboardInterrupt')

    def handle_request(self, request):
        request_splitted = request.split(':')
        if request_splitted[0] == 'set':
            self.translate_request_to_set(request_splitted[1:], request)
            return
        elif request_splitted[0] == 'close':
            self.translate_request_to_close(request_splitted[1:], request)
            return
        elif request_splitted[0] == 'open':
            self.translate_request_to_open(request_splitted[1:], request)
            return
        elif request_splitted[0] == 'wait':
            self.translate_request_to_wait()
            return
        elif request_splitted[0] == 'get_position':
            self.translate_request_to_get_position()
            return
        elif request_splitted[0] == 'get_status':
            self.translate_request_to_get_status()
            return
        else:
            print("Invalid request: " + str(request))
    
    def translate_request_to_set(self, request_splitted, request):
        # Check that the request have the right amount of inputs
        if test_if_within_function_limits(1, 4, len(request_splitted),
                                          'set', request):
            return
        request_values = []
        for i, value in enumerate(request_splitted):
            # The first seven digits are floats, so convert them to that
            if i <= 3:
                request_values.append(int(value))
            # The last two are bools
            else:
                request_values.append(convert_to_bool(value))
        self.gripper.set(*request_values)

    def translate_request_to_close(self, request_splitted, request):
        # Check that the request have the right amount of inputs
        if test_if_within_function_limits(0, 3, len(request_splitted),
                                          'close', request):
            return
        request_values = []
        for i, value in enumerate(request_splitted):
            # The first seven digits are floats, so convert them to that
            if i <= 2:
                request_values.append(int(value))
            # The last two are bools
            else:
                request_values.append(convert_to_bool(value))
        self.gripper.close(*request_values)

    def translate_request_to_open(self, request_splitted, request):
        # Check that the request have the right amount of inputs
        if test_if_within_function_limits(0, 3, len(request_splitted),
                                          'open', request):
            return
        request_values = []
        for i, value in enumerate(request_splitted):
            # The first seven digits are floats, so convert them to that
            if i <= 2:
                request_values.append(int(value))
            # The last two are bools
            else:
                request_values.append(convert_to_bool(value))
        self.gripper.open(*request_values)

    def translate_request_to_wait(self):
        # Wait until the arm is done moving
        self.gripper.wait()

        # Respond that the arm is done moving
        response = 'wait_done=True'

        # Update the response such that it can be sent
        self.response = self.encode_response(response)

    def translate_request_to_get_position(self):

        gripper_position = self.gripper.get_position()
        
        # Create a response containing the position of the UR
        response = str(gripper_position)

        # Update the response such that it can be sent
        self.response = self.encode_response(response)

    def translate_request_to_get_status(self):

        gripper_status = self.gripper.get_position()
        
        # Create a response containing the position of the UR
        response = str(gripper_status[0]) + ':' + str(gripper_status[1]) + ':' +\
                   str(gripper_status[1]) + ':' + str(gripper_status[2])

        # Update the response such that it can be sent
        self.response = self.encode_response(response)

    # Encodes the response, if using python 3
    def encode_response(self, response):
        # If using python 3, encode the response
        if not self.python_2:
            response = response.encode()

        return response
