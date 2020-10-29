import sys
import time

from .server.class_server import Server
from .server.server_data_helping_functions import check_request, convert_to_bool,\
                                                  test_if_within_function_limits
from .ur.class_ur import UR


class URServer:
    def __init__(self, server_port=4444, ip=None, port=None):
        # Whether the program is run in python 2 or not
        self.python_2 = (sys.version_info.major == 2)

        # The response, which gets sent to the user
        self.response = None

        # The server itself
        self.server = Server(server_port, 'UR server')

        # The connection to the ur arm
        self.ur = UR(ip, port)

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
            self.ur.shutdown()
            print('KeyboardInterrupt')

    def handle_request(self, request):
        request_splitted = request.split(':')
        if request_splitted[0] == 'move':
            self.translate_request_to_move(request_splitted[1:], request)
            return
        elif request_splitted[0] == 'move_relative':
            self.translate_request_to_move_relative(request_splitted[1:], request)
            return
        elif request_splitted[0] == 'move_tool':
            self.translate_request_to_move_tool(request_splitted[1:], request)
            return
        elif request_splitted[0] == 'speed':
            self.translate_request_to_speed(request_splitted[1:], request)
            return
        elif request_splitted[0] == 'wait':
            self.translate_request_to_wait()
            return
        elif request_splitted[0] == 'get_position':
            self.translate_request_to_get_position(request_splitted[1:], request)
            return
        elif request_splitted[0] == 'read':
            self.translate_request_to_read(request_splitted[1:])
            return
        else:
            self.translate_request_to_send_line(request)
            return

    def test_if_within_function_limits(self, lower, higher, lenght,
                                       function_name, request):
        # If there is not enough inputs provided then tell the user
        if lenght < lower:
            print('Not enough inputs recieved to make a ' + function_name +
                  ' command.' + '\nReceived: ' + str(request))
            return True
        # If there is too many inputs provided then tell the user
        elif lenght > higher:
            print('Too many inputs recieved to make a ' + function_name +
                  '.' + '\nReceived: ' + str(request))
            return True
        else:
            return False

    def translate_request_to_move(self, request_splitted, request):
        # Check that the request have the right amount of inputs
        if test_if_within_function_limits(3, 10, len(request_splitted),
                                          'move', request):
            return
        request_values = []
        for i, value in enumerate(request_splitted):
            # The first seven digits are floats, so convert them to that
            if i <= 7:
                request_values.append(float(value))
            # The last two are bools
            else:
                request_values.append(convert_to_bool(value))
        self.ur.move(*request_values)

    def translate_request_to_move_relative(self, request_splitted, request):
        # Check that the request have the right amount of inputs
        if test_if_within_function_limits(0, 9, len(request_splitted),
                                          'move_relative', request):
            return
        request_values = []
        for i, value in enumerate(request_splitted):
            # The first seven digits are floats, so convert them to that
            if i <= 7:
                request_values.append(float(value))
            # The last two are bools
            else:
                request_values.append(convert_to_bool(value))
        self.ur.move_relative(*request_values)

    def translate_request_to_move_tool(self, request_splitted, request):
        # Check that the request have the right amount of inputs
        if test_if_within_function_limits(0, 9, len(request_splitted),
                                          'move_tool', request):
            return
        request_values = []
        for i, value in enumerate(request_splitted):
            # The first seven digits are floats, so convert them to that
            if i <= 7:
                request_values.append(float(value))
            # The last two are bools
            else:
                request_values.append(convert_to_bool(value))
        self.ur.move_tool(*request_values)

    def translate_request_to_move_tool(self, request_splitted, request):
        # Check that the request have the right amount of inputs
        if test_if_within_function_limits(0, 9, len(request_splitted),
                                          'speed', request):
            return
        request_values = []
        for i, value in enumerate(request_splitted):
            # The first seven digits are floats, so convert them to that
            if i <= 7:
                request_values.append(float(value))
            # The last two are bools
            else:
                request_values.append(convert_to_bool(value))
        self.ur.speed(*request_values)

    def translate_request_to_wait(self):
        # Wait until the arm is done moving
        self.ur.wait()

        # Respond that the arm is done moving
        response = 'wait_done=True'

        # Update the response such that it can be sent
        self.response = self.encode_response(response)

    def translate_request_to_get_position(self, request_splitted, request):
        if test_if_within_function_limits(0, 1, len(request_splitted),
                                          'get_position', request):
            return
        request_values = []
        for i, value in enumerate(request_splitted):
            # The only value that get_position can have is a boolean
            request_values.append(convert_to_bool(value))
        ur_position = self.ur.get_position(*request_values)
        
        # Create a response containing the position of the UR
        response = str(ur_position[0]) + ':' + str(ur_position[1]) + ':' +\
                   str(ur_position[1])

        # Update the response such that it can be sent
        self.response = self.encode_response(response)

    def translate_request_to_read(self, request_splitted):
        # Update the values read from the UR
        self.ur.read()

        # Try to read all the values requested
        response = ''
        for i, value in enumerate(request_splitted):
            # Try to read the requested data
            try:
                response_value = str(self.ur.ur_data[value])
            # If it is not possible, then skip is
            except:
                continue

            # Add the read value to the response 
            response += value + '=' + response_value + ':'

        # Update the response such that it can be sent
        self.response = self.encode_response(response)

    def translate_request_to_send_line(self, request):
        # Add a new \n, since the old got removed
        self.ur.send_line(request + "\n")

    # Encodes the response, if using python 3
    def encode_response(self, response):
        # If using python 3, encode the response
        if not self.python_2:
            response = response.encode()

        return response
