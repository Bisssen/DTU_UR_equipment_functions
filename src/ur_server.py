import sys
import time

from .server.class_server import Server
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
                if request is None:
                    continue
                
                # If using python 3 then decide the request
                if not self.python_2:
                    request = request.decode()
                
                # React on the request sent
                self.handle_request(request)
        except KeyboardInterrupt:
            print('KeyboardInterrupt')


    def handle_request(self, request):
        request_splitted = request.split(':')
        if request_splitted[0] == 'move':
            self.translate_request_to_move(request_splitted[1:], request)
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
        if self.test_if_within_function_limits(3, 10, len(request_splitted),
                                               "move", request):
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

def convert_to_bool(value):
    if value == "True" or value == "true":
        return True
    else:
        return False
