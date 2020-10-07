import socket
import struct
import threading
import sys
import time

from . import config_ur


class communication_thread():
    def __init__(self):
        # Whether the program is run in python 2 or not
        self.python_2 = (sys.version_info.major == 2)

        # Creating the socket
        self.socket_ur = socket.socket(socket.AF_INET,
                                       socket.SOCK_STREAM)
        time_start = time.time()
        self.socket_ur.connect((config_ur.IP,
                                config_ur.PORT))

        # The thread keeps going as long as this variable is true
        self.running = True

        # The data which the thread optains from the ur
        self.data = 0

        # Read the message size
        self.message_size = self.get_message_size()

        # The thread which keeps receiving data
        self.receive_thread = threading.Thread(target=self.receive)

        # Starting the Thread
        print('    Starting communication thread...')
        self.receive_thread.start()

    def receive(self):
        while self.running:
            data = (self.socket_ur.recv(2048))
            self.data = self.transform_data(data)

    def shutdown(self):
        self.running = False
        self.receive_thread.join()
        self.socket_ur.close()

    def get_message_size(self):
        data = (self.socket_ur.recv(2048))
        message_size = self.transform_data_point(data, "message_size")
        self.data = self.transform_data(data)
        
        return message_size

    def transform_data_point(self, data, data_name):
        num = config_ur.DATA_MAP[data_name]
        data = data[num:num + config_ur.DATA_SIZE[data_name]]

        # Convert the data from \x hex notation to plain hex
        if self.python_2:
            data = data.encode("hex")
        else:
            data = data.hex()

        if len(data) == config_ur.DATA_SIZE[data_name] * 2:
            if self.python_2:
                data = struct.unpack(config_ur.DATA_TYPE[data_name],
                                     data.decode("hex"))[0]
            else:
                data = struct.unpack(config_ur.DATA_TYPE[data_name],
                                     bytes.fromhex(data))[0]
            return data
        else:
            return 0

    def transform_data(self, data):
        data_string = ''
        for data_type in config_ur.DATA_MAP:
            data_point = self.transform_data_point(data, data_type)
            data_string += data_type + ':' + str(data_point) + ';'
        return data_string
