import socket
import struct
import threading
import sys

from . import config_ur


class communication_thread():
    def __init__(self):
        # Whether the program is run in python 2 or not
        self.python_2 = (sys.version_info.major == 2)

        # Creating the socket
        self.socket_robot = socket.socket(socket.AF_INET,
                                          socket.SOCK_STREAM)
        self.socket_robot.connect((config_ur.IP,
                                   config_ur.PORT))

        # The thread keeps going as long as this variable is true
        self.running = True

        # The data which the thread optains from the ur
        self.data = 0

        # The thread which keeps receiving data
        self.receive_thread = threading.Thread(target=self.receive)

        # Starting the Thread
        print('    Starting communication thread...')
        self.receive_thread.start()

    def receive(self):
        while self.running:
            data = (self.socket_robot.recv(2048))
            self.data = self.transform_data(data)

    def shutdown(self):
        self.running = False
        self.receive_thread.join()
        self.socket_robot.close()

    def transform_data_point(self, data, data_type):
        num = config_ur.DATA_MAP[data_type]
        data = data[num:num+8]

        # Convert the data from \x hex notation to plain hex
        if self.python_2:
            data = data.encode("hex")
        else:
            data = data.hex()

        if len(data) == 16:
            if self.python_2:
                data = struct.unpack('!d', data.decode("hex"))[0]
            else:
                data = struct.unpack('!d', bytes.fromhex(data))[0]
            return data
        else:
            return 0


    def transform_data(self, data):
        data_string = ''
        for data_type in config_ur.DATA_MAP:
            data_point = self.transform_data_point(data, data_type)
            data_string += data_type + ':' + str(data_point) + ';'
        return data_string
