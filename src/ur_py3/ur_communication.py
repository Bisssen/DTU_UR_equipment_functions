import socket
import struct
import threading

from . import config_robot as cfg

class communication_thread():
    def __init__(self):
        # Whether the program is run in python 2 or not 
        self.python_2 = (sys.version_info.major == 2)

        # Creating the socket
        self.socket_robot = socket.socket(socket.AF_INET,
                                          socket.SOCK_STREAM)
        self.socket_robot.connect((cfg.SOCKETS['host ip'],
                                   cfg.SOCKETS['port send']))

        self.running = True

        self.data = 0

        self.receive_thread = threading.Thread(target=self.receive)

        print('    Starting communication thread...')
        self.receive_thread.start()


    def receive(self):
        while self.running:
            data = (self.socket_robot.recv(2048))
            self.data = transform_data(data)

    def shutdown(self):
        self.running = False
        self.receive_thread.join()
        self.socket_robot.close()


######################### READING DATA FROM ROBOT ###############################
def transform_data_point(data, data_type):
    # message size is the first 4 bits being sent
    if data_type is 'message_size':
        data = data[:4]
        data = data.hex()
        if len(data) == 8:
            data = struct.unpack('!i', bytes.fromhex(data))[0]
            return data
        else:
            return 0
    else:
        num = cfg.DATA_MAP[data_type]
        data = data[num:num+8]
        data = data.hex() #convert the data from \x hex notation to plain hex
        if len(data) == 16:
            data = struct.unpack('!d', bytes.fromhex(data))[0]
            return data
        else:
            return 0


def transform_data(data):
    data_string = ''
    for data_type in cfg.DATA_MAP:
        data_point = transform_data_point(data, data_type)
        data_string += data_type + ':' + str(data_point) + ';'
    return data_string

