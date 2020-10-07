import socket
import struct
import threading

import config_robot as cfg

class communicationThread():
    def __init__(self):
            ########################## OPENING SOCKET TO ####################################
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
	    print(len(data))
            self.data = transformData(data)

    def shutdown(self):
        self.running = False
        self.receive_thread.join()
        self.socket_robot.close()


######################### READING DATA FROM ROBOT ###############################
def transformDataPoint(data, data_type):
    num = cfg.DATA_MAP[data_type]
    data = data[num:num+8]
    data = data.encode("hex") #convert the data from \x hex notation to plain hex
    if len(data) == 16:
        data = struct.unpack('!d', data.decode("hex"))[0]
        return data
    else:
        return 0


def transformData(data):
    data_string = ''
    for data_type in cfg.DATA_MAP:
        data_point = transformDataPoint(data, data_type)
        data_string += data_type + ':' + str(data_point) + ';'
    return data_string
