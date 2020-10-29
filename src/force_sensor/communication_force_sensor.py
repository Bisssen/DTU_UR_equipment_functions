import socket
import threading
import sys

from . import config_force_sensor

class communication_thread():
    def __init__(self, ip, port):
        # Whether the program is run in python 2 or not
        self.python_2 = (sys.version_info.major == 2)

        # Creating the socket
        self.socket = socket.socket(socket.AF_INET,
                                    socket.SOCK_STREAM)
        self.socket.connect((ip, port))

        # The thread keeps going as long as this variable is true
        self.running = True

        # The data which the thread optains from the ur
        self.data = {}

        # The thread which keeps receiving data
        self.receive_thread = threading.Thread(target=self.receive)

        # Starting the Thread
        print('    Starting communication thread...')
        self.receive_thread.start()

    def receive(self):
        while self.running:
            data = (self.socket.recv(2048))
            data = self.transform_data(data)

    def shutdown(self):
        self.running = False
        self.receive_thread.join()
        self.socket.close()

    def transform_data(self, data):
        # Decode the data if using python 3
        if not self.python_2:
            data = data.decode()
        # Remove brackets from the string
        data = data[1:-1]
        # Split the data
        data = data.split(" , ")
        # Save the data
        for i, item in enumerate(config_force_sensor.FORCE_LIST):
            self.data[item] = float(data[i])
