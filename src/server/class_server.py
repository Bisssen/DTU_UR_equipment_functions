import socket
import sys

class Server():
    def __init__(self, port=4444, name='No name given'):
        # Whether the program is run in python 2 or not
        self.python_2 = (sys.version_info.major == 2)

        # Define the port which the socket is created on
        self.port = port

        # The name of the server (To identify them if multiple is running)
        self.name = name

        # Create the socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Allow the port to be reused
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('', self.port))
        # Allow only one to connect for now
        self.socket.listen(1)
        # Set the socket to not blocking
        self.socket.setblocking(0)

        self.client_socket = None

        # Bool that keeps track of whether or not a client have been accepted
        self.client_accepted = False

    # The main loop of the server
    def main_loop(self, data=None):
        # If there is no client socket then accept a new one
        if self.client_socket is None:
            self.accept_client()
        # Otherwise interact with the client socket
        else:    
            self.send_data(data)
            return self.read_data()

    # Function that attempts to accept a socket, if there is one waiting
    def accept_client(self):
        # Try to accept a client if there is any
        if self.python_2:
            try:
                (client, _) = self.socket.accept()
            except socket.error:
                return
        else:
            try:
                (client, _) = self.socket.accept()
            except BlockingIOError:
                return   

        # Save the client
        self.client_socket = client

        # Notify the user
        print('Client connected on server: ' + str(self.name))

    # Function which reads data from the client socket, if there is any
    def read_data(self):
        # Try to receive data if there is any
        if self.python_2:
            try:
                data = self.client_socket.recv(4096)
            except socket.error:
                return
        else:
            try:
                data = self.client_socket.recv(4096)
            except BlockingIOError:
                return

        # Check if the socket have disconnected 
        if len(data) == 0:
            # Notify the user
            print('Client disconnected from server: ' + str(self.name))
            # And reset the client socket
            self.client_socket = None

        return data

    # Function which sends data to the client socket, if there is any
    def send_data(self, data):
        # If there is not data, then do nothing
        if data is None:
            return
        if self.python_2:
            try:
                self.client_socket.send(data)
            except socket.error:
                print("Server: " + str(self.name) + " is sending too much data")
        else:
            try:
                self.client_socket.send(data)
            except BlockingIOError:
                print("Server: " + str(self.name) + " is sending too much data")

