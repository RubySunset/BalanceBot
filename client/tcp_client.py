import socket

class TCPClient:

    # Connection information.
    # TODO fill this in.
    SERVER_NAME = ''
    SERVER_PORT = 0
    ROBOT_NAME = ''
    ROBOT_PORT = 0

    BUFFER_SIZE = 1024
    TIMEOUT = 0

    def __init__(self):
        # Use TCP for all connections.
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print('Sockets initalised.')
    
    def reset(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print('TCP client reset.')
    
    def connect_to_server(self):
        print('Connecting to server...')
        self.server_socket.connect((self.SERVER_NAME, self.SERVER_PORT))
        print('Connected to server.')
        self.server_socket.settimeout(self.TIMEOUT)
        # Set the server socket to non-blocking so that we can skip if there are no messages to receive.
    
    def connect_to_robot(self):
        print('Connecting to robot...')
        self.robot_socket.connect((self.ROBOT_NAME, self.ROBOT_PORT))
        print('Connected to robot.')
        self.robot_socket.settimeout(self.TIMEOUT)
    
    def send_server(self, message):
        self.server_socket.send(message.encode())
    
    def send_robot(self, message):
        self.robot_socket.send(message.encode())
    
    def recv_server(self):
        message = ''
        while True:
            try:
                message += self.server_socket.recv(self.BUFFER_SIZE).decode()
            except:
                break
        return message

    def recv_robot(self):
        message = ''
        while True:
            try:
                message += self.robot_socket.recv(self.BUFFER_SIZE).decode()
            except:
                break
        return message
    
    def close(self):
        self.server_socket.close()
        self.robot_socket.close()
        print('Closed all connections.')