from enum import Enum
from maze_manager import *
from tcp_client import *

class ClientState(Enum):
    IDLE = 0 # Not doing anything.
    MOVING = 1 # Robot is in default navigation mode.
    JUNCTION = 2 # Waiting for results of junction mapping.

class Client:

    def __init__(self):
        self.manager = MazeManager()
        self.tcp_client = TCPClient()
        self.state = ClientState.IDLE
        self.pos = 0 # The last known position of the robot.
        self.angle = 0 # The last known orientation of the robot.
    
    # Reset to initial state.
    def reset(self):
        self.manager.reset()
        self.tcp_client.reset()
        self.state = ClientState.IDLE
        self.pos = 0
        self.angle = 0
    
    # Set up connections.
    def connect(self):
        self.tcp_client.connect_to_server()
        self.tcp_client.connect_to_robot()
    
    # Close all connections.
    def close(self):
        self.tcp_client.close()
    
    # Start the robot.
    def start(self):
        self.tcp_client.send_robot('s') # Tell the robot to start.
        self.state = ClientState.JUNCTION # The robot starts by surveying the maze; wait for the results.
    
    # Stop the robot.
    def stop(self):
        self.tcp_client.send_robot('e') # Tell the robot to stop.
        self.state = ClientState.IDLE
    
    # Process input messages from the server.
    def recv_server(self):
        message = self.tcp_client.recv_server()
        if message == '':
            return
        elif message == 's' and self.state == ClientState.IDLE:
            self.start()
        elif message == 'e' and self.state != ClientState.IDLE:
            self.stop()
        else:
            print('Error: received message ' + message + ' from server but current state is ' + self.state + '.')
    
    # Process input messages from the robot.
    def recv_robot(self):
        if self.state == ClientState.IDLE:
            return None
        message = self.tcp_client.recv_robot()
        if message == '':
            return None
        elif message[0] == 'm' and self.state == ClientState.MOVING:
            messages = message[1:].split(';')
            if len(messages) != 7:
                raise ValueError('Incorrect number of values received from robot (received ' + str(len(messages)) + ').')
            self.pos = (float(messages[0]), float(messages[1]))
            self.angle = float(messages[2])
            light = [float(messages[i]) for i in range(3, 7)] # Assume 4 light readings for the time being.
            command = self.manager.default_navigate(self.pos, self.angle, light)
            if command == 'j':
                self.state = ClientState.JUNCTION
            return command
        elif message[0] == 'j' and self.state == ClientState.JUNCTION:
            # TODO parse inputs in junction mode (depends on details of computer vision).
            # Use placeholders for now.
            self.pos = (0, 0)
            self.angle = 0
            light = [0, 0, 0, 0]
            walls = []
            self.manager.map_maze(walls)
            command = self.manager.junction_navigate(self.pos, self.angle, light)
            if command == 'e': # If we have reached the end.
                self.state = ClientState.IDLE
            else:
                self.state = ClientState.MOVING
            return command
        else:
            print('Error: received message ' + message + ' from robot but current state is ' + self.state + '.')
    
    # Send data to server.
    # TODO send relevant data to server.
    def send_to_server(self):
        pass
    
    # The main update loop.
    def update(self):
        
        # Parse inputs.
        self.recv_server()
        robot_command = self.recv_robot()
        
        # Robot commands:
        # s: start (enter moving mode).
        # e: end (go to idle).
        # j: scan at junction (enter junction mode).
        # f: move forwards.
        # b: move backwards.
        # {x} where x is an integer: rotate by x degrees. x>0: clockwise, else anticlockwise.
        
        # Send command to robot.
        if robot_command != None:
            self.tcp_client.send_robot(robot_command)
        
        # Send data to server.
        self.send_to_server()