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
    
    # Reset to initial state.
    def reset(self):
        self.manager.reset()
        self.tcp_client.reset()
        self.state = ClientState.IDLE
    
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
    
    # Parse input message from the server.
    # Input message: s(start), e(end).
    def parse_server(self):
        message = self.tcp_client.recv_server()
        if message == '':
            return
        elif message == 's' and self.state == ClientState.IDLE:
            self.start()
        elif message == 'e' and self.state != ClientState.IDLE:
            self.stop()
        else:
            print('Error: received message ' + message + ' from server but current state is ' + self.state + '.')
    
    # Parse input message from the robot and generate a command.
    # Input message: m{...} or j{...} depending on whether we are in moving or junction mode.
    def parse_robot(self):
        if self.state == ClientState.IDLE:
            return None
        message = self.tcp_client.recv_robot()
        if message == '':
            return None
        elif message[0] == 'm' and self.state == ClientState.MOVING:
            messages = message[1:].split(';')
            if len(messages) != 7:
                raise ValueError('Incorrect number of values received from robot (received ' + str(len(messages)) + ').')
            pos = (float(messages[0]), float(messages[1]))
            angle = float(messages[2])
            light = [float(messages[i]) for i in range(3, 7)] # Assume 4 light readings for the time being.
            return self.manager.default_navigate(pos, angle, light)
        elif message[0] == 'j' and self.state == ClientState.JUNCTION:
            # TODO parse inputs in junction mode (depends on details of computer vision).
            # Use placeholders for now.
            pos = (0, 0)
            angle = 0
            light = [0, 0, 0, 0]
            walls = []
            self.manager.map_maze(walls)
            self.state = ClientState.MOVING # Go back to the default moving state.
            return self.manager.junction_navigate(pos, angle, light)
        else:
            print('Error: received message ' + message + ' from robot but current state is ' + self.state + '.')
    
    # The main update loop.
    def update(self):
        
        # Parse inputs.
        self.parse_server()
        robot_command = self.parse_robot()

        if robot_command == 'j':
            self.state = ClientState.JUNCTION
        
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