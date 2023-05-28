from maze_manager import *

class MazeTop:

    # Size of arena.
    X_LIM = 3.6
    Y_LIM = 2.4

    def __init__(self):
        self.manager = MazeManager(self.X_LIM, self.Y_LIM)
    
    # Reset to initial state.
    def reset(self):
        self.manager.reset()
    
    # Set a start position.
    def set_start(self, pos):
        self.manager.set_start(pos)
    
    # Set an end position.
    def set_end(self, pos):
        self.manager.set_end(pos)
    
    # Receive data from robot in default mode.
    def receive_default(self, pos, angle, light):
        command = self.manager.default_navigate(pos, angle, light)
        # TODO send command to robot.
    
    # Receive data from robot in junction mode.
    def receive_junction(self, pos, angle, light, walls):
        self.manager.map_maze(walls)
        command = self.manager.junction_navigate(pos, angle, light)
        # TODO send command to robot.