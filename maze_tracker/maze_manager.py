from maze_tracker import *

class MazeManager:

    # Scaling of input x, y.
    INPUT_X = 1
    INPUT_Y = 1
    # Scaling of output x, y.
    OUTPUT_X = 1
    OUTPUT_Y = 1
    # Number of possible x, y coordinates (resolution).
    X_RES = 8
    Y_RES = 8

    def __init__(self):
        self.tracker = MazeTracker(self.X_RES, self.Y_RES)
        self.prev_pos = None
        self.prev_orient = None
        self.is_initial = True
    
    # Reset to initial state.
    def reset(self):
        self.tracker.reset()
        self.prev_pos = None
        self.prev_orient = None
        self.is_initial = True
    
    def add_start(self, pos):
        self.tracker.add_start(pos)
    
    def add_end(self, pos):
        self.tracker.add_end(pos)
    
    # Receive data from robot.
    def receive_data(self, input_pos, orient, light):
        pos = (int(input_pos[0] / self.INPUT_X * self.X_RES), int(input_pos[1] / self.INPUT_Y * self.Y_RES))
        if pos != self.prev_pos or orient != self.prev_orient or self.is_initial:
            self.is_initial = False
            self.prev_pos = pos
            self.prev_orient = orient
            command = self.tracker.update(pos, orient, light)
        else:
            command = -1 # No new command - continue with last command.
        # TODO send command to robot. For now, just return it from this function.
        return command
        # TODO send position and maze state to web server.
    
    # For testing purposes.
    def print_maze(self, pos, orient):
        self.tracker.maze_grid.print_maze(pos, orient)