from maze_grid import *
from maze_graph import *

# Main maze tracker class. Instantiate and use this in other modules.
class MazeTracker:

    def __init__(self, X_RES, Y_RES):
        self.X_RES = X_RES
        self.Y_RES = Y_RES
        self.maze_grid = MazeGrid(self.X_RES, self.Y_RES)
        # Other maze state data.
        self.is_initial = True # Initial state?
        self.is_finished = False # Are we finished (reached end and shortest path determined)?
        self.prev_command = 0 # The previous navigation command - used to force straight after turn.
        self.prev_pos = None # The previous robot position - used to determine when we enter/exit a cell.
        self.reverse_mode = False # Is the robot in reverse mode?
    
    # Reset the maze tracker to the initial state.
    def reset(self):
        self.maze_grid.reset()
        self.is_initial = True
        self.is_finished = False
        self.prev_command = 0
        self.prev_pos = None
        self.reverse_mode = False
    
    # Add a starting point.
    def add_start(self, pos):
        self.start_pos = pos
        self.maze_grid.add_start(pos)
    
    # Add an end point.
    def add_end(self, pos):
        self.end_pos = pos
        self.maze_grid.add_end(pos)
    
    # Generates a command to orient or move the robot to face or begin moving in the given direction.
    def correct_orientation(self, desired, actual):
        difference = (desired - actual) % 4
        if difference == 0:
            return 0
        elif difference == 1:
            return 2
        elif difference == 2:
            return 3
        elif difference == 3:
            return 1
        else:
            print('correct_orientation(): error, received desired direction ' + str(desired) + '.')
    
    # Updates the maze state based on the robot's position, orientation, and light readings.
    # Only call update when there is a change in either position or orientation.
    # Inputs:
    # Robot position (x, y).
    # Orientation (number from 0-3 corresponding to north, east, south, west).
    # Light sensor readings (north, east, south, west).
    # Output: navigation command
    def update(self, pos, orientation, light):
        # Internal navigation commands:
        # 0: forwards. 1: turn left. 2: turn right. 3: switch to reverse mode, then go forwards. 4: stop.
        # External navigation commands:
        # 0: forwards. 1: turn left. 2: turn right. 3: backwards. 4: stop.

        # Translate inputs to internal form.
        if self.reverse_mode:
            r_orient = (orientation + 2) % 4
            r_light = []
            for i in range(4):
                r_light.append(light[(i + 2) % 4])
        else:
            r_orient = orientation
            r_light = light

        # Normalise measurements.
        links = []
        for i in range(4):
            links.append(not r_light[(i - r_orient) % 4])
        
        # Update the maze state before navigating.
        if self.is_initial: # Update starting node on first iteration.
            self.prev_pos = self.start_pos
            self.maze_grid.visit_cell(pos, links)
            self.maze_grid.flood()
            print('Starting...')
        elif pos == self.end_pos and self.prev_pos != pos: # If we have just reached the end.
            self.maze_grid.visit_cell(pos, links)
            self.maze_grid.flood()
            if self.maze_grid.test_end():
                self.is_finished = True
        else:
            self.maze_grid.visit_cell(pos, links)
            self.maze_grid.flood()
        self.prev_pos = pos

        # Generate navigation command.
        if self.prev_command in (1, 2):
            self.prev_command = 0 # If the last command was to turn, always follow by going forwards (no need to recompute).
        elif self.is_finished:
            self.prev_command = 4 # Stop when end is reached.
        else:
            target_direction = self.maze_grid.navigate(pos)
            self.prev_command = self.correct_orientation(target_direction, r_orient)
        
        # Translate navigation command to external form.
        if self.prev_command == 3:
            self.reverse_mode = not self.reverse_mode
            r_orient = (r_orient + 2) % 4
            self.prev_command = 0
        if self.reverse_mode:
            if self.prev_command == 0:
                r_command = 3
            elif self.prev_command == 1:
                r_command = 1
            elif self.prev_command == 2:
                r_command = 2
            elif self.prev_command == 4:
                r_command = 4
            else:
                print('Invalid command.')
        else:
            r_command = self.prev_command
        # TODO explore the other option for turning, where the robot switches modes.
        
        # Update the maze state after navigating.
        if self.is_initial:
            self.is_initial = False
        
        return r_command