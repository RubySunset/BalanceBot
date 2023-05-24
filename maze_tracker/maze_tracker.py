from maze_grid import *
from maze_graph import *

# Main maze tracker class. Instantiate and use this in other modules.
class MazeTracker:

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
        self.maze_grid = MazeGrid(self.X_RES, self.Y_RES)
        self.maze_graph = MazeGraph()
        # Other maze state data.
        self.is_initial = True # Initial state?
        self.reached_end = False # Have we reached the end once already?
        self.is_discovered = False # Have we discovered the entire layout of the maze?
        self.prev_command = 0 # The previous navigation command - used to force straight after turn.
        self.prev_pos = None # The previous robot position - used to determine when we enter/exit a cell.
        self.reverse_mode = False # Is the robot in reverse mode?
    
    # Reset the maze tracker to the initial state.
    def reset(self):
        self.maze_grid.reset()
        self.maze_graph.reset()
        self.is_initial = True
        self.reached_end = False
        self.is_discovered = False
        self.prev_command = 0
        self.prev_pos = None
        self.reverse_mode = False
    
    # Add a starting point.
    def add_start(self, pos):
        self.start_pos = pos
        self.maze_grid.add_start(pos)
        self.maze_graph.add_start(pos)
    
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

    # # Turn the robot to face into the maze at the start.
    # # This is to avoid some pathological cases that result in infinite loops with looped mazes.
    # def initial_calibration(self, pos, orientation):
    #     # Note that we assume the robot always starts at one of the edges of the maze.
    #     if pos[1] == 0:
    #         desired_orientation = 2
    #     elif pos[0] == self.X_RES:
    #         desired_orientation = 3
    #     elif pos[1] == self.Y_RES:
    #         desired_orientation = 0
    #     elif pos[0] == 0:
    #         desired_orientation = 1
    #     else:
    #         print('Warning: robot is not on the edge of the maze.')
    #     return self.correct_orientation(desired_orientation, orientation)
    
    # Inputs:
    # Robot position (x, y).
    # Orientation (number from 0-3 corresponding to north, east, south, west).
    # Light sensor readings (north, east, south, west).
    # Output: navigation command
    def update(self, input_pos, orientation, light):
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
        pos = (int(input_pos[0] / self.INPUT_X * self.X_RES), int(input_pos[1] / self.INPUT_Y * self.Y_RES))

        # Update the maze state if we are in the discovery phase.
        if not self.is_discovered:
            if self.is_initial: # Update starting node on first iteration.
                self.prev_pos = self.start_pos
                self.maze_grid.update_links(pos, links)
                self.maze_graph.update_prev_vertex(self.start_pos)
                print('Starting...')
            elif pos == self.end_pos and not self.reached_end: # If we have reached the end for the first time.
                self.reached_end = True
                self.maze_grid.update_links(pos, links)
                self.maze_grid.entry_mark(pos, r_orient)
                self.maze_graph.add_edge(pos)
                print('End reached, continue exploring...')
            # elif cell.type == CellType.START and self.reached_end == True: # If we have looped back to the start.
            #     # self.is_discovered = True
            #     self.add_edge(norm_pos)
            #     if self.prev_pos != self.start_pos:
            #         self.entry_mark(norm_pos, orientation)
            #     self.last_junction = self.start_pos
            #     # self.find_shortest_path()
            #     print('Looped back to start...')
            elif r_light[0] or not r_light[1] or not r_light[3] or pos in (self.start_pos, self.end_pos):
                # If we are not in a corridor, or at the start or end points.
                self.maze_grid.add_junction(pos, links)
                if self.prev_pos != pos:
                    self.maze_grid.entry_mark(pos, r_orient)
                self.maze_graph.add_edge(pos)
            else: # If we are in a corridor.
                self.maze_grid.add_passage(pos, r_orient)
            self.prev_pos = pos
            target_direction, self.is_discovered = self.maze_grid.discover_maze(pos, r_orient, r_light, self.is_initial, self.reached_end, self.is_discovered)
            discovery_command = self.correct_orientation(target_direction, r_orient)
            if self.is_discovered:
                self.maze_graph.generate_remaining_path(pos, self.end_pos)
        
        # Generate navigation command.
        if self.prev_command in (1, 2):
            self.prev_command = 0
            # If the last command was to turn, always follow by going forwards.
            # This is to prevent infinite loops that can occour in 3 or 4-way junctions.
        else:
            if self.is_discovered:
                if pos == self.end_pos:
                    self.prev_command = 4 # Stop if we have reached the end (again).
                else:
                    target_direction = self.maze_graph.follow_path(pos, r_orient, self.end_pos)
                    if target_direction == 4:
                        self.prev_command = 4
                    else:
                        self.prev_command = self.correct_orientation(target_direction, r_orient)
            else:
                self.prev_command = discovery_command
        
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
        
        # Update state.
        if not self.is_discovered and self.prev_command == 0:
            self.maze_grid.exit_mark(pos, r_orient)
        if self.is_initial:
            self.is_initial = False
        
        return r_command