import math
from enum import Enum

# Maze grid cell type.
class CellType(Enum):
    EMPTY = 0 # Unvisited cell
    PASSAGE = 1
    JUNCTION = 2
    START = 3
    END = 4

# Maze grid cell.
# Type: one of CellType's values.
# Links: boolean[4] recording the presence of each of the 4 links to other cells.
# Marks: int[4] recording the number of marks on each entrance/exit to/from this cell (used by Tremaux's algorithm).
class Cell:
    def __init__(self, type, links, marks):
        self.type = type
        self.links = links
        self.marks = marks

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
        # Initialise maze grid (a convenient representation of the maze constructed in the discovery phase).
        self.maze_grid = []
        for i in range(self.X_RES):
            row = []
            for j in range(self.Y_RES):
                row.append(Cell(CellType.EMPTY, [], []))
            self.maze_grid.append(row)
        # Initialise maze graph adjacency list (used by pathfinding after discovery).
        self.maze_graph = {}
        # Initialise shortest path.
        self.shortest_path = []
        self.path_index = 0 # The next vertex on the path we aim to reach.
        # Other maze state data.
        self.is_initial = True # Initial state?
        self.reached_end = False # Have we reached the end once already?
        self.is_discovered = False # Have we discovered the entire layout of the maze?
        self.prev_command = 0 # The previous navigation command - used to force straight after turn.
        self.prev_vertex = None # The previous vertex (start, junction, end) - used to add edges to the maze graph.
        self.prev_pos = None # The previous robot position - used to determine when we enter/exit a cell.
        self.reverse_mode = False # Is the robot in reverse mode?
    
    # Reset the maze tracker to the state it was in after initialisation.
    def reset(self):
        for i in range(self.X_RES):
            for j in range(self.Y_RES):
                self.maze_grid[i][j] = Cell(CellType.EMPTY, [], [])
        self.maze_graph = {}
        self.shortest_path = []
        self.path_index = 0
        self.is_initial = True
        self.reached_end = False
        self.is_discovered = False
        self.prev_command = 0
        self.prev_vertex = None
        self.prev_pos = None
        self.reverse_mode = False
    
    # Promotes a cell to the given type in the maze grid.
    def promote_cell(self, pos, type, links):
        self.maze_grid[pos[0]][pos[1]] = Cell(type, links, [0, 0, 0, 0])
    
    # Add a mark when entering a junction with the given orientation.
    def entry_mark(self, pos, orientation):
        self.maze_grid[pos[0]][pos[1]].marks[(orientation + 2) % 4] += 1
    
    # # Add a mark when exiting a junction with the given orientation.
    # def exit_mark(self, pos, orientation):
    #     self.maze_grid[pos[0]][pos[1]].marks[orientation] += 1
    
    # Add an edge to the maze graph if it is valid.
    def add_edge(self, pos):
        if pos != self.prev_vertex and pos not in self.maze_graph[self.prev_vertex]: # Avoid self-edges and duplicate edges.
            self.maze_graph[self.prev_vertex].append(pos)
            if pos in self.maze_graph:
                self.maze_graph[pos].append(self.prev_vertex)
            else:
                self.maze_graph[pos] = [self.prev_vertex]
    
    # Add a starting point. We assume that we do not know the links of the starting point in advance.
    def add_start(self, pos):
        self.start_pos = pos
        self.promote_cell(pos, CellType.START, [])
        self.maze_graph[pos] = []
    
    # Add an end point. We assume that we do not know the links of the end point in advance.
    def add_end(self, pos):
        self.end_pos = pos
        self.promote_cell(pos, CellType.END, [])
    
    # Adds a new junction to the maze grid if one does not already exist in that cell.
    def add_junction(self, pos, links):
        if self.maze_grid[pos[0]][pos[1]].type == CellType.EMPTY:
            self.promote_cell(pos, CellType.JUNCTION, links)
    
    # Adds a new passage to the maze grid if one does not already exist in that cell.
    def add_passage(self, pos, orientation):
        if self.maze_grid[pos[0]][pos[1]].type == CellType.EMPTY:
            if orientation in (0, 2):
                self.promote_cell(pos, CellType.PASSAGE, [1, 0, 1, 0])
            else:
                self.promote_cell(pos, CellType.PASSAGE, [0, 1, 0, 1])
    
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
    
    # # Navigate using the wall follower algorithm in order to discover the maze.
    # def wall_follower(self, light):
    #     # Note that we use relative light readings to generate navigation commands using the wall follower algorithm.
    #     if not light[3]: # No boundary to left.
    #         return 1
    #     elif light[0] and not light[1]: # Boundary in front but none to the right.
    #         return 2
    #     elif light[0] and light[1]: # Boundaries everywhere except behind.
    #         return 3
    #     else:
    #         return 0
    
    # Navigate using the modified Tremaux algorithm in order to discover the maze.
    # Note that light is taken relative to the robot to determine corridors.
    def discover_maze(self, pos, orientation, light):

        # Continue forwards through a corridor.
        if not light[0] and light[1] and light[3]:
            return 0
        # Note that corridors have no need for mark updation.
        
        links = self.maze_grid[pos[0]][pos[1]].links
        marks = self.maze_grid[pos[0]][pos[1]].marks

        if self.is_initial:
            for i in range(4):
                if links[i]:
                    entry_mark = i # Consider the robot to initially have entered through a valid link.
                    break
        else:
            entry_mark = (orientation + 2) % 4

        # Apply Tremaux's algorithm.
        target_direction = 0
        others_unmarked = True # Are all the other entrances unmarked?
        num_entrances = 0
        for i in range(4):
            if links[i]:
                num_entrances += 1
            if links[i] and marks[i] != 0 and i != entry_mark:
                others_unmarked = False
                break
        if others_unmarked and num_entrances > 1:
            # We can only choose another entrance if there is another entrance to choose from...
            for i in range(4):
                if links[i] and i != entry_mark:
                    target_direction = i
                    break
        elif marks[entry_mark] < 2:
            target_direction = entry_mark
        else:
            min_link = None
            min_val = math.inf
            for i in range(4):
                if links[i] and marks[i] < min_val:
                    min_link = i
                    min_val = marks[i]
            target_direction = min_link

        # Check if we have explored the entire maze.
        if self.reached_end:
            self.is_discovered = True
            for i in range(self.X_RES):
                for j in range(self.Y_RES):
                    cell = self.maze_grid[i][j]
                    if cell.type in (CellType.EMPTY, CellType.PASSAGE):
                        continue # Only consider junctions, start, end.
                    for k in range(4):
                        if cell.links[k] and cell.marks[k] == 0:
                            self.is_discovered = False
                            break
                    if not self.is_discovered:
                        break
                if not self.is_discovered:
                    break
        if self.is_discovered:
            print('Maze discovered!')
        
        return self.correct_orientation(target_direction, orientation)
    
    # Compute the shortest path from given source to destination using Dijkstra's algorithm.
    # TODO replace Dijkstra's algorithm with A* search.
    def find_shortest_path(self, source, dest):
        # Construct helper objects.
        distance = {}
        prev = {}
        unvisited = []
        for vertex in self.maze_graph:
            distance[vertex] = math.inf
            prev[vertex] = None
            unvisited.append(vertex)
        # Construct shortest-distance tree.
        distance[source] = 0
        while len(unvisited) > 0:
            current = unvisited[0]
            if current == dest:
                break
            for vertex in unvisited:
                if distance[vertex] < distance[current]:
                    current = vertex
            unvisited.remove(current)
            for neighbour in self.maze_graph[current]:
                if neighbour not in unvisited:
                    continue
                alt_dist = distance[current] + math.dist(current, neighbour)
                if alt_dist < distance[neighbour]:
                    distance[neighbour] = alt_dist
                    prev[neighbour] = current
        # Determine shortest path to end node.
        shortest_path = []
        node = self.end_pos
        while node != None:
            shortest_path.insert(0, node)
            node = prev[node]
        return shortest_path
    
    # Navigate to the next stop on the shortest path.
    def follow_path(self, pos, orientation):
        next = self.shortest_path[self.path_index]
        if pos == next and next != self.end_pos:
            self.path_index += 1
            next = self.shortest_path[self.path_index]
            if pos[1] > next[1]:
                desired_orientation = 0
            elif pos[0] < next[0]:
                desired_orientation = 1
            elif pos[1] < next[1]:
                desired_orientation = 2
            elif pos[0] > next[0]:
                desired_orientation = 3
            return self.correct_orientation(desired_orientation, orientation)
        elif pos == next and next == self.end_pos:
            return 4
        else:
            return 0
    
    # Generate the complete shortest path from start to end, to send somewhere.
    def generate_complete_path(self):
        return self.find_shortest_path(self.start_pos, self.end_pos)
    
    # Generate the shortest path from start to robot's position, to send somewhere.
    def generate_partial_path(self, pos):
        return self.find_shortest_path(self.start_pos, pos)

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

        # Generate links to adjacent cells with absolute reference.
        links = []
        for i in range(4):
            links.append(not r_light[(i - r_orient) % 4])
        # Identify the current cell.
        norm_pos = (int(pos[0] / self.INPUT_X * self.X_RES), int(pos[1] / self.INPUT_Y * self.Y_RES))
        cell = self.maze_grid[norm_pos[0]][norm_pos[1]]

        # Update the maze state if we are in the discovery phase.
        if not self.is_discovered:
            if self.is_initial: # Update starting node on first iteration.
                cell.links = links
                self.prev_vertex = self.start_pos
                self.prev_pos = self.start_pos
                print('Starting...')
            elif cell.type == CellType.END and not self.reached_end: # If we have reached the end for the first time.
                cell.links = links
                self.reached_end = True
                self.add_edge(norm_pos)
                self.entry_mark(norm_pos, r_orient)
                self.prev_vertex = self.end_pos
                print('End reached, continue exploring...')
            # elif cell.type == CellType.START and self.reached_end == True: # If we have looped back to the start.
            #     # self.is_discovered = True
            #     self.add_edge(norm_pos)
            #     if self.prev_pos != self.start_pos:
            #         self.entry_mark(norm_pos, orientation)
            #     self.last_junction = self.start_pos
            #     # self.find_shortest_path()
            #     print('Looped back to start...')
            elif r_light[0] or not r_light[1] or not r_light[3] or cell.type in (CellType.START, CellType.END):
                # If we are not in a corridor, or at the start or end points.
                self.add_junction(norm_pos, links)
                self.add_edge(norm_pos)
                self.prev_vertex = norm_pos
                if self.prev_pos != norm_pos:
                    self.entry_mark(norm_pos, r_orient)
            else: # If we are in a corridor.
                self.add_passage(norm_pos, orientation)
            self.prev_pos = norm_pos
            discovery_command = self.discover_maze(norm_pos, r_orient, r_light)
        
        # Find the shortest path from current position to destination, when needed.
        if self.is_discovered and len(self.shortest_path) == 0:
            self.shortest_path = self.find_shortest_path(norm_pos, self.end_pos)
        
        # Generate navigation command.
        cell = self.maze_grid[norm_pos[0]][norm_pos[1]] # Re-evaluate cell as it may have been overwritten.
        if self.prev_command in (1, 2):
            self.prev_command = 0
            # If the last command was to turn, always follow by going forwards.
            # This is to prevent infinite loops that can occour in 3 or 4-way junctions.
        else:
            if self.is_discovered:
                if cell.type == CellType.END:
                    self.prev_command = 4 # Stop if we have reached the end (again).
                else:
                    self.prev_command = self.follow_path(norm_pos, r_orient)
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
        if cell.type not in (CellType.EMPTY, CellType.PASSAGE) and not self.is_discovered and self.prev_command == 0:
            cell.marks[r_orient] += 1 # Add an exit mark just as we are moving away from this junction.
        if self.is_initial:
            self.is_initial = False
        
        return r_command
    
    # Prints the maze in the console. Optionally also show the position and orientation of the robot.
    def print_maze(self, pos=None, orientation=None):
        print('--' * self.Y_RES)
        for j in range(self.Y_RES):
            for i in range(self.X_RES):
                if pos != None and pos[0] == i and pos[1] == j:
                    if orientation == None:
                        print('o ', end='')
                    else:
                        chars = ('↑', '→', '↓', '←')
                        print(chars[orientation] + ' ', end='')
                else:
                    cell = self.maze_grid[i][j]
                    if cell.type == CellType.EMPTY:
                        print('. ', end='')
                    elif cell.type == CellType.PASSAGE:
                        chars = ('──', '│ ')
                        print(chars[cell.links[0]], end='')
                    elif cell.type == CellType.JUNCTION:
                        k = 8*cell.links[0] + 4*cell.links[1] + 2*cell.links[2] + cell.links[3]
                        # chars = ('?', '⫞', '⫟', '◹', '⊦', '?', '◸', '▽', '⫠', '◿', '?', '◁', '◺', '△', '▷', '+')
                        chars = ('? ', '╢ ', '╤ ', '┐ ', '╟─', '? ', '┌─', '┬─', '╧', '┘ ', '? ', '┤ ', '└─', '┴─', '├─', '┼─')
                        print(chars[k], end='')
                    elif cell.type == CellType.START:
                        print('s ', end='')
                    elif cell.type == CellType.END:
                        print('e ', end='')
            print()
        print('--' * self.Y_RES)