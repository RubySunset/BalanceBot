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

# Main class for manipulating the maze grid. Import into other modules.
class MazeGrid:

    def __init__(self, X_RES, Y_RES):
        self.X_RES = X_RES
        self.Y_RES = Y_RES
        self.cell_grid = []
        for i in range(self.X_RES):
            row = []
            for j in range(self.Y_RES):
                row.append(Cell(CellType.EMPTY, [0, 0, 0, 0], [0, 0, 0, 0]))
            self.cell_grid.append(row)
    
    # Reset to initial state.
    def reset(self):
        for i in range(self.X_RES):
            for j in range(self.Y_RES):
                self.cell_grid[i][j] = Cell(CellType.EMPTY, [0, 0, 0, 0], [0, 0, 0, 0])
    
    # Returns the cell at the given position.
    def cell(self, pos):
        return self.cell_grid[pos[0]][pos[1]]
    
    # Change the type, and optionally the other attributes, of the given cell.
    def mutate(self, pos, type, new_links=None, new_marks=None):
        cell = self.cell(pos)
        cell.type = type
        if new_links != None:
            cell.links = new_links
        if new_marks != None:
            cell.marks = new_marks
    
    # Updates a cell's links.
    def update_links(self, pos, links):
        self.cell_grid[pos[0]][pos[1]].links = links
    
    # Adds a mark at the link of the given cell.
    def add_mark(self, pos, link):
        self.cell_grid[pos[0]][pos[1]].marks[link] += 1
    
    # Identifies a given cell as the start. We assume that we do not know the links of the starting point in advance.
    def add_start(self, pos):
        self.mutate(pos, CellType.START)
    
    # Identifies a given cell as the end. We assume that we do not know the links of the starting point in advance.
    def add_end(self, pos):
        self.mutate(pos, CellType.END)
    
    # Add a mark when entering a junction with the given orientation.
    def entry_mark(self, pos, orientation):
        self.add_mark(pos, (orientation + 2) % 4)
    
    # Add a mark when exiting a valid cell, i.e. junction or start/end.
    def exit_mark(self, pos, orientation):
        cell = self.cell(pos)
        if cell.type not in (CellType.EMPTY, CellType.PASSAGE):
            self.add_mark(pos, orientation)
    
    # Adds a new junction to the maze grid if one does not already exist in that cell.
    def add_junction(self, pos, links):
        if self.cell_grid[pos[0]][pos[1]].type == CellType.EMPTY:
            self.mutate(pos, CellType.JUNCTION, new_links=links)
    
    # Adds a new passage to the maze grid if one does not already exist in that cell.
    def add_passage(self, pos, orientation):
        if self.cell(pos).type == CellType.EMPTY:
            if orientation in (0, 2):
                self.mutate(pos, CellType.PASSAGE, new_links=[1, 0, 1, 0])
            else:
                self.mutate(pos, CellType.PASSAGE, new_links=[0, 1, 0, 1])
    
    # Navigate using the modified Tremaux algorithm in order to discover the maze.
    # Note that light is taken relative to the robot to determine corridors.
    def navigate(self, pos, orientation, light, is_initial):

        # Continue forwards through a corridor.
        if not light[0] and light[1] and light[3]:
            return orientation
        
        cell = self.cell(pos)
        links = cell.links
        marks = cell.marks

        if is_initial:
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
        candidate = None # A possible direction if all other entrances are unmarked.
        for i in range(4):
            if links[i]:
                num_entrances += 1
                if i == entry_mark:
                    continue
                elif marks[i] == 0 and candidate == None:
                    candidate = i
                elif marks[i] != 0:
                    others_unmarked = False
                    break
        if others_unmarked and num_entrances > 1:
            # We can only choose another entrance if there is another entrance to choose from...
            return candidate
        elif marks[entry_mark] < 2:
            return entry_mark
        else:
            min_link = None
            min_val = math.inf
            for i in range(4):
                if links[i] and marks[i] < min_val:
                    min_link = i
                    min_val = marks[i]
            return min_link
        
    # Find the Manhattan distance between two nodes.
    def manhattan_dist(self, source, dest):
        return abs(source[0] - dest[0]) + abs(source[1] - dest[1])
    
    # Finds the position of the cell connected to the cell with the given position via the given link.
    def adj_pos(self, pos, link):
        if link == 0:
            return (pos[0], pos[1] - 1)
        elif link == 1:
            return (pos[0] + 1, pos[1])
        elif link == 2:
            return (pos[0], pos[1] + 1)
        elif link == 3:
            return (pos[0] - 1, pos[1])
        else:
            print('adj_pos(): invalid link ' + str(link) + '.')

    # Test to see if we have discovered enough of the maze to determine the shortest path.
    def enough_discovered(self, end, dist_tree):
        enough = True
        for i in range(self.X_RES):
            for j in range(self.Y_RES):
                cell = self.cell_grid[i][j]
                if cell.type not in (CellType.JUNCTION, CellType.START):
                    continue # Only consider junctions and start point.
                for k in range(4):
                    if cell.links[k] and cell.marks[k] == 0:
                        adj_pos = self.adj_pos((i, j), k)
                        min_dist = dist_tree[(i, j)] + 1 + self.manhattan_dist(adj_pos, end)
                        if min_dist < dist_tree[end]:
                            enough = False
                            break
                if not enough:
                    break
            if not enough:
                break
        if enough:
            print('Sufficient portion of maze discovered!')
        return enough
    
    # Prints the maze grid in the console. Optionally also show the position and orientation of the robot.
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
                    cell = self.cell_grid[i][j]
                    if cell.type == CellType.EMPTY:
                        print('· ', end='')
                    elif cell.type == CellType.PASSAGE:
                        chars = ('──', '│ ')
                        print(chars[cell.links[0]], end='')
                    elif cell.type == CellType.JUNCTION:
                        k = 8*cell.links[0] + 4*cell.links[1] + 2*cell.links[2] + cell.links[3]
                        # chars = ('?', '⫞', '⫟', '◹', '⊦', '?', '◸', '▽', '⫠', '◿', '?', '◁', '◺', '△', '▷', '+')
                        chars = ('? ', '╢ ', '╤ ', '┐ ', '╟─', '? ', '┌─', '┬─', '╧ ', '┘ ', '? ', '┤ ', '└─', '┴─', '├─', '┼─')
                        print(chars[k], end='')
                    elif cell.type == CellType.START:
                        print('s ', end='')
                    elif cell.type == CellType.END:
                        print('e ', end='')
            print()
        print('--' * self.Y_RES)