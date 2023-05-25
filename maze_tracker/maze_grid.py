import math
from enum import Enum

# Maze grid cell type.
class CellType(Enum):
    EMPTY = 0 # Unvisited cell.
    VISITED = 1 # Visited but not start or end.
    START = 2
    END = 3

# Maze grid cell.
# type: one of CellType's values.
# links: boolean[4] recording the presence of each of the 4 links to other cells.
# f_num: the number assigned to this cell by flooding.
# Marks: int[4] recording the number of marks on each entrance/exit to/from this cell (used by Tremaux's algorithm).
class Cell:
    def __init__(self, type, links, f_num):
        self.type = type
        self.links = links
        self.f_num = f_num

# Main class for manipulating the maze grid. Import into other modules.
class MazeGrid:

    def __init__(self, X_RES, Y_RES):
        self.X_RES = X_RES
        self.Y_RES = Y_RES
        self.cell_grid = []
        for i in range(self.X_RES):
            row = []
            for j in range(self.Y_RES):
                links = [j != 0, i != X_RES - 1, j != Y_RES - 1, i != 0]
                # Initially assume there are no maze walls except for the outer boundaries.
                row.append(Cell(CellType.EMPTY, links, 0))
            self.cell_grid.append(row)
        self.shortest_path = []
        # Other maze state data.
        self.is_backtracking = False # Are we in the process of backtracking?
        self.backtrack_target = None # Where do we want to backtrack to?
        self.current_path = [] # The conceptual path we have traversed from start to current position.
    
    # Reset to initial state.
    def reset(self):
        for i in range(self.X_RES):
            for j in range(self.Y_RES):
                links = [j != 0, i != self.X_RES - 1, j != self.Y_RES - 1, i != 0]
                self.cell_grid[i][j] = Cell(CellType.EMPTY, links, 0)
        self.shortest_path = []
        self.is_backtracking = False
        self.backtrack_target = None
        self.current_path = []
    
    # Identifies a given cell as the start. We assume that we do not know the links of the starting point in advance.
    def add_start(self, pos):
        self.start_pos = pos
        self.cell_grid[pos[0]][pos[1]].type = CellType.START
        self.current_path.append(self.start_pos)
    
    # Identifies a given cell as the end. We assume that we do not know the links of the starting point in advance.
    def add_end(self, pos):
        self.end_pos = pos
        self.cell_grid[pos[0]][pos[1]].type = CellType.END
    
    # Visit a cell and update its links.
    def visit_cell(self, pos, links):
        cell = self.cell_grid[pos[0]][pos[1]]
        cell.links = links
        if cell.type == CellType.EMPTY:
            cell.type = CellType.VISITED
        if self.is_backtracking:
            if pos == self.backtrack_target:
                self.is_backtracking = False
            elif pos == self.current_path[-1]:
                self.current_path.pop()
        elif not self.is_backtracking and pos != self.current_path[-1]:
            self.current_path.append(pos)
    
    # Returns the position of the cell connected to the given cell by a given link.
    def adjacent_cell(self, pos, link):
        if link == 0:
            return (pos[0], pos[1] - 1)
        elif link == 1:
            return (pos[0] + 1, pos[1])
        elif link == 2:
            return (pos[0], pos[1] + 1)
        elif link == 3:
            return (pos[0] - 1, pos[1])
        else:
            print('adjacent_cell(): invalid link ' + str(link) + '.')
    
    # Flood the maze.
    def flood(self):
        for i in range(self.X_RES):
            for j in range(self.Y_RES):
                self.cell_grid[i][j].f_num = math.inf # Initially assume all cells are at infinity.
        self.cell_grid[self.end_pos[0]][self.end_pos[1]].f_num = 0 # End cell has flood number 0.
        queue = [] # Use a queue since flooding is a breadth-first operation.
        queue.append(self.end_pos)
        while len(queue) > 0:
            pos = queue.pop(0)
            cell = self.cell_grid[pos[0]][pos[1]]
            for i in range(4):
                if not cell.links[i]:
                    continue
                adj_pos = self.adjacent_cell(pos, i)
                adj_cell = self.cell_grid[adj_pos[0]][adj_pos[1]]
                if adj_cell.links[(i + 2) % 4] and adj_cell.f_num == math.inf:
                    # Both cells must have links to each other. (*)
                    # Only replace this cell's flood number if it has not already been replaced.
                    adj_cell.f_num = cell.f_num + 1
                    queue.append(adj_pos)
        # (*) it is possible for an undiscovered cell to appear to have a link to a cell with a wall facing it,
        # since undiscovered cells are assumed to have all links (apart from the outer boundaries).
        # Therefore we must check both cells for links.

    # Find the target direction.
    def navigate(self, pos):
        if self.is_backtracking:
            target = self.current_path[-1]
            desired_orientation = 0
            if pos[1] > target[1]:
                desired_orientation = 0
            elif pos[0] < target[0]:
                desired_orientation = 1
            elif pos[1] < target[1]:
                desired_orientation = 2
            elif pos[0] > target[0]:
                desired_orientation = 3
            return desired_orientation
        else:
            cell = self.cell_grid[pos[0]][pos[1]]
            min_val = math.inf
            min_link = 0
            for i in range(4):
                if cell.links[i]:
                    adj_pos = self.adjacent_cell(pos, i)
                    adj_cell = self.cell_grid[adj_pos[0]][adj_pos[1]]
                    if adj_cell.f_num < min_val:
                        min_val = adj_cell.f_num
                        min_link = i
            return min_link

    # Test to see if we are finished, or if there could be a shorter path.
    def test_end(self):
        pos = self.start_pos
        while pos != self.end_pos:
            cell = self.cell_grid[pos[0]][pos[1]]
            next_pos = None
            for i in range(4):
                if cell.links[i]:
                    adj_pos = self.adjacent_cell(pos, i)
                    adj_cell = self.cell_grid[adj_pos[0]][adj_pos[1]]
                    if adj_cell.f_num == cell.f_num - 1 and adj_cell.type != CellType.EMPTY:
                        next_pos = adj_pos
                        break
            if next_pos == None:
                self.is_backtracking = True
                self.backtrack_target = pos
                self.current_path.pop() # To remove the node we are currently on, the end node.
                print('Alternate path found, begin backtracking...')
                print(self.current_path)
                print(self.backtrack_target)
                return False
            pos = next_pos
        self.find_shortest_path()
        return True

    # Find the shortest path at the end using the flood numbers.
    def find_shortest_path(self):
        pos = self.start_pos
        self.shortest_path = []
        self.shortest_path.append(pos)
        while pos != self.end_pos:
            cell = self.cell_grid[pos[0]][pos[1]]
            for i in range(4):
                if cell.links[i]:
                    next_pos = self.adjacent_cell(pos, i)
                    if self.cell_grid[next_pos[0]][next_pos[1]].f_num < cell.f_num:
                        self.shortest_path.append(next_pos)
                        pos = next_pos
                        break

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
                    elif cell.type == CellType.START:
                        print('s ', end='')
                    elif cell.type == CellType.END:
                        print('e ', end='')
                    else:
                        k = 8*cell.links[0] + 4*cell.links[1] + 2*cell.links[2] + cell.links[3]
                        # chars = ('?', '⫞', '⫟', '◹', '⊦', '?', '◸', '▽', '⫠', '◿', '?', '◁', '◺', '△', '▷', '+')
                        chars = ('? ', '╢ ', '╤ ', '┐ ', '╟─', '──', '┌─', '┬─', '╧ ', '┘ ', '│ ', '┤ ', '└─', '┴─', '├─', '┼─')
                        print(chars[k], end='')
            print()
        print('--' * self.Y_RES)
    
    # Navigate using the modified Tremaux algorithm in order to discover the maze.
    # Note that light is taken relative to the robot to determine corridors.
    def discover_maze(self, pos, orientation, light, is_initial, reached_end, was_discovered):
        # Continue forwards through a corridor.
        # Note that corridors have no need for mark updation.
        if not light[0] and light[1] and light[3]:
            return orientation, was_discovered
        
        links = self.cell_grid[pos[0]][pos[1]].links
        marks = self.cell_grid[pos[0]][pos[1]].marks

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
        is_discovered = was_discovered
        if reached_end:
            is_discovered = True
            for i in range(self.X_RES):
                for j in range(self.Y_RES):
                    cell = self.cell_grid[i][j]
                    if cell.type in (CellType.EMPTY, CellType.PASSAGE):
                        continue # Only consider junctions, start, end.
                    for k in range(4):
                        if cell.links[k] and cell.marks[k] == 0:
                            is_discovered = False
                            break
                    if not is_discovered:
                        break
                if not is_discovered:
                    break
            if is_discovered:
                print('Maze discovered!')
        
        return target_direction, is_discovered