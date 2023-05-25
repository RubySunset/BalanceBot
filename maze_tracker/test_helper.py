import string
from maze_manager import *

RES = MazeManager.X_RES # Assume square matrix.

# Option 1: generate light grid from a grid where each cell if either filled (solid wall) or unfilled.
def light_grid_i(name, manager):
    # Initialise maze light grid.
    light_grid = []
    for i in range(RES):
        row = []
        for j in range(RES):
            row.append([])
        light_grid.append(row)

    # Initialise maze wall grid (open spaces and walls).
    wall_grid = []
    for i in range(RES):
        row = []
        for j in range(RES):
            row.append('.') # Fill with open spaces.
        wall_grid.append(row)

    # Load wall grid from file.
    file = open('test_mazes/' + name + '.txt', 'r')
    raw_lines = file.readlines()
    for j in range(len(raw_lines)):
        line = raw_lines[j].replace(' ', '').strip(string.whitespace)
        for i in range(len(line)):
            if line[i] == 'x':
                wall_grid[i][j] = 'x'
            else:
                wall_grid[i][j] = '.'
                if line[i] == 's':
                    manager.add_start((i, j))
                    pos = [i / RES, j / RES] # Always start robot at the designated starting point.
                elif line[i] == 'e':
                    manager.add_end((i, j))
    file.close()

    # Convert maze wall grid to maze light grid.
    for i in range(RES):
        for j in range(RES):
            light = [0, 0, 0, 0] # Start with all boundaries open.
            if j == 0 or wall_grid[i][j - 1] == 'x':
                light[0] = 1
            if i == RES - 1 or wall_grid[i + 1][j] == 'x':
                light[1] = 1
            if j == RES - 1 or wall_grid[i][j + 1] == 'x':
                light[2] = 1
            if i == 0 or wall_grid[i - 1][j] == 'x':
                light[3] = 1
            light_grid[i][j] = light

    return light_grid

# Option 2: generate light grid from a maze built with the tool.
def light_grid_ii(name, manager):
    # Initialise maze light grid.
    light_grid = []
    for i in range(RES):
        row = []
        for j in range(RES):
            row.append([])
        light_grid.append(row)
    
    # Initialise maze symbol grid.
    symbol_grid = []
    for j in range(RES):
        row = []
        for i in range(RES):
            row.append('·')
        symbol_grid.append(row)
    
    # Load symbol grid from file.
    file = open('test_mazes/' + name + '.txt', 'r', encoding='utf8')
    raw_lines = file.readlines()

    # Extract start and end positions.
    header = raw_lines[0].replace(' ', '').strip(string.whitespace)
    state = 0
    foo = ''
    start_pos = [0, 0]
    end_pos = [0, 0]
    for char in header[1:]: # Skip initial character 's'.
        if state == 0 and char == ',':
            start_pos[0] = int(foo)
            foo = ''
            state = 1
            continue
        elif state == 1 and char == 'e':
            start_pos[1] = int(foo)
            foo = ''
            state = 2
            continue
        elif state == 2 and char == ',':
            end_pos[0] = int(foo)
            foo = ''
            state = 3
            continue
        foo += char
    end_pos[1] = int(foo)
    manager.add_start((start_pos[0], start_pos[1]))
    manager.add_end((end_pos[0], end_pos[1]))

    # Extract maze.
    for j in range(0, RES):
        line = raw_lines[j + 1].strip(string.whitespace)
        for i in range(int((len(line) + 1) / 2)):
            symbol_grid[i][j] = line[i * 2]
    file.close()

    # Convert symbol grid to light grid.
    light_grid = []
    for i in range(RES):
        row = []
        for j in range(RES):
            symbol = symbol_grid[i][j]
            light = [1, 1, 1, 1]
            if symbol in ('│', '╧', '┘', '┤', '└', '┴', '├', '┼'):
                light[0] = 0
            if symbol in ('─', '╟', '┌', '┬', '└', '┴', '├', '┼'):
                light[1] = 0
            if symbol in ('│', '╤', '┐', '┌', '┬', '┤', '├', '┼'):
                light[2] = 0
            if symbol in ('─', '╢', '┐', '┬', '┘', '┤', '┴', '┼'):
                light[3] = 0
            row.append(light)
        light_grid.append(row)
    return light_grid