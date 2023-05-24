import string
import time
from maze_tracker import *

RES = MazeTracker.X_RES # Assume square matrix.
tracker = MazeTracker()

# Initialise robot.
pos = [0, 0] # Replace this with the starting position later.
orientation = 0
velocity = 1 / RES

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
file = open('test_mazes/maze8_4.txt', 'r')
raw_lines = file.readlines()
for j in range(len(raw_lines)):
    line = raw_lines[j].replace(' ', '').strip(string.whitespace)
    for i in range(len(line)):
        if line[i] == 'x':
            wall_grid[i][j] = 'x'
        else:
            wall_grid[i][j] = '.'
            if line[i] == 's':
                tracker.add_start((i, j))
                pos = [i / RES, j / RES] # Always start robot at the designated starting point.
            elif line[i] == 'e':
                tracker.add_end((i, j))
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

# print('Options:')
# print('a: Default')
# print('b: Animation')
# option = ''
# while option not in ('a', 'b'):
#     option = input()

iterations = 0
while True:

    # Check for timeout.
    iterations += 1
    if iterations > 1000:
        print('!! Timeout !!')
        break

    # Print current state of maze.
    norm_pos = (int(pos[0] * RES), int(pos[1] * RES))
    tracker.print_maze(norm_pos, orientation)

    # Update maze.
    light = []
    for i in range(RES):
        light.append(light_grid[norm_pos[0]][norm_pos[1]][(i + orientation) % 4])
    command = tracker.update(pos, orientation, light)

    # Simulate navigation command.
    if command in (0, 3):
        if command == 0:
            v = velocity
            print('Forwards')
        else:
            v = -velocity
            print('Backwards')
        if orientation == 0:
            pos[1] -= v
        elif orientation == 1:
            pos[0] += v
        elif orientation == 2:
            pos[1] += v
        elif orientation == 3:
            pos[0] -= v
    elif command == 1:
        orientation = (orientation - 1) % 4
        print('Turn left')
    elif command == 2:
        orientation = (orientation + 1) % 4
        print('Turn right')
    # elif command == 3:
    #     orientation = (orientation + 2) % 4
    #     # Conceptually, I will treat going into reverse the same as a full 180 rotation.
    #     print('Reverse')
    elif command == 4:
        print('Stop')
        break

    time.sleep(0.2)
    print('\n' * 20)

print('Maze graph:')
print(tracker.maze_graph)
print('Shortest path:')
print(tracker.shortest_path)