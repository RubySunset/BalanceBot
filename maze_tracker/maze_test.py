import time
from maze_manager import *
from test_helper import *

RES = MazeManager.X_RES # Assume square matrix.
manager = MazeManager()

# Initialise robot.
pos = [0, 0] # Replace this with the starting position later.
orientation = 0
velocity = 1 / RES * 1
last_command = None

light_grid = light_grid_i('maze8_5', manager)
# light_grid = light_grid_ii('maze8_3ii', manager)
pos = [manager.tracker.start_pos[0] / RES, manager.tracker.start_pos[1] / RES]

iterations = 0
while True:
    # Check for timeout.
    iterations += 1
    if iterations > 10000:
        print('!! Timeout !!')
        break

    # Print current state of maze.
    norm_pos = (int(pos[0] * RES), int(pos[1] * RES))
    manager.print_maze(norm_pos, orientation)

    # Update maze.
    light = []
    for i in range(RES):
        light.append(light_grid[norm_pos[0]][norm_pos[1]][(i + orientation) % 4])
    command = manager.receive_data(pos, orientation, light)
    if command == -1:
        print('(last command) ', end='')
        command = last_command # Continue with last command in absense of new one.
    else:
        last_command = command

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
    elif command == 4:
        print('Stop')
        break

    # time.sleep(0.2)
    # print('\n' * 20)

print('Iterations: ' + str(iterations))
print('Maze graph:')
print(manager.tracker.maze_graph.a_list)
print('Shortest path:')
print(manager.tracker.maze_graph.shortest_path)
# print('Marks (for Tremaux\'s algorithm)')
# for i in range(RES):
#     for j in range(RES):
#         print(i, j, tracker.maze_grid.cell_grid[i][j].marks)