import math
import time
from PIL import Image

class WallTracker:

    # D_TOL = 0.1 # The maximum distance between the endpoints of two lines for them to be considered the same.
    # G_TOL = 0.1 # The maximum difference in gradients between two lines for them to be considered parallel.

    PIXELS = 10 # How many 'pixels' we split each cell in the grid into when pathfinding.

    def __init__(self, X_LIM, Y_LIM, RES):
        self.X_LIM = X_LIM
        self.Y_LIM = Y_LIM
        self.GRID_RES = RES # The resolution of the grid.
        self.walls = [] # A list of pairs of points corresponding to lines representing walls in the maze.
        # Note that the first point is always closer to the left than the second point, unless they 
        # have the same x coordinate in which case the first point is closer to the top.
        self.start = None # The start position.
        self.end = None # The end position.
        self.external_path = [] # The path sent to the web server to display.
        # When the robot is still traversing the maze, this is the shortest path from the start to the robot.
        # Since we are using the wall follower algorithm for movement, this will simply be the sequence of
        # vertices traversed by the robot so far.
        # When the robot has reached the end and has surveyed the entire maze, this is the shortest path from
        # start to end.
    
    # Reset to initial state.
    def reset(self):
        self.walls = []
        self.start = None
        self.end = None
    
    # Discretise a point to fit it to the grid.
    def discretise_point(self, point):
        return (round(point[0] / self.GRID_RES) * self.GRID_RES, round(point[1] / self.GRID_RES) * self.GRID_RES)
    
    # Set a starting point.
    def set_start(self, pos):
        self.start = self.discretise_point(pos)
    
    # Set an end point.
    def set_end(self, pos):
        self.end = self.discretise_point(pos)
    
    # Update the path traversed by the robot so far.
    def update_path(self, pos):
        for i in range(len(self.external_path)):
            if self.external_path[i] == pos:
                self.external_path = self.external_path[0:i] # Remove loops (caused by dead ends).
                break
        self.external_path.append(pos)
    
    # Makes it so that the first point is closer to the top or else closer to the left than the second point.
    def rearrange_line(self, line):
        if line[0][0] != line[1][0]:
            if line[0][0] < line[1][0]:
                return line
            else:
                return (line[1], line[0])
        elif line[0][1] < line[1][1]:
            return line
        else:
            return (line[1], line[0])
    
    # Snap the positions of the endpoints to the grid.
    def discretise_line(self, line):
        return (self.discretise_point(line[0]), self.discretise_point(line[1]))

    # Rearrange + discretise.
    def normalise_line(self, line):
        return self.discretise_line(self.rearrange_line(line))
    
    # Find the gradient and y-intercept of the line.
    def find_params(self, line):
        g = (line[1][1] - line[0][1]) / (line[1][0] - line[0][0])
        c = line[0][1] - g * line[0][0] # Could also do this with second point.
        return g, c
    
    # Tests if the two given lines are overlapping and can be combined into a single line (returned).
    # Return None if not.
    def combine_overlapping(self, line1, line2):
        g1, c1 = self.find_params(line1)
        g2, c2 = self.find_params(line2)
        if abs(g1 - g2) < 0.01 and abs(c1 - c2) < 0.01: # If these are part of the same line (giving some allowance).
            pass # TODO
    
    # Add a new wall (a pair of points).
    def add_wall(self, input_line):
        line = self.normalise_line(input_line)
        for wall in self.walls:
            if line == wall:
                return # Do not add duplicate walls.
        self.walls.append(line)
    
    # Find the neighbours of the pixel.
    def find_neighbours(self, pixel, x_pixels, y_pixels):
        neighbours = []
        if pixel[1] != 0:
            neighbours.append((pixel[0], pixel[1] - 1))
            if pixel[0] != x_pixels - 1:
                neighbours.append((pixel[0] + 1, pixel[1] - 1))
        if pixel[0] != x_pixels - 1:
            neighbours.append((pixel[0] + 1, pixel[1]))
            if pixel[1] != y_pixels - 1:
                neighbours.append((pixel[0] + 1, pixel[1] + 1))
        if pixel[1] != y_pixels - 1:
            neighbours.append((pixel[0], pixel[1] + 1))
            if pixel[0] != 0:
                neighbours.append((pixel[0] - 1, pixel[1] + 1))
        if pixel[0] != 0:
            neighbours.append((pixel[0] - 1, pixel[1]))
            if pixel[1] != 0:
                neighbours.append((pixel[0] - 1, pixel[1] - 1))
        return neighbours

    # The heuristic for A* search.
    def h(self, pos, end):
        dx = abs(pos[0] - end[0])
        dy = abs(pos[1] - end[1])
        return dx + dy + (math.sqrt(2) - 2)*min(dx, dy)

    # Find the shortest path from start to end.
    def find_shortest_path(self):

        # Initial parameters.
        allowance = 0.05 # The distance we should maintain from walls in standard units.
        pixel_res = self.GRID_RES / self.PIXELS # The resolution of the pixel array.
        x_pixels = int(self.X_LIM / pixel_res) + 1 # The number of pixels in the x direction.
        y_pixels = int(self.Y_LIM / pixel_res) + 1 # The number of pixels in the y direction.
        p_allow = allowance / pixel_res # The distance we should maintain from walls in pixels.
        last_time = time.time()

        # Generate initial arrays.
        pixels = []
        f = []
        g = []
        prev = []
        Q = []
        for i in range(x_pixels):
            pixels_row = []
            f_row = []
            g_row = []
            prev_row = []
            for j in range(y_pixels):
                pixels_row.append(0)
                f_row.append(math.inf)
                g_row.append(math.inf)
                prev_row.append(None)
                Q.append((i, j))
            pixels.append(pixels_row)
            f.append(f_row)
            g.append(g_row)
            prev.append(prev_row)
        print(time.time() - last_time)
        last_time = time.time()

        # Add walls to pixel array.
        for wall in self.walls:
            p1 = [round(x / pixel_res) for x in wall[0]]
            p2 = [round(x / pixel_res) for x in wall[1]]
            diff = [p2[i] - p1[i] for i in range(2)] # Find the difference vector.
            p_dist = math.ceil(math.dist(p1, p2)) # Choose the upper bound on the length of the wall.
            unit_diff = [diff[i] / p_dist for i in range(2)] # Normalise the difference vector to get the direction.
            current = [p1[0], p1[1]]
            for i in range(p_dist + 1):
                # for j in range(x_pixels):
                #     for k in range(y_pixels):
                #         if math.dist(current, (j, k)) < p_allow:
                #             pixels[j][k] = 1 # Make this cell inaccessible.
                for j in range(max(0, math.floor(current[0] - p_allow)), min(math.ceil(current[0] + p_allow) + 1, x_pixels)):
                    for k in range(max(0, math.floor(current[1] - p_allow)), min(math.ceil(current[1] + p_allow) + 1, y_pixels)):
                        if math.dist(current, (j, k)) <= p_allow:
                            pixels[j][k] = 1 # Make this cell inaccessible.
                current = [current[i] + unit_diff[i] for i in range(2)]
        print(time.time() - last_time)
        last_time = time.time()

        # Use A* search to find the shortest path.
        start = (round(self.start[0] / pixel_res), round(self.start[1] / pixel_res))
        end = (round(self.end[0] / pixel_res), round(self.end[1] / pixel_res))
        open_set = {start}
        g[start[0]][start[1]] = 0
        f[start[0]][start[1]] = self.h(start, end)
        while len(open_set) > 0:
            current = None
            min_val = math.inf
            for pos in open_set:
                if f[pos[0]][pos[1]] < min_val:
                    current = pos
                    min_val = f[pos[0]][pos[1]]
            if current == end:
                break
            open_set.remove(current)
            neighbours = self.find_neighbours(current, x_pixels, y_pixels)
            for n in neighbours:
                if not pixels[n[0]][n[1]]:
                    t = g[current[0]][current[1]] + math.dist(current, n)
                    if t < g[n[0]][n[1]]:
                        prev[n[0]][n[1]] = current
                        g[n[0]][n[1]] = t
                        f[n[0]][n[1]] = t + self.h(n, end)
                        open_set.add(n)
        shortest_path = []
        current = end
        while current != None:
            shortest_path.append(current)
            current = prev[current[0]][current[1]]
        print(time.time() - last_time)
        last_time = time.time()

        # Generate image.
        image = Image.new('RGB', (x_pixels, y_pixels), 'white')
        img_pixels = image.load()
        for i in range(image.size[0]):
            for j in range(image.size[1]):
                if (i, j) in shortest_path:
                    img_pixels[i,j] = (0, 0, 255)
                else:
                    p = pixels[i][j] * 255
                    img_pixels[i,j] = (p, p, p)
        image.show()
        print(time.time() - last_time)
        last_time = time.time()
    
if __name__ == '__main__':
    tracker = WallTracker(3, 2, 0.1)
    tracker.set_start((0, 0))
    tracker.set_end((3, 2))
    tracker.add_wall(((0.5, 0), (0.5, 1.5)))
    tracker.add_wall(((1, 2), (1, 0.5)))
    tracker.add_wall(((1.5, 0.5), (1.5, 1.5)))
    tracker.add_wall(((1.5, 0.5), (2.5, 0.5)))
    tracker.add_wall(((2, 1), (2, 2)))
    tracker.find_shortest_path()
    # while True:
    #     x1 = float(input('Point 1 x coord: '))
    #     y1 = float(input('Point 1 y coord: '))
    #     x2 = float(input('Point 2 x coord: '))
    #     y2 = float(input('Point 2 y coord: '))
    #     tracker.add_wall(((x1, y1), (x2, y2)))
    #     for wall in tracker.walls:
    #         print(wall)