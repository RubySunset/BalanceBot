import math
import time
import heapq
from enum import Enum
from PIL import Image

class PixelType(Enum):
    EMPTY = 0
    START = 1
    END = 2
    WALL = 3
    COLLISION = 4

# Main class. Instantiate and use in other modules.
class MazeTracker:

    # D_TOL = 0.1 # The maximum distance between the endpoints of two lines for them to be considered the same.
    # G_TOL = 0.1 # The maximum difference in gradients between two lines for them to be considered parallel.

    PIXELS = 10 # How many 'pixels' we split each cell in the grid into when pathfinding.

    def __init__(self, X_LIM, Y_LIM, RES):
        self.X_LIM = X_LIM
        self.Y_LIM = Y_LIM
        self.GRID_RES = RES # The resolution of the grid.
        self.ALLOW = 0.1 # The distance we should maintain from walls in standard units.
        self.PIXEL_RES = self.GRID_RES / self.PIXELS # The resolution of the pixel array.
        self.X_PIXELS = int(self.X_LIM / self.PIXEL_RES) + 1 # The number of pixels in the x direction.
        self.Y_PIXELS = int(self.Y_LIM / self.PIXEL_RES) + 1 # The number of pixels in the y direction.
        self.P_ALLOW = self.ALLOW / self.PIXEL_RES # The distance we should maintain from walls in pixels.
        self.walls = [] # A list of pairs of points corresponding to lines representing walls in the maze.
        # Note that the first point is always closer to the left than the second point, unless they 
        # have the same x coordinate in which case the first point is closer to the top.
        self.start = None # The start position.
        self.end = None # The end position.
        self.external_path = [] # The path sent to the web server to display.
        # When the robot is still traversing the maze, this is the shortest path from the start to the robot.
        # Since we are using the wall follower algorithm for movement, this will simply be the sequence of
        # vertices traversed by the robot so far (with loops removed where necessary).
        # When the robot has reached the end and has surveyed the entire maze, this is the shortest path from
        # start to end.
        self.pixels = [] # The pixel array representing the state of the maze.
        for i in range(self.X_PIXELS):
            pixel_row = []
            for j in range(self.Y_PIXELS):
                pixel_row.append(PixelType.EMPTY)
            self.pixels.append(pixel_row)
    
    # Reset to initial state.
    def reset(self):
        self.walls = []
        self.start = None
        self.end = None
        self.external_path = []
        self.pixels = []
        for i in range(self.X_PIXELS):
            pixel_row = []
            for j in range(self.Y_PIXELS):
                pixel_row.append(PixelType.EMPTY)
            self.pixels.append(pixel_row)
    
    # Discretise a point to fit it to the grid. Also force it into the arena if it is out of bounds.
    def discretise_point(self, point):
        d_point = [round(point[0] / self.GRID_RES) * self.GRID_RES, round(point[1] / self.GRID_RES) * self.GRID_RES]
        if d_point[0] < 0:
            d_point[0] = 0
            print('Warning: point out of bounds (left).')
        elif d_point[0] > self.X_LIM:
            d_point[0] = self.X_LIM
            print('Warning: point out of bounds (right).')
        if d_point[1] < 0:
            d_point[1] = 0
            print('Warning: point out of bounds (top).')
        elif d_point[1] > self.Y_LIM:
            d_point[1] = self.Y_LIM
            print('Warning: point out of bounds (bottom).')
        return d_point
    
    # Convert a point in metres (standard units) to pixels.
    def to_pixels(self, point):
        return (round(point[0] / self.PIXEL_RES), round(point[1] / self.PIXEL_RES))
    
    # Set a starting point.
    def set_start(self, pos):
        self.start = self.discretise_point(pos)
        self.p_start = self.to_pixels(self.start)
        self.pixels[self.p_start[0]][self.p_start[1]] = PixelType.START
    
    # Set an end point.
    def set_end(self, pos):
        self.end = self.discretise_point(pos)
        self.p_end = self.to_pixels(self.end)
        self.pixels[self.p_end[0]][self.p_end[1]] = PixelType.END
    
    # Update the path traversed by the robot so far while we are still not at the end.
    def update_path(self, pos):
        for i in range(len(self.external_path)):
            if self.external_path[i] == pos:
                self.external_path = self.external_path[0:i] # Remove loops (caused by dead ends).
                break
        self.external_path.append(pos)
    
    # Rearrange the line so that the first point is closer to the top or else closer to the left than the second point.
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
    # Note that if the line is vertical, the gradient is math.inf and the y-intercept is replaced with the x-intercept.
    def find_params(self, line):
        try:
            g = (line[1][1] - line[0][1]) / (line[1][0] - line[0][0])
            c = line[0][1] - g * line[0][0] # Could also do this with second point.
        except ZeroDivisionError:
            g = math.inf
            c = line[0][0]
        return g, c
    
    # Tests if the two given lines are overlapping and can be combined into a single line (returned).
    # Return None if not.
    def combine_overlapping(self, line1, line2):
        g1, c1 = self.find_params(line1)
        g2, c2 = self.find_params(line2)
        if g1 == g2 and c1 == c2: # If the lines are collinear.
            if line1[0][0] <= line2[0][0] and line1[1][0] >= line2[1][0]: # If line1 contains line2.
                return line1
            elif line2[0][0] <= line1[0][0] and line2[1][0] >= line1[1][0]: # If line2 contains line1.
                return line2
            if g1 == 0: # If the lines are horizontal.
                if line1[0][0] < line2[0][0] and line1[1][0] >= line2[0][0]: # If line1 is to the left and overlapping line2.
                    return (line1[0], line2[1])
                elif line2[0][0] < line1[0][0] and line2[1][0] >= line1[0][0]: # If line2 is to the left and overlapping line1.
                    return (line2[0], line1[1])
                else:
                    return None
            elif g1 == math.inf: # If the lines are vertical.
                if line1[0][1] < line2[0][1] and line1[1][1] >= line2[0][1]: # If line1 is above and overlapping line2.
                    return (line1[0], line2[1])
                elif line2[0][1] < line1[0][1] and line2[1][1] >= line1[0][1]: # If line2 is above and overlapping line1.
                    return (line2[0], line1[1])
                else:
                    return None
            else: # If the line are diagonal.
                if line1[0][0] < line2[0][0]:
                    l1 = line1
                    l2 = line2
                else:
                    l1 = line2
                    l2 = line1
                x_span_total = l2[1][0] - l1[0][0] # The total x distance covered by the two lines.
                x_span_1 = line1[1][0] - line1[0][0]
                x_span_2 = line2[1][0] - line2[0][0]
                if x_span_1 + x_span_2 > x_span_total: # If the lines are overlapping.
                    return (l1[0], l2[1])
                else:
                    return None
    
    # Check if we can combine any walls.
    def combine_walls(self):
        have_combined = True # Initial iteration
        while have_combined:
            have_combined = False
            for i in range(len(self.walls)):
                for j in range(i, len(self.walls)):
                    if i == j:
                        continue
                    combined = self.combine_overlapping(self.walls[i], self.walls[j])
                    if combined != None:
                        self.walls.pop(i)
                        self.walls.pop(j - 1)
                        self.walls.append(combined)
                        have_combined = True
                        break
                if have_combined:
                    break
    
    # Add a new wall (a pair of points).
    def add_wall(self, input_line):
        line = self.normalise_line(input_line)
        for wall in self.walls:
            if line == wall:
                return # Do not add duplicate walls.
        self.walls.append(line)
    
    # Update the pixel array.
    def update_pixels(self):
        for i in range(self.X_PIXELS):
            for j in range(self.Y_PIXELS):
                if self.pixels[i][j] not in (PixelType.START, PixelType.END):
                    self.pixels[i][j] = PixelType.EMPTY # Clear the pixel array (but skip over start and end).
        for wall in self.walls:
            p1 = self.to_pixels(wall[0])
            p2 = self.to_pixels(wall[1])
            diff = [p2[i] - p1[i] for i in range(2)] # Find the difference vector.
            p_dist = math.ceil(math.dist(p1, p2)) # Choose the upper bound on the length of the wall.
            unit_diff = [diff[i] / p_dist for i in range(2)] # Normalise the difference vector to get the direction.
            current = [p1[0], p1[1]]
            for i in range(p_dist + 1):
                c = [round(x) for x in current]
                for j in range(max(0, math.floor(c[0] - self.P_ALLOW)), min(math.ceil(c[0] + self.P_ALLOW) + 1, self.X_PIXELS)):
                    for k in range(max(0, math.floor(c[1] - self.P_ALLOW)), min(math.ceil(c[1] + self.P_ALLOW) + 1, self.Y_PIXELS)):
                        pixel = self.pixels[j][k]
                        if pixel in (PixelType.START, PixelType.END, PixelType.WALL): # Skip over start, end, previously placed walls.
                            pass
                        elif c == [j, k]:
                            self.pixels[j][k] = PixelType.WALL # Mark this cell as a wall.
                        elif math.dist(c, (j, k)) <= self.P_ALLOW:
                            self.pixels[j][k] = PixelType.COLLISION # Mark this cell as inaccessible.
                current = [current[i] + unit_diff[i] for i in range(2)]
    
    # Add a list of walls.
    def add_walls(self, input_lines):
        for input_line in input_lines:
            self.add_wall(input_line)
        self.combine_walls()
        self.update_pixels()
    
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

        # Generate initial arrays.
        f = []
        g = []
        prev = []
        for i in range(self.X_PIXELS):
            f_row = []
            g_row = []
            prev_row = []
            for j in range(self.Y_PIXELS):
                f_row.append(math.inf)
                g_row.append(math.inf)
                prev_row.append(None)
            f.append(f_row)
            g.append(g_row)
            prev.append(prev_row)
        
        last_time = time.time()

        # Use A* search to find the shortest path.
        # TODO replace A* search with an any-angle path planning algorithm.
        g[self.p_start[0]][self.p_start[1]] = 0
        f[self.p_start[0]][self.p_start[1]] = self.h(self.p_start, self.p_end)
        open_pq = []
        heapq.heappush(open_pq, (f[self.p_start[0]][self.p_start[1]], self.p_start))
        open_set = {self.p_start}
        path_found = False
        while len(open_set) > 0:
            current_f, current_pos = heapq.heappop(open_pq)
            open_set.remove(current_pos)
            if current_pos == self.p_end:
                path_found = True
                break
            neighbours = self.find_neighbours(current_pos, self.X_PIXELS, self.Y_PIXELS)
            for n in neighbours:
                if self.pixels[n[0]][n[1]] not in (PixelType.WALL, PixelType.COLLISION): # If this is a passable area.
                    t = g[current_pos[0]][current_pos[1]] + math.dist(current_pos, n)
                    if t < g[n[0]][n[1]]:
                        prev[n[0]][n[1]] = current_pos
                        g[n[0]][n[1]] = t
                        f[n[0]][n[1]] = t + self.h(n, self.p_end)
                        if n not in open_set:
                            heapq.heappush(open_pq, (f[n[0]][n[1]], n))
                            open_set.add(n)
        if not path_found:
            raise Exception('Path not found.')
        self.external_path = []
        current = self.p_end
        while current != None:
            self.external_path.append(current)
            current = prev[current[0]][current[1]]
        print('A* search time:', round(time.time() - last_time, 3))
    
    # Render the pixel array.
    def render_pixels(self):
        last_time = time.time()
        image = Image.new('RGB', (self.X_PIXELS, self.Y_PIXELS), 'white')
        img_pixels = image.load()
        for i in range(image.size[0]):
            for j in range(image.size[1]):
                pixel = self.pixels[i][j]
                if pixel == PixelType.START:
                    colour = (0, 255, 0)
                elif pixel == PixelType.END:
                    colour = (255, 0, 0)
                elif (i, j) in self.external_path:
                    colour = (0, 0, 255)
                elif pixel == PixelType.EMPTY:
                    colour = (0, 0, 0)
                elif pixel == PixelType.WALL:
                    colour = (255, 255, 255)
                elif pixel == PixelType.COLLISION:
                    colour = (128, 128, 128)
                else:
                    raise ValueError('Incorrect pixel value: ' + str(pixel) + ' at ' + str((i, j)) + '.')
                img_pixels[i, j] = colour
        image.show()
        print('Image rendering time:', round(time.time() - last_time, 3))

def testing_1(tracker):
    tracker.set_start((0.2, 0.2))
    tracker.set_end((2.8, 1.8))

    walls = []

    # Add boundary walls.
    walls.append(((0, 0), (tracker.X_LIM, 0)))
    walls.append(((tracker.X_LIM, 0), (tracker.X_LIM, tracker.Y_LIM)))
    walls.append(((0, tracker.Y_LIM), (tracker.X_LIM, tracker.Y_LIM)))
    walls.append(((0, 0), (0, tracker.Y_LIM)))

    # Test walls to combine.
    walls.append(((0.5, 0), (0.5, 1)))
    walls.append(((0.5, 0.5), (0.5, 1.5)))

    walls.append(((1, 2), (1.5, 1.5)))
    walls.append(((2, 1), (2.5, 0.5)))
    walls.append(((1, 0.5), (2, 0.5)))
    walls.append(((1, 1), (1.5, 1)))
    walls.append(((2, 0.5), (2, 1)))
    walls.append(((1.5, 1), (1.5, 1.5)))
    walls.append(((3, 1), (2.5, 1.5)))
    walls.append(((2, 1.5), (2.5, 1.5)))

    tracker.add_walls(walls)

def testing_2(tracker):
    tracker.set_start((0.2, 0.2))
    tracker.set_end((2.8, 1.8))

    walls = []

    # Add boundary walls.
    walls.append(((0, 0), (tracker.X_LIM, 0)))
    walls.append(((tracker.X_LIM, 0), (tracker.X_LIM, tracker.Y_LIM)))
    walls.append(((0, tracker.Y_LIM), (tracker.X_LIM, tracker.Y_LIM)))
    walls.append(((0, 0), (0, tracker.Y_LIM)))

    walls.append(((0.5, 0), (0.5, 1.5)))
    walls.append(((1, 2), (1, 0.5)))
    walls.append(((1.5, 0.5), (1.5, 1.5)))
    walls.append(((1.5, 0.5), (2.5, 0.5)))
    walls.append(((2, 1), (2, 2)))

    tracker.add_walls(walls)
    
if __name__ == '__main__':
    tracker = MazeTracker(3, 2, 0.1)
    testing_1(tracker)
    print('Number of walls:', len(tracker.walls))
    tracker.find_shortest_path()
    tracker.render_pixels()