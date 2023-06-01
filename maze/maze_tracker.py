import math
import time
import heapq
from enum import Enum
from PIL import Image

class PixelType(Enum):
    EMPTY = 0
    START = 1
    END = 2
    PATH = 3
    CLEARED = 4

# Main class. Instantiate and use in other modules.
class MazeTracker:

    def __init__(self, X_LIM, Y_LIM, RES):

        # Define constants.
        self.X_LIM = X_LIM
        self.Y_LIM = Y_LIM
        self.GRID_RES = RES # The resolution of the grid.
        self.ALLOW = 0.1 # The distance we should maintain from walls in standard units.
        self.PIXEL_RES = self.GRID_RES / self.PIXELS # The resolution of the pixel array.
        self.X_PIXELS = int(self.X_LIM / self.PIXEL_RES) + 1 # The number of pixels in the x direction.
        self.Y_PIXELS = int(self.Y_LIM / self.PIXEL_RES) + 1 # The number of pixels in the y direction.
        self.P_ALLOW = self.ALLOW / self.PIXEL_RES # The distance we should maintain from walls in pixels.

        # Graph structures/variables.
        self.a_list = {} # An adjacency list for the graph representing possible paths the robot can take.
        self.marks = {} # A mapping from vertex positions to lists of link marks (used by Tremaux's algorithm).
        self.prev_vertex = None # The position of the last vertex reached.
        self.start = None # The start position.
        self.end = None # The end position.
        self.external_path = [] # The path sent to the web server to display.
        # When the robot is still traversing the maze, this is the shortest path from the start to the robot.
        # When the robot has reached the end and has surveyed the entire maze, this is the shortest path from start to end.

        # Other variables.
        self.pixels = [] # The pixel array representing the state of the maze.
        for i in range(self.X_PIXELS):
            pixel_row = []
            for j in range(self.Y_PIXELS):
                pixel_row.append(PixelType.EMPTY)
            self.pixels.append(pixel_row)
    
    # Reset to initial state.
    def reset(self):
        self.a_list = {}
        self.marks = {}
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
    
    # Visit a vertex, and generate the target angle for navigation.
    # Angles is a list of angles of the links to this node, calculating on the robot using light readings.
    # Angles should go clockwise from north in the range [-180, 180].
    def visit_vertex(self, raw_pos, angles):

        pos = self.discretise_point(raw_pos)

        # Update graph structures.
        if self.prev_vertex == None: # If we are at the start.
            self.a_list[pos] = {}
            self.marks[pos] = []
            for i in range(len(angles)):
                self.marks[pos].append(0)
        else:
            self.a_list[self.prev_vertex].add(pos)
            if pos in self.a_list:
                self.a_list[pos].add(self.prev_vertex)
            else:
                self.a_list[pos] = {self.prev_vertex}
                self.marks[pos] = []
                for i in range(len(angles)):
                    self.marks[pos].append(0)
        
        # Find entry angle.
        if self.prev_vertex == None: # If we are at the start.
            entry_angle = angles[0] # Assume we entered from a valid link, doesn't matter which one.
            closest = angles[0]
        else:
            diff = [pos[i] - self.prev_vertex[i] for i in range(2)]
            entry_angle = (270 - math.atan2(diff[1], diff[0])) % 360 - 180 # Convert from standard argument to our convention.
            # Find the link corresponding to the entry angle.
            closest = 0
            for i in range(len(angles)):
                if abs(angles[i] - entry_angle) < abs(angles[closest] - entry_angle):
                    closest = i
            # Apply entry mark.
            self.marks[pos][closest] += 1

        # Apply Tremaux's algorithm to find the target link.
        target_link = None
        others_unmarked = True # Are all the other entrances unmarked?
        candidate = None # A possible direciton if all other entrances are unmarked.
        for i in range(len(angles)):
            if i == closest:
                continue
            elif candidate == None and self.marks[pos][i] == 0:
                candidate = i
            elif self.marks[pos][i] != 0:
                others_unmarked = False
                break
        if others_unmarked and len(angles) > 1:
            # We can only choose another entrance if there is another entrance to choose from...
            target_link = candidate
        elif self.marks[pos] < 2:
            target_link = entry_angle # Go back the way we came.
        else:
            min_link = None
            min_val = math.inf
            for i in range(len(angles)):
                if self.marks[pos][i] < min_val:
                    min_link = i
                    min_val = self.marks[pos][i]
            target_link = min_link
        
        self.marks[pos][target_link] += 1 # Apply exit mark.
        self.prev_vertex = pos # Update previous vertex.
        
        return angles[target_link]