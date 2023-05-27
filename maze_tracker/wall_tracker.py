class WallTracker:

    # D_TOL = 0.1 # The maximum distance between the endpoints of two lines for them to be considered the same.
    # G_TOL = 0.1 # The maximum difference in gradients between two lines for them to be considered parallel.

    def __init__(self, RES):
        self.RES = RES # The resolution of the grid.
        self.walls = [] # A list of pairs of points corresponding to lines representing walls in the maze.
        # Note that the first point is always closer to the top than the second point, unless they 
        # have the same y coordinate in which case the first point is closer to the left.
        self.start = None # The start position.
        self.end = None # The end position.
    
    # Reset to initial state.
    def reset(self):
        self.walls = []
        self.start = None
        self.end = None
    
    # Set a starting point.
    def set_start(self, pos):
        self.start = pos
    
    # Set an end point.
    def set_end(self, pos):
        self.end = pos
    
    # Makes it so that the first point is closer to the top or else closer to the left than the second point.
    def rearrange_line(self, line):
        if line[0][1] != line[1][1]:
            if line[0][1] < line[1][1]:
                return line
            else:
                return (line[1], line[0])
        elif line[0][0] < line[1][0]:
            return line
        else:
            return (line[1], line[0])
    
    # Snap the positions of the endpoints to the grid.
    def discretise_line(self, line):
        d_line = [[None, None], [None, None]]
        for i in range(2):
            for j in range(2):
                d_line[i][j] = round(line[i][j] / self.RES) * self.RES
        return ((d_line[0][0], d_line[0][1]), (d_line[1][0], d_line[1][1]))

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

if __name__ == '__main__':
    tracker = WallTracker()
    while True:
        x1 = float(input('Point 1 x coord: '))
        y1 = float(input('Point 1 y coord: '))
        x2 = float(input('Point 2 x coord: '))
        y2 = float(input('Point 2 y coord: '))
        tracker.add_wall(((x1, y1), (x2, y2)))
        for wall in tracker.walls:
            print(wall)