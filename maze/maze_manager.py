from maze_tracker import *
from maze_bitmap import *

# Main class. Instantiate and use in other modules.
class MazeManager:

    # Size of arena.
    X_LIM = 3.6
    Y_LIM = 2.4

    RES = 0.1 # The resolution of the underlying grid that positions are snapped to.
    # The maximum distance between two points for them to be considered the same is RES/2.

    J_DIST = 0.5 # The distance we travel straight for after turning at a junction.

    def __init__(self):
        self.tracker = MazeTracker(self.RES, self.X_LIM, self.Y_LIM)
        self.bitmap = MazeBitmap(self, self.X_LIM, self.Y_LIM)
        self.reached_end = False # Have we reached the end?
        self.is_discovered = False # Have we discovered enough of the maze to find the shortest path?
        self.reverse_mode = False # Is the robot in reverse mode?
    
    # Reset to the initial state.
    def reset(self):
        self.tracker.reset()
        self.bitmap.reset()
        self.reached_end = False
        self.is_discovered = False
        self.reverse_mode = False
    
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
        return (d_point[0], d_point[1])
    
    # Set a starting point.
    def set_start(self, raw_pos):
        pos = self.discretise_point(raw_pos)
        self.tracker.set_start(pos)
        self.bitmap.set_start(pos)
        self.tracker.prev_vertex = self.tracker.start
    
    # Set an end point.
    def set_end(self, raw_pos):
        pos = self.discretise_point(raw_pos)
        self.tracker.set_end(pos)
        self.bitmap.set_end(pos)
    
    # Process sensor data from the robot to generate the next command.
    # Angle is given in degrees starting clockwise from north, [0, 360].
    # Light readings are a 4-tuple of booleans.
    def default_navigate(self, pos, angle, light):
        
        # Translate light readings to forwards/reverse-relative form.
        if self.reverse_mode:
            r_light = []
            for i in range(4):
                r_light.append(light[(i + 2) % 4])
        else:
            r_light = light

        if r_light[0]: # Maze boundary in front.
            print('Boundary ahead.')
            return 'j'
        elif abs(pos - self.tracker.prev_vertex) < self.J_DIST: # Force straight after junction.
            return None
        elif not r_light[1] or not r_light[3]: # Reached junction.
            print('Reached junction.')
            return 'j'
        else: # Default: in a corridor.
            return None

    # Generate a navigation command after a junction has been mapped.
    # Assume input angles are taken in degrees clockwise from north, [0, 360].
    # Assume output angles are taken in degrees clockwise from north, [-180, 180].
    def junction_navigate(self, pos, angle, link_angles):

        disc_pos = self.discretise_point(pos)

        # Check if we've reached the end.
        if abs(pos - self.tracker.end) < self.RES/2:
            self.reached_end = True
            print('End reached.')
        
        # Update relevant data structures.
        self.tracker.visit_vertex(disc_pos, link_angles) # Update graph structures.
        self.bitmap.update_pixels(self.tracker.a_list) # Update bitmap.
        if not self.is_discovered: # Update external path in discovery phase.
            self.tracker.generate_partial_path(pos) # Using the continuous position is intentional here.

        # Check if enough of the maze has been discovered.
        if self.reached_end and not self.is_discovered and self.tracker.enough_discovered():
            self.is_discovered = True
            self.tracker.generate_complete_path()
        
        # Find the target angle.
        if self.is_discovered:
            if self.reached_end:
                return 'e'
            else:
                target_angle = self.tracker.dijkstra_navigate(disc_pos)
        else:
            target_angle = self.tracker.tremaux_navigate(disc_pos, link_angles)

        self.tracker.prev_vertex = disc_pos # Update previous vertex.
        
        # Generate navigation command.
        diff = (target_angle - angle) % 360 # [0, 360]
        if diff > 90 and diff < 270: # If it would be quicker to reverse.
            self.reverse_mode = not self.reverse_mode
            rotation = diff - 180 # [-90, 90]
        else:
            rotation = (diff + 180) % 360 - 180 # [-90, 90]
        if self.reverse_mode:
            return 'b' + str(int(rotation))
        else:
            return 'f' + str(int(rotation))
    
    # Get the bitmap after calling junction_navigate().
    def get_bitmap(self):
        return self.bitmap.pixels