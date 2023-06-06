from maze_tracker import *
from maze_bitmap import *

# Main class. Instantiate and use in other modules.
class MazeManager:

    # Size of arena.
    X_LIM = 4
    Y_LIM = 4

    RES = 0.1 # The resolution of the underlying grid that positions are snapped to.
    # The maximum distance between two points for them to be considered the same is RES/2.

    J_DIST = 0.5 # The distance we travel straight for after turning at a junction.
    # LINK_DIST = 0.1 # The distance we travel straight for after detecting a side link.

    def __init__(self):
        self.tracker = MazeTracker()
        self.bitmap = MazeBitmap(self.X_LIM, self.Y_LIM)
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
        d_point = [round(point[0] / self.RES) * self.RES, round(point[1] / self.RES) * self.RES]
        if d_point[0] < 0:
            d_point[0] = 0
            print('Warning: point out of bounds (left): ' + str(d_point))
        elif d_point[0] > self.X_LIM:
            d_point[0] = self.X_LIM
            print('Warning: point out of bounds (right): ' + str(d_point))
        if d_point[1] < 0:
            d_point[1] = 0
            print('Warning: point out of bounds (top): ' + str(d_point))
        elif d_point[1] > self.Y_LIM:
            d_point[1] = self.Y_LIM
            print('Warning: point out of bounds (bottom): ' + str(d_point))
        return (d_point[0], d_point[1])
    
    # Set a starting point.
    def set_start(self, raw_pos):
        pos = self.discretise_point(raw_pos)
        self.tracker.set_start(pos)
        self.bitmap.set_start(pos)
    
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
            print('Boundary ahead - junction.')
            return 'j'
        elif math.dist(pos, self.tracker.prev_vertex) < self.J_DIST: # Force straight after junction.
            return None
        elif not r_light[1] or not r_light[3]: # Reached junction.
            print('Link to side - junction.')
            return 'j'
        else: # Default: in a corridor.
            return None

    # Generate a navigation command after a junction has been mapped.
    # Assume input angles are taken in degrees clockwise from north, [0, 360].
    # Assume output angles are taken in degrees clockwise from north, [-180, 180].
    def junction_navigate(self, pos, angle, link_angles):

        # Translate angle to forwards/reverse-relative form.
        if self.reverse_mode:
            r_angle = (angle + 180) % 360
        else:
            r_angle = angle

        disc_pos = self.discretise_point(pos)

        # Check if we've reached the end.
        if math.dist(pos, self.tracker.end) <= self.RES/2:
            self.reached_end = True
            print('End reached.')
            if self.is_discovered:
                print('Finished.')
                return 'e'
        
        # Update relevant data structures.
        self.tracker.visit_vertex(disc_pos, link_angles) # Update graph structures.
        self.bitmap.update_pixels(self.tracker.a_list) # Update bitmap.
        if not self.is_discovered: # Update external path in discovery phase.
            self.tracker.generate_partial_path(pos) # Using the continuous position is intentional here.
        
        # Apply entry mark and check if enough of the maze has been discovered.
        entry_link = self.tracker.entry_mark(disc_pos, link_angles)
        if self.reached_end and not self.is_discovered and self.tracker.enough_discovered():
            self.is_discovered = True
            self.tracker.generate_complete_path()
        
        # Find the target angle.
        if self.is_discovered:
            target_angle = self.tracker.dijkstra_navigate(disc_pos)
        else:
            target_angle = self.tracker.tremaux_navigate(disc_pos, link_angles, entry_link)
        print('Target angle:', target_angle)

        self.tracker.prev_vertex = disc_pos # Update previous vertex.
        
        # Generate navigation command.
        diff = (target_angle - r_angle) % 360 # [0, 360]
        if diff > 90 and diff < 270: # If it would be quicker to reverse.
            self.reverse_mode = not self.reverse_mode
            rotation = diff - 180 # [-90, 90]
        else:
            rotation = (diff + 180) % 360 - 180 # [-90, 90]
        if self.reverse_mode:
            return str(int(rotation)) + 'b'
        else:
            return str(int(rotation)) + 'f'
    
    # Get the bitmap after calling junction_navigate().
    def get_bitmap(self):
        return self.bitmap.pixels

if __name__ == '__main__':
    manager = MazeManager()
    manager.set_start((1, 1))
    manager.set_end((3, 3))
    while True:
        while True:
            try:
                x = float(input('Robot x: '))
                y = float(input('Robot y: '))
                angle = int(input('Robot angle: '))
                link_angles = []
                while True:
                    i = input('Link angle: ')
                    if i == '':
                        break
                    link_angles.append(int(i))
                break
            except:
                pass
        command = manager.junction_navigate((x, y), angle, link_angles)
        print('Command:', command)
        print()
        if command == 'e':
            manager.bitmap.render_pixels()