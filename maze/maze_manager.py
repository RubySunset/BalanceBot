from maze_tracker import *
from maze_bitmap import *

# Main class. Instantiate and use in other modules.
class MazeManager:

    # Size of arena.
    X_LIM = 3
    Y_LIM = 2

    RES = 0.25 # The resolution of the underlying grid that positions are snapped to.
    # The maximum distance between two points for them to be considered the same is RES/2.

    # J_DIST = 0.3 # The distance we travel straight for after turning at a junction.
    LINK_DIST = 0.18 # The distance we travel straight for after detecting a side link.
    FORCE_DIST = 0.05 # The distance we continue to force straight for after detecting a corridor.

    def __init__(self):
        self.tracker = MazeTracker()
        self.bitmap = MazeBitmap(self.X_LIM, self.Y_LIM)
        self.reached_end = False # Have we reached the end?
        self.is_discovered = False # Have we discovered enough of the maze to find the shortest path?
        self.reverse_mode = False # Is the robot in reverse mode?
        self.temp_pos = None
        self.is_link_delay = False # Are we delaying the detection of a junction via a side link?
        self.foo_pos = None
        self.is_force_delay = False # Are we delaying stopping forcing straight after a junction?
        self.is_forcing = False # Are we forcing straight after a junction?
    
    # Reset to the initial state.
    def reset(self):
        self.tracker.reset()
        self.bitmap.reset()
        self.reached_end = False
        self.is_discovered = False
        self.reverse_mode = False
        self.temp_pos = None
        self.is_link_delay = False
        self.is_forcing = False
    
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
        
        # Test if we can stop forcing straight.
        if self.is_forcing and not self.is_force_delay and r_light[1] and r_light[3]:
            self.is_force_delay = True
            self.foo_pos = pos # Use continuous position.
            print('Start force delay at', pos)
        elif self.is_forcing and self.is_force_delay and math.dist(self.foo_pos, pos) >= self.FORCE_DIST:
            self.is_forcing = False
            self.is_force_delay = False
            self.foo_pos = None
            print('Stop forcing straight.')

        # if math.dist(pos, self.tracker.end) <= self.RES/2: # If we have reached the end, maybe?
        #     print('Possibly reached end.')
        #     return 'j'
        if r_light[0]: # Maze boundary in front.
            if self.is_link_delay: # Reset link delay if one was active.
                self.is_link_delay = False
                self.temp_pos = None
                print('Reset link delay.')
            if self.is_force_delay: # Reset force delay if one was active.
                self.is_force_delay = False
                self.foo_pos = None
                print('Reset force delay.')
            print('Boundary ahead - junction.')
            return 'j'
        # elif math.dist(pos, self.tracker.prev_vertex) < self.J_DIST: # Force straight after junction.
        #     return None
        elif not self.is_forcing and (not r_light[1] or not r_light[3]): # Reached junction.
            if not self.is_link_delay:
                self.is_link_delay = True
                self.temp_pos = pos # Use continuous position.
                print('Start link delay at', pos)
                return None
            elif math.dist(self.temp_pos, pos) >= self.LINK_DIST:
                self.is_link_delay = False
                self.temp_pos = None
                print('Link to side - junction.')
                return 'j'
            else:
                return None
        else: # Default: in a corridor.
            if self.is_link_delay: # Abandon a link delay if there is one.
                self.is_link_delay = False
                self.temp_pos = None
                print('Abandon link delay.')
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
        
        # Rearrange link angles to have the first be closest to north.
        closest_index = 0
        closest_dist = math.inf
        for i in range(len(link_angles)):
            if self.tracker.mod_diff(link_angles[i], 0, 360) < closest_dist:
                closest_index = i
                closest_dist = self.tracker.mod_diff(link_angles[i], 0, 360)
        temp = []
        for i in range(len(link_angles)):
            temp.append(link_angles[(i + closest_index) % len(link_angles)])
        link_angles = temp
        print(link_angles)

        disc_pos = self.discretise_point(pos)

        # Check if we've reached the end.
        if disc_pos == self.tracker.end:
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
            if disc_pos == self.tracker.end:
                print('Finished.')
                return 'e'
            else:
                target_angle = self.tracker.dijkstra_navigate(disc_pos)
        else:
            target_angle = self.tracker.tremaux_navigate(disc_pos, link_angles, entry_link)
        print('Target angle:', target_angle)

        self.tracker.prev_vertex = disc_pos # Update previous vertex.
        self.is_forcing = True # Force straight after reaching a junction (and turning).
        
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