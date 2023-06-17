from maze_tracker import *
from maze_bitmap import *
from beacon_tri import *

# Main class. Instantiate and use in other modules.
class MazeManager:

    # Size of arena.
    X_LIM = 3
    Y_LIM = 2

    # RES = 0.25 # The resolution of the underlying grid that positions are snapped to.

    # J_DIST = 0.3 # The distance we travel straight for after turning at a junction.
    LINK_DIST = 0.15 # The distance we travel straight for after detecting a side link.
    FORCE_DIST = 0.05 # The distance we continue to force straight for after detecting a corridor.

    FRONT_LIGHT_T = 300
    SIDE_LIGHT_T = 300
    SCAN_LIGHT_T = 300

    def __init__(self):
        self.tracker = MazeTracker()
        self.bitmap = MazeBitmap(self.X_LIM, self.Y_LIM)
        self.beacon_tri = BeaconTri(self.X_LIM, self.Y_LIM)
        self.reached_end = False # Have we reached the end?
        self.is_discovered = False # Have we discovered enough of the maze to find the shortest path?
        # self.reverse_mode = False # Is the robot in reverse mode?
        self.temp_pos = None
        self.is_link_delay = False # Are we delaying the detection of a junction via a side link?
        self.foo_pos = None
        self.is_force_delay = False # Are we delaying stopping forcing straight after a junction?
        self.is_forcing = False # Are we forcing straight after a junction?
        self.last_vertex = None # The last vertex reached in continuous coords.
        self.last_angle = None
        self.last_dr_pos = None # The dead reckoning measurement at the last vertex.
        self.prev_dr_pos = None # The dead reckoning measurement in the previous iteration
        self.last_dr_angle = None
        self.prev_dr_angle = None
        self.robot_pos = None # The best estimate of the robot's current position.
        self.robot_angle = None # The best estimate of the robot's current orientation.
    
    # Reset to the initial state.
    def reset(self):
        self.tracker.reset()
        self.bitmap.reset()
        self.beacon_tri.reset()
        self.reached_end = False
        self.is_discovered = False
        # self.reverse_mode = False
        self.temp_pos = None
        self.is_link_delay = False
        self.foo_pos = None
        self.is_force_delay = False
        self.is_forcing = False
        self.last_vertex = None
        self.last_angle
        self.last_dr_pos = None
        self.prev_dr_pos = None
        self.last_dr_angle = None
        self.prev_dr_angle = None
        self.robot_pos = None
        self.robot_angle = None
    
    # # Discretise a point to fit it to the grid. Also force it into the arena if it is out of bounds.
    # def discretise_point(self, point):
    #     d_point = [round(point[0] / self.RES) * self.RES, round(point[1] / self.RES) * self.RES]
    #     if d_point[0] < 0:
    #         d_point[0] = 0
    #         print('Warning: point out of bounds (left): ' + str(d_point))
    #     elif d_point[0] > self.X_LIM:
    #         d_point[0] = self.X_LIM
    #         print('Warning: point out of bounds (right): ' + str(d_point))
    #     if d_point[1] < 0:
    #         d_point[1] = 0
    #         print('Warning: point out of bounds (top): ' + str(d_point))
    #     elif d_point[1] > self.Y_LIM:
    #         d_point[1] = self.Y_LIM
    #         print('Warning: point out of bounds (bottom): ' + str(d_point))
    #     return (d_point[0], d_point[1])
    
    # Set a starting point.
    def set_start(self, pos):
        # pos = self.discretise_point(raw_pos)
        self.tracker.set_start(pos)
        self.bitmap.set_start(pos)
    
    # Set an end point.
    def set_end(self, pos):
        # pos = self.discretise_point(raw_pos)
        self.tracker.set_end(pos)
        self.bitmap.set_end(pos)
    
    # Set the beacon positions.
    def set_beacons(self, beacon_pos):
        self.beacon_tri.set_beacons(beacon_pos)
    
    # Gets the best estimate of the robot's position in pixel coords.
    def get_pos(self):
        return self.bitmap.to_pixels(self.robot_pos)
    
    # Update the best estimate of the robot's current position using the current dead-reckoning measurement.
    def update_pos(self, dr_pos):
        if self.last_dr_pos == None:
            self.last_dr_pos = dr_pos
            self.robot_pos = self.last_vertex
        else:
            dr_change = (dr_pos[0] - self.last_dr_pos[0], dr_pos[1] - self.last_dr_pos[1])
            self.robot_pos = (self.last_vertex[0] + dr_change[0], self.last_vertex[1] + dr_change[1])
        self.prev_dr_pos = dr_pos
    
    # Gets the best estimate of the robot's position in degrees.
    def get_angle(self):
        return self.bitmap.to_pixels(self.robot_angle)
    
    # Update the best estimate of the robot's current angle using the dead-reckoning measurement.
    def update_angle(self, dr_angle):
        if self.last_dr_angle == None:
            self.last_dr_angle = dr_angle
            self.robot_angle = self.last_angle
        else:
            dr_change = dr_angle - self.last_dr_angle
            self.robot_angle = self.last_angle + dr_change
        self.prev_dr_angle = dr_angle
    
    # Determine whether any action is needed whilst the robot travels through a passage.
    # Takes dead reckoning position and light sensor readings.
    def default_navigate(self, dr_pos, dr_angle, front_light_raw, left_light_raw, right_light_raw):

        self.update_pos(dr_pos)
        self.update_angle(dr_angle)

        if not self.is_discovered:
            self.tracker.update_partial_path(self.robot_pos)
        
        # Apply thresholds to light readings.
        front_light = front_light_raw >= self.FRONT_LIGHT_T
        left_light = left_light_raw >= self.SIDE_LIGHT_T
        right_light = right_light_raw >= self.SIDE_LIGHT_T

        # # Translate light readings to forwards/reverse-relative form.
        # if self.reverse_mode:
        #     r_light = []
        #     for i in range(4):
        #         r_light.append(light[(i + 2) % 4])
        # else:
        #     r_light = light
        
        # Test if we can stop forcing straight.
        if self.is_forcing and not self.is_force_delay and left_light and right_light:
            self.is_force_delay = True
            self.foo_pos = self.robot_pos # Use continuous position.
            print('Start force delay at', self.robot_pos)
        elif self.is_forcing and self.is_force_delay and math.dist(self.foo_pos, self.robot_pos) >= self.FORCE_DIST:
            self.is_forcing = False
            self.is_force_delay = False
            self.foo_pos = None
            print('Stop forcing straight.')

        # if math.dist(pos, self.tracker.end) <= self.RES/2: # If we have reached the end, maybe?
        #     print('Possibly reached end.')
        #     return 'j'
        if front_light: # Maze boundary in front.
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
        elif not self.is_forcing and not (left_light and right_light): # Reached junction.
            if not self.is_link_delay:
                self.is_link_delay = True
                self.temp_pos = self.robot_pos # Use continuous position.
                print('Start link delay at', self.robot_pos)
                return None
            elif math.dist(self.temp_pos, self.robot_pos) >= self.LINK_DIST:
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
    # Assume that at this point, the robot is facing towards the first beacon.
    # Output turning angle is given in degrees clockwise from north, [-180, 180].
    def junction_navigate(self, alpha, beta, gamma, readings):

        # Triangulate.
        self.robot_pos = self.beacon_tri.find_pos(alpha, beta, gamma)
        self.robot_pos = (round(self.robot_pos[0], 3), round(self.robot_pos[1], 3)) # Mostly for convenience when testing...
        self.robot_angle = self.beacon_tri.find_angle(self.robot_pos)

        print('Vertex at', self.robot_pos)

        # Convert light sensor readings to link angles.
        light_scan = []
        for reading in readings:
            if reading >= self.SCAN_LIGHT_T:
                light_scan.append(True)
            else:
                light_scan.append(False)
        link_angles = []
        counting = False
        current_angle = self.robot_angle % 360 # Assume the scan starts and ends at the same angle.
        track_angle = current_angle
        scan_res = len(light_scan)
        for i in range(scan_res):
            current_angle += 360 / scan_res
            current_angle %= 360
            if not light_scan[i] and not counting:
                counting = True
                track_angle = current_angle
                # current_angle = self.robot_angle + i/scan_res * 360
            elif not light_scan[i] and counting:
                track_angle += 0.5/scan_res * 360
                track_angle %= 360
            elif light_scan[i] and counting:
                counting = False
                track_angle += 0.5/scan_res * 360
                track_angle %= 360
                link_angles.append(track_angle)
        if counting and not light_scan[0]:
            # start = 2*current_angle - 360
            # end = 2*link_angles[0] + 360
            # link_angles.append(((start + end) / 2) % 360)
            right_dist = self.tracker.mod_diff(self.robot_angle % 360, link_angles[0], 360)
            # left_dist = self.tracker.mod_diff(self.robot_angle, current_angle) * 2
            link_angles.append((track_angle + right_dist) % 360)
            link_angles.pop(0)
        elif counting and light_scan[0]:
            link_angles.append(track_angle)

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

        # # Translate angle to forwards/reverse-relative form.
        # if self.reverse_mode:
        #     r_angle = (self.robot_angle + 180) % 360
        # else:
        #     r_angle = self.robot_angle

        # disc_pos = self.discretise_point(self.robot_pos)

        # Check if we've reached the end.
        if math.dist(self.robot_pos, self.tracker.end) <= self.tracker.MIN_DIST:
            self.reached_end = True
            print('End reached.')
            if self.is_discovered:
                print('Finished.')
                return 'e'
        
        # Update relevant data structures.
        v_pos = self.tracker.visit_vertex(self.robot_pos, link_angles) # Update graph structures.
        print(self.tracker.a_list)
        self.bitmap.update_pixels(self.tracker.a_list) # Update bitmap.
        if not self.is_discovered: # Update external path in discovery phase.
            self.tracker.generate_partial_path(v_pos, self.robot_pos)
        
        # Apply entry mark and check if enough of the maze has been discovered.
        entry_link = self.tracker.entry_mark(v_pos, link_angles)
        if self.reached_end and not self.is_discovered and self.tracker.enough_discovered():
            self.is_discovered = True
            self.tracker.generate_complete_path()
        
        # Find the target angle.
        if self.is_discovered:
            if math.dist(self.robot_pos, self.tracker.end) <= self.tracker.MIN_DIST:
                print('Finished.')
                return 'e'
            else:
                target_angle = self.tracker.dijkstra_navigate(v_pos)
        else:
            target_angle = self.tracker.discovery_navigate(v_pos, link_angles, entry_link)
        print('Target angle:', target_angle)

        self.tracker.prev_vertex = v_pos # Update previous vertex.
        self.is_forcing = True # Force straight after reaching a junction (and turning).
        self.last_vertex = self.robot_pos
        self.last_dr_pos = self.prev_dr_pos
        self.last_angle = self.robot_angle
        self.last_dr_angle = self.prev_dr_angle
        
        # Generate navigation command.
        # diff = (target_angle - r_angle) % 360 # [0, 360]
        # if diff > 90 and diff < 270: # If it would be quicker to reverse.
        #     self.reverse_mode = not self.reverse_mode
        #     rotation = diff - 180 # [-90, 90]
        # else:
        #     rotation = (diff + 180) % 360 - 180 # [-90, 90]
        # if self.reverse_mode:
        #     return str(int(rotation)) + 'b'
        # else:
        #     return str(int(rotation)) + 'f'
        diff = (target_angle - self.robot_angle) % 360
        rotation = (diff + 180) % 360 - 180
        return str(int(rotation))
    
    # Get the bitmap after calling junction_navigate().
    def get_bitmap(self):
        return self.bitmap.pixels

    # Get the shortest path to display.
    def get_path(self):
        pixel_path = []
        for path_pos in self.tracker.external_path:
            pixel_path.append(self.bitmap.to_pixels(path_pos))
        return pixel_path

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