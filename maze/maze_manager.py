from maze_tracker import *
from maze_bitmap import *
from beacon_tri import *
from maze_db import *
# from light_cluster import *
from sonic_cluster import *

# Main class. Instantiate and use in other modules.
class MazeManager:

    # RES = 0.25 # The resolution of the underlying grid that positions are snapped to.

    # J_DIST = 0.3 # The distance we travel straight for after turning at a junction.

    # FL_T = 720
    # FM_T = 710
    # FR_T = 700
    # L_T = 420
    # R_T = 320

    def __init__(self, X_LIM=3, Y_LIM=2, pp=3, LINK_DIST=0.15, FORCE_DIST=0.05, MIN_DIST=0.25):
        self.X_LIM = X_LIM
        self.Y_LIM = Y_LIM
        self.LINK_DIST = LINK_DIST # The distance we travel straight for after detecting a side link.
        self.FORCE_DIST = FORCE_DIST # The distance we continue to force straight for after detecting a corridor.
        self.tracker = MazeTracker(MIN_DIST)
        self.bitmap = MazeBitmap(self.X_LIM, self.Y_LIM, pp)
        self.beacon_tri = BeaconTri(self.X_LIM, self.Y_LIM)
        self.maze_db = MazeDB()
        self.maze_db.set_c_string('mongodb://admin:secret@13.43.40.216:6000/admin?authMechanism=DEFAULT') # TODO remove this.
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
    
    # Set a starting point.
    def set_start(self, pos):
        # pos = self.discretise_point(raw_pos)
        self.tracker.set_start(pos)
        self.bitmap.set_start(pos)
        return self.bitmap.to_pixels(pos)
    
    # Set an end point.
    def set_end(self, pos):
        # pos = self.discretise_point(raw_pos)
        self.tracker.set_end(pos)
        self.bitmap.set_end(pos)
        return self.bitmap.to_pixels(pos)
    
    # Set the beacon positions.
    def set_beacons(self, beacon_pos):
        self.beacon_tri.set_beacons(beacon_pos)
    
    # Set the MongoDB connection string.
    def set_connection_string(self, connection_string):
        self.maze_db.set_c_string(connection_string)
    
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
        return self.robot_angle
    
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
    # Takes dead reckoning position and ultrasonic sensor readings.
    def default_navigate(self, dr_pos, dr_angle, front_dist, left_dist, right_dist):

        self.update_pos(dr_pos)
        self.update_angle(dr_angle)

        if not self.is_discovered:
            self.tracker.update_partial_path(self.robot_pos)
        
        # Apply thresholds to ultrasonic readings.
        front_clear = front_dist >= 10
        left_clear = left_dist >= 30
        right_clear = right_dist >= 30

        # # Translate light readings to forwards/reverse-relative form.
        # if self.reverse_mode:
        #     r_light = []
        #     for i in range(4):
        #         r_light.append(light[(i + 2) % 4])
        # else:
        #     r_light = light
        
        # Test if we can stop forcing straight.
        if self.is_forcing and not self.is_force_delay and not left_clear and not right_clear:
            self.is_force_delay = True
            self.foo_pos = self.robot_pos # Use continuous position.
            # print('Start force delay at', self.robot_pos)
        elif self.is_forcing and self.is_force_delay and math.dist(self.foo_pos, self.robot_pos) >= self.FORCE_DIST:
            self.is_forcing = False
            self.is_force_delay = False
            self.foo_pos = None
            # print('Stop forcing straight.')

        # if math.dist(pos, self.tracker.end) <= self.RES/2: # If we have reached the end, maybe?
        #     print('Possibly reached end.')
        #     return 'j'
        if not front_clear: # Maze boundary in front.
            if self.is_link_delay: # Reset link delay if one was active.
                self.is_link_delay = False
                self.temp_pos = None
                # print('Reset link delay.')
            if self.is_force_delay: # Reset force delay if one was active.
                self.is_force_delay = False
                self.foo_pos = None
                # print('Reset force delay.')
            # print('Boundary ahead - junction.')
            return 'j'
        # elif math.dist(pos, self.tracker.prev_vertex) < self.J_DIST: # Force straight after junction.
        #     return None
        elif not self.is_forcing and (left_clear or right_clear): # Reached junction.
            if not self.is_link_delay:
                self.is_link_delay = True
                self.temp_pos = self.robot_pos # Use continuous position.
                # print('Start link delay at', self.robot_pos)
                return None
            elif math.dist(self.temp_pos, self.robot_pos) >= self.LINK_DIST:
                self.is_link_delay = False
                self.temp_pos = None
                # print('Link to side - junction.')
                return 'j'
            else:
                return None
        else: # Default: in a corridor.
            if self.is_link_delay: # Abandon a link delay if there is one.
                self.is_link_delay = False
                self.temp_pos = None
                # print('Abandon link delay.')
            return None

    # Generate a navigation command after a junction has been mapped.
    # Assume that at this point, the robot is facing towards the first beacon.
    # Output turning angle is given in degrees clockwise from north, [-180, 180].
    def junction_navigate(self, alpha, beta, gamma, angles, left_dist, right_dist):

        # Triangulate.
        self.robot_pos = self.beacon_tri.find_pos(alpha, beta, gamma)
        self.robot_pos = (round(self.robot_pos[0], 3), round(self.robot_pos[1], 3)) # Mostly for convenience when testing...
        self.robot_angle = self.beacon_tri.find_angle(self.robot_pos)

        print('Vertex at', self.robot_pos)

        # link_angles = find_link_angles(left, self.robot_angle)
        # link_angles = find_link_angles_cluster(self.robot_angle, angles, left)
        link_angles = find_link_angles(angles, left_dist, right_dist, self.robot_angle)

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
                # self.maze_db.add_doc(self.get_bitmap(), self.get_path())
                # self.maze_db.add_doc(self.bitmap.to_pixels(self.tracker.start), self.bitmap.to_pixels(self.tracker.end), self.get_edges(), self.get_path())
                # TODO
                return 'e'
        
        # Update relevant data structures.
        v_pos = self.tracker.visit_vertex(self.robot_pos, link_angles, self.is_discovered) # Update graph structures.
        self.bitmap.update_pixels(self.tracker.a_list) # Update bitmap.
        if not self.is_discovered: # Update external path in discovery phase.
            self.tracker.generate_partial_path(v_pos, self.robot_pos)
        
        # Apply entry mark and check if enough of the maze has been discovered.
        # self.tracker.entry_mark(v_pos, link_angles)
        if self.reached_end and not self.is_discovered and self.tracker.enough_discovered():
            self.is_discovered = True
            self.tracker.generate_complete_path()
        
        # Find the target angle.
        if self.is_discovered:
            if math.dist(self.robot_pos, self.tracker.end) <= self.tracker.MIN_DIST:
                print('Finished.')
                # self.maze_db.add_doc(self.get_bitmap(), self.get_path())
                # self.maze_db.add_doc(self.bitmap.to_pixels(self.tracker.start), self.bitmap.to_pixels(self.tracker.end), self.get_edges(), self.get_path())
                # TODO
                return 'e'
            else:
                target_angle = self.tracker.dijkstra_navigate(v_pos)
        else:
            target_angle = self.tracker.discovery_navigate(v_pos, link_angles)
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
        temp = []
        for i in range(self.bitmap.X_PIXELS):
            row = []
            for j in range(self.bitmap.Y_PIXELS):
                pixel = self.bitmap.pixels[i][j]
                if pixel == PixelType.EMPTY:
                    row.append(0)
                elif pixel == PixelType.START:
                    row.append(1)
                elif pixel == PixelType.END:
                    row.append(2)
                elif pixel == PixelType.VERTEX:
                    row.append(3)
                elif pixel == PixelType.PATH:
                    row.append(4)
                elif pixel == PixelType.CLEARED:
                    row.append(5)
                else:
                    raise ValueError('Incorrect pixel value.')
            temp.append(row)
        return temp
    
    # Get the pairs of vertices in pixel coordinates.
    def get_edges(self):
        edges = []
        for v in self.tracker.a_list:
            for n in self.tracker.a_list[v]:
                a = self.bitmap.to_pixels(v)
                b = self.bitmap.to_pixels(n)
                if (a, b) not in edges and (b, a) not in edges:
                    edges.append((a, b))
        return edges

    # Get the shortest path to display in pixel coordinates.
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