from maze_tracker import *

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
        self.is_initial = True # Initial state?
        self.reached_end = False # Have we reached the end?
        self.prev_junction = None # Used to force straight after leaving a junction for a while.
        self.reverse_mode = False # Is the robot in reverse mode?
    
    # Reset to the initial state.
    def reset(self):
        self.tracker.reset()
        self.is_initial = True
        self.reached_end = False
        self.prev_junction = None
        self.reverse_mode = False
    
    # Set a starting point.
    def set_start(self, pos):
        self.tracker.set_start(pos)
        self.tracker.prev_vertex = self.tracker.start
    
    # Set an end point.
    def set_end(self, pos):
        self.tracker.set_end(pos)
    
    # Process sensor data from the robot to generate the next command.
    # Angle is given in degrees starting clockwise from north, [0, 360].
    # Light readings are a 4-tuple of booleans.
    # Return values:
    # f: go forwards.
    # b: go backwards.
    # j: scan at junction.
    # e: stop (end reached).
    def default_navigate(self, pos, angle, light):
        
        # Translate inputs to internal form.
        if self.reverse_mode:
            r_angle = (angle + 180) % 360
            r_light = []
            for i in range(4):
                r_light.append(light[(i + 2) % 4])
        else:
            r_angle = angle
            r_light = light

        if r_light[0]: # Maze boundary in front.
            print('Scan: boundary in front.')
            return 'j'
        elif abs(pos - self.prev_junction) < self.J_DIST: # Force straight after junction.
            pass # Continue straight
        elif not r_light[1] or not r_light[3]: # Reached junction.
            print('Junction - link to side.')
            return 'j'
        # Default: in a corridor.

        # If we need to go straight.
        if self.reverse_mode:
            return 'b'
        else:
            return 'f'

    # Generate a navigation command after a junction has been mapped.
    # Angle is given in degrees starting clockwise from north, [0, 360].
    # Link angles are the angles of paths from this junction.
    # Returns either a stop command if end reached, or a value in degrees to rotate by (+: clockwise, -: anticlockwise).
    def junction_navigate(self, pos, angle, link_angles):

        # Check if we've reached the end.
        if abs(pos - self.tracker.end) < self.RES/2:
            self.reached_end = True
            print('End reached.')
            return 'e'
        
        # Choose which link to go through and find the angle.
        target_deg = self.tracker.visit_vertex()
        diff = (target_deg - angle) % 360 # [0, 360]
        if diff > 90 and diff < 270: # If it would be quicker to reverse.
            self.reverse_mode = not self.reverse_mode
            return diff - 180 # [-90, 90]
        else:
            return (diff + 180) % 360 - 180 # [-90, 90]