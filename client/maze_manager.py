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
        # Note that for the time being, I will assume that the entire maze has been mapped by the time the end is reached.
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
    
    # Set an end point.
    def set_end(self, pos):
        self.tracker.set_end(pos)
    
    # Process sensor data from the robot to generate the next command.
    # Angle is given in degrees starting clockwise from north, [0, 360].
    # Light readings are a 4-tuple of booleans.
    # Return values:
    # f: go forwards.
    # b: go backwards.
    # j: stop at a junction (sweep and perform computer vision).
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

        # Find what to do next.
        if self.is_initial: # Initial update.
            self.is_initial = False
            print('Starting...')
            return 'j' # Always consider the starting point as a junction.
        elif abs(pos - self.end) < self.RES/2: # Reached the end.
            self.reached_end = True
            self.tracker.find_shortest_path(self)
            print('End reached.')
            return 'e'
        elif r_light[0]: # Maze boundary in front.
            print('Junction - boundary in front.')
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
    
    # Map the maze using the boundary walls from computer vision.
    def map_maze(self, walls):
        for wall in walls:
            self.tracker.add_wall(wall)
    
    # Generate a navigation command after a junction has been mapped.
    # Angle is given in degrees starting clockwise from north, [0, 360].
    # Light readings are booleans taken around the entire unit circle (clockwise).
    # Returns a value in degrees to rotate by (+: clockwise, -: anticlockwise).
    def junction_navigate(self, pos, angle, light):
        self.tracker.update_path(pos) # Update the path to send to the web server.
        mid = int(len(light) / 2) # Midpoint index of the light array.
        s_index = None
        count = 0
        for i in range(len(light)):
            if not light[i + mid] and s_index == None: # Start from the midpoint to find the leftmost link.
                s_index = i + mid
                count += 1
            if light[i + mid] and s_index != None:
                break
        if s_index == None:
            raise Exception('junction_navigate(): link not found.')
        target = s_index + int(count / 2) # The target direction in terms of the length of the input array.
        target_deg = int((target / len(light)) * 360) # Convert to degrees.
        diff = (target_deg - angle) % 360 # [0, 360]
        if diff > 90 and diff < 270: # If it would be quicker to reverse.
            self.reverse_mode = not self.reverse_mode
            return diff - 180 # [-90, 90]
        else:
            return (diff + 180) % 360 - 180 # [-90, 90]