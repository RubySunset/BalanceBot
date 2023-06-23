import math
from scipy.optimize import minimize

class BeaconTri:

    NUM_BEACONS = 3

    # beacon_pos is a list of beacon positions. Each element is a 2-tuple (x, y).
    # x_lim and y_lim are the dimensions of the arena.
    def __init__(self, x_lim, y_lim):
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.beacon_pos = []
        self.beacon_dist = []
    
    def reset(self):
        self.beacon_pos = []
        self.beacon_dist = []
    
    # Finds the difference between two numbers in modular arithmetic.
    def mod_diff(self, a, b, mod):
        diff = b - a
        if diff <= -mod/2:
            return diff + mod
        elif diff > mod/2:
            return diff - mod
        else:
            return diff
    
    def set_beacons(self, beacon_pos):
        if len(beacon_pos) != self.NUM_BEACONS:
            raise ValueError('Wrong number of beacons.')
        self.beacon_pos = beacon_pos
        self.beacon_dist = []
        for i in range(self.NUM_BEACONS - 1):
            row = []
            for j in range(i + 1, self.NUM_BEACONS):
                row.append(math.dist(beacon_pos[i], beacon_pos[j]))
            self.beacon_dist.append(row)
    
    def func(self, params):
        total = 0
        for i in range(self.NUM_BEACONS - 1):
            for j in range(i + 1, self.NUM_BEACONS):
                d = self.beacon_dist[i][j - i - 1]
                a = self.angles[i][j - i - 1]
                xi, yi = self.beacon_pos[i]
                xj, yj = self.beacon_pos[j]
                x, y = params
                val = (x-xi)**2 + (x-xj)**2 + (y-yi)**2 + (y-yj)**2 - 2*math.sqrt(((x-xi)**2 + (y-yi)**2)*((x-xj)**2 + (y-yj)**2))*math.cos(math.radians(a)) - d**2
                total += val**2
        return total
    
    # Triangules position for any number of beacons.
    def find_pos_general(self, angles):
        self.angles = angles
        res = minimize(self.func, [self.x_lim/2, self.y_lim/2], bounds=[(0, self.x_lim), (0, self.y_lim)]) # First guess is the centre of the arena.
        best_fit = res.x
        return (best_fit[0], best_fit[1])
    
    # Triangules position using three beacons.
    # alpha is the angle at beacon 1.
    # beta is the angle at beacon 2.
    # gamma is the angle at beacon 3.
    def find_pos(self, alpha, beta, gamma):
        a = self.mod_diff(alpha, beta, 360)
        b = self.mod_diff(alpha, gamma, 360)
        c = self.mod_diff(beta, gamma, 360)
        return self.find_pos_general([[a, b], [c]])
    
    # Finds the angle based on position.
    # Note that we assume the robot is facing beacon 1 at this point.
    def find_angle(self, pos):
        diff = (self.beacon_pos[0][0] - pos[0], pos[1] - self.beacon_pos[0][1])
        theta = math.degrees(math.atan2(diff[1], diff[0]))
        if theta >= -90:
            return 90 - theta
        else:
            return -theta - 270