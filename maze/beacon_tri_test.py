import random
import math
from beacon_tri import *

NUM_TESTS = 10000

# Assume a 3x3 arena for now.
X_LIM = 3
Y_LIM = 3

# Testing configurations.
beacon_pos = [(0, 0), (3, 0), (0, 3)]
# beacon_pos = [(1.5, 0), (0, 3), (3, 3)]

beacon_tri = BeaconTri(X_LIM, Y_LIM)
beacon_tri.set_beacons(beacon_pos)

passed = True
for k in range(NUM_TESTS):
    true_pos = (X_LIM*random.random(), Y_LIM*random.random())
    beacon_dist = []
    for i in range(beacon_tri.NUM_BEACONS):
        beacon_dist.append(math.dist(true_pos, beacon_pos[i]))
    angles = []
    for i in range(beacon_tri.NUM_BEACONS - 1):
        row = []
        for j in range(i + 1, beacon_tri.NUM_BEACONS):
            a = math.dist(beacon_pos[i], beacon_pos[j])
            b = beacon_dist[i]
            c = beacon_dist[j]
            row.append(math.degrees(math.acos((b**2 + c**2 - a**2)/(2*b*c))))
        angles.append(row)
    tri_pos = beacon_tri.find_pos_general(angles)
    error = math.dist(true_pos, tri_pos)
    print('True pos: ' + str(true_pos) + ', tri pos: ' + str(tri_pos) + ', error: ' + str(error))
    if abs(error) > 0.01: # Allow 1cm error.
        passed = False
        break

if passed:
    print('Test passed!')
else:
    print('Test failed.')