import pygame
import time
from enum import Enum
from maze_manager import *

# Mutable parameters.
FRONT_RANGE = 1 # The range of the front sensors.
SIDE_RANGE = 1 # The range of the side sensors.
SCAN_RANGE = 1 # The range of the side sensors when scanning.
SCAN_RES = 360
LINK_DIST = 0.15
FORCE_DIST = 0.05
MIN_DIST = 0.25

# Sensor parameters.
# FRONT_ANGLE = 20 # The angle of acceptance of the front sensors.
# SIDE_ANGLE = 15 # The angle of acceptance of the side sensors.
# FRONT_ANGLE = 5
# SIDE_ANGLE = 5
# SCAN_RES = 64 # The number of vectors we consider in a junction scan.

# Course-correction controller parameters.
# P_CC = 0.3 # Course correction proportional gain.
# I_CC = 0.0001 # Course correction integral gain.
# D_CC = 0.3 # Course correction derivative gain.
# MAX_OFFSET = 5 # The maximum amount of course-correction that can be applied, in degrees.
# UNBOUND_DIST = 0.3 # The distance for which unbounded CC is applied.

# Course-correction controller variables.
# cc_active = False
# cc_sum = 0
# cc_prev = 0
# cc_prev_left_dist = 0
# cc_prev_right_dist = 0
# cc_prev_width = 0
# cc_offset = 0
# cc_prev_vertex = None

# Other parameters.
PIXEL_RES = 0.01 # Metres/pixel
SPEED = 0.01 # The distance travelled by the robot between each update.

class PixelType(Enum):
    EMPTY = 0
    START = 1
    END = 2
    WALL = 3

# Convert a point in metres (standard units) to pixels.
def to_pixels(point):
    return (round(point[0] / PIXEL_RES), round(point[1] / PIXEL_RES))

# Render the pixel array.
def render_pixels():
    last_time = time.time()
    image = Image.new('RGB', (X_PIXELS, Y_PIXELS), 'white')
    img_pixels = image.load()
    for i in range(image.size[0]):
        for j in range(image.size[1]):
            pixel = pixels[i][j]
            if pixel == PixelType.START:
                colour = (0, 255, 0)
            elif pixel == PixelType.END:
                colour = (255, 0, 0)
            elif pixel == PixelType.EMPTY:
                colour = (0, 0, 0)
            elif pixel == PixelType.WALL:
                colour = (255, 255, 255)
            else:
                raise ValueError('Incorrect pixel value: ' + str(pixel) + ' at ' + str((i, j)) + '.')
            img_pixels[i, j] = colour
    image.show()
    print('Image rendering time:', round(time.time() - last_time, 3))

# Generate a bitmap representing the maze.
walls = []

def config1():
    walls.append(((0.5, 0), (0.5, 1.5)))
    walls.append(((1, 2), (1.5, 1.5)))
    walls.append(((2, 1), (2.5, 0.5)))
    walls.append(((1, 0.5), (2, 0.5)))
    walls.append(((1, 1), (1.5, 1)))
    walls.append(((2, 0.5), (2, 1)))
    walls.append(((1.5, 1), (1.5, 1.5)))
    walls.append(((3, 1), (2.5, 1.5)))
    walls.append(((2, 1.5), (2.5, 1.5)))

def config2():
    walls.append(((0.5, 0), (0.5, 1.5)))
    walls.append(((1, 2), (1, 0.5)))
    walls.append(((1.5, 0), (1.5, 1.5)))
    walls.append(((2, 2), (2, 0.5)))
    walls.append(((2, 0.5), (2.5, 0.5)))
    walls.append(((2.5, 0.5), (2.5, 2)))

def config3():
    walls.append(((0.5, 0), (0.5, 1.5)))
    walls.append(((1, 0.5), (1, 1.5)))
    walls.append(((1, 0.5), (2.5, 0.5)))
    walls.append(((1, 1.5), (2.5, 1.5)))
    walls.append(((1.5, 1), (2.5, 1)))
    walls.append(((2.5, 1.5), (2.5, 2)))

def config4():
    walls.append(((0.5, 0), (0.5, 1.5)))
    walls.append(((0.5, 1.5), (1, 1.5)))
    walls.append(((1, 0.5), (1, 1)))
    walls.append(((1, 0.5), (2.5, 0.5)))
    walls.append(((2.5, 0.5), (2.5, 1)))
    walls.append(((2.5, 1.5), (2.5, 2)))
    walls.append(((1.5, 1.5), (2.5, 1.5)))
    walls.append(((1.5, 1), (2, 1)))
    walls.append(((2, 1), (2, 1.5)))

def config5():
    walls.append(((0.5, 0.5), (0.5, 1)))
    walls.append(((0.5, 1), (1, 1)))
    walls.append(((0.5, 1.5), (1, 1.5)))
    walls.append(((1, 0.5), (1.5, 0.5)))
    walls.append(((1.5, 0.5), (1.5, 1)))
    walls.append(((1.5, 1.5), (2, 1.5)))
    walls.append(((2, 1), (2, 1.5)))
    walls.append(((2, 0.5), (2.5, 0.5)))
    walls.append(((2.5, 0.5), (2.5, 1)))
    walls.append(((2.5, 1.5), (3, 1.5)))

def config6():
    walls.append(((0.5, 0), (0.5, 1.5)))
    walls.append(((0.5, 1.5), (2.5, 0)))
    walls.append(((0.5, 2), (1.5, 1.25)))
    walls.append(((1.5, 1.25), (1.5, 2)))
    walls.append(((2, 1), (2.5, 0.5)))
    walls.append(((2, 1), (2, 1.5)))
    walls.append(((2.5, 0.5), (2.5, 1.5)))
    walls.append(((2, 1.5), (2.5, 1.5)))

def config7():
    walls.append(((0.5, 0.5), (1, 0.5)))
    walls.append(((0, 1), (1, 1)))
    walls.append(((1, 1), (1, 1.5)))
    walls.append(((0.5, 1.5), (1, 1.5)))
    walls.append(((1.5, 0.5), (2, 0.5)))
    walls.append(((1.5, 0.5), (1.5, 1)))
    walls.append(((2.5, 0.5), (2.5, 2)))
    walls.append(((1.5, 1), (2.5, 2)))
    walls.append(((1.5, 1.5), (1.5, 2)))
    walls.append(((1.5, 1.5), (2, 2)))

def config8():
    walls.append(((0.5, 0.5), (1, 0.5)))
    walls.append(((0, 1), (1, 1)))
    walls.append(((1, 1), (1, 1.5)))
    walls.append(((0.5, 1.5), (1, 1.5)))
    walls.append(((1.5, 0.5), (2, 0.5)))
    walls.append(((1.5, 0.5), (1.5, 1)))
    walls.append(((2.5, 0.5), (2.5, 1.5)))
    walls.append(((2, 1), (2.5, 1.5)))
    walls.append(((2, 1), (2.5, 1)))
    walls.append(((1.5, 1), (2.5, 2)))
    walls.append(((1.5, 1.5), (1.5, 2)))
    walls.append(((1.5, 1.5), (2, 2)))

def config9():
    walls.append(((0, 0.5), (1, 0.5)))
    walls.append(((1, 0.5), (1, 3.5)))
    walls.append(((0.5, 3.5), (1, 3.5)))
    walls.append(((0.5, 1), (0.5, 3.5)))

    # walls.append(((1.5, 0.5), (3.5, 0.5)))
    # walls.append(((3.5, 0.5), (3.5, 1.5)))
    # walls.append(((2, 1.5), (3.5, 1.5)))
    # walls.append(((2, 1.5), (2, 3.5)))
    # walls.append(((1.5, 0.5), (1.5, 3.5)))
    walls.append(((1.5, 0.5), (4, 0.5)))
    walls.append(((4, 0.5), (4, 2.5)))
    walls.append(((4, 2.5), (2, 2.5)))
    walls.append(((2, 2.5), (2, 3.5)))
    walls.append(((1.5, 0.5), (1.5, 3.5)))
    
    # walls.append(((4.25, 0), (4.25, 2.5)))
    # walls.append(((3, 2.5), (4.25, 2.5)))
    # walls.append(((2, 3.5), (4.5, 3.5)))
    walls.append(((4.5, 0), (4.5, 3)))
    walls.append(((4.5, 3), (2.5, 3)))
    walls.append(((1.5, 3.5), (5, 3.5)))
    
    walls.append(((5, 0), (5, 3)))
    walls.append(((5, 3), (6, 4)))

    walls.append(((5, 3.5), (5, 4)))

    # temp = []
    # for wall in walls:
    #     temp.append(((wall[0][0] / 2, wall[0][1] / 2), (wall[1][0] / 2, wall[1][1] / 2)))
    # walls.clear()
    # for wall in temp:
    #     walls.append(wall)

# Configs to test: 4, 8

config4()
WALL_WIDTH = 0.08
X_LIM = 3
Y_LIM = 2
pp = 3
start = (0.25, 0.25)
end = (2.75, 1.75)

# config9()
# WALL_WIDTH = 0.08
# X_LIM = 6
# Y_LIM = 4
# pp = 2
# start = (0.25, 0.25)
# end = (2.75, 1.75)

X_PIXELS = int(X_LIM / PIXEL_RES) + 1 # The number of pixels in the x direction.
Y_PIXELS = int(Y_LIM / PIXEL_RES) + 1 # The number of pixels in the y direction.

pygame.init()
screen = pygame.display.set_mode((X_PIXELS * pp, Y_PIXELS * pp))

# Add boundary walls.
walls.append(((0, 0), (X_LIM, 0)))
walls.append(((X_LIM, 0), (X_LIM, Y_LIM)))
walls.append(((0, Y_LIM), (X_LIM, Y_LIM)))
walls.append(((0, 0), (0, Y_LIM)))

pixels = []
for i in range(X_PIXELS):
    pixel_row = []
    for j in range(Y_PIXELS):
        pixel_row.append(PixelType.EMPTY)
    pixels.append(pixel_row)
for wall in walls:
    p1 = to_pixels(wall[0])
    p2 = to_pixels(wall[1])
    diff = [p2[i] - p1[i] for i in range(2)] # Find the difference vector.
    p_dist = math.ceil(math.dist(p1, p2)) # Choose the upper bound on the length of the wall.
    unit_diff = [diff[i] / p_dist for i in range(2)] # Normalise the difference vector to get the direction.
    current = [p1[0], p1[1]]
    for i in range(p_dist + 1):
        c = [round(x) for x in current]
        for j in range(max(0, math.floor(c[0] - WALL_WIDTH/PIXEL_RES)), min(math.ceil(c[0] + WALL_WIDTH/PIXEL_RES) + 1, X_PIXELS)):
            for k in range(max(0, math.floor(c[1] - WALL_WIDTH/PIXEL_RES)), min(math.ceil(c[1] + WALL_WIDTH/PIXEL_RES) + 1, Y_PIXELS)):
                pixel = pixels[j][k]
                if pixel in (PixelType.START, PixelType.END, PixelType.WALL): # Skip over start, end, previously placed walls.
                    pass
                elif math.dist(c, (j, k)) <= WALL_WIDTH/PIXEL_RES:
                    pixels[j][k] = PixelType.WALL # Mark this cell as inaccessible.
        current = [current[i] + unit_diff[i] for i in range(2)]


# render_pixels()

manager = MazeManager(X_LIM, Y_LIM, pp=pp, LINK_DIST=LINK_DIST, FORCE_DIST=FORCE_DIST, MIN_DIST=MIN_DIST)
manager.bitmap.update_walls(walls, WALL_WIDTH)

# Set up arena.
manager.set_start(start)
manager.set_end(end)
manager.set_beacons([(0, 0), (3, 0), (0, 3)])

# Initialise robot.
pos = [start[0], start[1]] # Assume robot is initially at start position.
ppos = to_pixels(start) # Pixel position.
angle = 180 # Assume robot is initially pointing south.
target_angle = 180 # The target angle from the last rotation command.
# reverse_mode = False # Whether or not the robot is going in reverse.
robot_path = []

prev_dist_L = 10000
prev_dist_R = 10000

iterations = 0

while True:

    # Check for timeout.
    iterations += 1
    # print('Iteration', iterations)
    if iterations > 10000:
        print('!! Timeout !!')
        break

    # loci = []
    # for i in range(4):
    #     loci.append((0.25 * math.sin(i/4*math.pi + math.radians(angle)), -0.25 * math.cos(i/4*math.pi + math.radians(angle))))
    # loci_p = [to_pixels(loci[i]) for i in range(4)]
    # # loci = ((0, -0.25), (0.25, 0), (0, 0.25), (-0.25, 0))
    # for i in range(4):
    #     p = [ppos[j] + loci_p[i][j] for j in range(2)]
    #     for j in range(max(0, math.floor(ppos[0] - 0.4/PIXEL_RES)), min(math.ceil(ppos[0] + 0.4/PIXEL_RES) + 1, X_PIXELS)):
    #         for k in range(max(0, math.floor(ppos[1] - 0.4/PIXEL_RES)), min(math.ceil(ppos[1] + 0.4/PIXEL_RES) + 1, Y_PIXELS)):
    #             if pixels[j][k] == PixelType.WALL and math.dist([j, k], p) <= 0.15/PIXEL_RES:
    #                 raw_readings[i] += 1
    
    # f = 0
    # l = 0
    # r = 0
    # for i in range(max(0, math.floor(ppos[0] - max(FRONT_RANGE, SIDE_RANGE)/PIXEL_RES)), min(math.ceil(ppos[0] + max(FRONT_RANGE, SIDE_RANGE)/PIXEL_RES) + 1, X_PIXELS)):
    #     for j in range(max(0, math.floor(ppos[1] - max(FRONT_RANGE, SIDE_RANGE)/PIXEL_RES)), min(math.ceil(ppos[1] + max(FRONT_RANGE, SIDE_RANGE)/PIXEL_RES) + 1, Y_PIXELS)):
    #         if pixels[i][j] == PixelType.WALL:
    #             dist = math.dist((i, j), ppos)
    #             arg = (90 - (math.degrees(math.atan2(ppos[1] - j, i - ppos[0])) % 360)) % 360
    #             adj_arg = (arg - angle) % 360
    #             if dist <= FRONT_RANGE/PIXEL_RES and (adj_arg <= FRONT_ANGLE/2 or adj_arg >= 360 - FRONT_ANGLE/2):
    #                 f = 1000
    #             elif dist <= SIDE_RANGE/PIXEL_RES and adj_arg >= 90 - SIDE_ANGLE/2 and adj_arg <= 90 + SIDE_ANGLE/2:
    #                 l = 1000
    #             # elif dist <= FRONT_RANGE/PIXEL_RES and adj_arg >= 180 - FRONT_ANGLE/2 and adj_arg <= 180 + FRONT_ANGLE/2:
    #             #     light[2] = True
    #             elif dist <= SIDE_RANGE/PIXEL_RES and adj_arg >= 270 - SIDE_ANGLE/2 and adj_arg <= 270 + SIDE_ANGLE/2:
    #                 r = 1000
    pixel_dist = [1000, 1000, 1000]
    sensor_range = (FRONT_RANGE, SIDE_RANGE, SIDE_RANGE)
    add_angle = (0, -90, 90)
    for i in range(3):
        sensor_angle = (angle + add_angle[i]) % 360
        unit_v = (math.sin(math.radians(sensor_angle)), -math.cos(math.radians(sensor_angle)))
        current = [ppos[0], ppos[1]]
        for j in range(round(sensor_range[i]/PIXEL_RES)):
            p = [round(current[k]) for k in range(2)]
            if pixels[p[0]][p[1]] == PixelType.WALL:
                pixel_dist[i] = j
                break
            current = [current[k] + unit_v[k] for k in range(2)]
    # Each pixel is 1cm anyway.
    
    if iterations == 1:
        command = 'j'
    else:
        command = manager.default_navigate((pos[0] + iterations*0.001, pos[1] + iterations*0.001), angle + iterations*0.01, pixel_dist[0], pixel_dist[1], pixel_dist[2])
        # Simulate drift (linear w.r.t. time).

    if command == 'j':

        cc_prev_vertex = pos
        prev_dist_L = 10000
        prev_dist_R = 10000

        # loci = []
        # for i in range(8):
        #     loci.append((0.25 * math.sin(i/4*math.pi), -0.25 * math.cos(i/4*math.pi)))
        # loci_p = [to_pixels(loci[i]) for i in range(8)]
        # for i in range(8):
        #     p = [ppos[j] + loci_p[i][j] for j in range(2)]
        #     for j in range(max(0, math.floor(ppos[0] - 0.4/PIXEL_RES)), min(math.ceil(ppos[0] + 0.4/PIXEL_RES) + 1, X_PIXELS)):
        #         for k in range(max(0, math.floor(ppos[1] - 0.4/PIXEL_RES)), min(math.ceil(ppos[1] + 0.4/PIXEL_RES) + 1, Y_PIXELS)):
        #             if pixels[j][k] == PixelType.WALL and math.dist([j, k], p) <= 0.15/PIXEL_RES:
        #                 light_scan_raw[i] += 1

        # light_scan = []
        # for i in range(8):
        #     light_scan.append(False)
        # args = [0, 45, 90, 135, 180, 225, 270, 315]
        # for i in range(max(0, math.floor(ppos[0] - 0.5/PIXEL_RES)), min(math.ceil(ppos[0] + 0.5/PIXEL_RES) + 1, X_PIXELS)):
        #     for j in range(max(0, math.floor(ppos[1] - 0.5/PIXEL_RES)), min(math.ceil(ppos[1] + 0.5/PIXEL_RES) + 1, Y_PIXELS)):
        #         if pixels[i][j] == PixelType.WALL:
        #             dist = math.dist((i, j), ppos)
        #             arg = (90 - (math.degrees(math.atan2(ppos[1] - j, i - ppos[0])) % 360)) % 360
        #             for k in range(8):
        #                 if dist <= 0.5/PIXEL_RES and abs(args[k] - arg) <= 5:
        #                     light_scan[k] = True
        # link_angles = []
        # for i in range(8):
        #     if not light_scan[i]:
        #         link_angles.append(math.degrees(math.pi/4 * i))

        # Simulate angles for triangulation.
        # beacon_dist = []
        # for i in range(3):
        #     beacon_dist.append(math.dist(pos, manager.beacon_tri.beacon_pos[i]))
        # beacon_angles = []
        # pairs = ((0, 1), (0, 2), (1, 2))
        # for pair in pairs:
        #     a = math.dist(manager.beacon_tri.beacon_pos[pair[0]], manager.beacon_tri.beacon_pos[pair[1]])
        #     b = beacon_dist[pair[0]]
        #     c = beacon_dist[pair[1]]
        #     beacon_angles.append(math.degrees(math.acos((b**2 + c**2 - a**2)/(2*b*c))))
        beacon_angles = []
        for i in range(3):
            diff = (manager.beacon_tri.beacon_pos[i][0] - pos[0], pos[1] - manager.beacon_tri.beacon_pos[i][1])
            arg = (90 - (math.degrees(math.atan2(diff[1], diff[0])) % 360)) % 360
            beacon_angles.append(arg)
        
        # Simulate turning the robot to face the first beacon.
        diff = (manager.beacon_tri.beacon_pos[0][0] - pos[0], pos[1] - manager.beacon_tri.beacon_pos[0][1])
        theta = math.degrees(math.atan2(diff[1], diff[0]))
        if theta >= -90:
            angle = 90 - theta
        else:
            angle = -theta - 270
        
        # light_scan = []
        # for i in range(SCAN_RES):
        #     light_scan.append(0)
        # for i in range(max(0, math.floor(ppos[0] - SCAN_RANGE/PIXEL_RES)), min(math.ceil(ppos[0] + SCAN_RANGE/PIXEL_RES) + 1, X_PIXELS)):
        #     for j in range(max(0, math.floor(ppos[1] - SCAN_RANGE/PIXEL_RES)), min(math.ceil(ppos[1] + SCAN_RANGE/PIXEL_RES) + 1, Y_PIXELS)):
        #         if pixels[i][j] == PixelType.WALL and math.dist((i, j), ppos) <= SCAN_RANGE/PIXEL_RES:
        #             arg = (90 - (math.degrees(math.atan2(ppos[1] - j, i - ppos[0])) % 360)) % 360
        #             adj_arg = (arg - angle) % 360
        #             light_scan[int(adj_arg/360 * SCAN_RES)] = 1000
        
        # scan_angles = []
        # scan_left = []
        # for i in range(max(0, math.floor(ppos[0] - SCAN_RANGE/PIXEL_RES)), min(math.ceil(ppos[0] + SCAN_RANGE/PIXEL_RES) + 1, X_PIXELS)):
        #     for j in range(max(0, math.floor(ppos[1] - SCAN_RANGE/PIXEL_RES)), min(math.ceil(ppos[1] + SCAN_RANGE/PIXEL_RES) + 1, Y_PIXELS)):
        #         arg = (90 - (math.degrees(math.atan2(ppos[1] - j, i - ppos[0])) % 360)) % 360
        #         adj_arg = (arg - angle) % 360
        #         scan_angles.append((arg + 90) % 360)
        #         if pixels[i][j] == PixelType.WALL and math.dist((i, j), ppos) <= SCAN_RANGE/PIXEL_RES:
        #             # light_scan[int(adj_arg/360 * SCAN_RES)] = 1000
        #             scan_left.append(1000)
        #         else:
        #             scan_left.append(0)

        scan_angles = []
        scan_left = []
        scan_right = []
        for i in range(SCAN_RES):
            scan_angles.append(int(i / SCAN_RES * 360))
            scan_left.append(1000)
            scan_right.append(1000)
        # for i in range(max(0, math.floor(ppos[0] - SCAN_RANGE/PIXEL_RES)), min(math.ceil(ppos[0] + SCAN_RANGE/PIXEL_RES) + 1, X_PIXELS)):
        #     for j in range(max(0, math.floor(ppos[1] - SCAN_RANGE/PIXEL_RES)), min(math.ceil(ppos[1] + SCAN_RANGE/PIXEL_RES) + 1, Y_PIXELS)):
        #         if pixels[i][j] == PixelType.WALL and math.dist((i, j), ppos) <= SCAN_RANGE/PIXEL_RES:
        #             arg = (90 - (math.degrees(math.atan2(ppos[1] - j, i - ppos[0])) % 360)) % 360
        #             left_arg = (arg - angle + 90) % 360
        #             scan_left[int(left_arg/360 * SCAN_RES)] = 0
        #             right_arg = (arg - angle - 90) % 360
        #             scan_right[int(right_arg/360 * SCAN_RES)] = 0
        for i in range(SCAN_RES):
            left_angle = (angle + int(i / SCAN_RES * 360) - 90) % 360
            unit_v = (math.sin(math.radians(left_angle)), -math.cos(math.radians(left_angle)))
            current = [ppos[0], ppos[1]]
            for j in range(round(SCAN_RANGE/PIXEL_RES)):
                p = [round(current[k]) for k in range(2)]
                if pixels[p[0]][p[1]] == PixelType.WALL:
                    scan_left[i] = j
                    break
                current = [current[k] + unit_v[k] for k in range(2)]
        # Each pixel is 1cm anyway.

        # command = manager.junction_navigate(beacon_angles[0], beacon_angles[1], beacon_angles[2], light_scan)
        command = manager.junction_navigate(beacon_angles[0], beacon_angles[1], beacon_angles[2], scan_angles, scan_left, scan_right)
        # if command[0] == 'e':
        #     print('Reached end.')
        #     break
        # elif command[-1] == 'f':
        #     reverse_mode = False
        # elif command[-1] == 'b':
        #     reverse_mode = True
        # else:
        #     raise Exception('Invalid command: ' + command)
        # rotation = int(command[:len(command) - 1])
        if command[0] == 'e':
            print('Reached end.')
            break
        rotation = int(command)
        angle += rotation # Apply rotation instantaneously.
    
    if pixel_dist[1] <= 10:
        total_offset = 0
        while pixel_dist[1] - prev_dist_L <= 0:
            prev_dist_L = pixel_dist[1]
            angle += 1
            total_offset += 1
            angle %= 360
            for i in range(3):
                sensor_angle = (angle + add_angle[i]) % 360
                unit_v = (math.sin(math.radians(sensor_angle)), -math.cos(math.radians(sensor_angle)))
                current = [ppos[0], ppos[1]]
                for j in range(round(sensor_range[i]/PIXEL_RES)):
                    p = [round(current[k]) for k in range(2)]
                    if pixels[p[0]][p[1]] == PixelType.WALL:
                        pixel_dist[i] = j
                        break
                    current = [current[k] + unit_v[k] for k in range(2)]
        # angle -= total_offset * 0.4
        angle %= 360
        prev_dist_L = -10000
        prev_dist_R = 10000
    elif pixel_dist[2] <= 10:
        total_offset = 0
        while pixel_dist[2] - prev_dist_R <= 0:
            prev_dist_R = pixel_dist[2]
            angle -= 1
            total_offset -= 1
            angle %= 360
            for i in range(3):
                sensor_angle = (angle + add_angle[i]) % 360
                unit_v = (math.sin(math.radians(sensor_angle)), -math.cos(math.radians(sensor_angle)))
                current = [ppos[0], ppos[1]]
                for j in range(round(sensor_range[i]/PIXEL_RES)):
                    p = [round(current[k]) for k in range(2)]
                    if pixels[p[0]][p[1]] == PixelType.WALL:
                        pixel_dist[i] = j
                        break
                    current = [current[k] + unit_v[k] for k in range(2)]
        # angle += total_offset * 0.4
        angle %= 360
        prev_dist_R = -10000
        prev_dist_L = 10000
    
    # # Apply course correction.
    # if pixel_dist[1] <= 30 and pixel_dist[2] <= 30: # If we are in a corridor.
    #     # Model the light sensor readings.
    #     closest_left = (math.inf, math.inf)
    #     closest_right = (math.inf, math.inf)
    #     # if reverse_mode:
    #     #     r_angle = (angle + 180) % 360
    #     # else:
    #     #     r_angle = angle
    #     for i in range(max(0, math.floor(ppos[0] - 0.5/PIXEL_RES)), min(math.ceil(ppos[0] + 0.5/PIXEL_RES) + 1, X_PIXELS)):
    #         for j in range(max(0, math.floor(ppos[1] - 0.5/PIXEL_RES)), min(math.ceil(ppos[1] + 0.5/PIXEL_RES) + 1, Y_PIXELS)):
    #             if pixels[i][j] == PixelType.WALL and math.dist((i, j), ppos) <= SIDE_RANGE/PIXEL_RES:
    #                 arg = (90 - (math.degrees(math.atan2(ppos[1] - j, i - ppos[0])) % 360)) % 360
    #                 adj_arg = (arg - angle) % 360
    #                 if adj_arg >= 90 - 10/2 and adj_arg <= 90 + 10/2 and math.dist((i, j), ppos) < math.dist(closest_right, ppos):
    #                     closest_right = (i, j)
    #                 elif adj_arg >= 270 - 10/2 and adj_arg <= 270 + 10/2 and math.dist((i, j), ppos) < math.dist(closest_left, ppos):
    #                     closest_left = (i, j)
    #     left_dist = math.dist(closest_left, ppos)
    #     right_dist = math.dist(closest_right, ppos)
    #     # Controller logic.
    #     balance = left_dist - right_dist
    #     width = left_dist + right_dist
    #     if cc_active:
    #         width_diff = width - cc_prev_width
    #         left_diff = left_dist - cc_prev_left_dist
    #         right_diff = right_dist - cc_prev_right_dist
    #         if abs(width - cc_prev_width) < 1.5 and abs(abs(left_diff) - abs(right_diff)) < 1.5:
    #             cc_sum += balance
    #             if cc_prev == None:
    #                 diff = 0
    #             else:
    #                 diff = balance - cc_prev
    #             delta = P_CC*balance + I_CC*cc_sum + D_CC*diff
    #             if cc_prev_vertex != None and math.dist(cc_prev_vertex, pos) <= UNBOUND_DIST:
    #                 angle -= delta
    #             else:
    #                 cc_offset -= delta
    #                 if abs(cc_offset) > MAX_OFFSET and abs(cc_offset) < MAX_OFFSET + delta:
    #                     if delta > 0:
    #                         delta -= abs(cc_offset) - MAX_OFFSET
    #                     else:
    #                         delta += abs(cc_offset) - MAX_OFFSET
    #                     angle -= delta
    #                 elif abs(cc_offset) <= MAX_OFFSET:
    #                     angle -= delta
    #                 else:
    #                     cc_offset += delta
    #     # if abs(diff) < CUTOFF_CC:
    #     #     angle -= P_CC*balance + I_CC*cc_sum + D_CC*diff
    #     # elif balance > 0:
    #     #     angle -= 0.5
    #     # else:
    #     #     angle += 0.5
    #     cc_active = True
    #     cc_prev = balance
    #     cc_prev_left_dist = left_dist
    #     cc_prev_right_dist = right_dist
    #     cc_prev_width = width
    # else:
    #     cc_active = False
    #     cc_sum = 0
    #     cc_offset = 0
    
    # Update robot position.
    # if reverse_mode:
    #     direction = (angle + 180) % 360
    # else:
    #     direction = angle
    direction = angle
    direction_2 = math.radians((90 - direction) % 360)
    pos[0] += SPEED * math.cos(direction_2)
    pos[1] -= SPEED * math.sin(direction_2)
    ppos = to_pixels(pos)
    robot_path.append((ppos[0], ppos[1]))

    # Check to display map.
    # if iterations % 1000 == 0:
    #     manager.bitmap.render_pixels_debug(robot_path, walls, WALL_WIDTH)
    
    pygame.surfarray.blit_array(screen, manager.bitmap.get_bitmap_debug((ppos[0], ppos[1]), robot_path, manager.get_path()))
    pygame.display.update()

    events = pygame.event.get()
    for event in events:
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                pygame.quit()

pygame.quit()
manager.bitmap.render_pixels_debug(walls, WALL_WIDTH)