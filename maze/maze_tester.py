import pygame
import time
from enum import Enum
from maze_manager import *

# Sensor parameters.
FRONT_RANGE = 0.2 # The range of the front sensors.
SIDE_RANGE = 0.4 # The range of the side sensors.
SCAN_RANGE = 0.4 # The range of the side sensors when scanning.
FRONT_ANGLE = 45 # The angle of acceptance of the front sensors.
SIDE_ANGLE = 20 # The angle of acceptance of the side sensors.
SCAN_RES = 64 # The number of vectors we consider in a junction scan.

# Course-correction controller parameters.
P_CC = 0.3 # Course correction proportional gain.
I_CC = 0.0001 # Course correction integral gain.
D_CC = 0.3 # Course correction derivative gain.
MAX_OFFSET = 10 # The maximum amount of course-correction that can be applied, in degrees.
UNBOUND_DIST = 0.3 # The distance for which unbounded CC is applied.

# Course-correction controller variables.
cc_active = False
cc_sum = 0
cc_prev = 0
cc_prev_left_dist = 0
cc_prev_right_dist = 0
cc_prev_width = 0
cc_offset = 0
cc_prev_vertex = None

# Other parameters.
PIXEL_RES = 0.01 # Metres/pixel
WALL_WIDTH = 0.05
SPEED = 0.01 # The distance travelled by the robot between each update.

# Notes:
# Angle of acceptance of sensors is not well modelled since the robot is simulated as a point, whereas in reality
# the sensors will be some distance away from the center of the robot.

X_LIM = 3
Y_LIM = 2
X_PIXELS = int(X_LIM / PIXEL_RES) + 1 # The number of pixels in the x direction.
Y_PIXELS = int(Y_LIM / PIXEL_RES) + 1 # The number of pixels in the y direction.

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

pygame.init()
screen = pygame.display.set_mode((X_PIXELS * 3, Y_PIXELS * 3))

# Generate a bitmap representing the maze.
walls = []

# Add boundary walls.
walls.append(((0, 0), (X_LIM, 0)))
walls.append(((X_LIM, 0), (X_LIM, Y_LIM)))
walls.append(((0, Y_LIM), (X_LIM, Y_LIM)))
walls.append(((0, 0), (0, Y_LIM)))

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

config6()

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

manager = MazeManager()
manager.bitmap.update_walls(walls, WALL_WIDTH)

# Set up arena.
start = (0.25, 0.25)
manager.set_start(start)
end = (2.75, 1.75)
manager.set_end(end)
manager.set_beacons([(0, 0), (3, 0), (0, 3)])

# Initialise robot.
pos = [start[0], start[1]] # Assume robot is initially at start position.
ppos = (to_pixels(start)[0], to_pixels(start)[1]) # Pixel position.
angle = 180 # Assume robot is initially pointing south.
light = [0, 0, 0, 0] # Array of light readings (forwards, right, back, left).
target_angle = 180 # The target angle from the last rotation command.
# reverse_mode = False # Whether or not the robot is going in reverse.
robot_path = []

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
    
    front_light_raw = 0
    left_light_raw = 0
    right_light_raw = 0
    for i in range(max(0, math.floor(ppos[0] - max(FRONT_RANGE, SIDE_RANGE)/PIXEL_RES)), min(math.ceil(ppos[0] + max(FRONT_RANGE, SIDE_RANGE)/PIXEL_RES) + 1, X_PIXELS)):
        for j in range(max(0, math.floor(ppos[1] - max(FRONT_RANGE, SIDE_RANGE)/PIXEL_RES)), min(math.ceil(ppos[1] + max(FRONT_RANGE, SIDE_RANGE)/PIXEL_RES) + 1, Y_PIXELS)):
            if pixels[i][j] == PixelType.WALL:
                dist = math.dist((i, j), ppos)
                arg = (90 - (math.degrees(math.atan2(ppos[1] - j, i - ppos[0])) % 360)) % 360
                adj_arg = (arg - angle) % 360
                if dist <= FRONT_RANGE/PIXEL_RES and (adj_arg <= FRONT_ANGLE/2 or adj_arg >= 360 - FRONT_ANGLE/2):
                    front_light_raw = 1000
                elif dist <= SIDE_RANGE/PIXEL_RES and adj_arg >= 90 - SIDE_ANGLE/2 and adj_arg <= 90 + SIDE_ANGLE/2:
                    left_light_raw = 1000
                # elif dist <= FRONT_RANGE/PIXEL_RES and adj_arg >= 180 - FRONT_ANGLE/2 and adj_arg <= 180 + FRONT_ANGLE/2:
                #     light[2] = True
                elif dist <= SIDE_RANGE/PIXEL_RES and adj_arg >= 270 - SIDE_ANGLE/2 and adj_arg <= 270 + SIDE_ANGLE/2:
                    right_light_raw = 1000
    
    if iterations == 1:
        command = 'j'
    else:
        command = manager.default_navigate((pos[0] + iterations*0.001, pos[1] + iterations*0.001), angle + iterations*0.01, front_light_raw, left_light_raw, right_light_raw)
        # Simulate drift (linear w.r.t. time).

    if command == 'j':

        cc_prev_vertex = pos

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
        beacon_dist = []
        for i in range(3):
            beacon_dist.append(math.dist(pos, manager.beacon_tri.beacon_pos[i]))
        beacon_angles = []
        pairs = ((0, 1), (0, 2), (1, 2))
        for pair in pairs:
            a = math.dist(manager.beacon_tri.beacon_pos[pair[0]], manager.beacon_tri.beacon_pos[pair[1]])
            b = beacon_dist[pair[0]]
            c = beacon_dist[pair[1]]
            beacon_angles.append(math.degrees(math.acos((b**2 + c**2 - a**2)/(2*b*c))))
        
        # Simulate turning the robot to face the first beacon.
        diff = (manager.beacon_tri.beacon_pos[0][0] - pos[0], pos[1] - manager.beacon_tri.beacon_pos[0][1])
        theta = math.degrees(math.atan2(diff[1], diff[0]))
        if theta >= -90:
            angle = 90 - theta
        else:
            angle = -theta - 270
        
        light_scan = []
        for i in range(SCAN_RES):
            light_scan.append(0)
        for i in range(max(0, math.floor(ppos[0] - SCAN_RANGE/PIXEL_RES)), min(math.ceil(ppos[0] + SCAN_RANGE/PIXEL_RES) + 1, X_PIXELS)):
            for j in range(max(0, math.floor(ppos[1] - SCAN_RANGE/PIXEL_RES)), min(math.ceil(ppos[1] + SCAN_RANGE/PIXEL_RES) + 1, Y_PIXELS)):
                if pixels[i][j] == PixelType.WALL and math.dist((i, j), ppos) <= SCAN_RANGE/PIXEL_RES:
                    arg = (90 - (math.degrees(math.atan2(ppos[1] - j, i - ppos[0])) % 360)) % 360
                    adj_arg = (arg - angle) % 360
                    light_scan[int(adj_arg/360 * SCAN_RES)] = 1000

        command = manager.junction_navigate(beacon_angles[0], beacon_angles[1], beacon_angles[2], light_scan)
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
    
    # Apply course correction.
    if left_light_raw and right_light_raw: # If we are in a corridor.
        # Model the light sensor readings.
        closest_left = (math.inf, math.inf)
        closest_right = (math.inf, math.inf)
        # if reverse_mode:
        #     r_angle = (angle + 180) % 360
        # else:
        #     r_angle = angle
        for i in range(max(0, math.floor(ppos[0] - 0.5/PIXEL_RES)), min(math.ceil(ppos[0] + 0.5/PIXEL_RES) + 1, X_PIXELS)):
            for j in range(max(0, math.floor(ppos[1] - 0.5/PIXEL_RES)), min(math.ceil(ppos[1] + 0.5/PIXEL_RES) + 1, Y_PIXELS)):
                if pixels[i][j] == PixelType.WALL and math.dist((i, j), ppos) <= SIDE_RANGE/PIXEL_RES:
                    arg = (90 - (math.degrees(math.atan2(ppos[1] - j, i - ppos[0])) % 360)) % 360
                    adj_arg = (arg - angle) % 360
                    if adj_arg >= 90 - SIDE_ANGLE/2 and adj_arg <= 90 + SIDE_ANGLE/2 and math.dist((i, j), ppos) < math.dist(closest_right, ppos):
                        closest_right = (i, j)
                    elif adj_arg >= 270 - SIDE_ANGLE/2 and adj_arg <= 270 + SIDE_ANGLE/2 and math.dist((i, j), ppos) < math.dist(closest_left, ppos):
                        closest_left = (i, j)
        left_dist = math.dist(closest_left, ppos)
        right_dist = math.dist(closest_right, ppos)
        # Controller logic.
        balance = left_dist - right_dist
        width = left_dist + right_dist
        if cc_active:
            width_diff = width - cc_prev_width
            left_diff = left_dist - cc_prev_left_dist
            right_diff = right_dist - cc_prev_right_dist
            if abs(width - cc_prev_width) < 1.5 and abs(abs(left_diff) - abs(right_diff)) < 1.5:
                cc_sum += balance
                if cc_prev == None:
                    diff = 0
                else:
                    diff = balance - cc_prev
                delta = P_CC*balance + I_CC*cc_sum + D_CC*diff
                if cc_prev_vertex != None and math.dist(cc_prev_vertex, pos) <= UNBOUND_DIST:
                    angle -= delta
                else:
                    cc_offset -= delta
                    if abs(cc_offset) > MAX_OFFSET and abs(cc_offset) < MAX_OFFSET + delta:
                        if delta > 0:
                            delta -= abs(cc_offset) - MAX_OFFSET
                        else:
                            delta += abs(cc_offset) - MAX_OFFSET
                        angle -= delta
                    elif abs(cc_offset) <= MAX_OFFSET:
                        angle -= delta
                    else:
                        cc_offset += delta
        # if abs(diff) < CUTOFF_CC:
        #     angle -= P_CC*balance + I_CC*cc_sum + D_CC*diff
        # elif balance > 0:
        #     angle -= 0.5
        # else:
        #     angle += 0.5
        cc_active = True
        cc_prev = balance
        cc_prev_left_dist = left_dist
        cc_prev_right_dist = right_dist
        cc_prev_width = width
    else:
        cc_active = False
        cc_sum = 0
        cc_offset = 0
    
    # Update robot position.
    # if reverse_mode:
    #     direction = (angle + 180) % 360
    # else:
    #     direction = angle
    direction = angle
    direction_2 = math.radians((90 - direction) % 360)
    pos[0] += SPEED * math.cos(direction_2)
    pos[1] -= SPEED * math.sin(direction_2)
    ppos = (to_pixels(pos)[0], to_pixels(pos)[1])
    robot_path.append(ppos)

    # Check to display map.
    # if iterations % 1000 == 0:
    #     manager.bitmap.render_pixels_debug(robot_path, walls, WALL_WIDTH)
    
    pygame.surfarray.blit_array(screen, manager.bitmap.get_bitmap_debug(ppos, robot_path, manager.get_path()))
    pygame.display.update()
    # time.sleep(0.1)

pygame.quit()
manager.bitmap.render_pixels_debug(robot_path)