from enum import Enum
from maze_manager import *

X_LIM = 3
Y_LIM = 2
PIXEL_RES = 0.01 # Metres/pixel
X_PIXELS = int(X_LIM / PIXEL_RES) + 1 # The number of pixels in the x direction.
Y_PIXELS = int(Y_LIM / PIXEL_RES) + 1 # The number of pixels in the y direction.

WALL_WIDTH = 0.01
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

# Add boundary walls.
walls.append(((0, 0), (X_LIM, 0)))
walls.append(((X_LIM, 0), (X_LIM, Y_LIM)))
walls.append(((0, Y_LIM), (X_LIM, Y_LIM)))
walls.append(((0, 0), (0, Y_LIM)))

# Add all other walls.
walls.append(((0.5, 0), (0.5, 1.5)))
walls.append(((1, 2), (1.5, 1.5)))
walls.append(((2, 1), (2.5, 0.5)))
walls.append(((1, 0.5), (2, 0.5)))
walls.append(((1, 1), (1.5, 1)))
walls.append(((2, 0.5), (2, 1)))
walls.append(((1.5, 1), (1.5, 1.5)))
walls.append(((3, 1), (2.5, 1.5)))
walls.append(((2, 1.5), (2.5, 1.5)))

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
        for j in range(max(0, math.floor(c[0] - WALL_WIDTH)), min(math.ceil(c[0] + WALL_WIDTH) + 1, X_PIXELS)):
            for k in range(max(0, math.floor(c[1] - WALL_WIDTH)), min(math.ceil(c[1] + WALL_WIDTH) + 1, Y_PIXELS)):
                pixel = pixels[j][k]
                if pixel in (PixelType.START, PixelType.END, PixelType.WALL): # Skip over start, end, previously placed walls.
                    pass
                elif math.dist(c, (j, k)) <= WALL_WIDTH:
                    pixels[j][k] = PixelType.WALL # Mark this cell as inaccessible.
        current = [current[i] + unit_diff[i] for i in range(2)]

render_pixels()

manager = MazeManager()

# Set up arena.
start = (0.2, 0.2)
manager.set_start(start)
end = (2.8, 1.8)
manager.set_end(end)

# Initialise robot.
pos = [start[0], start[1]] # Assume robot is initially at start position.
ppos = [to_pixels(start)[0], to_pixels(start)[1]] # Pixel position.
angle = 180 # Assume robot is initially pointing south.
light = [0, 0, 0, 0] # Array of light readings (forwards, right, back, left).
target_angle = 180 # The target angle from the last rotation command.
reverse_mode = False # Whether or not the robot is going in reverse.

iterations = 0
while True:
    # Check for timeout.
    iterations += 1
    if iterations > 1000000:
        print('!! Timeout !!')
        break

    raw_readings = [0, 0, 0, 0]
    # Update sensor readings.
    # for i in range(max(0, math.floor(ppos[0] - PIXEL_RANGE)), min(math.ceil(ppos[0] + PIXEL_RANGE) + 1, X_PIXELS)):
    #     for j in range(max(0, math.floor(ppos[1] - PIXEL_RANGE)), min(math.ceil(ppos[1] + PIXEL_RANGE) + 1, Y_PIXELS)):
    #         if pixels[i][j] == PixelType.WALL:
    #             arg = (90 - (math.degrees(math.atan2(ppos[1] - j, i - ppos[0])) % 360)) % 360
    #             adj_arg = (arg - angle) % 360
    #             if adj_arg < 30 or adj_arg >= 330:
    #                 raw_readings[0] += 1
    #             elif adj_arg >= 60 and adj_arg < 120:
    #                 raw_readings[1] += 1
    #             elif adj_arg >= 150 and adj_arg < 210:
    #                 raw_readings[2] += 1
    #             elif adj_arg >= 240 and adj_arg < 300:
    #                 raw_readings[3] += 1
    loci = []
    for i in range(4):
        loci.append((0.25 * math.sin(i/4*math.pi + math.radians(angle)), -0.25 * math.cos(i/4*math.pi + math.radians(angle))))
    loci_p = [to_pixels(loci[i]) for i in range(4)]
    # loci = ((0, -0.25), (0.25, 0), (0, 0.25), (-0.25, 0))
    for i in range(4):
        p = [ppos[j] + loci_p[i][j] for j in range(2)]
        for j in range(max(0, math.floor(ppos[0] - 0.4/PIXEL_RES)), min(math.ceil(ppos[0] + 0.4/PIXEL_RES) + 1, X_PIXELS)):
            for k in range(max(0, math.floor(ppos[1] - 0.4/PIXEL_RES)), min(math.ceil(ppos[1] + 0.4/PIXEL_RES) + 1, Y_PIXELS)):
                if pixels[j][k] == PixelType.WALL and math.dist([j, k], p) <= 0.15/PIXEL_RES:
                    raw_readings[i] += 1
    for i in range(4):
        if raw_readings[i] > 0:
            light[i] = True
        else:
            light[i] = False
    
    if iterations == 1:
        command = 'j'
    else:
        command = manager.default_navigate(pos, angle, light)

    if command == 'j':
        # Perform light sensor scan with limited range - take 8 orientations here.
        light_scan_raw = []
        for i in range(8):
            light_scan_raw.append(0)
        # args = [math.pi/2, math.pi/4, 0, -math.pi/4, -math.pi/2, -3*math.pi/4, math.pi, 3*math.pi/4, math.pi/2]
        # for i in range(max(0, math.floor(ppos[0] - PIXEL_RANGE)), min(math.ceil(ppos[0] + PIXEL_RANGE) + 1, X_PIXELS)):
        #     for j in range(max(0, math.floor(ppos[1] - PIXEL_RANGE)), min(math.ceil(ppos[1] + PIXEL_RANGE) + 1, Y_PIXELS)):
        #         if pixels[i][j] == PixelType.WALL:
        #             arg = math.atan2(ppos[1] - j, i - ppos[0])
        #             if abs(arg) <= math.pi/8:
        #                 light_scan[2] = True
        #             elif abs(arg) <= 3*math.pi/8:
        #                 if arg > 0:
        #                     light_scan[1] = True
        #                 else:
        #                     light_scan[3] = True
        #             elif abs(arg) <= 5*math.pi/8:
        #                 if arg > 0:
        #                     light_scan[0] = True
        #                 else:
        #                     light_scan[4] = True
        #             elif abs(arg) <= 7*math.pi/8:
        #                 if arg > 0:
        #                     light_scan[7] = True
        #                 else:
        #                     light_scan[5] = True
        #             else:
        #                 light_scan[6] = True
        loci = []
        for i in range(8):
            loci.append((0.25 * math.sin(i/4*math.pi), -0.25 * math.cos(i/4*math.pi)))
        loci_p = [to_pixels(loci[i]) for i in range(8)]
        for i in range(8):
            p = [ppos[j] + loci_p[i][j] for j in range(2)]
            for j in range(max(0, math.floor(ppos[0] - 0.4/PIXEL_RES)), min(math.ceil(ppos[0] + 0.4/PIXEL_RES) + 1, X_PIXELS)):
                for k in range(max(0, math.floor(ppos[1] - 0.4/PIXEL_RES)), min(math.ceil(ppos[1] + 0.4/PIXEL_RES) + 1, Y_PIXELS)):
                    if pixels[j][k] == PixelType.WALL and math.dist([j, k], p) <= 0.15/PIXEL_RES:
                        light_scan_raw[i] += 1
        light_scan = []
        for i in range(8):
            if light_scan_raw[i] > 0:
                light_scan.append(True)
            else:
                light_scan.append(False)
        link_angles = []
        for i in range(8):
            if not light_scan[i]:
                link_angles.append(math.degrees(math.pi/4 * i))
        print(link_angles)
        command = manager.junction_navigate(pos, angle, link_angles)
        if command[0] == 'e':
            print('Reached end.')
            break
        elif command[-1] == 'f':
            reverse_mode = False
        elif command[-1] == 'b':
            reverse_mode = True
        else:
            raise Exception('Invalid command: ' + command)
        rotation = int(command[:len(command) - 1])
        angle += rotation # Apply rotation instantaneously.
        print(angle)
    
    # Update robot position.
    if reverse_mode:
        direction = (angle + 180) % 360
    else:
        direction = angle
    direction_2 = math.radians((90 - direction) % 360)
    pos[0] += SPEED * math.cos(direction_2)
    pos[1] -= SPEED * math.sin(direction_2)
    ppos = [to_pixels(pos)[0], to_pixels(pos)[1]]
    print(pos)

manager.bitmap.render_pixels()