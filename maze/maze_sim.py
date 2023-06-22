from enum import Enum
from maze_manager import *
import pygame

class PixelType(Enum):
    EMPTY = 0
    START = 1
    END = 2
    WALL = 3

class MazeSim:

    # Mutable parameters.
    FRONT_RANGE = 1 # The range of the front sensors.
    SIDE_RANGE = 1 # The range of the side sensors.
    SCAN_RANGE = 1 # The range of the side sensors when scanning.
    SCAN_RES = 360
    LINK_DIST = 0.15
    FORCE_DIST = 0.05
    MIN_DIST = 0.25

    # Other parameters.
    PIXEL_RES = 0.01 # Metres/pixel
    SPEED = 0.01 # The distance travelled by the robot between each update.

    # Convert a point in metres (standard units) to pixels.
    def to_pixels(self, point):
        return (round(point[0] / self.PIXEL_RES), round(point[1] / self.PIXEL_RES))

    # Generate a bitmap representing the maze.
    walls = []

    def config1(self):
        self.walls.append(((0.5, 0), (0.5, 1.5)))
        self.walls.append(((0.5, 1.5), (1, 1.5)))
        self.walls.append(((1, 0.5), (1, 1)))
        self.walls.append(((1, 0.5), (2.5, 0.5)))
        self.walls.append(((2.5, 0.5), (2.5, 1)))
        self.walls.append(((2.5, 1.5), (2.5, 2)))
        self.walls.append(((1.5, 1.5), (2.5, 1.5)))
        self.walls.append(((1.5, 1), (2, 1)))
        self.walls.append(((2, 1), (2, 1.5)))

    def config2(self):
        self.walls.append(((0.5, 0.5), (0.5, 1)))
        self.walls.append(((0.5, 1), (1, 1)))
        self.walls.append(((0.5, 1.5), (1, 1.5)))
        self.walls.append(((1, 0.5), (1.5, 0.5)))
        self.walls.append(((1.5, 0.5), (1.5, 1)))
        self.walls.append(((1.5, 1.5), (2, 1.5)))
        self.walls.append(((2, 1), (2, 1.5)))
        self.walls.append(((2, 0.5), (2.5, 0.5)))
        self.walls.append(((2.5, 0.5), (2.5, 1)))
        self.walls.append(((2.5, 1.5), (3, 1.5)))

    def config3(self):
        self.walls.append(((0.5, 0.5), (1, 0.5)))
        self.walls.append(((0, 1), (1, 1)))
        self.walls.append(((1, 1), (1, 1.5)))
        self.walls.append(((0.5, 1.5), (1, 1.5)))
        self.walls.append(((1.5, 0.5), (2, 0.5)))
        self.walls.append(((1.5, 0.5), (1.5, 1)))
        self.walls.append(((2.5, 0.5), (2.5, 1.5)))
        self.walls.append(((2, 1), (2.5, 1.5)))
        self.walls.append(((2, 1), (2.5, 1)))
        self.walls.append(((1.5, 1), (2.5, 2)))
        self.walls.append(((1.5, 1.5), (1.5, 2)))
        self.walls.append(((1.5, 1.5), (2, 2)))
    
    def __init__(self, config_num):
        
        if config_num == 1:
            self.config1()
        elif config_num == 2:
            self.config2()
        elif config_num == 3:
            self.config3()
        else:
            raise ValueError('Incorrect config number.')
        self.WALL_WIDTH = 0.08
        self.X_LIM = 3
        self.Y_LIM = 2
        self.pp = 3
        self.start = (0.25, 0.25)
        self.end = (2.75, 1.75)

        self.X_PIXELS = int(self.X_LIM / self.PIXEL_RES) + 1 # The number of pixels in the x direction.
        self.Y_PIXELS = int(self.Y_LIM / self.PIXEL_RES) + 1 # The number of pixels in the y direction.

        # Add boundary walls.
        self.walls.append(((0, 0), (self.X_LIM, 0)))
        self.walls.append(((self.X_LIM, 0), (self.X_LIM, self.Y_LIM)))
        self.walls.append(((0, self.Y_LIM), (self.X_LIM, self.Y_LIM)))
        self.walls.append(((0, 0), (0, self.Y_LIM)))

        self.pixels = []
        for i in range(self.X_PIXELS):
            pixel_row = []
            for j in range(self.Y_PIXELS):
                pixel_row.append(PixelType.EMPTY)
            self.pixels.append(pixel_row)
        for wall in self.walls:
            p1 = self.to_pixels(wall[0])
            p2 = self.to_pixels(wall[1])
            diff = [p2[i] - p1[i] for i in range(2)] # Find the difference vector.
            p_dist = math.ceil(math.dist(p1, p2)) # Choose the upper bound on the length of the wall.
            unit_diff = [diff[i] / p_dist for i in range(2)] # Normalise the difference vector to get the direction.
            current = [p1[0], p1[1]]
            for i in range(p_dist + 1):
                c = [round(x) for x in current]
                for j in range(max(0, math.floor(c[0] - self.WALL_WIDTH/self.PIXEL_RES)), min(math.ceil(c[0] + self.WALL_WIDTH/self.PIXEL_RES) + 1, self.X_PIXELS)):
                    for k in range(max(0, math.floor(c[1] - self.WALL_WIDTH/self.PIXEL_RES)), min(math.ceil(c[1] + self.WALL_WIDTH/self.PIXEL_RES) + 1, self.Y_PIXELS)):
                        pixel = self.pixels[j][k]
                        if pixel in (PixelType.START, PixelType.END, PixelType.WALL): # Skip over start, end, previously placed walls.
                            pass
                        elif math.dist(c, (j, k)) <= self.WALL_WIDTH/self.PIXEL_RES:
                            self.pixels[j][k] = PixelType.WALL # Mark this cell as inaccessible.
                current = [current[i] + unit_diff[i] for i in range(2)]


        # render_pixels()

        self.manager = MazeManager(self.X_LIM, self.Y_LIM, pp=self.pp, LINK_DIST=self.LINK_DIST, FORCE_DIST=self.FORCE_DIST, MIN_DIST=self.MIN_DIST)
        self.manager.bitmap.update_walls(self.walls, self.WALL_WIDTH)

        # Set up arena.
        self.manager.set_start(self.start)
        self.manager.set_end(self.end)
        self.manager.set_beacons([(0, 0), (3, 0), (0, 3)])

        # Initialise robot.
        self.pos = [self.start[0], self.start[1]] # Assume robot is initially at start position.
        self.ppos = self.to_pixels(self.start) # Pixel position.
        self.angle = 180 # Assume robot is initially pointing south.
        self.target_angle = 180 # The target angle from the last rotation command.
        # reverse_mode = False # Whether or not the robot is going in reverse.
        self.robot_path = []

        self.prev_dist_L = 10000
        self.prev_dist_R = 10000

        self.iterations = 0

    def update(self):

        self.iterations += 1

        pixel_dist = [1000, 1000, 1000]
        sensor_range = (self.FRONT_RANGE, self.SIDE_RANGE, self.SIDE_RANGE)
        add_angle = (0, -90, 90)
        for i in range(3):
            sensor_angle = (self.angle + add_angle[i]) % 360
            unit_v = (math.sin(math.radians(sensor_angle)), -math.cos(math.radians(sensor_angle)))
            current = [self.ppos[0], self.ppos[1]]
            for j in range(round(sensor_range[i]/self.PIXEL_RES)):
                p = [round(current[k]) for k in range(2)]
                if self.pixels[p[0]][p[1]] == PixelType.WALL:
                    pixel_dist[i] = j
                    break
                current = [current[k] + unit_v[k] for k in range(2)]
        # Each pixel is 1cm anyway.
        
        if self.iterations == 1:
            command = 'j'
        else:
            command = self.manager.default_navigate((self.pos[0] + self.iterations*0.001, self.pos[1] + self.iterations*0.001), self.angle + self.iterations*0.01, pixel_dist[0], pixel_dist[1], pixel_dist[2])
            # Simulate drift (linear w.r.t. time).

        if command == 'j':

            self.prev_dist_L = 10000
            self.prev_dist_R = 10000

            beacon_angles = []
            for i in range(3):
                diff = (self.manager.beacon_tri.beacon_pos[i][0] - self.pos[0], self.pos[1] - self.manager.beacon_tri.beacon_pos[i][1])
                arg = (90 - (math.degrees(math.atan2(diff[1], diff[0])) % 360)) % 360
                beacon_angles.append(arg)
            
            # Simulate turning the robot to face the first beacon.
            diff = (self.manager.beacon_tri.beacon_pos[0][0] - self.pos[0], self.pos[1] - self.manager.beacon_tri.beacon_pos[0][1])
            theta = math.degrees(math.atan2(diff[1], diff[0]))
            if theta >= -90:
                self.angle = 90 - theta
            else:
                self.angle = -theta - 270

            scan_angles = []
            scan_left = []
            scan_right = []
            for i in range(self.SCAN_RES):
                scan_angles.append(int(i / self.SCAN_RES * 360))
                scan_left.append(1000)
                scan_right.append(1000)
            for i in range(self.SCAN_RES):
                left_angle = (self.angle + int(i / self.SCAN_RES * 360) - 90) % 360
                unit_v = (math.sin(math.radians(left_angle)), -math.cos(math.radians(left_angle)))
                current = [self.ppos[0], self.ppos[1]]
                for j in range(round(self.SCAN_RANGE/self.PIXEL_RES)):
                    p = [round(current[k]) for k in range(2)]
                    if self.pixels[p[0]][p[1]] == PixelType.WALL:
                        scan_left[i] = j
                        break
                    current = [current[k] + unit_v[k] for k in range(2)]
            # Each pixel is 1cm anyway.

            # command = manager.junction_navigate(beacon_angles[0], beacon_angles[1], beacon_angles[2], light_scan)
            command = self.manager.junction_navigate(beacon_angles[0], beacon_angles[1], beacon_angles[2], scan_angles, scan_left, scan_right)
            if command[0] == 'e':
                print('Reached end.')
                return False
            rotation = int(command)
            self.angle += rotation # Apply rotation instantaneously.
        
        if pixel_dist[1] <= 10:
            total_offset = 0
            while pixel_dist[1] - self.prev_dist_L <= 0:
                self.prev_dist_L = pixel_dist[1]
                self.angle += 1
                total_offset += 1
                self.angle %= 360
                for i in range(3):
                    sensor_angle = (self.angle + add_angle[i]) % 360
                    unit_v = (math.sin(math.radians(sensor_angle)), -math.cos(math.radians(sensor_angle)))
                    current = [self.ppos[0], self.ppos[1]]
                    for j in range(round(sensor_range[i]/self.PIXEL_RES)):
                        p = [round(current[k]) for k in range(2)]
                        if self.pixels[p[0]][p[1]] == PixelType.WALL:
                            pixel_dist[i] = j
                            break
                        current = [current[k] + unit_v[k] for k in range(2)]
            # angle -= total_offset * 0.4
            self.angle %= 360
            self.prev_dist_L = -10000
            self.prev_dist_R = 10000
        elif pixel_dist[2] <= 10:
            total_offset = 0
            while pixel_dist[2] - self.prev_dist_R <= 0:
                self.prev_dist_R = pixel_dist[2]
                self.angle -= 1
                total_offset -= 1
                self.angle %= 360
                for i in range(3):
                    sensor_angle = (self.angle + add_angle[i]) % 360
                    unit_v = (math.sin(math.radians(sensor_angle)), -math.cos(math.radians(sensor_angle)))
                    current = [self.ppos[0], self.ppos[1]]
                    for j in range(round(sensor_range[i]/self.PIXEL_RES)):
                        p = [round(current[k]) for k in range(2)]
                        if self.pixels[p[0]][p[1]] == PixelType.WALL:
                            pixel_dist[i] = j
                            break
                        current = [current[k] + unit_v[k] for k in range(2)]
            # angle += total_offset * 0.4
            self.angle %= 360
            self.prev_dist_R = -10000
            self.prev_dist_L = 10000
        
        direction = self.angle
        direction_2 = math.radians((90 - direction) % 360)
        self.pos[0] += self.SPEED * math.cos(direction_2)
        self.pos[1] -= self.SPEED * math.sin(direction_2)
        self.ppos = self.to_pixels(self.pos)
        self.robot_path.append((self.ppos[0], self.ppos[1]))

        return True

if __name__ == '__main__':
    sim = MazeSim(3)
    pygame.init()
    screen = pygame.display.set_mode((sim.X_PIXELS * sim.pp, sim.Y_PIXELS * sim.pp))
    while sim.update():
        edges = sim.manager.get_edges()

        pygame.surfarray.blit_array(screen, sim.manager.bitmap.get_bitmap_debug((sim.ppos[0], sim.ppos[1]), sim.robot_path, sim.manager.get_path()))
        pygame.display.update()
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
    pygame.quit()
    sim.manager.bitmap.render_pixels_debug(sim.walls, sim.WALL_WIDTH)