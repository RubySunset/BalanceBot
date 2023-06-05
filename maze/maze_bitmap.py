import math
import time
from enum import Enum
from PIL import Image

class PixelType(Enum):
    EMPTY = 0
    START = 1
    END = 2
    VERTEX = 3
    PATH = 4
    CLEARED = 5

# Main class. Instantiate and use in other modules.
class MazeBitmap:

    PIXEL_RES = 0.01 # How many metres a single pixel is.
    WIDTH = 0.3 # The width of passages in standard units.

    def __init__(self, X_LIM, Y_LIM):
        self.X_LIM = X_LIM
        self.Y_LIM = Y_LIM
        self.X_PIXELS = int(self.X_LIM / self.PIXEL_RES) + 1 # The number of pixels in the x direction.
        self.Y_PIXELS = int(self.Y_LIM / self.PIXEL_RES) + 1 # The number of pixels in the y direction.
        self.P_WIDTH = self.WIDTH / self.PIXEL_RES # The width of passages in pixels.
        self.pixels = [] # The pixel array representing the state of the maze.
        for i in range(self.X_PIXELS):
            pixel_row = []
            for j in range(self.Y_PIXELS):
                pixel_row.append(PixelType.EMPTY)
            self.pixels.append(pixel_row)
        self.painted_links = [] # Pairs of vertices whose links have already been painted.
    
    # Reset to initial state.
    def reset(self):
        self.pixels = []
        for i in range(self.X_PIXELS):
            pixel_row = []
            for j in range(self.Y_PIXELS):
                pixel_row.append(PixelType.EMPTY)
            self.pixels.append(pixel_row)
        self.painted_links = []
    
    # Convert a point in metres (standard units) to pixels.
    def to_pixels(self, point):
        return (round(point[0] / self.PIXEL_RES), round(point[1] / self.PIXEL_RES))
    
    # Set a starting point.
    def set_start(self, pos):
        self.start = self.to_pixels(pos)
        self.pixels[self.start[0]][self.start[1]] = PixelType.START
    
    # Set an end point.
    def set_end(self, pos):
        self.end = self.to_pixels(pos)
        self.pixels[self.end[0]][self.end[1]] = PixelType.END
    
    # Update the pixel array.
    def update_pixels(self, a_list):
        # start_time = time.time()
        # for i in range(self.X_PIXELS):
        #     for j in range(self.Y_PIXELS):
        #         if self.pixels[i][j] not in (PixelType.START, PixelType.END):
        #             self.pixels[i][j] = PixelType.EMPTY # Clear the pixel array (but skip over start and end).
        for v in a_list: # Iterate through vertices.
            for n in a_list[v]: # Iterate through neighbours.
                if (v, n) in self.painted_links or (n, v) in self.painted_links: # If this link has already been painted.
                    continue
                if v == n:
                    print('Self-link.')
                    continue
                self.painted_links.append((v, n))
                p1 = self.to_pixels(v)
                p2 = self.to_pixels(n)
                if self.pixels[p1[0]][p1[1]] not in (PixelType.START, PixelType.END):
                    self.pixels[p1[0]][p1[1]] = PixelType.VERTEX
                if self.pixels[p2[0]][p2[1]] not in (PixelType.START, PixelType.END):
                    self.pixels[p2[0]][p2[1]] = PixelType.VERTEX
                diff = [p2[i] - p1[i] for i in range(2)] # Find the difference vector.
                p_dist = math.ceil(math.dist(p1, p2)) # Choose the upper bound on the length of the wall.
                unit_diff = [diff[i] / p_dist for i in range(2)] # Normalise the difference vector to get the direction.
                current = [p1[0], p1[1]]
                for i in range(p_dist + 1):
                    c = [round(x) for x in current]
                    for j in range(max(0, math.floor(c[0] - self.P_WIDTH/2)), min(math.ceil(c[0] + self.P_WIDTH/2) + 1, self.X_PIXELS)):
                        for k in range(max(0, math.floor(c[1] - self.P_WIDTH/2)), min(math.ceil(c[1] + self.P_WIDTH/2) + 1, self.Y_PIXELS)):
                            pixel = self.pixels[j][k]
                            if pixel not in (PixelType.EMPTY, PixelType.CLEARED): # Only paint over empty space or cleared areas.
                                pass
                            elif c == [j, k]:
                                self.pixels[j][k] = PixelType.PATH # Mark this cell as a path.
                            elif math.dist(c, (j, k)) <= self.P_WIDTH/2:
                                self.pixels[j][k] = PixelType.CLEARED # Mark this cell as cleared.
                    current = [current[i] + unit_diff[i] for i in range(2)]
        self.prev_a_list = a_list
        # print('Pixel array update time:', round(time.time() - start_time, 3))
    
    # Render the pixel array.
    def render_pixels(self):
        # last_time = time.time()
        image = Image.new('RGB', (self.X_PIXELS, self.Y_PIXELS))
        img_pixels = image.load()
        for i in range(image.size[0]):
            for j in range(image.size[1]):
                pixel = self.pixels[i][j]
                if pixel == PixelType.START:
                    colour = (0, 255, 0)
                elif pixel == PixelType.END:
                    colour = (255, 0, 0)
                # elif (i, j) in self.external_path:
                #     colour = (0, 0, 255)
                elif pixel == PixelType.EMPTY:
                    colour = (0, 0, 0)
                elif pixel == PixelType.VERTEX:
                    colour = (0, 0, 255)
                elif pixel == PixelType.PATH:
                    colour = (255, 255, 255)
                elif pixel == PixelType.CLEARED:
                    colour = (128, 128, 128)
                else:
                    raise ValueError('Incorrect pixel value: ' + str(pixel) + ' at ' + str((i, j)) + '.')
                img_pixels[i, j] = colour
        image.show()
        # print('Image rendering time:', round(time.time() - last_time, 3))
    
if __name__ == '__main__':
    bitmap = MazeBitmap(3, 2)
    bitmap.set_start((0.3, 0.3))
    bitmap.set_end((2.7, 1.7))
    # a_list = {(0.3, 0.3): {(2.7, 1.7)}, (2.7, 1.7): {(0.3, 0.3)}}
    a_list = {(0.3, 0.3): {(1, 0.3), (0.3, 1), (1, 1)}, (1, 0.3): {(1, 1)}, (0.3, 1): {(0.3, 0.3)}}
    bitmap.update_pixels(a_list)
    a_list = {(0.3, 0.3): {(1, 0.3), (0.3, 1), (1, 1)}, (1, 0.3): {(1, 1)}, (0.3, 1): {(0.3, 0.3)}, (1, 1): {(0.3, 0.3), (2, 1.5)}, (2, 1.5): {(1, 1), (2.7, 1.7)}, (2.7, 1.7): {(2, 1.5)}}
    bitmap.update_pixels(a_list)
    bitmap.render_pixels()