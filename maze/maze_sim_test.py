import pygame
from maze_sim import *

sim = MazeSim(2, False, 1)
pygame.init()
screen = pygame.display.set_mode((sim.X_PIXELS * sim.pp, sim.Y_PIXELS * sim.pp))
while sim.update():
    edges = sim.manager.get_edges()
    path = sim.manager.get_path()
    pos = sim.manager.get_pos()
    angle = sim.manager.get_angle()

    pygame.surfarray.blit_array(screen, sim.manager.bitmap.get_bitmap_debug((sim.ppos[0], sim.ppos[1]), sim.robot_path, sim.manager.get_path()))
    pygame.display.update()
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                pygame.quit()
pygame.quit()
sim.manager.bitmap.render_pixels_debug(sim.walls, sim.WALL_WIDTH)