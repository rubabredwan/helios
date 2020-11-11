import pygame
import sys
import utm
import json
import numpy
from telemetry import Telemetry

tm = Telemetry()
boundary = tm.get_borderPoints(768)
obstacles = tm.get_obstacles(768)
waypoints = tm.get_waypoints(768)

pygame.init()
pygame.display.set_caption('simulation')
screen = pygame.display.set_mode((768, 768))
print(pygame.display.list_modes())
print(pygame.display.Info())

background = pygame.image.load('bg.jpg').convert()

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
    screen.blit(background, (0, 0))
    for i in range(len(boundary)):
        pygame.draw.line(screen, (255, 0, 0), (boundary[i][0], boundary[i][1]),
                         (boundary[i - 1][0], boundary[i - 1][1]),
                         width=2)
    for i in obstacles:
        pygame.draw.circle(screen, (255, 255, 0), (i[0], i[1]), i[2])
    font = pygame.font.SysFont(None, 24)
    for i in range(len(waypoints)):
        # pygame.draw.circle(screen, (0, 0, 255), (i[0], i[1]), 3 )
        font = pygame.font.SysFont(None, 24)
        img = font.render(str(i + 1), True, (0, 0, 255))
        rect = img.get_rect()
        pygame.draw.rect(img, (0, 0, 255), rect, 1)
        screen.blit(img, (waypoints[i][0], waypoints[i][1]))

    pygame.display.update()
