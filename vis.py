import pygame
import sys
import utm
import json
import numpy

def amin(x):
  res = None
  for i in x:
    if res is None or i < res:
      res = i
  return res

def amax(x):
  res = None
  for i in x:
    if res is None or i > res:
      res = i
  return res

def normalize(aer):
  mn, mx = amin(aer), amax(aer)
  for i in range(len(aer)):
    aer[i] = (aer[i] - mn) / (mx - mn)
    aer[i] = int(aer[i] * 768)
  return aer

def process(pts):
  xc = []
  yc = []
  for pt in pts:
    (x, y, p, q) = utm.from_latlon(pt['latitude'], pt['longitude'])
    xc.append(x)
    yc.append(y)

  return (normalize(xc), normalize(yc))

with open('data.json') as f:
  data = json.load(f)

boundary = process(data['flyZones'][0]['boundaryPoints'])
print(boundary)


pygame.init()
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
  for i in range(len(boundary[0])):
    pygame.draw.aaline(screen, (255, 0, 0), (boundary[0][i], boundary[1][i]), (boundary[0][i-1], boundary[1][i-1]))
  pygame.display.update()
