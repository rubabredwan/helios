import utm
import json
import copy
import sys

sample_item = {
  'AMSLAltAboveTerrain': None,
  'Altitude': 50,
  'AltitudeMode': 1,
  'autoContinue': True,
  'command': 22,
  'doJumpId': 1,
  'frame': 3,
  'params': [],
  'type':'SimpleItem'
}

sample_plan = {
  'fileType': 'Plan',
  'geoFence': {
    'circles': [],
    'polygons': [],
    'version': 2
  },
  'groundStation': 'QGroundControl',
  'mission': {
    'cruiseSpeed': 15,
    'firmwareType': 12,
    'hoverSpeed': 5,
    'items': [],
    'plannedHomePosition': [],
    'vehicleType': 2,
    'version': 2
  },
  'rallyPoints': {
    'points': [],
    'version': 2
  },
  'version': 1
}

def generate_plan(boundary, obstacles, waypoints, center):
  (xc, yc, zc, dc) = utm.from_latlon(center['latitude'], center['longitude'])
  poly = []
  plan = copy.copy(sample_plan)
  for pt in boundary:
    (x, y) = utm.to_latlon(xc + pt[0] / 3.281, yc + pt[1] / 3.281, zc, dc)
    poly.append([x, y])
  plan['geoFence']['polygons'] = [{
    'inclusion': True,
    'polygon': poly,
    'version': 1,
  }]
  obst = []
  for pt in obstacles:
    (x, y) = utm.to_latlon(xc + pt[0][0] / 3.281, yc + pt[0][1] / 3.281, zc, dc)
    obst.append({
      "circle": {
        "center": [x, y],
        "radius": pt[1] / 3.281,
      },
      "inclusion": False,
      "version": 1,
    })

  plan['geoFence']['circles'] = obst
  wayp = []
  for i in range(len(waypoints)):
    item = copy.copy(sample_item)
    item['command'] = 22 if i == 0 else 16
    item['doJumpId'] = i + 1
    (x, y) = utm.to_latlon(xc + waypoints[i][0] / 3.281, yc + waypoints[i][1] / 3.281, zc, dc)
    item['params'] = [(15 if i == 0 else 0), 0, 0, None, x, y, 50]
    wayp.append(item)
  plan['mission']['items'] = wayp
  plan['mission']['plannedHomePosition'] = [center['latitude'], center['longitude'], 50]
  with open('aza.plan', 'w') as f:
    json.dump(plan, f)
    



