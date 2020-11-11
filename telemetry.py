import utm
import json
import numpy as np
import math


def get_cord(pts):
    points = []
    for pt in pts:
        (x, y, p, q) = utm.from_latlon(pt['latitude'], pt['longitude'])
        points.append((x, y))
    return np.array(points)


def get_val(pts, key):
    vals = []
    for pt in pts:
        vals.append(pt[key])
    return np.array(vals)


class Telemetry:
    def __init__(self, file='data.json'):
        with open(file) as f:
            data = json.load(f)
        self.mns = np.array([float('inf'), float('inf')])
        self.mxs = np.array([-float('inf'), -float('inf')])
        self.bPoints = self.process(data['flyZones'][0]['boundaryPoints'])
        self.wPoints = self.process(data['waypoints'])
        self.oPoints = self.process(data['stationaryObstacles'])
        self.oRadius = get_val(data['stationaryObstacles'], 'radius')
        self.scale = max(self.mxs[0] - self.mns[0], self.mxs[1] - self.mns[1])
        self.bPoints = (self.bPoints - self.mns) / self.scale
        self.wPoints = (self.wPoints - self.mns) / self.scale
        self.oPoints = (self.oPoints - self.mns) / self.scale
        self.oRadius = self.oRadius / (2 * self.scale)

    def process(self, locations):
        points = get_cord(locations)
        self.mns = np.fmin(np.amin(points, 0), self.mns)
        self.mxs = np.fmax(np.amax(points, 0), self.mxs)
        return points

    def get_borderPoints(self, pixels):
        pts = self.bPoints.tolist()
        for i in range(len(pts)):
            for j in range(2):
                pts[i][j] = int(pts[i][j] * pixels)
        return pts

    def get_obstacles(self, pixels):
        pts = self.oPoints.tolist()
        for i in range(len(pts)):
            for j in range(2):
                pts[i][j] = int(pts[i][j] * pixels)
            pts[i].append(int(self.oRadius[i] * pixels))
        return pts

    def get_waypoints(self, pixels):
        pts = self.wPoints.tolist()
        for i in range(len(pts)):
            for j in range(2):
                pts[i][j] = int(pts[i][j] * pixels)
        return pts
