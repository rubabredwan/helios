import utm
import json


def to_mer_list(pts, cen):
    points = []
    for pt in pts:
        points.append((to_mer(pt)[0] - to_mer(cen)[0], to_mer(pt)[1] - to_mer(cen)[1]))
    return points


def to_mer(pt):
    (x, y, p, q) = utm.from_latlon(pt['latitude'], pt['longitude'])
    return x * 3.281, y * 3.281


def get_val(pts, key):
    values = []
    for pt in pts:
        values.append(pt[key])
    return values


class Telemetry:
    def __init__(self, file='data.json'):
        with open(file) as f:
            data = json.load(f)
        center = data['mapCenterPos']
        print(to_mer(center))
        self.bPoints = to_mer_list(data['flyZones'][0]['boundaryPoints'], center)
        self.wPoints = to_mer_list(data['waypoints'], center)
        self.oPoints = to_mer_list(data['stationaryObstacles'], center)
        print(self.wPoints)
        self.oRadius = get_val(data['stationaryObstacles'], 'radius')
        for i in range(len(self.oPoints)):
            self.oPoints[i] = (self.oPoints[i], self.oRadius[i])
