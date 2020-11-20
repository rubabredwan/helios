from os import path

from telemetry import Telemetry
from trajectory_GUI import TrajectoryGUI
from trajectory_algorithm import trajectory_algorithm
import vpython


w_t = []
o_t = []
b_t = []
if path.exists('data.json'):
    tm = Telemetry()

    b_t = tm.bPoints
    o_t = tm.oPoints
    w_t = tm.wPoints
    c_t = tm.center

    scaled_path = trajectory_algorithm(w_t, o_t, b_t)

    GUI = TrajectoryGUI(650, w_t, o_t, b_t, c_t, scaled_path)

else:
    GUI = TrajectoryGUI(650, w_t, o_t, b_t, (0, 0), [])

GUI.update()
GUI.events()