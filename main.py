from os import path

from telemetry import Telemetry
from trajectory_GUI import TrajectoryGUI
from trajectory_algorithm import trajectory_algorithm
import vpython
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--blank", default=False, type=bool)
args = parser.parse_args()

if args.blank == False and path.exists('data.json'):
  tm = Telemetry()

  b_t = tm.bPoints
  o_t = tm.oPoints
  w_t = tm.wPoints
  c_t = tm.center

  scaled_path = trajectory_algorithm(w_t, o_t, b_t)

  GUI = TrajectoryGUI(650, w_t, o_t, b_t, c_t, scaled_path)

else:
  GUI = TrajectoryGUI(650, [], [], [], {
      'latitude': 32.2318851,
      'longitude': -110.9522981
  }, [])

GUI.update()
GUI.events()
