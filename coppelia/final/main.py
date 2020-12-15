import time
import sim
import numpy as np 
from math import sin, cos, pi

from mpl_toolkits import mplot3d
from sklearn.cluster import DBSCAN

from robot import robot
from tools.viz import plot_clusters2d, plot_clusters3d, plot_2d, plot_grid
from tools.planning import build_grid, build_graph, astar, get_launch_point, path2waypoints
from tools.perception import spin_do
from tools.locomotion import rotate_to, polyline
from tools.launcher import get_launch_params, fire

# Constants
sit = 1.8   # Sit at 1.8 meters from the target, give them space
fol = pi/6  # Field of Launch : Don't fire from an extreme angle (target gamma +/- pi/12)

# Actions ---------------------------------------------------------------------
motor_names = ['Omnirob_FLwheel_motor', 'Omnirob_FRwheel_motor', 'Omnirob_RRwheel_motor', 'Omnirob_RLwheel_motor']
r = robot('Omnirob', motor_names)  # Create an instance of our robot

# Initial perception --------------------------------------
# Spin around 360 degrees, take 10 pictures with the depth sensor, aggregate results in `points`
robo_coords = r.get_position(-1)[0:2]
points = spin_do(r, sim, n=10)

# identify obstacles as clusters, project 3d point cloud onto the ground,
db = DBSCAN(eps=0.425).fit(points)
labels = db.labels_
n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
points_on_ground = points[:,0:2]

# Build graph for planning --------------------------------
# Also gets robot coordinates transformed to gridworld and transformation parameters
gridworld, robo_coords, tf_params = build_grid(points_on_ground, n_clusters_, labels, robo_coords, extend=True)
translation, scale, m = tf_params  # Looks like (array([dx,dy]), scale, world frame size)
G = build_graph(gridworld)

# Identify target & find a destination to fire from -------
target = r.get_object_position('Bill_head') # TODO: use a sphere and opencv for this
_, _, gamma = r.get_object_orientation('Bill_head') # Target orientation in Euler angles
lp = None
fire_range = sit
while not lp:
    lp = get_launch_point(target[0:2], fire_range, fol, gamma, gridworld, translation, scale)
    fire_range += 0.1

# A* there ------------------------------------------------
path = astar(G, tuple(robo_coords), lp) # make starting coords a tuple so astar can hash them
waypoints = path2waypoints(path, translation, m/len(gridworld), 0.5)
polyline(waypoints, r)

# Rotate & fire!! -----------------------------------------
robo_coords = r.get_position(-1)
dx = target[0] - robo_coords[0]
dy = target[1] - robo_coords[1]
gamma_d = -np.arctan2(dx, dy)
rotate_to(gamma_d, r)
v, theta = get_launch_params(r)
fire(v, theta, r)

r.close_connection()