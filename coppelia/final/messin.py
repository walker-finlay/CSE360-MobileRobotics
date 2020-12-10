import time
import numpy as np 
from math import pi
from mpl_toolkits import mplot3d
from sklearn.cluster import DBSCAN
from tools.viz import plot_clusters, plot_2d, plot_grid, plot_3d
from tools.planning import build_grid, build_graph, astar, get_launch_point, path2waypoints, transform2d
from tools.perception import laser2world

from matplotlib import pyplot as plt

# Constants
sit = 1.8   # Sit at 1.8 meters from the target, give them space
fol = pi/6  # Field of Launch : Don't fire from an extreme angle (target gamma +/- pi/12)

# Action ----------------------------------------------------------------------
robo_coords = np.array([-0.65, -2.1])
points = np.genfromtxt('points.csv', delimiter=',')
print("got points")

points_on_ground = points[:,0:2]
db = DBSCAN(eps=0.5).fit(points_on_ground)
labels = db.labels_
n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)


# Build graph for A* --------------------------------------
gridworld, robo_coords, tf_params = build_grid(points_on_ground, n_clusters_, labels, robo_coords, extend=True)
translation, scale, m = tf_params 
G = build_graph(gridworld)

goal = np.array([0, 1.027])
goal = transform2d(goal, translation=-translation, scale=scale)
# plot_grid(gridworld, r_c=robo_coords, goal=goal)

# Identify target & find a destination to fire from -------
target = np.array([0, 1.027, 1.13])
gamma = -1.5
lp = None
fire_range = sit
while not lp:
    lp = get_launch_point(target[0:2], fire_range, fol, gamma, gridworld, translation, scale)
    fire_range += 0.1

# A* there ------------------------------------------------
path = astar(G, tuple(robo_coords), lp)
waypoints = path2waypoints(path, translation, m/len(gridworld), 0.5)
print("done")