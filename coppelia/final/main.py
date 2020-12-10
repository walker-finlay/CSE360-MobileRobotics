import time
import sim
import numpy as np 
from math import sin, cos, pi

from mpl_toolkits import mplot3d
from sklearn.cluster import DBSCAN

from robot import robot
from tools.viz import plot_clusters2d, plot_clusters3d, plot_2d, plot_grid
from tools.planning import build_grid, build_graph, astar, get_launch_point, path2waypoints
from tools.perception import laser2world, laser_points, spin_do, sense

# Constants
sit = 1.8   # Sit at 1.8 meters from the target, give them space
fol = pi/6  # Field of Launch : Don't fire from an extreme angle (target gamma +/- pi/12)


def fire():
    return

# Actions ---------------------------------------------------------------------
motor_names = ['Omnirob_FLwheel_motor', 'Omnirob_FRwheel_motor', 'Omnirob_RRwheel_motor', 'Omnirob_RLwheel_motor']
r = robot('Omnirob', motor_names)  # Create an instance of our robot

# Initial perception --------------------------------------
# Spin around 360 degrees, take 10 pictures with the depth sensor, aggregate results in `points`
robo_coords = r.get_position(-1)[0:2]
points = spin_do(2*pi, r, sim, f=sense, n=10)

# Project 3d point cloud onto the ground, identify obstacles as clusters
points_on_ground = points[:,0:2]
db = DBSCAN(eps=0.425).fit(points)
labels = db.labels_
n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
plot_clusters3d(db, points)

# Build graph for A* --------------------------------------
# Also gets robot coordinates transformed to gridworld and transformation parameters
# for example: bottom left,top right = (-10,-10),(10,10) => (0,0),(20,20) => (0,0),(50,50)
gridworld, robo_coords, tf_params = build_grid(points_on_ground, n_clusters_, labels, robo_coords, extend=True)
translation, scale, m = tf_params  # Looks like (array([dx,dy]), scale, world frame size)
G = build_graph(gridworld)

# Identify target & find a destination to fire from -------
target = r.get_object_position('Bill_head')
_, _, gamma = r.get_object_orientation('Bill_head') # Target orientation in Euler angles
lp = None
fire_range = sit
while not lp:
    lp = get_launch_point(target[0:2], fire_range, fol, gamma, gridworld, translation, scale)
    fire_range += 0.1

# A* there ------------------------------------------------
path = astar(G, tuple(robo_coords), lp) # make starting coords a tuple so astar can hash them
n = len(gridworld)
waypoints = path2waypoints(path, translation, m/len(gridworld), 0.5)
# Execute waypoints as a polyline trajectory
dt = 5
for wp in waypoints:
    print(wp)
    tf = wp[1]
    time_steps = np.linspace(0, tf, dt)
    robot_position = r.get_position()[0:2]
    desired_position = wp[0]
    a1 = (desired_position - robot_position) / tf
    a0 = robot_position
    for t in time_steps:
        point_traj = a1 * t + a0
        vel_traj = a1
        # Sensing
        robot_position = r.get_position()[0:2]
        # Trajectory tracker
        u = 100 * (point_traj - robot_position) + vel_traj
        vx, vy = u
        r.send_motor_velocities([-vy - vx, vy - vx, vy + vx, -vy + vx])
        time.sleep(tf/dt)
r.send_motor_velocities([0,0,0,0])

# Rotate & fire!! -----------------------------------------
gamma = r.get_orientation(-1)

r.close_connection()