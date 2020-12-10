import time
import sim
import numpy as np 
from math import sin, cos, pi

from mpl_toolkits import mplot3d
from sklearn.cluster import DBSCAN

from robot import robot
from tools.viz import plot_clusters, plot_2d, plot_grid
from tools.planning import build_grid, build_graph, astar, get_launch_point, path2waypoints
from tools.perception import laser2world

# Constants
sit = 1.8   # Sit at 1.8 meters from the target, give them space
fol = pi/6  # Field of Launch : Don't fire from an extreme angle (target gamma +/- pi/12)


def fire():
    return

def laser_points():
    return_code, data = sim.simxGetStringSignal(r.client_id, "measuredDataAtThisTime", sim.simx_opmode_blocking)
    if return_code == sim.simx_return_ok:
        measured_data = np.array(sim.simxUnpackFloats(data))
        measured_data = np.split(measured_data, len(measured_data)/3)
    else:
        print("Eror %d"%return_code)
    return np.array(measured_data)

def spin_do(deg: float, f = None, n:int = 1, dbg: bool=False):
    """Method for rotating by `deg` degrees - 
    Also used for initial mapping by calling `f`() n times during the rotation
    """
    data = np.empty((0,3))
    init_gamma = r.get_orientation()
    if n:
        data = np.concatenate((data, f(debug=dbg, d=init_gamma)))
        action_angles = np.linspace(2*pi/n, 2*pi, n) + init_gamma
        action_angles = action_angles
    r.send_motor_velocities([2,2,2,2])                          # Start the spin so we can compare curr to init
    time.sleep(1)
    curr_gamma = r.get_orientation()
    while abs(curr_gamma - init_gamma) > 0.05:
        r.send_motor_velocities([2,2,2,2])
        curr_gamma = r.get_orientation()
        if f:                                                   
            mask = abs(action_angles - curr_gamma%(2*pi)) < 0.1 # Are we at one of the angles, and which one?
            if np.any(mask):
                action_angles = action_angles[~mask]            # Do it (sense_async)
                data = np.concatenate((data, f(debug=dbg, d=curr_gamma)))
    r.send_motor_velocities([0,0,0,0])
    if f:
        return data

def sense(debug=False, d=None):
    if debug:
        print(f'sensing... d={d}')
    points = laser_points()
    points = laser2world(points,r)
    return points
    
# Actions ---------------------------------------------------------------------

motor_names = ['Omnirob_FLwheel_motor', 'Omnirob_FRwheel_motor', 'Omnirob_RRwheel_motor', 'Omnirob_RLwheel_motor']
r = robot('Omnirob', motor_names)  # Create an instance of our robot

# Initial perception --------------------------------------
# Spin around 360 degrees, take 10 pictures with the depth sensor, aggregate results in `points`
robo_coords = r.get_position(-1)[0:2]
points = spin_do(2*pi, f=sense, n=10)

# Project 3d point cloud onto the ground, identify obstacles as clusters
points_on_ground = points[:,0:2]
db = DBSCAN(eps=0.5).fit(points_on_ground)
labels = db.labels_
n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
# plot_clusters(db, points_on_ground)

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
waypoints = path2waypoints(path, n, m, 0.5)
# Execute waypoints as a polyline trajectory
# i = 0
# for wp in waypoints:
#     print(wp)
#     tf = wp[1]
#     time_steps = np.linspace(0, tf, 500)
#     robot_position = r.get_position()[0:2]
#     desired_position = wp[0]
#     a1 = (desired_position - robot_position) / tf
#     a0 = robot_position
#     for t in time_steps:
#         point_traj = a1 * t + a0
#         vel_traj = a1
#         # Sensing
#         robot_position = r.get_position()[0:2]
#         # Trajectory tracker
#         u = 10 * (point_traj - robot_position) + vel_traj
#         vx, vy = u
#         r.send_motor_velocities([-vy - vx, vy - vx, vy + vx, -vy + vx])
#         time.sleep(tf/500)


r.close_connection()