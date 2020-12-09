import time
from math import pi, sqrt, sin, cos, atan2
from robot import robot
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import sim
from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets import make_blobs
from sklearn.preprocessing import StandardScaler

class Point:
    """Purely for readability, for what it's worth"""
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Constants -------------------------------------
theta_f = pi/4  # The angle at which we want the projectile to land
g = 9.80665

def get_launch_params(d):
    """Takes a target (x,y), (actually (x,z) in the robot frame) and
    returns launch parameters: initial velocity `v_0`, and initial angle `theta_0`"""
    x,y = d
    c = Point(x,-y)
    v_f = sqrt((c.x**2*g) / (c.x*sin(2*theta_f) - 2*c.y*(cos(theta_f))**2))
    t_f = (v_f*sin(theta_f) + sqrt((v_f**2*(sin(theta_f))**2) - (2*g*c.y))) / g
    v_fx = v_f*cos(theta_f)
    v_fy = v_f*sin(theta_f)-g*t_f
    v_0 = sqrt(v_fx**2 + v_fy**2)
    theta_0 = atan2(-v_fy, v_fx)
    return v_0, theta_0

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

def laser2world(points):
    scanner_frame = r.get_object_position('fast3DLaserScanner_sensor')
    scanner_orientation = r.get_object_orientation('fast3DLaserScanner')[2]
    R = np.array([[cos(scanner_orientation), -sin(scanner_orientation), 0],
        [sin(scanner_orientation), cos(scanner_orientation), 0],
        [0, 0, 1]])
    points = R.dot(points.T).T                          # Rotate
    points = np.add(points, scanner_frame)              # Translate
    points = points[np.logical_not(points[:,2] < 0)]    # Remove points on the ground
    return points

def spin(degrees, f=None, n=1):
    return

# Testing ------------------------------------------
motor_names = ['Omnirob_FLwheel_motor', 'Omnirob_FRwheel_motor', 'Omnirob_RRwheel_motor', 'Omnirob_RLwheel_motor']
r = robot('Omnirob', motor_names)  # Create an instance of our robot

target = r.get_object_position('Bill_head')
alpha, beta, gamma = r.get_object_orientation('Bill_head')

# Get 3d scanner info
points = laser_points()
points = laser2world(points)
points_on_ground = points[:,0:2]

db = DBSCAN(eps=0.5).fit(points_on_ground)
core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
core_samples_mask[db.core_sample_indices_] = True
labels = db.labels_

print(labels)

# Number of clusters in labels, ignoring noise if present.
n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
n_noise_ = list(labels).count(-1)

print('Estimated number of clusters: %d' % n_clusters_)

# #############################################################################
# Plot result
# import matplotlib.pyplot as plt
cmap = plt.cm.get_cmap("Spectral")
# Black removed and is used for noise instead.
unique_labels = set(labels)
colors = [cmap(each)
          for each in np.linspace(0, 1, len(unique_labels))]
for k, col in zip(unique_labels, colors):
    if k == -1:
        # Black used for noise.
        col = [0, 0, 0, 1]

    class_member_mask = (labels == k)

    xy = points_on_ground[class_member_mask & core_samples_mask]
    plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
             markeredgecolor='k', markersize=14)

    xy = points_on_ground[class_member_mask & ~core_samples_mask]
    plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
             markeredgecolor='k', markersize=6)

plt.title('Estimated number of clusters: %d' % n_clusters_)
plt.show(block=True)

# r.send_motor_velocities([2,2,2,2])
# time.sleep(1)
# while abs(r.get_orientation(-1)) > 0.1:
#     r.send_motor_velocities([2,2,2,2])
#     time.sleep(0.1)
# r.send_motor_velocities([0,0,0,0])


# Viz
# fig = plt.figure()
# # ax = plt.axes(projection='3d')
# ax = plt.axes()
# # ax.scatter3D(points[:,0],points[:,1],points[:,2])
# ax.plot(points_on_ground[:,0],points_on_ground[:,1], 'ko')
# plt.show(block=True)

# Step 1: Build an initial map
#   Turn around 360 degrees

# Every n degrees take a pic with the depth sensor
#       Transform point cloud to world frame coordinates
# note: the floor is not at exactly z=0 -.387 floor rwt sensor vs .375 sensor height 
# Or maybe it is - in which case the 
#           Remove all points with z == 0 (or less than some constant)
#           Project the point cloud onto the ground 
#           Set the coordinates occupied to 1 in gridworld
#   Find the target? - placed a circle there so I can use Hough algorithm



# Step 2: Re-orient
#   draw the person's view as an arc around them
#   go to an unoccupied spot with plenty of space
#   Make sure the person can see you
#       Take a picture with the depth sensor

# Instead of keeping a 3d representation of the world in memory at all times
#   Use virtual potential fields to orient
#   Take a picture with the depth sensor
#   See if there is a flight path
#       bfs until an unoccupied space is found - both on the ground and the z-value point where it previously collided

r.close_connection()


# Stage 1
# Get the position of the ball
# Fire it
# Inspect
# Log
# Reset 
# Repeat

# Stage 2
# Read logs
# For each parameter that led to hits
# Try the range again with finer differences
# Try a buffer at both ends in case we missed some

