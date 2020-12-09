import time
from math import pi, sqrt, sin, cos, atan2
from robot import robot
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import sim

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

# Testing ------------------------------------------
motor_names = ['Omnirob_FLwheel_motor', 'Omnirob_FRwheel_motor', 'Omnirob_RRwheel_motor', 'Omnirob_RLwheel_motor']
r = robot('Omnirob', motor_names)  # Create an instance of our robot

target = r.get_object_position('Bill_head')
alpha, beta, gamma = r.get_object_orientation('Bill_head')

# Get 3d scanner info
points = laser_points()
scanner_frame = r.get_object_position('fast3DLaserScanner')
scanner_orientation = r.get_object_orientation('fast3DLaserScanner')[2]
R = np.array([[cos(scanner_orientation), -sin(scanner_orientation), 0],
    [sin(scanner_orientation), cos(scanner_orientation), 0],
    [0, 0, 1]])
points = R.dot(points.T).T                          # Rotate
points = np.add(points, scanner_frame)              # Translate
points = points[np.logical_not(points[:,2] < 0)]    # Remove points on the ground

# Viz
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.scatter3D(points[:,0],points[:,1],points[:,2])
plt.show(block=True)

# Step 1: Build an initial map
#   Turn around 360 degrees

# r.send_motor_velocities([2,2,2,2])
# time.sleep(1)
# while abs(r.get_orientation(-1)) > 0.1:
#     r.send_motor_velocities([2,2,2,2])
#     time.sleep(0.1)
# r.send_motor_velocities([0,0,0,0])
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

