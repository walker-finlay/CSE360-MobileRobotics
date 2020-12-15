from math import pi, sqrt, sin, cos, atan2
import numpy as np
from . import planning
import time

class Point:
    """Purely for readability, for what it's worth"""
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Constants -------------------------------------
theta_f = pi/4  # The angle at which we want the projectile to land
g = 9.80665

def transform3d(points, translation=None, rotation=None, scale=None):
    if rotation is not None:
        R = np.array([[cos(rotation), -sin(rotation), 0],
                      [sin(rotation), cos(rotation), 0],
                      [0,0,1]])
        points = R.dot(points.T).T
    if translation is not None:
        points = (points + translation)
    if scale is not None:
        points = (points * scale)
    return points

def get_launch_params(r):
    projectile_coords = r.get_object_relative_position('projectile')
    target = r.get_object_relative_position('Bill_head')
    # Transform target to projectile frame
    mouth_offset = np.array([0,0.0771, 0.0552])
    _,x,y = transform3d(target, translation=-(projectile_coords + mouth_offset))
    print(f'target in projectile frame: {x,y}')
    # Calculations
    c = Point(x,-y)
    v_f = sqrt((c.x**2*g) / (c.x*sin(2*theta_f) - 2*c.y*(cos(theta_f))**2))
    t_f = (v_f*sin(theta_f) + sqrt((v_f**2*(sin(theta_f))**2) - (2*g*c.y))) / g
    v_fx = v_f*cos(theta_f)
    v_fy = v_f*sin(theta_f)-g*t_f
    v_0 = sqrt(v_fx**2 + v_fy**2)
    theta_0 = atan2(-v_fy, v_fx)
    return v_0, theta_0


def fire(v, theta, r):
    r.set_joint_position('launcher_revolute_joint', theta)
    while abs(r.get_joint_position('launcher_revolute_joint') - theta) > 0.025: time.sleep(1)
    r.set_joint_velocity('launcher_prismatic_joint', v)