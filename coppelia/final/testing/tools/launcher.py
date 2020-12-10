from math import pi, sqrt, sin, cos, atan2

class Point:
    """Purely for readability, for what it's worth"""
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Constants -------------------------------------
theta_f = pi/4  # The angle at which we want the projectile to land
g = 9.80665

def get_launch_params(d):
    """Takes a target (x,y), (actually (y,z) in the robot frame) and
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