from math import sin, cos, pi
import numpy as np
import time

def laser2world(points, r):
    """Convert point cloud in sensor frame to points in world frame
    and remove non-relevant points on the ground (we already know where that is!)"""
    scanner_frame = r.get_object_position('fast3DLaserScanner_sensor')
    scanner_orientation = r.get_object_orientation('fast3DLaserScanner')[2]
    R = np.array([[cos(scanner_orientation), -sin(scanner_orientation), 0],
        [sin(scanner_orientation), cos(scanner_orientation), 0],
        [0, 0, 1]])
    points = points[np.logical_not(points[:,2] < -scanner_frame[2])]    # Remove points on the ground
    points = R.dot(points.T).T                                          # Rotate
    points = np.add(points, scanner_frame)                              # Translate
    return points

def laser_points(r, sim):
    return_code, data = sim.simxGetStringSignal(r.client_id, "measuredDataAtThisTime", sim.simx_opmode_blocking)
    if return_code == sim.simx_return_ok:
        measured_data = np.array(sim.simxUnpackFloats(data))
        measured_data = np.split(measured_data, len(measured_data)/3)
    else:
        print("Eror %d"%return_code)
    return np.array(measured_data)

def sense(r, sim, debug=False, d=None):
    if debug:
        print(f'sensing... d={d}')
    points = laser_points(r, sim)
    points = laser2world(points,r)
    return points

# FIXME: There must be an easier way to do dependency injection here...
def spin_do(r, sim, f = sense, n:int = None, dbg: bool=False):
    """Method for rotating by `deg` degrees - 
    Also used for initial mapping by calling `f`() n times during the rotation
    """
    data = np.empty((0,3))
    init_gamma = r.get_orientation()
    if n is not None:
        data = np.concatenate((data, f(r, sim, debug=dbg, d=init_gamma)))
        action_angles = np.linspace(2*pi/n, 2*pi, n) + init_gamma
        action_angles = action_angles
    r.send_motor_velocities([2,2,2,2])                          # Start the spin so we can compare curr to init
    time.sleep(1)
    curr_gamma = r.get_orientation()
    while abs(curr_gamma - init_gamma) > 0.05:
        r.send_motor_velocities([2,2,2,2])
        curr_gamma = r.get_orientation()
        if f is not None:                                                   
            mask = abs(action_angles - curr_gamma%(2*pi)) < 0.1 # Are we at one of the angles, and which one?
            if np.any(mask):
                action_angles = action_angles[~mask]            # Do it (sense_async)
                data = np.concatenate((data, f(r, sim, debug=dbg, d=curr_gamma)))
    r.send_motor_velocities([0,0,0,0])
    if f is not None:
        return data

    