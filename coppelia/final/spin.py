from robot import robot
from math import pi
import sim
import numpy as np
import time
from tools.perception import laser2world


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


motor_names = ['Omnirob_FLwheel_motor', 'Omnirob_FRwheel_motor', 'Omnirob_RRwheel_motor', 'Omnirob_RLwheel_motor']
r = robot('Omnirob', motor_names)  # Create an instance of our robot
points = spin_do(2*pi, f=sense, n=10)
np.savetxt('more_points.csv', points, delimiter=',')
r.close_connection()
