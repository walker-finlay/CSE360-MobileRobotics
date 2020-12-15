import numpy as np
from math import pi, inf
import time

def rotate_to(gamma_d, r):
    while True:
        gamma = r.get_orientation(-1)
        diff = gamma_d - gamma
        u = -2 * diff # Proportional controller
        r.send_motor_velocities([u,u,u,u])
        if (abs(diff) < 0.001): 
            r.send_motor_velocities([0,0,0,0])
            break

def polyline(waypoints, r):
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