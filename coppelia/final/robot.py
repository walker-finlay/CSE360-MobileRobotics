"""
Communication with coppeliasim\n
Forked from https://github.com/dsaldana/CSE360-MobileRobotics
"""

import sim
import time
import numpy as np
from numpy import array
import pylab
from math import pi
pylab.interactive(True)

# Put these in __init__()?
sim.simxFinish(-1)  # Close opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim
class robot():
    
    def __init__(self, frame_name, motor_names=[], client_id=0):  
        # If there is an existing connection
        if client_id:
                self.client_id = client_id
        else:
            self.client_id = self.open_connection()
            
        self.motors = self._get_handlers(motor_names) 
        
        # Robot frame
        self.frame =  self._get_handler(frame_name)
            
        
    def open_connection(self):
        sim.simxFinish(-1)  # just in case, close all opened connections
        self.client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim 
        
        if clientID != -1:
            print('Robot connected')
        else:
            print('Connection failed')
        return clientID
        
    def close_connection(self):    
        sim.simxGetPingTime(self.client_id)  # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
        sim.simxFinish(self.client_id)  # Now close the connection to CoppeliaSim:
        print('Connection closed')
    
    def isConnected(self):
        c,result = sim.simxGetPingTime(self.client_id)
        # Return true if the robot is connected
        return result > 0         
        
    def _get_handler(self, name):
        err_code, handler = sim.simxGetObjectHandle(self.client_id, name, sim.simx_opmode_blocking)
        return handler
    
    def _get_handlers(self, names):
        handlers = []
        for name in names:
            handler = self._get_handler(name)
            handlers.append(handler)
        
        return handlers

    def send_motor_velocities(self, vels):
        for motor, vel in zip(self.motors, vels):
            err_code = sim.simxSetJointTargetVelocity(self.client_id, 
                                                      motor, vel, sim.simx_opmode_streaming)      
            
    def set_position(self, position, relative_object=-1):
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)        
        sim.simxSetObjectPosition(clientID, self.frame, relative_object, position, sim.simx_opmode_oneshot)
        
    def simtime(self):
        return sim.simxGetLastCmdTime(self.client_id)
    
    def get_position(self, relative_object=-1):
        # Get position relative to an object, -1 for global frame
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)
        res, position = sim.simxGetObjectPosition(self.client_id, self.frame, relative_object, sim.simx_opmode_blocking)        
        return array(position)

    def get_orientation(self, relative_object=-1):
        # Get orientation relative to an object, -1 for global frame
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)
        res, position = sim.simxGetObjectOrientation(self.client_id, self.frame, relative_object, sim.simx_opmode_blocking)        
        return array(position)[2]

    def normal_gamma(self):
        """Get rotation about z as a positive number"""
        res, orientation = sim.simxGetObjectOrientation(self.client_id, self.frame, -1, sim.simx_opmode_blocking)
        orientation = orientation[2]
        if orientation < 0: orientation = orientation + 2*pi
        return orientation
    
    def get_object_position(self, object_name):
        # Get Object position in the world frame
        err_code, object_h = sim.simxGetObjectHandle(self.client_id, object_name, sim.simx_opmode_blocking)
        res, position = sim.simxGetObjectPosition(self.client_id, object_h, -1, sim.simx_opmode_blocking)
        return array(position)

    def get_object_orientation(self, object_name):
        # Get Object orientation in the world frame
        err_code, object_h = sim.simxGetObjectHandle(self.client_id, object_name, sim.simx_opmode_blocking)
        res, orientation = sim.simxGetObjectOrientation(self.client_id, object_h, -1, sim.simx_opmode_blocking)
        return array(orientation)
    
    def get_object_relative_position(self, object_name):        
        # Get Object position in the robot frame
        err_code, object_h = sim.simxGetObjectHandle(self.client_id, object_name, sim.simx_opmode_blocking)
        res, position = sim.simxGetObjectPosition(self.client_id, object_h, self.frame, sim.simx_opmode_blocking)
        return array(position)

    def laser_points(self):
        return_code, data = sim.simxGetStringSignal(self.client_id, "measuredDataAtThisTime", sim.simx_opmode_blocking)
        if return_code == sim.simx_return_ok:
            measured_data = np.array(sim.simxUnpackFloats(data))
            measured_data = np.split(measured_data, len(measured_data)/3)
        else:
            print("Eror %d"%return_code)
        return np.array(measured_data)

    def spin_do(self, deg: float, n:int = 1, dbg: bool=False):
        """Method for rotating by `deg` degrees - 
        Also used for initial mapping by calling `f`() n times during the rotation
        """
        data = np.empty((0,3))
        init_gamma = self.get_orientation()
        if n:
            data = np.concatenate((data, self.sense(debug=dbg, d=init_gamma)))
            action_angles = np.linspace(2*pi/n, 2*pi, n) + init_gamma
            action_angles = action_angles
        self.send_motor_velocities([2,2,2,2])                          # Start the spin so we can compare curr to init
        time.sleep(1)
        curr_gamma = self.get_orientation()
        while abs(curr_gamma - init_gamma) > 0.05:
            self.send_motor_velocities([2,2,2,2])
            curr_gamma = self.get_orientation()                
            mask = abs(action_angles - curr_gamma%(2*pi)) < 0.1 # Are we at one of the angles, and which one?
            if np.any(mask):
                action_angles = action_angles[~mask]            # Do it (sense_async)
                data = np.concatenate((data, self.sense(debug=dbg, d=curr_gamma)))
        self.send_motor_velocities([0,0,0,0])
        return data

    def sense(self, debug=False, d=None):
        if debug:
            print(f'sensing... d={d}')
        points = self.laser_points()
        return points

    def fire(self, v, theta):
        pass