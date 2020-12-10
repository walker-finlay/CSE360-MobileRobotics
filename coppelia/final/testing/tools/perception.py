from math import sin, cos
import numpy as np

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
