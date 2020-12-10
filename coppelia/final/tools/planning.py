import numpy as np
from math import sin, cos, pi, sqrt

# Constants
omnirob_bounding_box = [0.356, 0.598, 0.2925]

def build_grid(points_on_ground, n_clusters_, labels, robo_coords, gw_n=50, extend=False):
    """gw_n by gw_n grid building based on data from 3d laser scanner and clustering using dbscan. 
    Optionally scale up the obstacles so that rob doesn't crash into them. 
    Returns gridworld, robot coordinates in grid world, and transformation parameters - 
    tf_params = (translation, scale)"""
    # Get map bounds based on current data + plenty of space for rob to get around obstacles
    map_min = points_on_ground.min(axis=0) 
    map_max = points_on_ground.max(axis=0) 
    map_min = np.minimum(robo_coords, map_min) - omnirob_bounding_box[1]*2  # Rob should be on the map duh
    map_max = np.maximum(robo_coords, map_max) + omnirob_bounding_box[1]*2  # |

    gw_points = points_on_ground

    gridworld = np.zeros((gw_n, gw_n), dtype=np.uint8)

    # Shifting obstacles to positive coordinates
    # World
    world_max_shifted = map_max
    if map_min[0] < 0:
        gw_points[:,0] -= map_min[0]
        robo_coords[0] -= map_min[0]
        world_max_shifted[0] -= map_min[0]
    if map_min[1] < 0:
        gw_points[:,1] -= map_min[1]
        robo_coords[1] -= map_min[1]
        world_max_shifted[1] -= map_min[1]

    # Scaling obstacles to grid size
    global_n = np.amax(world_max_shifted)
    scale = gw_n/global_n
    gw_points *= scale
    robo_coords *= scale

    # # Create bounding box around obstacles, set to 1 in gridworld
    for n in np.arange(0,n_clusters_,dtype=np.intp):
        current_obstacle_data = gw_points[labels == n]
        lower_left = (current_obstacle_data.min(axis=0) + 0.5 ).astype(np.intc)     # Looks like:   [xmin, ymin]
        upper_right = (current_obstacle_data.max(axis=0) + 0.5 ).astype(np.intc)    # |             [xmax, ymax]
        if extend:  # Make room for rob to get close
            lower_left += int(omnirob_bounding_box[1]/1.75)
            upper_right += int(omnirob_bounding_box[1]/1.75)
        gridworld[lower_left[0]:upper_right[0], lower_left[1]:upper_right[1]] = 1

    robo_coords = (np.array(robo_coords) + 0.5).astype(np.intc)
    return gridworld, robo_coords, (map_min, scale, global_n)

# Directly from workshop 3 - neat! --------------------------------------------
def build_graph(grid):
    directions = np.array([
        [0,1],      # North
        [1,0],      # East
        [0,-1],     # South
        [-1,0],     # West
        [1,1],      # NE
        [1,-1],     # SE
        [-1,-1],    # SW
        [-1,1]      # NW
    ])
    """Build adjacency list for bfs"""
    G = {}
    n = len(grid)
    for i in range(0, n):
        for j in range(0, n):
            current = (i,j)
            G[current] = []
            for direction in directions:
                next_node = (i+direction[0], j+direction[1])
                if next_node[0] >= 0 and next_node[0] < n and next_node[1] >= 0 and next_node[1] < n and grid[next_node] == 0:
                    G[current].append(next_node)
    return G

def backtrace(node, parent):
    path = [node]
    while parent[node]:
        path.append(parent[node])
        node = parent[node]
    path.reverse()
    return path


def path2waypoints(path, translation, scale, t):
    """turn path from nxn grid into waypoints
    in mxm space with time t to get to adjacent squares
    centered at (0,0)"""
    time = 0
    waypoints = []
    i = 0
    time_diag = t * (1 + (sqrt(2)/2))
    path = transform2d(path, translation=translation/scale, scale=scale)
    for square in path:
        time = t
        if i < len(path)-1: # It should travel slower on diagonals
            next_square = path[i+1]
            if abs(next_square[0] - square[0]) > 0.1 and abs(next_square[1] - square[1]) > 0.1:
                time = time_diag
        waypoints.append([square,time])
        i += 1
    return waypoints

def _d(start,goal):
    "Euclidian distance"
    (x1,y1),(x2,y2) = start,goal
    dx = x2-x1
    dy = y2-y1
    return np.sqrt((dx**2)+(dy**2))

def astar(G, s, ds):
    from math import inf
    from heapq import heappush, heappop

    def _h(n):
        "Heuristic for distance to goal = euclidian"
        return _d(n,ds)

    open_set = []
    heappush(open_set,s)
    came_from = {}
    came_from[s] = None
    n=len(G)
    g_score = np.full((n,n),inf, dtype=np.float32)
    g_score[s] = 0
    f_score = np.full((n,n),inf, dtype=np.float32)
    f_score[s] = _h(s)
    while open_set:
        current = heappop(open_set)
        if current == ds:
            return backtrace(current, came_from)
        for neighbor in G[current]:
            tentative_gscore = g_score[current] + _d(current, neighbor)
            if tentative_gscore < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_gscore
                f_score[neighbor] = g_score[neighbor] + _h(neighbor)
                if neighbor not in open_set:
                    heappush(open_set, neighbor)
    return None

def transform2d(points, translation=np.array([0,0]), scale=1.0, rotation=None):
    if rotation:
        R = np.array([[cos(rotation), -sin(rotation)],
                      [sin(rotation), cos(rotation)]])
        points = R.dot(points.T).T
    points = (points + translation)
    points = (points * scale)
    return points

def get_launch_point(center, r, theta, gamma, gridworld, t, s):
    """Get an open launch space on this arc"""
    r *= s # Scale radius to gridworld size
    checks = int(abs(((r*sin(theta/2))-(r*sin(-theta/2))))+1) # Number of cells to check

    center = transform2d(center, translation=-t, scale=s)
    theta_range = np.arange(gamma-theta/2, gamma+theta/2, theta/checks)
    for theta in theta_range:
        x = int(center[0] + r*cos(theta) + 0.5)
        y = int(center[1] + r*sin(theta) + 0.5)
        if gridworld[x,y] == 0: return (x,y)
    return None