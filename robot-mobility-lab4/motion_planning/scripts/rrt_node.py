#!/usr/bin/env python3
"""
Part B: RRT / RRT* local planner + Pure Pursuit (simulator).
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

Planning, every PLAN_PERIOD seconds:
  1. build an occupancy grid in the car frame from the latest LaserScan
  2. goal = the waypoint GOAL_DISTANCE ahead of the car on the global route
  3. route up to the goal is free  -> follow the global waypoints
     route is blocked              -> RRT(*) from the car to the goal,
                                      straighten the path, follow it instead
     RRT found nothing             -> stop
Control, on every pose message: Pure Pursuit on the route or on the RRT path.
"""
import math
import os
import random

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from scipy import ndimage
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# ----------------------------- parameters ----------------------------------
WAYPOINT_FILE = 'levine.csv'   # global route (closed loop) in pure_pursuit/waypoints/
WHEELBASE = 0.33               # m
MAX_STEER = 0.4189             # rad
LASER_X = 0.275                # the LiDAR is this far in front of the car origin (m)

# occupancy grid, in the car frame (x forward, y left)
GRID_RES = 0.05                # m per cell
GRID_X_MIN = -0.5              # from 0.5 m behind the car ...
GRID_X_MAX = 5.0               # ... to 5 m in front of it
GRID_HALF_WIDTH = 2.5          # and 2.5 m to each side
INFLATION = 0.22               # grow obstacles by half the car width (0.155) + margin

# goal and RRT
PLAN_PERIOD = 0.1              # s between two plans
GOAL_DISTANCE = 2.5            # m along the route from the car to the goal
MAX_ITER = 300                 # RRT iterations per plan
STEP = 0.3                     # max length of a new tree edge (m)
GOAL_BIAS = 0.1                # probability of sampling the goal itself
GOAL_TOLERANCE = 0.3           # a node this close to the goal may connect to it (m)
USE_RRT_STAR = True            # False = plain RRT
NEAR_RADIUS = 0.6              # RRT*: neighbours considered for parent / rewiring (m)

# Pure Pursuit
ROUTE_LOOKAHEAD = 0.9          # m, when following the global waypoints
ROUTE_SPEED = 2.0              # m/s
PATH_LOOKAHEAD = 0.6           # m, when following an RRT path (shorter to not cut corners)
PATH_SPEED = 1.0               # m/s
SEARCH_WINDOW = 50             # waypoints checked ahead of the last closest one

GRID_WIDTH = int(round((GRID_X_MAX - GRID_X_MIN) / GRID_RES))    # columns (x)
GRID_HEIGHT = int(round(2 * GRID_HALF_WIDTH / GRID_RES))         # rows (y)


# ------------------------------ helpers ------------------------------------
def load_waypoints(filename):
    folder = os.path.join(get_package_share_directory('pure_pursuit'), 'waypoints')
    return np.loadtxt(os.path.join(folder, filename), delimiter=',', skiprows=1)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def to_car_frame(px, py, pose):
    """Map point -> car frame of pose = (x, y, yaw)."""
    x, y, yaw = pose
    dx, dy = px - x, py - y
    return (math.cos(yaw) * dx + math.sin(yaw) * dy,
            -math.sin(yaw) * dx + math.cos(yaw) * dy)


def to_map_frame(px, py, pose):
    """Car frame point -> map frame (inverse of to_car_frame)."""
    x, y, yaw = pose
    return (x + math.cos(yaw) * px - math.sin(yaw) * py,
            y + math.sin(yaw) * px + math.cos(yaw) * py)


def pure_pursuit_steer(target, pose):
    x_car, y_car = to_car_frame(target[0], target[1], pose)
    curvature = 2.0 * y_car / (x_car ** 2 + y_car ** 2)
    return max(-MAX_STEER, min(MAX_STEER, math.atan(WHEELBASE * curvature)))


def disk(radius):
    """Boolean disk of cells, used to inflate the obstacles."""
    n = int(math.ceil(radius / GRID_RES))
    yy, xx = np.mgrid[-n:n + 1, -n:n + 1]
    return (xx ** 2 + yy ** 2) * GRID_RES ** 2 <= radius ** 2


# cells within INFLATION of the car origin (see build_grid)
_cols, _rows = np.meshgrid(np.arange(GRID_WIDTH), np.arange(GRID_HEIGHT))
CAR_AREA = np.hypot(GRID_X_MIN + (_cols + 0.5) * GRID_RES,
                    -GRID_HALF_WIDTH + (_rows + 0.5) * GRID_RES) <= INFLATION


class TreeNode:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent        # index of the parent in the tree list (None = root)


# ------------------------------- node --------------------------------------
class RRT(Node):
    def __init__(self):
        super().__init__('rrt_node')
        self.waypoints = load_waypoints(WAYPOINT_FILE)
        self.closest = None         # index of the waypoint closest to the car
        self.pose = None            # (x, y, yaw) in the map frame
        self.scan = None            # latest LaserScan
        self.mode = 'stop'          # 'route', 'rrt' or 'stop'
        self.path = None            # RRT path in the map frame (list of (x, y))

        self.create_subscription(Odometry, '/ego_racecar/odom', self.pose_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/rrt/grid', 10)
        self.waypoints_pub = self.create_publisher(Marker, '/rrt/waypoints', 10)
        self.goal_pub = self.create_publisher(Marker, '/rrt/goal', 10)
        self.target_pub = self.create_publisher(Marker, '/rrt/target', 10)
        self.tree_pub = self.create_publisher(Marker, '/rrt/tree', 10)
        self.path_pub = self.create_publisher(Marker, '/rrt/path', 10)
        self.create_timer(PLAN_PERIOD, self.plan)
        self.create_timer(1.0, self.publish_waypoints)

    # ---------------------------- callbacks --------------------------------
    def scan_callback(self, msg):
        self.scan = msg

    def pose_callback(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.update_closest()

        if self.mode == 'stop':
            self.publish_drive(0.0, 0.0)
            return
        if self.mode == 'route':
            target = self.route_target(ROUTE_LOOKAHEAD)
            speed = ROUTE_SPEED
        else:
            target = self.path_target(PATH_LOOKAHEAD)
            speed = PATH_SPEED
        self.publish_drive(pure_pursuit_steer(target, self.pose), speed)
        self.publish_sphere(self.target_pub, target, (1.0, 0.1, 0.1))

    # ----------------------------- planning --------------------------------
    def plan(self):
        if self.pose is None or self.scan is None:
            return
        pose = self.pose                     # use one pose for the whole plan
        grid = self.build_grid(self.scan)

        # goal: walk GOAL_DISTANCE along the route; if that point is inside an
        # obstacle, keep walking (at most 1 m) until it is free
        goal_i = self.route_index_ahead(GOAL_DISTANCE)
        goal = to_car_frame(*self.waypoints[goal_i], pose)
        for _ in range(10):
            if self.is_free(grid, goal):
                break
            goal_i = (goal_i + 1) % len(self.waypoints)
            goal = to_car_frame(*self.waypoints[goal_i], pose)
        else:
            self.mode = 'stop'
            self.publish_all(grid, pose, [], None)
            return

        # is the route itself (car -> waypoints -> goal) free?
        route = [(0.0, 0.0)]
        i = self.closest
        while i != goal_i:
            i = (i + 1) % len(self.waypoints)
            route.append(to_car_frame(*self.waypoints[i], pose))
        if all(self.segment_free(grid, a, b) for a, b in zip(route[:-1], route[1:])):
            self.mode = 'route'
            self.path = None
            self.publish_all(grid, pose, [], goal)
            return

        # blocked: plan around the obstacle
        tree, path = self.rrt(grid, goal)
        if path is None:
            self.mode = 'stop'
            self.get_logger().warn('RRT found no path, stopping')
        else:
            self.mode = 'rrt'
            path = self.shortcut(grid, path)
            self.path = [to_map_frame(x, y, pose) for x, y in densify(path, GRID_RES)]
        self.publish_all(grid, pose, tree, goal)

    def build_grid(self, scan):
        """True = blocked. Scan hits are obstacles, everything else is free."""
        ranges = np.array(scan.ranges)
        angles = scan.angle_min + scan.angle_increment * np.arange(len(ranges))
        hit = np.isfinite(ranges) & (ranges < scan.range_max)
        xs = LASER_X + ranges[hit] * np.cos(angles[hit])      # hit points, car frame
        ys = ranges[hit] * np.sin(angles[hit])
        cols = np.floor((xs - GRID_X_MIN) / GRID_RES).astype(int)
        rows = np.floor((ys + GRID_HALF_WIDTH) / GRID_RES).astype(int)
        inside = (cols >= 0) & (cols < GRID_WIDTH) & (rows >= 0) & (rows < GRID_HEIGHT)

        hits = np.zeros((GRID_HEIGHT, GRID_WIDTH), dtype=bool)
        hits[rows[inside], cols[inside]] = True
        grid = ndimage.binary_dilation(hits, disk(INFLATION))
        # the car is standing where it is: remove the inflation (not the real
        # hits) around it, otherwise a car that got a bit too close to an
        # obstacle could never plan a way out
        grid[CAR_AREA] = hits[CAR_AREA]
        return grid

    @staticmethod
    def is_free(grid, point):
        col = math.floor((point[0] - GRID_X_MIN) / GRID_RES)
        row = math.floor((point[1] + GRID_HALF_WIDTH) / GRID_RES)
        if not (0 <= col < GRID_WIDTH and 0 <= row < GRID_HEIGHT):
            return False                 # outside the grid counts as blocked
        return not grid[row, col]

    def segment_free(self, grid, a, b):
        """Check points every half cell along the straight line a -> b."""
        n = int(math.dist(a, b) / (GRID_RES / 2)) + 1
        for k in range(n + 1):
            t = k / n
            if not self.is_free(grid, (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))):
                return False
        return True

    # -------------------------------- RRT ----------------------------------
    def rrt(self, grid, goal):
        """RRT (or RRT*) from the car (0, 0) to goal, both in the car frame.
        Returns (tree, path) with path = list of points, or None."""
        tree = [TreeNode(0.0, 0.0)]
        goal_nodes = []                              # nodes that can reach the goal
        x_max = min(goal[0] + 0.5, GRID_X_MAX)       # only sample in front of the car

        for _ in range(MAX_ITER):
            # sample: the goal itself sometimes, otherwise a random free point
            if random.random() < GOAL_BIAS:
                sample = goal
            else:
                sample = (random.uniform(0.0, x_max),
                          random.uniform(-GRID_HALF_WIDTH, GRID_HALF_WIDTH))
                if not self.is_free(grid, sample):
                    continue

            # nearest node, then steer: move at most STEP towards the sample
            near_i = min(range(len(tree)),
                         key=lambda i: math.dist((tree[i].x, tree[i].y), sample))
            nearest = (tree[near_i].x, tree[near_i].y)
            d = math.dist(nearest, sample)
            if d < 1e-6:
                continue
            k = min(STEP, d) / d
            new = (nearest[0] + k * (sample[0] - nearest[0]),
                   nearest[1] + k * (sample[1] - nearest[1]))
            if not self.segment_free(grid, nearest, new):
                continue

            parent = near_i
            if USE_RRT_STAR:
                neighbours = [i for i, n in enumerate(tree)
                              if math.dist((n.x, n.y), new) < NEAR_RADIUS]
                # choose parent: the neighbour that gives the shortest path to new
                best = cost(tree, near_i) + d * k
                for i in neighbours:
                    c = cost(tree, i) + math.dist((tree[i].x, tree[i].y), new)
                    if c < best and self.segment_free(grid, (tree[i].x, tree[i].y), new):
                        parent, best = i, c
            tree.append(TreeNode(new[0], new[1], parent))
            new_i = len(tree) - 1

            if USE_RRT_STAR:
                # rewire: a neighbour becomes a child of new if that is shorter
                for i in neighbours:
                    if i == 0 or i == parent:
                        continue
                    c = best + math.dist(new, (tree[i].x, tree[i].y))
                    if c < cost(tree, i) and self.segment_free(grid, new, (tree[i].x, tree[i].y)):
                        tree[i].parent = new_i

            # close to the goal and a free straight line to it?
            if math.dist(new, goal) < GOAL_TOLERANCE and self.segment_free(grid, new, goal):
                goal_nodes.append(new_i)
                if not USE_RRT_STAR:
                    break                    # RRT: the first path is good enough
                # RRT*: keep growing the tree until MAX_ITER to improve the path

        if not goal_nodes:
            return tree, None
        best_i = min(goal_nodes, key=lambda i: cost(tree, i) + math.dist((tree[i].x, tree[i].y), goal))
        return tree, find_path(tree, best_i) + [goal]

    def shortcut(self, grid, path):
        """Straighten the zig-zag RRT path: from each point jump directly to
        the farthest later point that can be reached in a free straight line."""
        out, i = [path[0]], 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self.segment_free(grid, path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out

    # --------------------------- route helpers -----------------------------
    def update_closest(self):
        """Closest waypoint, searched only a little ahead of the previous one."""
        n = len(self.waypoints)
        if self.closest is None:
            candidates = range(n)
        else:
            candidates = [(self.closest + k) % n for k in range(SEARCH_WINDOW)]
        self.closest = min(candidates, key=lambda i: math.dist(self.waypoints[i], self.pose[:2]))

    def route_index_ahead(self, distance):
        """Index of the waypoint `distance` metres further along the loop."""
        i, travelled = self.closest, 0.0
        while travelled < distance:
            j = (i + 1) % len(self.waypoints)
            travelled += math.dist(self.waypoints[i], self.waypoints[j])
            i = j
        return i

    def route_target(self, lookahead):
        """First waypoint ahead of the car that is at least `lookahead` away."""
        i = self.closest
        while math.dist(self.waypoints[i], self.pose[:2]) < lookahead:
            i = (i + 1) % len(self.waypoints)
        return self.waypoints[i]

    def path_target(self, lookahead):
        """Same on the RRT path, which ends at the goal (not a loop)."""
        dists = [math.dist(p, self.pose[:2]) for p in self.path]
        i = dists.index(min(dists))
        while i < len(self.path) - 1 and dists[i] < lookahead:
            i += 1
        return self.path[i]

    # -------------------------------- RViz ---------------------------------
    def publish_drive(self, steer, speed):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def publish_all(self, grid, pose, tree, goal):
        # grid and tree are in the car frame -> drawn in the car's TF frame
        g = OccupancyGrid()
        g.header.frame_id = 'ego_racecar/base_link'
        g.info.resolution = GRID_RES
        g.info.width, g.info.height = GRID_WIDTH, GRID_HEIGHT
        g.info.origin.position.x = GRID_X_MIN
        g.info.origin.position.y = -GRID_HALF_WIDTH
        g.info.origin.orientation.w = 1.0
        g.data = (grid.astype(np.int8) * 100).ravel().tolist()
        self.grid_pub.publish(g)

        edges = []
        for n in tree:
            if n.parent is not None:
                edges += [(tree[n.parent].x, tree[n.parent].y), (n.x, n.y)]
        self.publish_lines(self.tree_pub, 'ego_racecar/base_link', Marker.LINE_LIST,
                           edges, 0.01, (0.6, 0.6, 0.6))
        self.publish_lines(self.path_pub, 'map', Marker.LINE_STRIP,
                           self.path if self.mode == 'rrt' else [], 0.05, (1.0, 0.0, 1.0))
        self.publish_sphere(self.goal_pub,
                            None if goal is None else to_map_frame(*goal, pose), (0.1, 0.9, 0.1))

    def publish_waypoints(self):
        self.publish_lines(self.waypoints_pub, 'map', Marker.POINTS,
                           self.waypoints, 0.06, (0.1, 0.4, 1.0))

    def publish_lines(self, pub, frame, marker_type, points, width, rgb):
        m = make_marker(frame, marker_type, width, rgb)
        if len(points) < 2:
            m.action = Marker.DELETE         # nothing to draw: remove the old one
        m.points = [Point(x=float(x), y=float(y)) for x, y in points]
        pub.publish(m)

    def publish_sphere(self, pub, point, rgb):
        m = make_marker('map', Marker.SPHERE, 0.25, rgb)
        if point is None:
            m.action = Marker.DELETE
        else:
            m.pose.position.x, m.pose.position.y = float(point[0]), float(point[1])
        pub.publish(m)


# --------------------------- tree functions --------------------------------
def cost(tree, i):
    """Length of the tree path from the root to node i (walk up the parents)."""
    c = 0.0
    while tree[i].parent is not None:
        p = tree[i].parent
        c += math.dist((tree[i].x, tree[i].y), (tree[p].x, tree[p].y))
        i = p
    return c


def find_path(tree, i):
    """Points from the root to node i."""
    path = []
    while i is not None:
        path.append((tree[i].x, tree[i].y))
        i = tree[i].parent
    return path[::-1]


def densify(path, spacing):
    """Add points every `spacing` metres so Pure Pursuit has smooth targets."""
    out = [path[0]]
    for a, b in zip(path[:-1], path[1:]):
        n = max(1, int(math.dist(a, b) / spacing))
        out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                for k in range(1, n + 1)]
    return out


def make_marker(frame, marker_type, size, rgb):
    m = Marker()
    m.header.frame_id = frame
    m.type = marker_type
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = size
    m.color.r, m.color.g, m.color.b = rgb
    m.color.a = 1.0
    return m


def main(args=None):
    # let Ctrl+C raise a plain KeyboardInterrupt (caught below) instead of
    # rclpy shutting down in the middle of a callback
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        rclpy.spin(RRT())
    except KeyboardInterrupt:   # Ctrl+C
        pass


if __name__ == '__main__':
    main()
