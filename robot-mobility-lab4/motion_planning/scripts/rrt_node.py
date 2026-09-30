#!/usr/bin/env python3
"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

How it runs (simulator):
  scan_callback  every PLAN_PERIOD s : occupancy grid in the car frame
  pose_callback  every pose message  : when a new grid is ready, plan():
      goal = waypoint GOAL_DISTANCE ahead on the global route
      route to the goal free -> follow the global waypoints
      route blocked          -> RRT(*) to the goal (main loop in rrt()),
                                straighten the path, follow it instead
      RRT found nothing      -> stop
    then Pure Pursuit on the route or on the RRT path
"""
import numpy as np
from numpy import linalg as LA
import math

import rclpy
from rclpy.node import Node as ROSNode    # renamed: the tree node class below is also "Node"
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid

# TODO: import as you need
import os
import random
from rclpy.signals import SignalHandlerOptions
from scipy import ndimage
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker

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


# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
    def __init__(self, x=None, y=None, parent=None):
        self.x = x
        self.y = y
        self.parent = parent  # the parent Node object (None for the root)
        self.cost = None # only used in RRT*  (cost() recomputes it from the parents)
        self.is_root = False

# class def for RRT
class RRT(ROSNode):
    def __init__(self):
        super().__init__('rrt_node')
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file
        pose_topic = "ego_racecar/odom"
        scan_topic = "/scan"

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        self.pose_sub_ = self.create_subscription(
            #PoseStamped,
            Odometry,
            pose_topic,
            self.pose_callback,
            1)
        self.pose_sub_

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            1)
        self.scan_sub_

        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/rrt/grid', 10)
        self.waypoints_pub = self.create_publisher(Marker, '/rrt/waypoints', 10)
        self.goal_pub = self.create_publisher(Marker, '/rrt/goal', 10)
        self.target_pub = self.create_publisher(Marker, '/rrt/target', 10)
        self.tree_pub = self.create_publisher(Marker, '/rrt/tree', 10)
        self.path_pub = self.create_publisher(Marker, '/rrt/path', 10)
        self.create_timer(1.0, self.publish_waypoints)

        # class attributes
        # TODO: maybe create your occupancy grid here
        self.grid = None            # occupancy grid (True = blocked), car frame
        self.grid_pose = None       # car pose when that grid was built
        self.grid_time = None       # when that grid was built
        self.new_grid = False       # a grid that has not been planned on yet
        self.waypoints = load_waypoints(WAYPOINT_FILE)
        self.closest = None         # index of the waypoint closest to the car
        self.pose = None            # (x, y, yaw) in the map frame
        self.mode = 'stop'          # 'route', 'rrt' or 'stop'
        self.path = None            # RRT path in the map frame (list of (x, y))
        self.goal = None            # current goal in the car frame (used by sample)
        self.x_max = GRID_X_MAX     # sampling limit in x (used by sample)

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args:
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        if self.pose is None:
            return
        # the simulator sends ~250 scans/s: rebuild the grid only every PLAN_PERIOD
        now = self.get_clock().now()
        if self.grid_time is not None and (now - self.grid_time).nanoseconds < PLAN_PERIOD * 1e9:
            return
        self.grid = self.build_grid(scan_msg)
        self.grid_pose = self.pose          # the grid is in the car frame of this pose
        self.grid_time = now
        self.new_grid = True

    def pose_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args:
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        p = pose_msg.pose.pose              # Odometry in the simulator
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.update_closest()

        # plan once for every new occupancy grid (goal, route check, RRT)
        if self.new_grid:
            self.new_grid = False
            self.plan()

        # Pure Pursuit on the global route or on the RRT path
        if self.mode == 'stop':
            self.publish_drive(0.0, 0.0)
            return None
        if self.mode == 'route':
            target = self.route_target(ROUTE_LOOKAHEAD)
            speed = ROUTE_SPEED
        else:
            target = self.path_target(PATH_LOOKAHEAD)
            speed = PATH_SPEED
        self.publish_drive(pure_pursuit_steer(target, self.pose), speed)
        self.publish_sphere(self.target_pub, target, (1.0, 0.1, 0.1))

        return None

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        # sometimes the goal itself, so the tree grows towards it
        if random.random() < GOAL_BIAS:
            return self.goal
        # otherwise a random free point in front of the car (a few tries)
        for _ in range(100):
            x = random.uniform(0.0, self.x_max)
            y = random.uniform(-GRID_HALF_WIDTH, GRID_HALF_WIDTH)
            if self.is_free(self.grid, (x, y)):
                return (x, y)
        return None

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node = min(range(len(tree)),
                           key=lambda i: math.dist((tree[i].x, tree[i].y), sampled_point))
        return nearest_node

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        dx = sampled_point[0] - nearest_node.x
        dy = sampled_point[1] - nearest_node.y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return None
        k = min(STEP, d) / d                # move at most STEP towards the sample
        new_node = Node(nearest_node.x + k * dx, nearest_node.y + k * dy)
        return new_node

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        return not self.segment_free(self.grid, (nearest_node.x, nearest_node.y),
                                     (new_node.x, new_node.y))

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        # close enough AND a free straight line to the goal (a wall could be in between)
        if math.dist((latest_added_node.x, latest_added_node.y), (goal_x, goal_y)) > GOAL_TOLERANCE:
            return False
        return not self.check_collision(latest_added_node, Node(goal_x, goal_y))

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        node = latest_added_node
        while node is not None:             # walk up the parents to the root
            path.append(node)
            node = node.parent
        path.reverse()                      # root -> goal
        return path



    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        # length of the tree path from the root, recomputed from the parents so
        # it is always correct after rewiring
        c = 0.0
        while node.parent is not None:
            c += self.line_cost(node, node.parent)
            node = node.parent
        return c

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return math.dist((n1.x, n1.y), (n2.x, n2.y))

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = [n for n in tree if self.line_cost(n, node) < NEAR_RADIUS]
        return neighborhood

    # ======================= added: planning ===============================
    def plan(self):
        """Goal on the route, then follow the route or plan around the obstacle."""
        pose = self.grid_pose                # the pose the grid belongs to

        # goal: walk GOAL_DISTANCE along the route; if that point is inside an
        # obstacle, keep walking (at most 1 m) until it is free
        goal_i = self.route_index_ahead(GOAL_DISTANCE)
        goal = to_car_frame(*self.waypoints[goal_i], pose)
        for _ in range(10):
            if self.is_free(self.grid, goal):
                break
            goal_i = (goal_i + 1) % len(self.waypoints)
            goal = to_car_frame(*self.waypoints[goal_i], pose)
        else:
            self.mode = 'stop'
            self.publish_all(pose, [], None)
            return

        # is the route itself (car -> waypoints -> goal) free?
        route = [(0.0, 0.0)]
        i = self.closest
        while i != goal_i:
            i = (i + 1) % len(self.waypoints)
            route.append(to_car_frame(*self.waypoints[i], pose))
        if all(self.segment_free(self.grid, a, b) for a, b in zip(route[:-1], route[1:])):
            self.mode = 'route'
            self.path = None
            self.publish_all(pose, [], goal)
            return

        # blocked: plan around the obstacle
        tree, path = self.rrt(goal)
        if path is None:
            self.mode = 'stop'
            self.get_logger().warn('RRT found no path, stopping')
        else:
            self.mode = 'rrt'
            path = self.shortcut(path)
            self.path = [to_map_frame(x, y, pose) for x, y in densify(path, GRID_RES)]
        self.publish_all(pose, tree, goal)

    def rrt(self, goal):
        """The main RRT (or RRT*) loop, from the car (0, 0) to goal, both in the
        car frame. Returns (tree, path) with path = list of (x, y), or None."""
        self.goal = goal
        self.x_max = min(goal[0] + 0.5, GRID_X_MAX)  # only sample in front of the car
        root = Node(0.0, 0.0)
        root.is_root = True
        tree = [root]
        goal_nodes = []                              # nodes that can reach the goal

        for _ in range(MAX_ITER):
            sampled_point = self.sample()
            if sampled_point is None:
                continue
            nearest_node = tree[self.nearest(tree, sampled_point)]
            new_node = self.steer(nearest_node, sampled_point)
            if new_node is None or self.check_collision(nearest_node, new_node):
                continue
            new_node.parent = nearest_node

            if USE_RRT_STAR:
                neighbours = self.near(tree, new_node)
                # choose parent: the neighbour that gives the shortest path to new_node
                best = self.cost(tree, nearest_node) + self.line_cost(nearest_node, new_node)
                for n in neighbours:
                    c = self.cost(tree, n) + self.line_cost(n, new_node)
                    if c < best and not self.check_collision(n, new_node):
                        new_node.parent, best = n, c
            tree.append(new_node)

            if USE_RRT_STAR:
                # rewire: a neighbour becomes a child of new_node if that is shorter
                for n in neighbours:
                    if n.is_root or n is new_node.parent:
                        continue
                    if best + self.line_cost(new_node, n) < self.cost(tree, n) \
                            and not self.check_collision(new_node, n):
                        n.parent = new_node

            if self.is_goal(new_node, goal[0], goal[1]):
                goal_nodes.append(new_node)
                if not USE_RRT_STAR:
                    break                    # RRT: the first path is good enough
                # RRT*: keep growing the tree until MAX_ITER to improve the path

        if not goal_nodes:
            return tree, None
        best_node = min(goal_nodes, key=lambda n: self.cost(tree, n)
                        + math.dist((n.x, n.y), goal))
        goal_node = Node(goal[0], goal[1], parent=best_node)
        return tree, [(n.x, n.y) for n in self.find_path(tree, goal_node)]

    def shortcut(self, path):
        """Straighten the zig-zag RRT path: from each point jump directly to
        the farthest later point that can be reached in a free straight line."""
        out, i = [path[0]], 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self.segment_free(self.grid, path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out

    # ======================= added: occupancy grid =========================
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

    # ======================= added: route helpers ==========================
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

    # ======================= added: publishing =============================
    def publish_drive(self, steer, speed):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def publish_all(self, pose, tree, goal):
        # grid and tree are in the car frame -> drawn in the car's TF frame
        g = OccupancyGrid()
        g.header.frame_id = 'ego_racecar/base_link'
        g.info.resolution = GRID_RES
        g.info.width, g.info.height = GRID_WIDTH, GRID_HEIGHT
        g.info.origin.position.x = GRID_X_MIN
        g.info.origin.position.y = -GRID_HALF_WIDTH
        g.info.origin.orientation.w = 1.0
        g.data = (self.grid.astype(np.int8) * 100).ravel().tolist()
        self.grid_pub.publish(g)

        edges = []
        for n in tree:
            if n.parent is not None:
                edges += [(n.parent.x, n.parent.y), (n.x, n.y)]
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

def main(args=None):
    # let Ctrl+C raise a plain KeyboardInterrupt (caught below) instead of
    # rclpy shutting down in the middle of a callback
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    print("RRT Initialized")
    rrt_node = RRT()
    try:
        rclpy.spin(rrt_node)
    except KeyboardInterrupt:   # Ctrl+C
        pass

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
