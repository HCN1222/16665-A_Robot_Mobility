#!/usr/bin/env python3
"""
Part A: Pure Pursuit (simulator).

Every time a new pose arrives:
  1. find the waypoint about LOOKAHEAD metres ahead of the car
  2. transform it into the car frame (x forward, y left)
  3. curvature = 2*y / d^2  ->  steering = atan(WHEELBASE * curvature)
  4. publish the drive command (steering clipped to +-MAX_STEER)
"""
import math
import os

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# ----------------------------- parameters ----------------------------------
WAYPOINT_FILE = 'levine.csv'   # file in pure_pursuit/waypoints/, a closed loop
LOOKAHEAD = 1.0                # L (m): how far ahead the target point is
WHEELBASE = 0.33               # distance between front and rear axle (m)
MAX_STEER = 0.4189             # steering limit of the simulated car (rad)
STRAIGHT_SPEED = 3.0           # m/s when driving (almost) straight
CORNER_SPEED = 1.5             # m/s when steering hard
CORNER_STEER = 0.15            # rad, above this we use CORNER_SPEED
SEARCH_WINDOW = 50             # waypoints checked ahead of the last closest one


def load_waypoints(filename):
    """Read the x,y columns of a CSV in pure_pursuit/waypoints/ (skip header)."""
    folder = os.path.join(get_package_share_directory('pure_pursuit'), 'waypoints')
    return np.loadtxt(os.path.join(folder, filename), delimiter=',', skiprows=1)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.waypoints = load_waypoints(WAYPOINT_FILE)   # shape (N, 2), closed loop
        self.closest = None                              # index of closest waypoint

        self.create_subscription(Odometry, '/ego_racecar/odom', self.pose_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.waypoints_pub = self.create_publisher(Marker, '/pure_pursuit/waypoints', 10)
        self.target_pub = self.create_publisher(Marker, '/pure_pursuit/target', 10)
        # re-send the waypoints every second so RViz always shows them
        self.create_timer(1.0, self.publish_waypoints)
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        # 1. waypoint to track
        target = self.find_target(x, y)

        # 2. target in the car frame (rotate the offset by -yaw)
        dx, dy = target[0] - x, target[1] - y
        x_car = math.cos(yaw) * dx + math.sin(yaw) * dy     # ahead of the car
        y_car = -math.sin(yaw) * dx + math.cos(yaw) * dy    # left of the car (+)

        # 3. curvature of the arc through the target, then steering angle
        curvature = 2.0 * y_car / (x_car ** 2 + y_car ** 2)
        steer = math.atan(WHEELBASE * curvature)
        steer = max(-MAX_STEER, min(MAX_STEER, steer))

        # 4. publish (slow down in corners)
        speed = STRAIGHT_SPEED if abs(steer) < CORNER_STEER else CORNER_SPEED
        drive = AckermannDriveStamped()
        drive.drive.steering_angle = steer
        drive.drive.speed = speed
        self.drive_pub.publish(drive)
        self.publish_target(target)

    def find_target(self, x, y):
        """Closest waypoint (searched only a little ahead of the previous one so
        we never jump to another part of the track), then walk forward along the
        loop until a waypoint is at least LOOKAHEAD away from the car."""
        n = len(self.waypoints)
        if self.closest is None:
            candidates = range(n)                                        # first call
        else:
            candidates = [(self.closest + k) % n for k in range(SEARCH_WINDOW)]
        self.closest = min(candidates, key=lambda i: math.dist(self.waypoints[i], (x, y)))

        i = self.closest
        while math.dist(self.waypoints[i], (x, y)) < LOOKAHEAD:
            i = (i + 1) % n                                              # wrap around the loop
        return self.waypoints[i]

    # ------------------------------ RViz ---------------------------------
    def publish_waypoints(self):
        m = make_marker(Marker.POINTS, 0.08, (0.1, 0.4, 1.0))
        m.points = [Point(x=float(px), y=float(py)) for px, py in self.waypoints]
        self.waypoints_pub.publish(m)

    def publish_target(self, target):
        m = make_marker(Marker.SPHERE, 0.25, (1.0, 0.1, 0.1))
        m.pose.position.x, m.pose.position.y = float(target[0]), float(target[1])
        self.target_pub.publish(m)


def make_marker(marker_type, size, rgb):
    m = Marker()
    m.header.frame_id = 'map'
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
        rclpy.spin(PurePursuit())
    except KeyboardInterrupt:   # Ctrl+C
        pass


if __name__ == '__main__':
    main()
