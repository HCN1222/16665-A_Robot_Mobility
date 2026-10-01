#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
# TODO CHECK: include needed ROS msg type headers and libraries
import math
import os
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
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


def make_marker(marker_type, size, rgb):
    m = Marker()
    m.header.frame_id = 'map'
    m.type = marker_type
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = size
    m.color.r, m.color.g, m.color.b = rgb
    m.color.a = 1.0
    return m


class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers
        self.create_subscription(Odometry, '/ego_racecar/odom', self.pose_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.waypoints_pub = self.create_publisher(Marker, '/pure_pursuit/waypoints', 10)
        self.target_pub = self.create_publisher(Marker, '/pure_pursuit/target', 10)
        # re-send the waypoints every second so RViz always shows them
        self.create_timer(1.0, self.publish_waypoints)

        self.waypoints = load_waypoints(WAYPOINT_FILE)   # shape (N, 2), closed loop
        self.closest = None                              # index of closest waypoint
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')

    def pose_callback(self, pose_msg):
        x = pose_msg.pose.pose.position.x                # Odometry in the simulator
        y = pose_msg.pose.pose.position.y
        yaw = yaw_from_quaternion(pose_msg.pose.pose.orientation)

        # TODO: find the current waypoint to track using methods mentioned in lecture
        target = self.find_target(x, y)

        # TODO: transform goal point to vehicle frame of reference
        dx, dy = target[0] - x, target[1] - y
        x_car = math.cos(yaw) * dx + math.sin(yaw) * dy     # ahead of the car
        y_car = -math.sin(yaw) * dx + math.cos(yaw) * dy    # left of the car (+)

        # TODO: calculate curvature/steering angle
        curvature = 2.0 * y_car / (x_car ** 2 + y_car ** 2)
        steer = math.atan(WHEELBASE * curvature)

        # TODO: publish drive message, don't forget to limit the steering angle.
        steer = max(-MAX_STEER, min(MAX_STEER, steer))
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

    def publish_waypoints(self):
        m = make_marker(Marker.POINTS, 0.08, (0.1, 0.4, 1.0))
        m.points = [Point(x=float(px), y=float(py)) for px, py in self.waypoints]
        self.waypoints_pub.publish(m)

    def publish_target(self, target):
        m = make_marker(Marker.SPHERE, 0.25, (1.0, 0.1, 0.1))
        m.pose.position.x, m.pose.position.y = float(target[0]), float(target[1])
        self.target_pub.publish(m)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    try:
        rclpy.spin(pure_pursuit_node)
    except KeyboardInterrupt:
        pass

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
