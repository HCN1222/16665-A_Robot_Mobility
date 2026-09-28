#!/usr/bin/env python3
"""
Waypoint logger (simulator).

Drive one full lap with the keyboard and stop close to where you started:
the route is used as a closed loop, so the last point connects back to the
first one. A point is saved every MIN_DISTANCE metres; the CSV is rewritten
after every point, so Ctrl+C at any time keeps what was recorded.

  ros2 run pure_pursuit waypoint_logger.py
"""
import csv
import math
import os

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

OUTPUT_FILE = 'waypoints.csv'   # written in the directory you run the command from
MIN_DISTANCE = 0.1              # m between saved points


class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        self.waypoints = []
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/waypoint_logger/path', 10)
        self.get_logger().info(f'Saving to {os.path.abspath(OUTPUT_FILE)} - drive the car!')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.waypoints and math.dist(self.waypoints[-1], (x, y)) < MIN_DISTANCE:
            return
        self.waypoints.append((x, y))
        self.get_logger().info(f'Logged waypoint {len(self.waypoints)}: ({x:.2f}, {y:.2f})')

        with open(OUTPUT_FILE, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            writer.writerows(self.waypoints)

        # show everything recorded so far in RViz
        m = Marker()
        m.header.frame_id = 'map'
        m.type = Marker.POINTS
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.08
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 1.0
        m.points = [Point(x=px, y=py) for px, py in self.waypoints]
        self.marker_pub.publish(m)


def main(args=None):
    # let Ctrl+C raise a plain KeyboardInterrupt (caught below) instead of
    # rclpy shutting down in the middle of a callback
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        rclpy.spin(WaypointLogger())
    except KeyboardInterrupt:   # Ctrl+C
        pass


if __name__ == '__main__':
    main()
