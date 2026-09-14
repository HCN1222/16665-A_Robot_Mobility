#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        # TODO: create ROS subscribers and publishers.
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        self.yaw_rate = 0.     # yaw rate [rad/s] from odom, used for the lateral velocity (Deliverable 4)
        self.braking = False   # so the brake warning is logged once per event

        # Tunable parameters: override with `--ros-args -p ttc_threshold:=0.8` etc.
        # brake when the smallest iTTC in the cone drops below this many seconds
        self.declare_parameter('ttc_threshold', 1.0)
        # only beams within +/- this angle of the direction of travel are considered:
        # the projection v*cos(theta) assumes a stationary point on every beam, so a beam
        # hitting a parallel wall obliquely produces a spurious small iTTC; the cone keeps
        # the node from false-positive braking while driving down a hallway
        self.declare_parameter('fov_half_angle_deg', 45.0)
        # Deliverable 4: project the complete (v_x, v_y) velocity vector onto each beam
        self.declare_parameter('use_lateral_velocity', True)
        # kinematic bicycle geometry, CoM == lidar mount (see launch/ego_racecar.xacro)
        self.declare_parameter('l_r', 0.275)
        self.declare_parameter('l_f', 0.055)
        # keep braking whenever any beam in the cone is closer than this, even if the
        # computed iTTC looks safe (e.g. once stopped, closing speed is 0 -> iTTC = inf,
        # but a still-held throttle command would otherwise let the car creep back in)
        self.declare_parameter('min_stop_distance', 0.4)

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x
        self.yaw_rate = odom_msg.twist.twist.angular.z

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        ttc_threshold = self.get_parameter('ttc_threshold').value
        half_fov = np.radians(self.get_parameter('fov_half_angle_deg').value)

        ranges = np.asarray(scan_msg.ranges, dtype=np.float64)
        angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment

        # Velocity of the lidar/CoM in the body frame. For a no-slip kinematic bicycle with
        # rear-axle speed v and steering angle delta, yaw rate omega = v*tan(delta)/(l_f+l_r)
        # and the lateral velocity of a point l_r ahead of the rear axle is
        #   v_y = omega*l_r = v*tan(delta)*l_r/(l_f+l_r).
        # Odometry already reports omega, so it is used directly instead of delta.
        v_x = self.speed
        v_y = self.yaw_rate * self.get_parameter('l_r').value \
            if self.get_parameter('use_lateral_velocity').value else 0.0

        # range rate = projection of the velocity vector onto each beam, negated:
        # a beam pointing where we are heading has a shrinking range (negative rate)
        range_rate = -(v_x * np.cos(angles) + v_y * np.sin(angles))
        closing = np.maximum(-range_rate, 0.0)   # {-rdot}_+

        # drop inf/nan and out-of-range beams, beams outside the cone, and beams we are
        # moving away from (closing == 0 -> iTTC = inf)
        if v_x >= 0.0:
            in_cone = np.abs(angles) <= half_fov
        else:  # reversing: look at the rear-facing beams instead
            in_cone = np.abs(angles) >= np.pi - half_fov
        valid = (np.isfinite(ranges)
                 & (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max)
                 & in_cone & (closing > 1e-3))

        ttc = np.full(ranges.shape, np.inf)
        ttc[valid] = ranges[valid] / closing[valid]
        i_min = int(np.argmin(ttc))
        min_ttc = ttc[i_min]

        # proximity floor: an obstacle already inside this range in the cone keeps the
        # brake on regardless of iTTC (see min_stop_distance above)
        min_stop_distance = self.get_parameter('min_stop_distance').value
        in_cone_ranges = ranges[np.isfinite(ranges) & (ranges >= scan_msg.range_min)
                                 & (ranges <= scan_msg.range_max) & in_cone]
        too_close = in_cone_ranges.size > 0 and in_cone_ranges.min() < min_stop_distance

        # TODO: publish command to brake
        if min_ttc < ttc_threshold or too_close:
            if not self.braking:
                self.get_logger().warn(
                    'EMERGENCY BRAKE: iTTC=%.2fs at %.0f deg (range %.2f m, v_x=%.2f, v_y=%.2f)'
                    % (min_ttc, np.degrees(angles[i_min]), ranges[i_min], v_x, v_y))
                self.braking = True
            brake_msg = AckermannDriveStamped()
            brake_msg.header.stamp = self.get_clock().now().to_msg()
            brake_msg.drive.speed = 0.0
            brake_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(brake_msg)
        else:
            # publish nothing while safe so we never fight teleop / the wall follower
            self.braking = False

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
