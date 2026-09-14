#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

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

        # brake_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.brake_pub = self.create_publisher(Bool, '/emergency_brake', 1)

        self.yaw_rate = 0.
        # self.braking = False

        self.declare_parameter('ttc_threshold', 1.0)
        self.declare_parameter('fov_half_angle_deg', 45.0)
        self.declare_parameter('use_lateral_velocity', True)
        # Assumption from Readme
        # a kinematic bicycle model with no lateral slip at the tires,
        # the Centre of Mass (CoM) is at the same position as the lidar sensor mount,
        # l_r = 0.275 and l_f = 0.055 (refer to launch/ego_racecar.xacro)
        self.declare_parameter('l_r', 0.275)
        self.declare_parameter('l_f', 0.055)
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

        # yaw rate omega = v*tan(delta)/(l_f+l_r)
        # v_y = omega*l_r = v*tan(delta)*l_r/(l_f+l_r).
        v_x = self.speed
        v_y = self.yaw_rate * self.get_parameter('l_r').value \
            if self.get_parameter('use_lateral_velocity').value else 0.0

        range_rate = -(v_x * np.cos(angles) + v_y * np.sin(angles))
        closing = np.maximum(-range_rate, 0.0)   # {-rdot}_+

        # drop inf/nan and out-of-range beams, beams outside the cone, and beams we are
        # moving away from (closing == 0 -> iTTC = inf)
        in_cone = ( np.abs(angles) <= half_fov ) if v_x >= 0.0 \
            else np.abs(angles) >= np.pi - half_fov

        valid = (np.isfinite(ranges)
                 & (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max)
                 & in_cone & (closing > 1e-3))

        ttc = np.full(ranges.shape, np.inf)
        ttc[valid] = ranges[valid] / closing[valid]
        i_min = int(np.argmin(ttc))
        min_ttc = ttc[i_min]

        min_stop_distance = self.get_parameter('min_stop_distance').value
        in_cone_ranges = ranges[np.isfinite(ranges) & (ranges >= scan_msg.range_min)
                                 & (ranges <= scan_msg.range_max) & in_cone]
        too_close = in_cone_ranges.size > 0 and in_cone_ranges.min() < min_stop_distance

        # TODO: publish command to brake
        if min_ttc < ttc_threshold or too_close:
            # if not self.braking:
            #     self.get_logger().warn(
            #         'EMERGENCY BRAKE: iTTC=%.2fs at %.0f deg (range %.2f m, v_x=%.2f, v_y=%.2f)'
            #         % (min_ttc, np.degrees(angles[i_min]), ranges[i_min], v_x, v_y))
            # self.braking = True
            self.brake_pub.publish(Bool(data=True))
            brake_msg = AckermannDriveStamped()
            brake_msg.header.stamp = self.get_clock().now().to_msg()
            brake_msg.drive.speed = 0.0
            brake_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(brake_msg)
        else:
            # if self.braking:
            self.brake_pub.publish(Bool(data=False))
            # self.braking = False

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
