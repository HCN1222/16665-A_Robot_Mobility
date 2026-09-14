#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.scan_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.brake_sub = self.create_subscription(Bool, '/emergency_brake', self.brake_callback, 10)
        self.emergency_brake = False

        # TODO: set PID gains
        self.declare_parameter('kp', 0.9)
        self.declare_parameter('kd', 0.15)
        self.declare_parameter('ki', 0.0)
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.ki = self.get_parameter('ki').value

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0
        self.prev_time = None

        # TODO: store any necessary values you think you'll need
        
        # Levine hallway is 1.7 m wide
        self.declare_parameter('desired_distance', 0.85)
        # predict future position for earlier turn
        self.declare_parameter('lookahead', 0.8)
        # angle between beams a and b
        self.declare_parameter('theta_deg', 45.0)
        self.declare_parameter('max_speed', 1.5)
        # sim servo limit is +/-0.4189 rad ~= +/-24 deg
        self.declare_parameter('max_steering_deg', 24.0)
        self.declare_parameter('integral_limit', 1.0)

        self.desired_distance = self.get_parameter('desired_distance').value
        self.lookahead = self.get_parameter('lookahead').value
        self.theta = np.radians(self.get_parameter('theta_deg').value)
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering = np.radians(self.get_parameter('max_steering_deg').value)
        self.integral_limit = self.get_parameter('integral_limit').value

        self.steering_angle = 0.0
        self.angle_min = 0.0
        self.angle_increment = 1.0
        self.range_min = 0.0
        self.range_max = np.inf

    def brake_callback(self, msg):
        self.emergency_brake = msg.data

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        n = len(range_data)
        idx = int(round((angle - self.angle_min) / self.angle_increment))
        idx = min(max(idx, 0), n - 1)

        for offset in range(0, 10):
            for i in (idx - offset, idx + offset):
                if 0 <= i < n:
                    r = range_data[i]
                    if np.isfinite(r) and self.range_min <= r <= self.range_max:
                        return float(r)
                    
        # if no valid data
        return float(self.range_max)

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        # beam b: perpendicular to the car's x-axis, pointing at the left wall (+90 deg)
        # beam a: theta closer to the front of the car (+90 deg - theta)
        b = self.get_range(range_data, np.pi / 2.0)
        a = self.get_range(range_data, np.pi / 2.0 - self.theta)

        # From Readme ^_^
        alpha = np.arctan2(a * np.cos(self.theta) - b, a * np.sin(self.theta))
        D_t = b * np.cos(alpha)
        D_t1 = D_t + self.lookahead * np.sin(alpha)

        # The README's e = desired - D is written for a right wall
        # For left wll following, we have to flip it
        return D_t1 - dist

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.prev_time is None else now - self.prev_time
        self.prev_time = now

        derivative = 0.0
        if dt > 0.0:
            self.integral += error * dt
            self.integral = float(np.clip(self.integral, -self.integral_limit, self.integral_limit))
            derivative = (error - self.prev_error) / dt
        self.prev_error = error

        angle = self.kp * error + self.ki * self.integral + self.kd * derivative
        angle = float(np.clip(angle, -self.max_steering, self.max_steering))
        self.steering_angle = angle

        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'ego_racecar/base_link'
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = float(velocity)
        self.drive_pub.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        if self.emergency_brake:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.header.frame_id = 'ego_racecar/base_link'
            drive_msg.drive.steering_angle = 0.0
            drive_msg.drive.speed = 0.0
            self.drive_pub.publish(drive_msg)
            return

        # TODO: replace with error calculated by get_error()
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        error = self.get_error(msg.ranges, self.desired_distance)
        self.error = error

        # TODO: calculate desired car velocity based on error
        # step the speed down as the steering angle grows (README speed schedule);
        # use the last commanded angle, which is what the error just produced
        steer_deg = abs(np.degrees(self.steering_angle))
        if steer_deg < 10.0:
            velocity = self.max_speed
        # 10 ~ 20
        elif steer_deg < 20.0:
            velocity = min(self.max_speed, 1.0)
        # > 20
        else:
            velocity = min(self.max_speed, 0.5)

        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
