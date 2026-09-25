#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from rclpy.qos import qos_profile_sensor_data

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: Subscribe to LIDAR
        # TODO: Publish to drive
        self.scan_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, qos_profile_sensor_data)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.proc_scan_msg = None
        self.preprocessed_scan_pub = self.create_publisher(LaserScan, '/processed_scan', 10)

    def publish_preprocessed_scan(self, ranges, angle_min, angle_max, angle_increment, frame_id=''):
        """Publish the preprocessed LiDAR scan for visualization/debugging."""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.angle_min = float(angle_min)
        msg.angle_max = float(angle_max)
        msg.angle_increment = float(angle_increment)
        msg.range_min = 0.0
        msg.range_max = 30.0
        msg.ranges = [float(r) for r in ranges]

        self.preprocessed_scan_pub.publish(msg)

    def publish_drive(self, steering=0.0, speed=0.0):
        """Publish steering in radians and speed in meters per second."""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)

        self.drive_pub.publish(msg)

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        values = np.array(ranges, dtype=float, copy=True)
        n = len(values)

        max_distance = 3.0
        LIDAR_RANGE_MAX = 30.0
        LIDAR_RANGE_MIN = 0.0

        # deal with invalid values
        invalid = (
            np.isnan(values)
            | (values < LIDAR_RANGE_MIN)
            | (np.isfinite(values) & (values > LIDAR_RANGE_MAX))
        )
        values[invalid] = 0.0
        values = np.clip(values, 0.0, max_distance)

        # prefix[k] = sum(values[:k])
        prefix = np.zeros(n + 1, dtype=float)
        prefix[1:] = np.cumsum(values)

        # Window [left, right): left neighbor, self, right neighbor.
        indices = np.arange(n)
        left = np.maximum(indices - 2, 0)
        right = np.minimum(indices + 3, n)

        window_sums = prefix[right] - prefix[left]
        window_sizes = right - left
        proc_ranges = window_sums / window_sizes

        # leave invalid positions out of smoothing
        proc_ranges[invalid] = 0.0

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges"""
        best_start = 0
        best_end = 0
        current_start = 0

        for i, distance in enumerate(free_space_ranges):
            if distance <= 0.0:
                current_start = i + 1
            elif i + 1 - current_start > best_end - best_start:
                best_start = current_start
                best_end = i + 1

        return best_start, best_end
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        if start_i == end_i:
            return start_i

        gap = np.asarray(ranges[start_i:end_i])
        candidates = np.flatnonzero(gap == np.max(gap))

        # Break ties by choosing the candidate closest to the gap center.
        center = (len(gap) - 1) / 2.0
        best_local = candidates[np.argmin(np.abs(candidates - center))]

        return start_i + int(best_local)

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message"""
        ranges = data.ranges

        proc_ranges = self.preprocess_lidar(ranges)
        self.proc_scan_msg = proc_ranges
        self.publish_preprocessed_scan(
            proc_ranges, data.angle_min, data.angle_max, data.angle_increment, data.header.frame_id
        )
        # TODO:

        #filter out the back of the car
        angles = ( data.angle_min + np.arange( len( proc_ranges ) ) * data.angle_increment )

        front_mask = np.abs(angles) <= np.deg2rad(70.0)
        proc_ranges[~front_mask] = 0.0


        #Find closest point to LiDAR
        valid_indices = np.flatnonzero( proc_ranges > 0.0 )
        closest_index = valid_indices[ np.argmin( proc_ranges[ valid_indices ] ) ]
        closest_distance = proc_ranges[ closest_index ]

        # Expand the nearest obstacle into an angular safety bubble.
        bubble_radius = 0.25
        angle_step = abs( data.angle_increment )
        if angle_step == 0.0:
            return

        # Eliminate all points inside 'bubble' (set them to zero) 
        if closest_distance <= bubble_radius:
            proc_ranges[:] = 0.0
        else:
            bubble_angle = np.arcsin( bubble_radius / closest_distance )
            bubble_size = int( np.ceil( bubble_angle / angle_step ) )

            start = max( 0, closest_index - bubble_size )
            end = min( len( proc_ranges ), closest_index + bubble_size + 1 )
            proc_ranges[ start:end ] = 0.0

        #Find max length gap 
        start_i, end_i = self.find_max_gap( proc_ranges )

        #Find the best point in the gap 
        best_index = self.find_best_point( start_i, end_i, proc_ranges )

        #Publish Drive message
        
        # Calculate each beam's angle while preserving original indices.
        target_angle = angles[best_index]
        steering = target_angle
        steering = float(np.clip(target_angle, -0.4, 0.4))

        # Slow down for larger steering commands.
        speed = 0.5 if abs(steering) > 0.2 else 1.0

        self.publish_drive(steering, speed)

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()