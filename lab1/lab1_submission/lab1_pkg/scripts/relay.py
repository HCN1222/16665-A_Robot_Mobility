#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class Relay(Node):
    def __init__(self):
        super().__init__('relay')

        self.publisher_ = self.create_publisher(
            msg_type=AckermannDriveStamped,
            topic='drive_relay',
            qos_profile=10
        )
        self.subscription = self.create_subscription(
            msg_type=AckermannDriveStamped,
            topic='drive',
            callback=self.listener_callback,
            qos_profile=10
        )

    def listener_callback(self, msg):
        new_msg = AckermannDriveStamped()
        new_msg.drive.speed = msg.drive.speed * 3
        new_msg.drive.steering_angle = msg.drive.steering_angle * 3

        self.publisher_.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Relay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
