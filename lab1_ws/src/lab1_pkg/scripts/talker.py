#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        self.declare_parameter(name='v', value=0.0)
        self.declare_parameter(name='d', value=0.0)

        self.publisher_ = self.create_publisher(
            msg_type=AckermannDriveStamped,
            topic='drive',
            qos_profile=10
        )

        self.timer = self.create_timer(
            timer_period_sec=0.0,
            callback=self.timer_callback
        )

    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.get_parameter('v').get_parameter_value().double_value
        msg.drive.steering_angle = self.get_parameter('d').get_parameter_value().double_value

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
