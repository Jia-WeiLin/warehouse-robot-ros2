import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped


class TwistConverter(Node):

    def __init__(self):
        super().__init__("twist_converter")
        self.publisher =self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
)
        self.subscription =self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
)
    def cmd_vel_callback(self, msg):
        new_msg = TwistStamped()

        new_msg.header.stamp = self.get_clock().now().to_msg()

        new_msg.twist.linear = msg.linear
        new_msg.twist.angular = msg.angular

        self.publisher.publish(new_msg)
def main(args=None):
    rclpy.init(args=args)

    node = TwistConverter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()