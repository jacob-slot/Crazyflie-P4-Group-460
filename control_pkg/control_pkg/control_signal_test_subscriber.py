import rclpy
from rclpy.node import Node

from interfaces.msg import RPYT


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            RPYT,
            'control_signals',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):

        self.get_logger().info('I heard roll: "%s"' % msg.roll)
        """
        self.get_logger().info('I heard pitch: "%s"' % msg.pitch)
        self.get_logger().info('I heard yaw: "%s"' % msg.yaw)
        self.get_logger().info('I heard thrust: "%s"' % msg.thrust)
        """

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()