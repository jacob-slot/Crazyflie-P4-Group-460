import rclpy
from rclpy.node import Node

from interfaces.msg import RPYT
from interfaces.msg import PoseRPY


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('ref')
        self.publisher_ = self.create_publisher(PoseRPY, 'ref_pose', 10)
        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = PoseRPY()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing ref:')
        self.get_logger().info('Publishing: "%s"' % msg.x)
        self.get_logger().info('Publishing: "%s"' % msg.y)
        self.get_logger().info('Publishing: "%s"' % msg.z)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    ref = MinimalPublisher()

    rclpy.spin(ref)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ref.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()