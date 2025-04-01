import rclpy
from rclpy.node import Node

from interfaces.msg import RPYT
from interfaces.msg import PoseRPY


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseRPY, 'location', 10)
        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = PoseRPY()
        msg.x = float(5-self.i)
        msg.y = float(5-self.i)
        msg.z = float(5-self.i)
        self.publisher_.publish(msg)
        
        self.get_logger().info('Publishing location:')
        self.get_logger().info('Publishing: "%s"' % msg.x)
        self.get_logger().info('Publishing: "%s"' % msg.y)
        self.get_logger().info('Publishing: "%s"' % msg.z)
        
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()