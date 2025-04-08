import rclpy
from rclpy.node import Node

from interfaces.msg import RPYT
from geometry_msgs.msg import Pose





class Simulator(Node):

    def __init__(self):
        super().__init__('Simulator')

        self.start_time = self.get_clock().now().nanoseconds/1000000000

        # Create the publisher for the control signals
        self.position_publisher = self.create_publisher(Pose, 'location', 10)

        # Create the subscriptions for the reference and pose
        self.command_subscription = self.create_subscription(
            RPYT,
            'control_signals',
            self.listener_callback,
            10)
        self.command_subscription 

        self.start_pose = Pose()
        self.start_pose.position.x = 0.0
        self.start_pose.position.y = 0.0
        self.start_pose.position.z = 0.0
        self.start_pose.orientation.x = 0.0
        self.start_pose.orientation.y = 0.0
        self.start_pose.orientation.z = 0.0

        self.position_publisher.publish(self.start_pose)


    def listener_callback(self, msg):

        position = Pose()

        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw
        thrust = msg.thrust

        self.get_logger().info('Roll: "%s"' % roll)
        self.get_logger().info('Pitch: "%s"' % pitch)


        xdd = (thrust * pitch)/ 0.036
        ydd = -(thrust * roll)/ 0.036
        zdd = (thrust/ 0.036) - 9.82

        # Get the current time
        current_time = self.get_clock().now().nanoseconds/1000000000

        # Calculate the time step
        time_step = current_time - self.start_time
        self.start_time = current_time
        # Get the current position
        x = (0.5 * xdd * time_step**2)
        y = (0.5 * ydd * time_step**2)
        z = (0.5 * zdd * time_step**2)

        position.position.x = x
        position.position.y = y
        position.position.z = z
        position.orientation.y = roll
        position.orientation.x = pitch
        position.orientation.z = yaw

        self.position_publisher.publish(position)

        


        
        


def main(args=None):
    rclpy.init(args=args)

    Simulation = Simulator()

    rclpy.spin(Simulation)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()