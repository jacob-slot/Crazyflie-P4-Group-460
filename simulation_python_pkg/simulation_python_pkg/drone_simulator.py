import rclpy
from rclpy.node import Node

from interfaces.msg import RPYT
from interfaces.msg import PoseRPY
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import time



class Simulator(Node):

    def __init__(self):
        super().__init__('Simulator')

        

        # Create the publisher for the control signals
        self.position_publisher = self.create_publisher(PoseRPY, 'location', 10)

        self.plot_publisher = self.create_publisher(Pose,'plot',10)

        self.ready_publisher = self.create_publisher(Bool,'ready',10)

        # Create the subscriptions for the reference and pose
        self.command_subscription = self.create_subscription(
            RPYT,
            'control_signals',
            self.listener_callback,
            10)
        self.command_subscription 

        self.start_pose = PoseRPY()
        self.start_pose.x = 0.0
        self.start_pose.y = 0.0
        self.start_pose.z = 0.0
        self.start_pose.x_vel = 0.0
        self.start_pose.y_vel = 0.0
        self.start_pose.z_vel = 0.0
        self.start_pose.roll = 0.0
        self.start_pose.pitch = 0.0
        self.start_pose.yaw = 0.0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.xd = 0.0
        self.yd = 0.0
        self.zd = 0.0


        
        self.ready_publisher.publish(Bool(data=True))
        time.sleep(3)
        self.start_time = self.get_clock().now().nanoseconds/1000000000
        self.position_publisher.publish(self.start_pose)


    def listener_callback(self, msg):


        position = PoseRPY()
        plotd = Pose()

        g = 9.82
        m = 0.036

        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw
        thrust = msg.thrust

        xdd = (g * pitch)
        ydd = (g * roll)
        zdd = (thrust / m) - g

        # Get the current time
        current_time = self.get_clock().now().nanoseconds/1000000000

        # Calculate the time step
        time_step = current_time - self.start_time
        self.start_time = current_time
        # Get the current position  
        self.x += (0.5 * xdd * time_step**2)
        self.y += (0.5 * ydd * time_step**2)
        self.z += (0.5 * zdd * time_step**2)
        self.xd += (1.0 * xdd * time_step**1)
        self.yd += (1.0 * ydd * time_step**1)
        self.zd += (1.0 * zdd * time_step**1)

        position.x = self.x
        position.y = self.y
        position.z = self.z
        position.x_vel = self.xd
        position.y_vel = self.yd
        position.z_vel = self.zd
        position.roll = roll
        position.pitch = pitch
        position.yaw = yaw

        plotd.position.x = self.x
        plotd.position.y = self.y
        plotd.position.z = self.z
        plotd.orientation.x = roll
        plotd.orientation.y = pitch
        plotd.orientation.z = yaw

        self.get_logger().info('Publishing x: "%s"' % position.x)
        self.get_logger().info('Publishing y: "%s"' % position.y)
        self.get_logger().info('Publishing z: "%s"' % position.z)

        self.position_publisher.publish(position)
        self.plot_publisher.publish(plotd)
        time.sleep(0.1)

        


        
        


def main(args=None):
    rclpy.init(args=args)

    Simulation = Simulator()

    rclpy.spin(Simulation)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Simulation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()