import numpy as np

import rclpy
from rclpy.node import Node

from interfaces.msg import RPYT
from interfaces.msg import PoseRPY
from geometry_msgs.msg import Pose




class Controller(Node):

    

    def __init__(self):
        super().__init__('Controller')


        # Initialize the last reference, pose and error variables
        self.last_ref = PoseRPY()
        self.last_pose = PoseRPY()
        self.last_error = [0, 0, 0]
        self.first_ref = False

        # Initialize the integral term for the PID controller
        self.start_time = self.get_clock().now().nanoseconds/1000000000
        self.integral = [0, 0, 0]

        # Create the publisher for the control signals
        self.control_publisher = self.create_publisher(RPYT, 'control_signals', 10)

        # Create the subscriptions for the reference and pose
        self.ref_subscription = self.create_subscription(
            PoseRPY,
            'ref_pose',
            self.listener_callback_ref,
            10)
        self.ref_subscription 

        self.pose_subscription = self.create_subscription(
            Pose,
            'location',
            self.listener_callback_pose,
            10)
        self.pose_subscription

        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        


    def timer_callback(self):
        print("Timer callback")

    def listener_callback_ref(self, msg):
        """ Save the last reference """
        self.last_ref = msg
        self.first_ref = True

    def listener_callback_pose(self, msg):
        """ Save the last pose and update the command signal """
        self.last_pose = msg
        if self.first_ref == True:
            self.pos_to_rpy()


    def Yaw_transform(self):
        """
        Transforms the reference and pose vectors to the yaw frame
        """

        #Save the XYZ vector of the reference 
        ref_vector = [self.last_ref.x, self.last_ref.y, self.last_ref.z]

        #Create the yaw matrix
        yaw_matrix = np.array([[np.cos(self.last_pose.orientation.z), -np.sin(self.last_pose.orientation.z), 0], [np.sin(self.last_pose.orientation.z), np.cos(self.last_pose.orientation.z), 0], [0, 0, 1]])
        
        #Transform the reference vector to the yaw frame
        new_ref = np.dot(yaw_matrix, ref_vector)

        return new_ref
        

    def PID(self):
        """
        PID controller
        """

        #Initialize the control signal and the PID gains
        control_signal = [0, 0, 0]
        Kp = [ 1, 1, 1 ]
        Ki = [ 1, 1, 1 ]
        Kd = [ 1, 1, 1 ]

        #Calculate the time difference
        time = self.get_clock().now().nanoseconds/1000000000
        dt = time - self.start_time
        self.start_time = time

        #Transform the reference and pose vectors to the yaw frame
        new_ref = self.Yaw_transform()
        
        #Calculate the error
        error = new_ref - [self.last_pose.position.x, self.last_pose.position.y, self.last_pose.position.z]
        self.get_logger().info('Error: "%s"' % error)

        #Calculate the integral term
        self.integral += error*dt

        #Calculate the control signals for roll, pitch and thrust
        for i in [1,0,2]:
            control_signal[i] = Kp[i]*error[i] + Ki[i]*self.integral[i] + Kd[i]*(error[i] - self.last_error[i])/dt

        self.last_error = error

        return control_signal
    

    def pos_to_rpy(self):
        """
        Converts the position to roll, pitch and yaw, applies the PID controller and publishes the control signals
        """

        #Calculate the control signals
        control_signal = self.PID()

        #Create the control signal message
        msg = RPYT()
        msg.roll = control_signal[0]
        msg.pitch = control_signal[1]
        msg.thrust = control_signal[2]

        #Publish the control signals
        self.control_publisher.publish(msg)

        #Print the control signals
        self.get_logger().info('Publishing roll: "%s"' % msg.roll)
        self.get_logger().info('Publishing pitch: "%s"' % msg.pitch)
        self.get_logger().info('Publishing yaw: "%s"' % msg.yaw)
        self.get_logger().info('Publishing thrust: "%s"' % msg.thrust)
        


        
        


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()