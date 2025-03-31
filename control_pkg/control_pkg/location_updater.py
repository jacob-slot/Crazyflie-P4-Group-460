import numpy as np

import rclpy
from rclpy.node import Node

from interfaces.msg import RPYT
from interfaces.msg import PoseRPY
from std_msgs.msg import Int32




class Controller(Node):

    
    def __init__(self):
        super().__init__('Controller')

        # Initialize the last reference, pose and error variables
        self.last_ref = PoseRPY()
        self.last_pose = PoseRPY()
        self.position_number = 0

        # Create the publisher for the next reference
        self.status_publisher = self.create_publisher(Int32, 'next_ref', 10)

        # Create the subscriptions for the reference and pose
        self.ref_subscription = self.create_subscription(
            PoseRPY,
            'ref_pose',
            self.listener_callback_ref,
            10)
        self.ref_subscription 

        self.pose_subscription = self.create_subscription(
            PoseRPY,
            
            'location',
            self.listener_callback_pose,
            10)
        self.pose_subscription
        

    def listener_callback_ref(self, msg):
        """ Save the last reference """
        self.last_ref = msg

    def listener_callback_pose(self, msg):
        """ Save the last pose and update the command signal """
        self.last_pose = msg
        self.update_position()


    def update_position(self):
        """ Update the position of the drone """

        error = [0, 0, 0]

        # Get the last reference and pose
        ref = [self.last_ref.x, self.last_ref.y, self.last_ref.z]
        pose = [self.last_pose.x, self.last_pose.y, self.last_pose.z]

        # Calculate the error
        for i in range(len(ref)):
            error[i] = ref[i] - pose[i]
        
        self.get_logger().info('Error: "%s"' % error[0])

        # If within error margin, send the next reference
        if abs(error[0]) < 0.1 and abs(error[1]) < 0.1 and abs(error[2]) < 0.1:
            self.position_number += 1
            msg = Int32()

            # Publish the control signal
            msg.data = self.position_number
            self.status_publisher.publish(msg)




    
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