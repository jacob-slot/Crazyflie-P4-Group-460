import numpy as np

import rclpy
from rclpy.node import Node

from interfaces.msg import RPYT
from interfaces.msg import PoseRPY
from std_msgs.msg import int32




class Controller(Node):

    
    def __init__(self):
        super().__init__('Controller')

        # Initialize the last reference, pose and error variables
        self.last_ref = PoseRPY()
        self.last_pose = PoseRPY()
        self.position_number = int32()
        self.position_number.data = 0

        time = self.get_clock().now().nanoseconds/1000000000

        # Create the publisher for the next reference
        self.status_publisher = self.create_publisher(int32, 'next_ref', 10)

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

        # Get the last reference and pose
        ref = self.last_ref
        pose = self.last_pose

        # Calculate the error
        error = ref - pose

        # If within error margin, send the next reference
        if error[0] < 0.1 and error[1] < 0.1 and error[2] < 0.1 and dt > 3:
            self.position_number += 1
            msg = int32()

            # Publish the control signal
            msg.data = 0
            self.status_publisher.publish(msg)

            # update the time
            dt = self.get_clock().now().nanoseconds/1000000000 - time
            time = self.get_clock().now().nanoseconds/1000000000




    
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