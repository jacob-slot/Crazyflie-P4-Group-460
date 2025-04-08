import numpy as np

import rclpy
from rclpy.node import Node

from interfaces.msg import PoseRPY
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose




class Controller(Node):

    
    def __init__(self):
        super().__init__('Controller')

        # Initialize the last reference, pose and error variables
        self.last_ref = [0, 0, 0]
        self.last_pose = [0, 0, 0]
        self.position_number = 0
        self.first_ref = False
        self.dt = 0

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
            Pose,
            'location',
            self.listener_callback_pose,
            10)
        self.pose_subscription

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.dt += 1
        

    def listener_callback_ref(self, msg):
        """ Save the last reference """
        self.last_ref[0] = msg.x
        self.last_ref[1] = msg.y
        self.last_ref[2] = msg.z

        #self.get_logger().info('Reference: "%s"' % self.last_ref[0])
        self.first_ref = True

    def listener_callback_pose(self, msg):
        """ Save the last pose and update the command signal """
        self.last_pose[0] = msg.position.x
        self.last_pose[1] = msg.position.y
        self.last_pose[2] = msg.position.z

        #self.get_logger().info('Pose: "%s"' % self.last_pose[0])
        if self.first_ref == True:
            self.update_position()


    def update_position(self):
        """ Update the position of the drone """

        error = [0, 0, 0]

        # Get the last reference and pose
        ref = self.last_ref
        pose = self.last_pose

        # Calculate the error
        for i in range(len(ref)):
            error[i] = ref[i] - pose[i]
        
        #self.get_logger().info('Error: "%s"' % error[0])

        # If within error margin, send the next reference
        if abs(error[0]) < 0.1 and abs(error[1]) < 0.1 and abs(error[2]) < 0.1 and self.dt > 3:
            self.position_number += 1
            msg = Int32()

            # Publish the control signal
            msg.data = self.position_number
            self.get_logger().info('Update: "%s"' % msg.data)
            self.status_publisher.publish(msg)
            self.dt = 0
            




    
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