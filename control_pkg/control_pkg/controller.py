import numpy as np

import rclpy
from rclpy.node import Node

from interfaces.msg import RPYT
from interfaces.msg import PoseRPY
from std_msgs.msg import Bool





class Controller(Node):

    

    def __init__(self):
        super().__init__('Controller')


        # Initialize the last reference, pose and error variables
        self.last_ref = PoseRPY()
        self.last_pose = PoseRPY()
        self.last_error = [0, 0, 0]
        self.last_vel_error = [0, 0, 0]
        self.first_ref = False
        self.ready = False

        # Initialize the integral term for the PID controller
        self.start_time = 0
        self.integral = [0, 0, 0]
        self.vel_integral = [0, 0, 0]

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
            PoseRPY,
            'vrpn/Crazyflie/pose_rpy',
            self.listener_callback_pose,
            10)
        self.pose_subscription

        self.ready_subscription = self.create_subscription(
            Bool,
            'ready',
            self.listener_callback_ready,
            10)
        self.ready_subscription

        self.land_subscription = self.create_subscription(
            Bool,
            'land',
            self.listener_callback_land,
            10)
        self.land_subscription

    def listener_callback_land(self, msg):
        if msg == True:
            raise SystemExit('Landing now.')

    def listener_callback_ready(self, msg):
        self.ready = msg.data
        self.start_time = self.get_clock().now().nanoseconds/1000000000


    def listener_callback_ref(self, msg):
        """ Save the last reference """
        self.last_ref = msg
        self.first_ref = True

    def listener_callback_pose(self, msg):
        """ Save the last pose and update the command signal """
        self.last_pose = msg
        if self.first_ref == True and self.ready == True:
            self.pos_to_rpy()


    def PID(self):
        """
        PID controller
        """

        #Initialize the control signal and the PID gains
        vel_ref = [0, 0 ,0]
        control_signal = [0, 0, 0]

        #PID gains x y z x_vel y_vel z_vel
        Kp = [ 4.0, 4.0, 3.0, 3.0, 3.0, 3.0]
        Ki = [ 0.5, 0.5, 0.5, 0.0, 0.0, 0.0]
        Kd = [ 0.1, 0.1, 0.0, 0.4, 0.4, 0.4]

        #Calculate the time difference
        time = self.get_clock().now().nanoseconds/1000000000
        dt = max(time - self.start_time, 1e-6)
        self.start_time = time

        #Calculate the error
        ref = [self.last_ref.x, self.last_ref.y, self.last_ref.z]
        pose = [self.last_pose.x, self.last_pose.y, self.last_pose.z]
        error = [0, 0, 0]
        for i in [1,0,2]:
            error[i] = ref[i] - pose[i]
            #Calculate the integral term
            self.integral[i] += error[i]*dt
            if self.integral[i] > 100.0:
                self.integral[i] = 100.0
            if self.integral[i] < -100.0:
                self.integral[i] = -100.0

        vel_error = [0, 0, 0]
        last_vel = [self.last_pose.x_vel, self.last_pose.y_vel, self.last_pose.z_vel]

        #Calculate the control signals for roll, pitch and thrust
        for i in [1,0,2]:
            vel_ref[i] = Kp[i]*error[i] + Ki[i]*self.integral[i] + Kd[i]*(error[i] - self.last_error[i])/dt


            vel_error[i] = vel_ref[i] - last_vel[i]
            self.vel_integral[i] += vel_error[i]*dt
            if self.vel_integral[i] > 10.0:
                self.vel_integral[i] = 10.0
            if self.vel_integral[i] < -10.0:
                self.vel_integral[i] = -10.0

            control_signal[i] = Kp[i+3]*vel_error[i] + Ki[i+3]*self.vel_integral[i] + Kd[i+3]*(vel_error[i] - self.last_vel_error[i])/dt
            if control_signal[i] > 10.0 and i != 2:
                control_signal[i] = 10.0
            if control_signal[i] < -10.0 and i != 2:
                control_signal[i] = -10.0
            if control_signal[i] < 0.0 and i == 2:
                control_signal[i] = 0.0
            if control_signal[i] > 0.5 and i == 2:
                control_signal[i] = 0.5
            self.get_logger().info('Control signal %d: "%s"' % (i, control_signal[i]))



        self.last_vel_error = vel_error
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
        # self.get_logger().info('Publishing roll: "%s"' % msg.roll)
        # self.get_logger().info('Publishing pitch: "%s"' % msg.pitch)
        # self.get_logger().info('Publishing yaw: "%s"' % msg.yaw)
        # self.get_logger().info('Publishing thrust: "%s"' % msg.thrust)
        


        
        


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