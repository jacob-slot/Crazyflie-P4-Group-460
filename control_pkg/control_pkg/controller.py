import numpy as np

import rclpy
from rclpy.node import Node

from interfaces.msg import RPYT # type: ignore
from interfaces.msg import PoseRPY # type: ignore
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time



class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        # Set up the QoS profile for the subscriptions
        qos_profile = QoSProfile(depth=10)  # You can adjust the depth if needed
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE


        # Initialize the last reference, pose, error variables and ready signals
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
            qos_profile)
        self.ref_subscription 
        
        self.pose_subscription = self.create_subscription(
            PoseRPY,
            'vrpn_mocap/Crazyflie/pose_rpy',
            self.listener_callback_pose,
            qos_profile
        )
        self.pose_subscription

        # Create the subscription for the ready signal and landing signal
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
        """ Recieve the landing signal and land the drone """
        if msg == True:
            raise SystemExit('Landing now.')

    def listener_callback_ready(self, msg):
        """ Recieve the ready signal and start the timer"""
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

        #PID gains x y z x_vel y_vel z_vel
        Kp = [ 1.0, 1.0, 0.8, 12.5, 12.5, 4.0]
        Ki = [ 0.0, 0.0, 0.4, 0.0, 0.0, 0.0]
        Kd = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.3]

        # # Inside the PID method
        # Kp = np.array([2.0, 2.0, 1.6, 25.0, 25.0, 2.4]) * 0.5
        # Ki = np.array([0.0, 0.0, 2.4, 1.0, 1.0, 0.0]) * 0.5
        # Kd = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.6]) * 0.5

        #Calculate the time difference
        time = self.get_clock().now().nanoseconds/1000000000
        dt = max(time - self.start_time, 1e-6)
        self.start_time = time

        #Initialize the control signals and velocity reference
        vel_ref = [0, 0 ,0]
        control_signal = [0, 0, 0]

        #initialize the error and velocity error
        error = [0, 0, 0]
        vel_error = [0, 0, 0]

        #Get the last reference, pose and velocity
        ref = [self.last_ref.x, self.last_ref.y, self.last_ref.z]
        pose = [self.last_pose.x, self.last_pose.y, self.last_pose.z]
        last_vel = [self.last_pose.x_vel, self.last_pose.y_vel, self.last_pose.z_vel]

        for i in [1,0,2]:
            #Calculate the error
            error[i] = ref[i] - pose[i]

            #Calculate the integral term
            self.integral[i] += error[i]*dt
            if self.integral[i] > 0.5:
                self.integral[i] = 0.5
            if self.integral[i] < -0.5:
                self.integral[i] = -0.5

            #Calculate the reference velocity
            vel_ref[i] = Kp[i]*error[i] + Ki[i]*self.integral[i] + Kd[i]*(error[i] - self.last_error[i])/dt

            if vel_ref[i] > 3.0:
                vel_ref[i] = 3.0
            if vel_ref[i] < -3.0:
                vel_ref[i] = -3.0
            
            
            #Calculate the velocity error
            vel_error[i] = vel_ref[i] - last_vel[i]

            #Calculate the velocity integral term
            self.vel_integral[i] += vel_error[i]*dt
            if self.vel_integral[i] > 10.0:
                self.vel_integral[i] = 10.0
            if self.vel_integral[i] < -10.0:
                self.vel_integral[i] = -10.0

            #Calculate the control signal
            control_signal[i] = Kp[i+3]*vel_error[i] + Ki[i+3]*self.vel_integral[i] + Kd[i+3]*(vel_error[i] - self.last_vel_error[i])/dt

            #Invert the control signal for roll
            if i == 1:
                control_signal[i] = -1*control_signal[i]

            #Limit the pitch and roll signals
            if control_signal[i] > 8.0 and i != 2:
                control_signal[i] = 8.0
            if control_signal[i] < -8.0 and i != 2:
                control_signal[i] = -8.0

            #Limit the thrust signal
            if control_signal[i] < -1.2 and i == 2:
                control_signal[i] = -1.2
            if control_signal[i] > 1.2 and i == 2:
                control_signal[i] = 1.2


        self.last_vel_error = vel_error
        self.last_error = error
        

        return control_signal
    
    

    def pos_to_rpy(self):
        """
        Finds control signals through PID controller and publishes them
        """

        #Calculate the control signals
        control_signal = self.PID()

        #Create the control signal message
        msg = RPYT()
        msg.roll = control_signal[1]
        msg.pitch = control_signal[0]
        msg.thrust = control_signal[2]

        #Publish the control signals
        self.control_publisher.publish(msg)
        time.sleep(0.01)


        

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