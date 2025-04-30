import numpy as np
import rclpy
from rclpy.node import Node
import time
from threading import Thread
import motioncapture # type: ignore
import cflib.crtp   # type: ignore
from cflib.crazyflie import Crazyflie # type: ignore
from cflib.crazyflie.log import LogConfig   # type: ignore
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie     # type: ignore
from cflib.utils import uri_helper  # type: ignore
from cflib.utils.reset_estimator import reset_estimator # type: ignore

# Import custom message types
from interfaces.msg import RPYT # type: ignore
from interfaces.msg import CfLog # type: ignore
from interfaces.msg import PoseRPY # type: ignore
from std_msgs.msg import Bool

# Use vicon for mocap data or not (flowdeck)
vicon = True

# Constants
HOST_NAME = '192.168.1.33'
MOCAP_TYPE = 'vicon'
RIGID_BODY = 'Crazyflie'
ORIENTATION_STANDARD_DEVIATION = 8.0e-3
RADIO_URI = 'radio://0/80/2M/E7E7E7E7'
MIN_THRUST = 10000.0
BASE_THRUST = 47000.0
MAX_THRUST = 60000.0
CONTROL_LOOP_PERIOD = 0.01

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

class DroneInterfaceNode(Node):
    def __init__(self, crazyflie):
        self.crazyflie = crazyflie
        self.rpyt = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.prev_thrust = 0
        self.setpoint_control = False

        # Initialize the node with the name 'drone_interface'
        super().__init__('drone_interface')

        # Create the subscription to the control signals topic
        self.rpyt_subscription = self.create_subscription(RPYT, 'control_signals', self.signal_received, 10)
        self.land_subscription = self.create_subscription(Bool, 'land', self.land_drone, 10)
        
        # Create the publishers
        self.log_publisher = self.create_publisher(CfLog, 'CfLog', 10)
        self.ready_publisher = self.create_publisher(Bool, 'ready', 10)

        if not vicon:
            self.pose_publisher = self.create_publisher(PoseRPY, 'vrpn_mocap/Crazyflie/pose_rpy', 10)

        # Set up logging
        logconf = LogConfig(name='Motors', period_in_ms=20)
        logconf.add_variable('pm.vbatMV', 'uint16_t')
        logconf.add_variable('controller.cmd_thrust', 'float')
        logconf.add_variable('controller.roll', 'float')
        logconf.add_variable('controller.pitch', 'float')
        logconf.add_variable('controller.yaw', 'float')
        #logconf.add_variable('stateEstimate.x', 'float')
        #logconf.add_variable('stateEstimate.y', 'float')
        #logconf.add_variable('stateEstimate.z', 'float')
        #logconf.add_variable('stateEstimate.qx', 'float')
        #logconf.add_variable('stateEstimate.qy', 'float')
        #logconf.add_variable('stateEstimate.qz', 'float')
        #logconf.add_variable('stateEstimate.qw', 'float')

        crazyflie.log.add_config(logconf)
        logconf.data_received_cb.add_callback(self.publish_log_data)
        logconf.start()

        # For GUI and logging purposes, send FALSE in tLOG_INT16he ready topic when the drone is about to take off using high-level commander
        #self.ready_publisher.publish(Bool(data=False))

        # Send zero setpoint to the Crazyflie to unlock thrust protection
        self.crazyflie.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1) 
        
        
        self.crazyflie.commander.send_notify_setpoint_stop()
        time.sleep(0.1)
        
        crazyflie.high_level_commander.takeoff(1, 2.0)
        time.sleep(3.0)
        crazyflie.high_level_commander.go_to(0, 0, 1, 0, 1, relative=False)
        time.sleep(1.2)
        
        
        self.ready_publisher.publish(Bool(data=True))
        self.get_logger().info('Crazyflie is ready and flying.')

        self.create_timer(CONTROL_LOOP_PERIOD, self.send_rpyt)
        

    def send_rpyt(self):
        if self.setpoint_control:
            self.get_logger().info('Sending RPYT setpoint: roll: {}, pitch: {}, yaw: {}, thrust: {}'.format(self.rpyt[0], self.rpyt[1], self.rpyt[2], self.rpyt[3]))
            self.crazyflie.commander.send_setpoint(self.rpyt[0], self.rpyt[1], self.rpyt[2], int(self.rpyt[3]))

    def land_drone(self, msg):
        if msg.data == False: return

        self.setpoint_control = False
        self.crazyflie.commander.send_notify_setpoint_stop()
        #self.crazyflie.high_level_commander.go_to(0, 0, 0.5, 0, 4, relative=False)
        #time.sleep(5.0)
        self.crazyflie.high_level_commander.land(0.0, 3.0)
        time.sleep(4.0)

        # end the link
        self.create_publisher(Bool, 'land', 10).publish(Bool(data=False))
        self.crazyflie.high_level_commander.stop()
        self.crazyflie.platform.send_arming_request(False)
        time.sleep(1.0)
        self.crazyflie.close_link()
        self.get_logger().info('Crazyflie disarmed and program ended.')
        rclpy.shutdown()
        exit(0)

    def signal_received(self, msg):
        self.get_logger().info('Received RPYT signal: roll: {}, pitch: {}, yaw: {}, thrust: {}'.format(msg.roll, msg.pitch, msg.yaw, msg.thrust))
        self.setpoint_control = True

        # Convert the RPYT message to the Crazyflie setpoint.
        self.rpyt[0] = float(msg.roll)
        self.rpyt[1] = float(msg.pitch)
        self.rpyt[2] = float(msg.yaw)
        
        # Map the range 0 to 1.2 to a new range of 10000 to 60000
        thrust = float(msg.thrust)
        thrust = thrust * MIN_THRUST + BASE_THRUST
        if thrust <= MAX_THRUST:
            self.rpyt[3] = thrust
        else:
            self.rpyt[3] = MAX_THRUST
            self.get_logger().warn('Thrust value is too high. Setting thrust to 60000.')
        
    def publish_log_data(self, timestamp, data, x):
        msg = CfLog()
        
        '''
        # Get position data from Crazyflie
        msg.x = float(data['stateEstimate.x'])
        msg.y = float(data['stateEstimate.y'])
        msg.z = float(data['stateEstimate.z'])
        msg.quat_x = float(data['stateEstimate.qx'])
        msg.quat_y = float(data['stateEstimate.qy'])
        msg.quat_z = float(data['stateEstimate.qz'])
        msg.quat_w = float(data['stateEstimate.qw'])
        '''
        
        '''
        # Get position data from vicon
        msg.x = float(latest_pose[0])
        msg.y = float(latest_pose[1])
        msg.z = float(latest_pose[2])
        msg.quat_x = float(latest_pose[3])
        msg.quat_y = float(latest_pose[4])
        msg.quat_z = float(latest_pose[5])
        msg.quat_w = float(latest_pose[6])'''

        # Get reference RPYT and battery voltage from Crazyflie
        msg.roll_signal = float(data['controller.roll'])
        msg.pitch_signal = float(data['controller.pitch'])
        msg.yaw_signal = float(data['controller.yaw'])
        msg.thrust_signal = float(data['controller.cmd_thrust'])
        msg.battery_voltage = float(data['pm.vbatMV'])

        # Save the previous thrust value
        self.prev_thrust = data['controller.cmd_thrust']

        # Publish the log data
        self.log_publisher.publish(msg)

        if not vicon:
            msg2 = PoseRPY()
            msg2.x = float(data['stateEstimate.x'])
            msg2.y = float(data['stateEstimate.y'])
            msg2.z = float(data['stateEstimate.z'])
            
            self.pose_publisher.publish(msg2)


class MocapWrapper(Thread):
    def __init__(self, body_name):
        Thread.__init__(self)

        self.body_name = body_name
        self.on_pose = None
        self._stay_open = True

        self.start()

    def close(self):
        self._stay_open = False

    def run(self):
        mc = motioncapture.connect(MOCAP_TYPE, {'hostname': HOST_NAME})
        while self._stay_open:
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name == self.body_name:
                    if self.on_pose:
                        pos = obj.position
                        quat = obj.rotation
                        global latest_pose
                        latest_pose = [pos[0], pos[1], pos[2], quat.x, quat.y, quat.z, quat.w]
                        self.on_pose([pos[0], pos[1], pos[2], obj.rotation])


def send_extpose_quat(cf, x, y, z, quat):
    cf.extpos.send_extpose(x, y, z, quat.x, quat.y, quat.z, quat.w)    

def main(args=None):

    print("Connecting to the Crazyflie...")
    cflib.crtp.init_drivers()

    if vicon:
        print("Connecting to the vicon...")
        mocap_wrapper = MocapWrapper(RIGID_BODY)
    else:
        print('Vicon is disabled. No mocap data will be sent to the Crazyflie.')

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        crazyflie = scf.cf

        if vicon:
            # Set up a callback to handle data from the mocap system
            mocap_wrapper.on_pose = lambda pose: send_extpose_quat(crazyflie, pose[0], pose[1], pose[2], pose[3])

        # adjust orientation sensitivity
        crazyflie.param.set_value('locSrv.extQuatStdDev', ORIENTATION_STANDARD_DEVIATION)

        # Activate kalman estimator
        crazyflie.param.set_value('stabilizer.estimator', '2')

        # Set the std deviation for the quaternion data pushed into the kalman filter. The default value seems to be a bit too low.
        crazyflie.param.set_value('locSrv.extQuatStdDev', 0.06)

        # Set the yaw commands to absolute angle (deg) instead of yaw rate (deg/s)
        crazyflie.param.set_value('flightmode.stabModeYaw', 1)

        # Activate mellinger_controller
        #crazyflie.param.set_value('stabilizer.controller', '2')

        if vicon:
            reset_estimator(crazyflie)

        # Arm the Crazyflie
        print("Arming the Crazyflie...")
        crazyflie.platform.send_arming_request(True)
        time.sleep(1.0)

        # Start the node
        print("Starting the drone interface node...")
        rclpy.init(args=args)
        node = DroneInterfaceNode(crazyflie) # Pass a reference to the Crazyflie object to the node
        rclpy.spin(node)
        # AFTER THE NODE IS CLOSED
        node.destroy_node()
        rclpy.shutdown()

    if vicon:
        mocap_wrapper.close()

if __name__ == '__main__':
    main()