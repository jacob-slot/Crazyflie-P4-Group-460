import numpy as np
import rclpy
from rclpy.node import Node
import time
from threading import Thread
import motioncapture
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

# Import custom message types
from interfaces.msg import RPYT
from interfaces.msg import CfLog

vicon = True

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
host_name = '192.168.1.33'
mocap_system_type = 'vicon'
rigid_body_name = 'Crazyflie'
# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
orientation_std_dev = 8.0e-3

class DroneInterfaceNode(Node):
    def __init__(self, myFlie):
        self.myFlie = myFlie
        #self.roll = 0
        #self.pitch = 0
        #self.yaw = 0
        #self.thrust = 46550
        #self.setPointControl = False
        self.prev_thrust = 0

        # Initialize the node with the name 'drone_interface'
        super().__init__('drone_interface')

        # Create the subscription to the control signals topic
        self.subscription = self.create_subscription(RPYT, 'control_signals', self.listener_callback, 10)
        
        # Create the publisher for the CfLog topic
        self.publisher = self.create_publisher(CfLog, 'CfLog', 10)

        # Set up logging
        logconf = LogConfig(name='Motors', period_in_ms=20)
        logconf.add_variable('pm.vbatMV', 'uint16_t')
        logconf.add_variable('controller.cmd_thrust', 'float')
        logconf.add_variable('controller.roll', 'float')
        logconf.add_variable('controller.pitch', 'float')
        logconf.add_variable('controller.yaw', 'float')

        myFlie.log.add_config(logconf)
        logconf.data_received_cb.add_callback(self.log_data_callback)
        logconf.start()
        
        # create new txt file with unique name
        #timestamp = time.strftime("%Y%m%d-%H%M%S")
        #self.log_file = open(f'log_{timestamp}.txt', 'w')
        #self.log_file.write(str(self.thrust) + '\n')

        # Send zero setpoint to the Crazyflie to unlock thrust protection
        self.myFlie.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)
        self.myFlie.commander.send_notify_setpoint_stop()
        time.sleep(0.1)

        myFlie.high_level_commander.takeoff(0.5, 2.0)
        time.sleep(3.0)
        myFlie.high_level_commander.go_to(-1, 0, 1, 0, 2, relative=False)
        time.sleep(3.0)

        for i in range (3):
            for i in range(15):
                myFlie.commander.send_setpoint(0, 8.0, 0, int(self.prev_thrust))
                time.sleep(0.1)

            myFlie.commander.send_setpoint(0, -10, 0, int(self.prev_thrust))
            time.sleep(0.1)
            myFlie.commander.send_notify_setpoint_stop()
            #time.sleep(0.05)
            myFlie.high_level_commander.go_to(-1, 0, 1, 0, 5, relative=False)
            time.sleep(6.0)

        self.land_drone()

        #self.create_timer(0.1, self.send_rpyt)
        #self.create_timer(6, self.back_to_center)
        

    def send_rpyt(self):
        #self.setPointControl = True
        self.myFlie.commander.send_setpoint(self.roll, self.pitch, self.yaw, self.thrust)

    def land_drone(self):
        #self.setPointControl = False
        self.myFlie.commander.send_notify_setpoint_stop()
        self.myFlie.high_level_commander.land(0.0, 5.0)
        time.sleep(6.0)

        # end the link
        self.myFlie.high_level_commander.stop()
        self.myFlie.platform.send_arming_request(False)
        time.sleep(1.0)
        self.myFlie.close_link()
        self.get_logger().info('Crazyflie disarmed and program ended.')
        rclpy.shutdown()
        exit(0)

    def listener_callback(self, msg):

        # Convert the RPYT message to the Crazyflie setpoint, which is in degrees and integer thrust
        self.thrust = int(msg.thrust*1000)
        # convert degrees to radians
        self.roll = msg.roll * 180 / np.pi
        self.pitch = msg.pitch * 180 / np.pi
        self.yaw = msg.yaw * 180 / np.pi

        self.get_logger().info(f'Received setpoint: roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}, thrust={self.thrust}')
        
    def log_data_callback(self, timestamp, data, x):

        msg = CfLog()

        # Get position data from vicon
        msg.x = float(latest_pose[0])
        msg.y = float(latest_pose[1])
        msg.z = float(latest_pose[2])
        msg.quat_x = float(latest_pose[3])
        msg.quat_y = float(latest_pose[4])
        msg.quat_z = float(latest_pose[5])
        msg.quat_w = float(latest_pose[6])

        # Get reference RPYT and battery voltage from Crazyflie
        msg.roll_signal = float(data['controller.roll'])
        msg.pitch_signal = float(data['controller.pitch'])
        msg.yaw_signal = float(data['controller.yaw'])
        msg.thrust_signal = float(data['controller.cmd_thrust'])
        msg.battery_voltage = float(data['pm.vbatMV'])

        # Save the previous thrust value
        self.prev_thrust = data['controller.cmd_thrust']

        # Publish the log data
        self.publisher.publish(msg)


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
        mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
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
        mocap_wrapper = MocapWrapper(rigid_body_name)
    else:
        print('Vicon is disabled. No mocap data will be sent to the Crazyflie.')

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        myFlie = scf.cf

        if vicon:
            # Set up a callback to handle data from the mocap system
            mocap_wrapper.on_pose = lambda pose: send_extpose_quat(myFlie, pose[0], pose[1], pose[2], pose[3])

        # adjust orientation sensitivity
        myFlie.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)

        # Activate kalman estimator
        myFlie.param.set_value('stabilizer.estimator', '2')

        # Set the std deviation for the quaternion data pushed into the kalman filter. The default value seems to be a bit too low.
        myFlie.param.set_value('locSrv.extQuatStdDev', 0.06)

        # Set the yaw commands to absolute angle (deg) instead of yaw rate (deg/s)
        myFlie.param.set_value('flightmode.stabModeYaw', 1)

        # Activate mellinger_controller
        #myFlie.param.set_value('stabilizer.controller', '2')

        if vicon:
            reset_estimator(myFlie)

        # Arm the Crazyflie
        print("Arming the Crazyflie...")
        myFlie.platform.send_arming_request(True)
        time.sleep(1.0)

        # Start the node
        print("Starting the drone interface node...")
        rclpy.init(args=args)
        node = DroneInterfaceNode(myFlie) # Pass a reference to the Crazyflie object to the node
        rclpy.spin(node)
        # AFTER THE NODE IS CLOSED
        node.destroy_node()
        rclpy.shutdown()

    if vicon:
        mocap_wrapper.close()

if __name__ == '__main__':
    main()