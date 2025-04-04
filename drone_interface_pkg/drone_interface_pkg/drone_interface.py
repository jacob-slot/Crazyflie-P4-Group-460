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
#from interfaces.msg import Cf_log

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

        # Initialize the node with the name 'drone_interface'
        super().__init__('drone_interface')

        # Create the subscription to the control signals topic
        self.subscription = self.create_subscription(RPYT, 'control_signals', self.listener_callback, 10)
        
        # Create the publisher for the cf_log topic
        #self.publisher = self.create_publisher(Cf_log, 'cf_log', 10)

        # Set up logging
        logconf = LogConfig(name='Motors', period_in_ms=20)
        logconf.add_variable('motor.m1', 'uint16_t')
        logconf.add_variable('motor.m2', 'uint16_t')
        logconf.add_variable('motor.m3', 'uint16_t')
        logconf.add_variable('motor.m4', 'uint16_t')
        logconf.add_variable('pm.vbatMV', 'uint16_t')
        myFlie.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_data_callback)

        logconf.start()

        
        myFlie.high_level_commander.takeoff(1.0, 2.0)
        time.sleep(3.0)
        myFlie.high_level_commander.go_to(0, 0, 0.1, 0.0, 1.0)
        time.sleep(20.0)
        myFlie.high_level_commander.land(0.0, 4.0)
        time.sleep(5.0)
        
        
        self.commander = myFlie.commander
        # doesnt work> self.commander.flightmode.stabModeYaw = 1        

    # Print the received control signal
    def listener_callback(self, msg):

        # Convert the RPYT message to the Crazyflie setpoint, which is in degrees and integer thrust
        thrust = int(msg.thrust*1000)
        # convert degrees to radians
        roll = msg.roll * 180 / np.pi
        pitch = msg.pitch * 180 / np.pi
        yaw = msg.yaw * 180 / np.pi

        # Send the setpoint to the Crazyflie
        self.commander.send_setpoint(roll, pitch, yaw, thrust) 
        time.sleep(0.01)
        self.get_logger().info(f'Received setpoint: roll={roll}, pitch={pitch}, yaw={yaw}, thrust={thrust}')
        


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

def log_data_callback(timestamp, data, x):

    #print(data["motor.m1"])
    
    #print("logging")
    return
    msg = Cf_log()
    msg.m1 = data["motor.m1"]
    msg.m2 = data["motor.m2"]
    msg.m3 = data["motor.m3"]
    msg.m4 = data["motor.m4"]
    msg.vbatMV = data["pm.vbatMV"]
    msg.x = latest_pose[0]
    msg.y = latest_pose[1]
    msg.z = latest_pose[2]
    msg.qx = latest_pose[3]
    msg.qy = latest_pose[4]
    msg.qz = latest_pose[5]
    msg.qw = latest_pose[6]
    
    # Publish the log data
    self.publisher.publish(msg)

def main(args=None):

    print("Connecting to the Crazyflie...")
    cflib.crtp.init_drivers()

    print("Connecting to the vicon...")
    mocap_wrapper = MocapWrapper(rigid_body_name)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        myFlie = scf.cf
        # Set up a callback to handle data from the mocap system
        mocap_wrapper.on_pose = lambda pose: send_extpose_quat(myFlie, pose[0], pose[1], pose[2], pose[3])

        # adjust orientation sensitivity
        myFlie.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)

        # Activate kalman estimator
        myFlie.param.set_value('stabilizer.estimator', '2')

        # Set the std deviation for the quaternion data pushed into the kalman filter. The default value seems to be a bit too low.
        myFlie.param.set_value('locSrv.extQuatStdDev', 0.06)

        # Activate mellinger_controller
        #myFlie.param.set_value('stabilizer.controller', '2')

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
    mocap_wrapper.close()

if __name__ == '__main__':
    main()