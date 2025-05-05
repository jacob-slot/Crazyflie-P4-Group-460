import rclpy
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from std_msgs.msg import Int32
from interfaces.msg import PoseRPY # type: ignore
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

pathfinding_path = os.path.join(get_package_share_directory('pathfinding'), 'waypoints.txt')

class Pathfinding(Node):
    
    def __init__(self):
        super().__init__('pathfinding_coords')

        # Set up the QoS profile for the subscriptions
        qos_profile = QoSProfile(depth=10)  # You can adjust the depth if needed
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE

        self.first_pose = False

        self.get_logger().info('Pathfinding node has been started.')
        self.publisher = self.create_publisher(PoseRPY, 'ref_pose', 10)

        self.land_publisher = self.create_publisher(Bool, 'land', 10)

        self.subscriber = self.create_subscription(Int32, 'next_ref', self.listen_position, 10)

        self.pose_subscriber = self.create_subscription(PoseRPY, 'vrpn_mocap/Crazyflie/pose_rpy', self.listen_pose, qos_profile)
        #get waypoints from txt file
        self.waypoints = [[0, 0, 0]]
        
        with open(pathfinding_path, 'r') as file:
            for line in file:
                coords = line.strip().split(',')
                if len(coords) == 3:
                    x, y, z = map(float, coords)
                    self.waypoints.append((x, y, z))

        self.array_index = 0
        self.timer = self.create_timer(1, self.timer_callback)


    def listen_position(self, msg):
        #get the value next_ref
        self.get_logger().info('Received next_ref: %d' % msg.data)
        self.array_index = msg.data
        
    def listen_pose(self, msg):
        self.get_logger().info('Received pose: x=%f, y=%f, z=%f' % (msg.x, msg.y, msg.z))
        if not self.first_pose:
            self.first_pose = True
            self.waypoints[0] = [msg.x, msg.y, 1.0]
            self.destroy_subscription(self.pose_subscriber)

    def timer_callback(self):
        if self.first_pose:
            self.send_position()

    def send_position(self):
        #send waypoint with array index of next_ref

         # Adjust for 0-based index

        if len(self.waypoints) == self.array_index:
            #If all waypoints have been achieved, send 0,0,z to return home
            self.land_publisher.publish(Bool(data=True))
            raise SystemExit('All waypoints have been achieved, landing now.')
        else:
            self.publisher.publish(PoseRPY(
            x=self.waypoints[self.array_index][0],
            y=self.waypoints[self.array_index][1],
            z=self.waypoints[self.array_index][2],
            roll=0.0,
            pitch=0.0,
            yaw=0.0
            ))
            self.get_logger().info('Sending waypoint: %s' % str(self.waypoints[self.array_index]))

        
def main(args=None):
    rclpy.init(args=args)
    pathfinding = Pathfinding()
    rclpy.spin(pathfinding)
    pathfinding.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

