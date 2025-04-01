import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from interfaces.msg import PoseRPY

class Pathfinding(Node):
    
    def __init__(self):
        super().__init__('pathfinding_coords')
        self.get_logger().info('Pathfinding node has been started.')
        self.publisher = self.create_publisher(PoseRPY, 'ref_pose', 10)
        self.subscriber = self.create_subscription(Int32, 'next_ref', self.listen_position, 10)
        #get waypoints from txt file
        self.waypoints = []
        
        with open('/home/jonas/ros2_ws_p4/src/Crazyflie-P4-Group-460/pathfinding/pathfinding/waypoints.txt', 'r') as file:
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
        self.array_index = msg.data - 1

    def timer_callback(self):
        self.send_position()

    def send_position(self):
        #send waypoint with array index of next_ref

         # Adjust for 0-based index

        if len(self.waypoints) < self.array_index:
            #If all waypoints have been achieved, send 0,0,z to return home
            self.publisher.publish(PoseRPY(
            x=0.0,
            y=0.0,
            z=self.waypoints[1][2],#extract z value
            roll=0.0,
            pitch=0.0,
            yaw=0.0
            ))
            self.get_logger().info('ALL WAYPOINTS COMPLETED - (0.0.z) sent')
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

