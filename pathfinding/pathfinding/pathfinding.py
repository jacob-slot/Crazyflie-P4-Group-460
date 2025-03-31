import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from interfaces.msg import PoseRPY

class Pathfinding(Node):
    
    def __init__(self):
        super().__init__('pathfinding_coords')
        self.get_logger().info('Pathfinding node has been started.')
        self.publisher = self.create_publisher(PoseRPY, 'pathfinding_coords', 10)
        self.subscriber = self.create_subscription(Int32, 'next_ref', self.listen_position, 10)
        #get waypoints from txt file
        self.waypoints = []
        with open('/home/rauh/ros2_pathfinding/src/Crazyflie-P4-Group-460/pathfinding/pathfinding/waypoints.txt', 'r') as file:
            for line in file:
                coords = line.strip().split(',')
                if len(coords) == 3:
                    x, y, z = map(float, coords)
                    self.waypoints.append((x, y, z))
        self.send_position(1) #send first waypoint to Crazyflie


    def listen_position(self, msg):
        #get the value next_ref
        self.get_logger().info('Received next_ref: %d' % msg.data)
        self.send_position(msg.data)

    def send_position(self, msg):
        #send waypoint with array index of next_ref

        array_index = msg - 1 # Adjust for 0-based index

        if len(self.waypoints) < msg:
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
            x=self.waypoints[array_index][0],
            y=self.waypoints[array_index][1],
            z=self.waypoints[array_index][2],
            roll=0.0,
            pitch=0.0,
            yaw=0.0
            ))
            self.get_logger().info('Sending waypoint: %s' % str(self.waypoints[array_index]))

        
def main(args=None):
    rclpy.init(args=args)
    pathfinding = Pathfinding()
    rclpy.spin(pathfinding)
    pathfinding.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

