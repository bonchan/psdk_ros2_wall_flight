import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import numpy as np
import time
from psdk_interfaces.msg import PositionFused
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
import math
from scipy.spatial.transform import Rotation as R
from wall_flight_interfaces.msg import TargetPoints

class DroneSim(Node):
    def __init__(self):
        super().__init__('dronesim')

        # Publisher for the drone's position and orientation
        self.pose_publisher = self.create_publisher(PoseStamped, '/wall_flight/sim/drone_pose', 10)

        # Publisher for the markers representing target_a and target_b
        self.marker_publisher = self.create_publisher(Marker, '/wall_flight/sim/target_markers', 10)

        self.path_publisher = self.create_publisher(Path, '/wall_flight/sim/drone_path', 10)
        
        # Set up a timer to publish both pose and markers periodically
        self.timer = self.create_timer(0.1, self.publish_pose_and_markers)


        
        # Create a subscriber for the wrapper
        self.position_sub = self.create_subscription(PositionFused, '/wrapper/psdk_ros2/position_fused', self.position_sub_callback, 10)
        self.attitude_sub = self.create_subscription(QuaternionStamped, '/wrapper/psdk_ros2/attitude', self.attitude_sub_callback, 10)

        self.target_points_sub = self.create_subscription(TargetPoints, '/wall_flight/targets', self.target_points_callback, 10)

        
        # Initialize drone position (start at the origin)
        self.drone_position = self.create_point(0.0, 0.0, 0.0)
        self.drone_orientation = Quaternion()
        self.drone_path = Path()
        self.drone_path.header.frame_id = 'map'

        self.target_a = None
        self.target_b = None
        self.target_c = None
        
        
    def publish_pose_and_markers(self):
        # heading = self.get_heading_from_quaternion(self.drone_orientation)
        

        # Publish the drone's pose
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = self.drone_position.x
        pose_msg.pose.position.y = self.drone_position.y
        pose_msg.pose.position.z = self.drone_position.z
        # pose_msg.pose.orientation.z = math.sin(self.droe_yaw / 2.0)
        # pose_msg.pose.orientation.w = math.cos(self.droe_yaw / 2.0)
        pose_msg.pose.orientation = self.drone_orientation
        self.pose_publisher.publish(pose_msg)

        # Publish markers
        if self.target_a:
            self.publish_marker(self.target_a, 'target_a', [0.0, 1.0, 0.0])  # Red color for target_a
        if self.target_b:
            self.publish_marker(self.target_b, 'target_b', [0.0, 0.0, 1.0])  # Green color for target_b
        if self.target_c:
            self.publish_marker(self.target_c, 'target_c', [1.0, 0.0, 0.0])  # Green color for target_c

        self.publish_drone_path(pose_msg)

    def publish_marker(self, position, marker_id, color):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = marker_id
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Define start and end points for the vertical line
        start_point = [position.x, position.y, 0.0]  # Start at ground level
        end_point = [position.x, position.y, 20.0]  # Go up to target height

        marker.points = [
            self.create_point(*start_point),
            self.create_point(*end_point)
        ]

        # Set marker properties
        marker.scale.x = 0.1  # Line width
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0  # Opacity

        self.marker_publisher.publish(marker)

    def publish_drone_path(self, pose_stamped):
        self.drone_path.poses.append(pose_stamped)
        if len(self.drone_path.poses) > 1000:
            self.drone_path.poses.pop(0)
        self.path_publisher.publish(self.drone_path)

    def create_point(self, x, y, z):
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point
    
    # def get_heading_from_quaternion(self, quaternion):
    #     # Convert the quaternion (x, y, z, w) to Euler angles
    #     rotation = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    #     roll, pitch, yaw = rotation.as_euler('xyz', degrees=True)  # Extract in degrees for easier interpretation


    #     print(f"Heading (yaw): {yaw} degrees")
    #     return yaw 
    
    def position_sub_callback(self, position_fused:PositionFused):
        self.drone_position = position_fused.position

    def attitude_sub_callback(self, attitude:QuaternionStamped):
        self.drone_orientation = attitude.quaternion

    def target_points_callback(self,targets:TargetPoints):
        self.target_a = targets.target_a
        self.target_b = targets.target_b
        self.target_c = targets.target_c

def main(args=None):
    rclpy.init(args=args)
    drone_sim = DroneSim()
    rclpy.spin(drone_sim)
    drone_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
