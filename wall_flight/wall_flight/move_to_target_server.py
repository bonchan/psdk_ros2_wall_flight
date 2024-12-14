import rclpy
import time
import math
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from wall_flight_interfaces.action import MoveToTarget
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
from psdk_interfaces.msg import PositionFused
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Quaternion

import numpy as np
from scipy.spatial.transform import Rotation as R

from wall_flight.enum.enums import NavigationType


import threading

class MoveToTargetServerNode(Node):

    def __init__(self):
        super().__init__('move_to_target_server')
        self.move_to_target_server = ActionServer(self,MoveToTarget, 'move_to_target', execute_callback=self.excecute_callback)
        
        self.position_sub = self.create_subscription(PositionFused, '/wrapper/psdk_ros2/position_fused', self.position_sub_callback, 10)
        self.attitude_sub = self.create_subscription(QuaternionStamped, '/wrapper/psdk_ros2/attitude', self.attitude_sub_callback, 10)


        self.FLUvelocity_yawrate_pub = self.create_publisher(Joy, '/wrapper/psdk_ros2/flight_control_setpoint_FLUvelocity_yawrate', 10)
        self.ENUposition_yaw_pub = self.create_publisher(Joy, '/wrapper/psdk_ros2/flight_control_setpoint_ENUposition_yaw', 10)



        self.drone_position:Point = None
        self.drone_orientation:Quaternion = None

        self.h_tolerance = 1.5
        self.v_tolerance = 0.2

        self.max_th = 2.5

        self.scaled_speed = 0.1
        self.last_distances = []


        self.spin_thread = threading.Thread(target=self.spin_background)
        self.spin_thread.start()

        self.get_logger().info('move_to_target_server created')

    def spin_background(self):
        rclpy.spin(self)

    def excecute_callback(self, goal_handle: ServerGoalHandle):
        # get request from goal
        goal_type = goal_handle.request.goal_type
        target_position = goal_handle.request.target_position
        height = goal_handle.request.height
        position_tolerance = goal_handle.request.position_tolerance
        target_orientation = goal_handle.request.target_orientation
        orientation_tolerance = goal_handle.request.orientation_tolerance
        max_speed = goal_handle.request.max_speed

        feedback = MoveToTarget.Feedback()
        
        # execute the action
        self.get_logger().info('Executing goal...')
        self.get_logger().info(f'Going to {target_position}')

        distance = 0.0
        

        if goal_type == NavigationType.HORIZONTAL.name:
            distance = self.get_lateral_distance_to_target(target_position)
            

            while not distance < self.h_tolerance:
                # Ensure we have a current position
                if self.drone_position is None:
                    time.sleep(0.1)  # Wait if position is not yet received
                    self.get_logger().info('waiting drone_position')
                    continue

                # Scale speed only if within 10 meters of the target
                if distance < 3.0:
                    # Adjust speed proportionally, with a minimum speed threshold
                    target_speed = 0.5  # Define minimum speed to prevent stopping
                elif distance < 9.0:
                    # Adjust speed proportionally, with a minimum speed threshold
                    target_speed = 1.1  # Define minimum speed to prevent stopping
                else:
                    # Use the full max speed if farther than 10 meters
                    target_speed = max_speed
                    # self.get_logger().info(f'max_speed: {scaled_speed} | distance: {distance}')



                if self.scaled_speed < target_speed:
                    self.scaled_speed += 0.1
                elif self.scaled_speed > target_speed:
                    self.scaled_speed -= 0.1
                

                front_back, left_right, up_down = self.get_local_direction(self.drone_position, self.drone_orientation, target_position)
                pitch, roll = self.combine_speeds(front_back, left_right, self.scaled_speed)
                throttle = 0.0

                joy_cmd = Joy()
                joy_cmd.axes = [
                        pitch,
                        roll, # this is reversed
                        0.0,
                        -math.radians(0.0),
                    ]
                
                # self.get_logger().info(f'Sending FLU Joy command: {joy_cmd}')

                self.FLUvelocity_yawrate_pub.publish(joy_cmd)

                # Update feedback
                feedback.current_position = self.drone_position
                goal_handle.publish_feedback(feedback)

                distance = self.get_lateral_distance_to_target(target_position)

                self.last_distances.append(distance)
                if len(self.last_distances) > 10:
                    self.last_distances.pop(0)

                    
                    avg = sum(self.last_distances) / len(self.last_distances)

                    self.get_logger().info(f'speed: {self.scaled_speed} | distance: {distance} | lastAVG: {avg}')

                    if avg < (1.3 * self.h_tolerance):
                        self.get_logger().info(f'|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||')
                        break
                else:
                    self.get_logger().info(f'speed: {self.scaled_speed} | distance: {distance} ')


                # time.sleep(3.0)

        if goal_type == NavigationType.VERTICAL.name:

            distance = self.get_vertical_distance_to_target(height)

            while not distance < self.v_tolerance:
                joy_cmd = Joy()
                joy_cmd.axes = [
                        0.0,
                        0.0, # this is reversed
                        distance,
                        -math.radians(0.0),
                    ]
                
                # self.get_logger().info(f'Sending ENU Joy command: {joy_cmd}')

                self.FLUvelocity_yawrate_pub.publish(joy_cmd)

                distance = self.get_vertical_distance_to_target(height)
                time.sleep(0.2)










        # once done, set the goal final state
        self.get_logger().info(f'Arrived to {target_position}')
        goal_handle.succeed()

        # send the result
        result = MoveToTarget.Result()
        result.reached_position = self.drone_position

        return result
    
    def get_lateral_distance_to_target(self, target_position):
        # Define a threshold to consider as "reached"
        if self.drone_position is None:
            return False

        dx = self.drone_position.x - target_position.x
        dy = self.drone_position.y - target_position.y
        dz = 0.0  # self.drone_position.z - target_position.z
        distance = (dx**2 + dy**2 + dz**2) ** 0.5
        # self.get_logger().info(f'distance: {distance}')
        return distance
    
    def get_vertical_distance_to_target(self, desired_height):
        # Define a threshold to consider as "reached"
        if self.drone_position is None:
            return False

        distance = desired_height - self.drone_position.z
        self.get_logger().info(f'vertical distance: {desired_height} - {self.drone_position.z} = {distance}')
        return distance
    
    def get_abs_vertical_distance_to_target(self, desired_height):
        # Define a threshold to consider as "reached"
        if self.drone_position is None:
            return False

        distance = abs(self.drone_position.z - desired_height)
        self.get_logger().info(f'vertical distance: {self.drone_position.z} - {desired_height} = {distance}')
        return distance
    
    def get_local_direction(self, drone_position, drone_orientation, target_position):
        # Step 1: Compute the vector from the drone to the target in the global frame
        vector_to_target = np.array([target_position.x, target_position.y, target_position.z]) - np.array([drone_position.x, drone_position.y, drone_position.z])

        # Step 2: Convert drone_orientation from quaternion to a rotation object
        rotation = R.from_quat([drone_orientation.x, drone_orientation.y, drone_orientation.z, drone_orientation.w])

        # Step 3: Invert the rotation to go from global frame to the drone's local frame
        vector_to_target_local = rotation.inv().apply(vector_to_target)

        # Step 4: Extract the X (front/back) and Y (left/right) components in the local frame
        front_back = vector_to_target_local[0]  # X component
        left_right = vector_to_target_local[1]  # Y component
        up_down = vector_to_target_local[2]  # Y component

        return front_back, left_right, up_down

    def combine_speeds(self, pitch, roll, max_speed):
        pitch *= 1000
        roll *= 1000
        # Calculate the magnitude of the combined speed
        magnitude = math.sqrt(pitch**2 + roll**2)
        
        # Scale down if the magnitude exceeds max_speed
        if magnitude > max_speed:
            scale = max_speed / magnitude
            pitch *= scale
            roll *= scale
        
        return pitch, roll
    
    def position_sub_callback(self, position_fused:PositionFused):
        if self.drone_position == None:
            self.get_logger().info(f'got the drone position: {position_fused.position}')
        self.drone_position = position_fused.position

    def attitude_sub_callback(self, attitude:QuaternionStamped):
        self.drone_orientation = attitude.quaternion

def main(args=None):
    rclpy.init(args=args)
    node = MoveToTargetServerNode()
    
    # Use MultiThreadedExecutor to handle callbacks concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
