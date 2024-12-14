import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from wall_flight_interfaces.action import MoveToTarget
from wall_flight_interfaces.msg import TargetPoints
from geometry_msgs.msg import Point
from std_msgs.msg import String
from psdk_interfaces.msg import PositionFused, WidgetPress
from std_srvs.srv import Trigger
from wall_flight.enum.enums import NavigationType

from sensor_msgs.msg import Joy
import math


class FlightControl(Node):

    def __init__(self):
        super().__init__('flight_control')

        self.move_to_target_client = ActionClient(self, MoveToTarget, 'move_to_target')

        self.position_sub = self.create_subscription(PositionFused, '/wrapper/psdk_ros2/position_fused', self.position_sub_callback, 10)

        self.widget_commands_subscriber = self.create_subscription(String, '/wall_flight/widget_commands', self.widget_commands_callback, 10)
        self.widget_press_subscriber = self.create_subscription(WidgetPress, '/wrapper/psdk_ros2/widget_press', self.widget_press_callback, 10)



        self.target_points_pub = self.create_publisher(TargetPoints, '/wall_flight/targets', 10)

        self.obtain_ctrl_authority_cli = self.create_client(Trigger, '/wrapper/psdk_ros2/obtain_ctrl_authority')
        self.release_ctrl_authority_cli = self.create_client(Trigger, '/wrapper/psdk_ros2/release_ctrl_authority')

        # only for test
        self.FLUvelocity_yawrate_pub = self.create_publisher(Joy, '/wrapper/psdk_ros2/flight_control_setpoint_FLUvelocity_yawrate', 10)

        # self.timer = self.create_timer(1.0, self.timer_callback)

        self.drone_position = None

        self.target_a = None
        self.target_b = None
        self.target_c = None

        self.max_speed = 7.2 / 3.6

        self.started = False

        self.targets = ['A', 'B', 'C']  # Array of target positions [A, B, C]
        self.height_delta = 3.0  # Height change value
        self.height_max = 20.0
        self.direction = 1  # 1 means moving forward (A -> B -> C), -1 means moving backward (C -> B -> A)
        self.current_target_index = 0  # Start at the first target (A)
        self.state = 'moving'

    # def timer_callback(self):
    #     pass

    def next_step(self):
        if not self.started:
            self.get_logger().info('Finished: released auth')
            return

        # If we are moving, we move to the next target
        if self.state == 'moving':
            current_target = self.targets[self.current_target_index]
            print(f"Moving to target {current_target}...")

            if current_target == 'A':
                t = self.target_a
            if current_target == 'B':
                t = self.target_b
            if current_target == 'C':
                t = self.target_c

            self.send_goal(NavigationType.HORIZONTAL.name, t, 0.0, self.max_speed)
            
            # Check if we are at target C, if so, issue height command in the next step
            if (current_target == 'C' and self.direction==1) or (current_target == 'A' and self.direction == -1):
                self.state = 'height_command'  # Set state to issue height command next
            else:
                # Move to the next target
                self.current_target_index += self.direction
                
                # If we've reached the end of the array, reverse the direction
                if self.current_target_index == len(self.targets) or self.current_target_index == -1:
                    self.direction *= -1
                    self.current_target_index = max(0, min(self.current_target_index, len(self.targets) - 1))
            
        # If we are issuing a height command, issue the height change
        elif self.state == 'height_command':
            print(f"Issuing height command: {self.height_delta}")
            # Implement the logic for changing height here
            # E.g., send a command to change height by self.height_delta
            
            height_goal = self.drone_position.z + self.height_delta
            
            self.send_goal(NavigationType.VERTICAL.name, self.target_a, height_goal, self.max_speed)

            # After issuing height command, return to moving state
            self.state = 'moving'
            # Reverse direction after height command
            self.direction *= -1


    def start(self):
        t = Trigger.Request()
        self.obtain_ctrl_authority_cli.call_async(t)
        time.sleep(1)
        self.started = True
        self.send_goal(NavigationType.VERTICAL.name, self.target_a, self.height_delta, self.max_speed)

    def stop(self):
        t = Trigger.Request()
        self.release_ctrl_authority_cli.call_async(t)
        time.sleep(1)
        self.started = False

    def misc(self):
        t = Trigger.Request()
        self.obtain_ctrl_authority_cli.call_async(t)
        time.sleep(1)
        joy_cmd = Joy()
        joy_cmd.axes = [
                10.0,
                0.0, # this is reversed
                0.0,
                -math.radians(0.0),
            ]
        
        self.get_logger().info(f'Sending FLU Joy command: {joy_cmd}')

        self.FLUvelocity_yawrate_pub.publish(joy_cmd)


        time.sleep(10)
        self.get_logger().info(f'10 sec')
        


    def send_goal(self, goal_type, target_position, height, max_speed):
        # wait for the server
        self.move_to_target_client.wait_for_server()

        # create the goal
        goal = MoveToTarget.Goal()
        goal.goal_type = goal_type
        goal.target_position = target_position
        goal.height = height
        goal.max_speed = max_speed

        # send the goal
        self.get_logger().info(f'sending the goal: {goal}')
        self.move_to_target_client.send_goal_async(goal).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        
    
    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'result: {result.reached_position}')
        self.next_step()

    def position_sub_callback(self, position_fused:PositionFused):
        if self.drone_position == None:
            self.get_logger().info(f'got the drone position: {position_fused.position}')
        self.drone_position = position_fused.position

    def widget_commands_callback(self, command:String):
        self.get_logger().info(f'widget_commands_callback: {command.data}')
        if command.data == 'start_movement':
            self.start()
        elif command.data == 'stop_movement':
            self.stop()
        elif command.data == 'misc':
            self.misc()

        elif self.drone_position:
            target_points_msg = TargetPoints()

            if command.data == 'set_target_A':
                self.target_a = self.drone_position
            if command.data == 'set_target_B':
                self.target_b = self.drone_position
            if command.data == 'set_target_C':
                self.target_c = self.drone_position
            if self.target_a:
                target_points_msg.target_a = self.target_a
            if self.target_b:
                target_points_msg.target_b = self.target_b
            if self.target_c:
                target_points_msg.target_c = self.target_c
            self.get_logger().info(f'target_points_msg: {target_points_msg}')
            self.target_points_pub.publish(target_points_msg)


    def widget_press_callback(self, command:WidgetPress):
        self.get_logger().info(f'widget_press_callback: {command}')

        if self.drone_position:
            if command.index == 0:
                if command.value == 1:
                    self.target_a = self.drone_position
            elif command.index == 1:
                if command.value == 1:
                    self.target_b = self.drone_position
            elif command.index == 2:
                if command.value == 1:
                    self.target_c = self.drone_position
            elif command.index == 3:
                if command.value == 1:
                    # self.target_a = self.drone_position
                    pass
            elif command.index == 4:
                if command.value == 1:
                    self.start()
                elif command.value == 0:
                    self.stop()
        
            target_points_msg = TargetPoints()

            if self.target_a:
                target_points_msg.target_a = self.target_a
            if self.target_b:
                target_points_msg.target_b = self.target_b
            if self.target_c:
                target_points_msg.target_c = self.target_c
            self.get_logger().info(f'target_points_msg: {target_points_msg}')
            self.target_points_pub.publish(target_points_msg)





def main(args=None):
    rclpy.init(args=args)
    node  = FlightControl()

    # node.create_goals()

    # step = node.goals.pop(0)
    # print(step)
    # node.send_goal(step['ty'], step['t'], node.max_speed)

    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
