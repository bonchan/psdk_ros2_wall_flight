import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk

class DroneGUI(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.command_publisher = self.create_publisher(String, '/wall_flight/widget_commands', 10)
        
        # Setup the Tkinter GUI
        self.root = tk.Tk()
        self.root.title("Drone Command GUI")
        
        # Target selection buttons
        tk.Button(self.root, text="Set Target A", command=lambda: self.set_target("A"), width=20, height=2).pack(pady=5)
        tk.Button(self.root, text="Set Target B", command=lambda: self.set_target("B"), width=20, height=2).pack(pady=5)
        tk.Button(self.root, text="Set Target C", command=lambda: self.set_target("C"), width=20, height=2).pack(pady=5)

        # Start button
        tk.Button(self.root, text="Start", command=self.start_movement, width=20, height=2).pack(pady=10)
        tk.Button(self.root, text="Stop", command=self.stop_movement, width=20, height=2).pack(pady=10)

        tk.Button(self.root, text="MISC", command=self.misc, width=20, height=2).pack(pady=10)

        # Run Tkinter loop
        self.root.protocol("WM_DELETE_WINDOW", self.shutdown)
        self.root.mainloop()

    def set_target(self, target):
        msg = String()
        msg.data = f"set_target_{target}"
        self.command_publisher.publish(msg)
        self.get_logger().info(f'Set target: {target}')

    def start_movement(self):
        msg = String()
        msg.data = "start_movement"
        self.command_publisher.publish(msg)
        self.get_logger().info("Start movement")
    
    def stop_movement(self):
        msg = String()
        msg.data = "stop_movement"
        self.command_publisher.publish(msg)
        self.get_logger().info("Stop movement")

    def misc(self):
        msg = String()
        msg.data = "misc"
        self.command_publisher.publish(msg)
        self.get_logger().info("Misc")

    def shutdown(self):
        self.root.quit()
        rclpy.shutdown()

def main():
    rclpy.init()
    gui_node = DroneGUI()
    rclpy.spin(gui_node)

if __name__ == '__main__':
    main()