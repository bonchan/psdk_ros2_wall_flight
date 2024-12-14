from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, Shutdown
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('wall_flight')
    
    # Define paths for the RViz2 configuration file
    # rviz_config_path = os.path.join(package_dir, 'config', 'dronesim_rviz_config.rviz')
    rviz_config_path = os.path.join('/home/bon/dji/psdk_ros2_ws/src/wall_flight/config', 'dronesim_rviz_config.rviz')

    print(rviz_config_path)
    
    # Define the nodes
    dronesim_node = Node(
        package='wall_flight',
        executable='dronesim',  # The name of the executable for your dronesim node
        name='dronesim_node',
        output='screen',
        # on_exit=Shutdown()
    )

    dronegui_node = ExecuteProcess(
        cmd=['ros2', 'run', 'wall_flight', 'dronegui'],
        output='screen',
        # on_exit=Shutdown()
    )

    # Define RViz2 with configuration
    rviz2 = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_path],
        output='screen',
        # on_exit=Shutdown()
    )

    Shutdown(),

    return LaunchDescription([
        dronesim_node,
        rviz2,
        dronegui_node,
    ])
