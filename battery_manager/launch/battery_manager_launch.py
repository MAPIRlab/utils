import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the launch directory
    my_dir = get_package_share_directory('battery_manager')

    # params
    params_yaml_file = os.path.join(my_dir, 'battery_manager_params.yaml')
        
    logger = LaunchConfiguration("log_level")
    
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
            ),

        # BATTERY_MANAGER
        Node(
            package='battery_manager',
            executable='battery_manager',
            name='battery_manager',
            output='screen',
            prefix="xterm -e",
            parameters=[params_yaml_file]            
            )
    ])
