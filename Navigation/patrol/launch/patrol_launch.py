import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the launch directory
    my_dir = get_package_share_directory('patrol')
    params_yaml_file = os.path.join(my_dir, 'launch', 'patrol_params.yaml')
        
    logger = LaunchConfiguration("log_level")
    
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
            ),

        # PATROL
        Node(
            package='patrol',
            executable='patrol_times',
            name='patrol_times',
            output='screen',
            parameters=[params_yaml_file]
            )
    ])
