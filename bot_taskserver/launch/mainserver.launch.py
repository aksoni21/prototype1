from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    
    task_server_node=Node(
        package='bot_taskserver',
        executable='simple_taskserver_node',
    )
 
    
    
    return LaunchDescription([
        task_server_node,
    ])