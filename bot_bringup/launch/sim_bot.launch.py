from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bot_desc"),
            "launch", 
            "bot.launch.py")
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bot_controller"),
            "launch", 
            "controller.launch.py"),
        launch_arguments={"is_sim":"True"}.items()
    )

    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bot_misa"),
            "launch", 
            "misa.launch.py"),
        launch_arguments={"is_sim":"True"}.items()
    )

    # remote_interface = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("bot_taskserver"),
    #         "launch", 
    #         "mainserver.launch.py"),
    # )

    return LaunchDescription([
        moveit,
        gazebo,
        controller,
        # remote_interface,
    ])
