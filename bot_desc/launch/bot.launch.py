from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
import os
from os.path import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    urdf_file = 'bot.urdf.xacro'
    # package_desc = "bot_desc"

    proto_bot_desc = get_package_share_directory("bot_desc")
    proto_desc_prefix = get_package_prefix("bot_desc")

    # robot_desc_path = os.path.join(
    #     get_package_share_directory(package_desc), 'urdf', urdf_file)
    # rviz_config_dir = os.path.join(get_package_share_directory(
    #     package_desc), 'rviz',  'bot.rviz')


    model_path=os.path.join(proto_bot_desc, 'models')
    model_path += pathsep + os.path.join(proto_desc_prefix,'share')
    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(proto_bot_desc, 'urdf', 'bot.urdf.xacro'),
        description="Absolute path to the URDF model file"
    )


    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]))

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen',
        # arguments=['-d', rviz_config_dir]
    )
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
            # '--verbose',
        ],
        output='screen',
    )
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=[
            'gzclient',
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
            # '--verbose',
        ],
        output='screen',
        # arguments=[os.path.join(get_package_prefix("my_robot_bringup"), "share","my_world.world")],
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-entity", "proto_bot", "-topic", "robot_description"],
        output='screen',
    )
    return LaunchDescription([
        # joint_state_publisher_node,
        # rviz_node,
        env_variable,
        model_arg,
        robot_state_publisher_node,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawn_entity,
    ])
