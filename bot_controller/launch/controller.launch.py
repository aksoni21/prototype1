from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition


def generate_launch_description():

    # this determines if is_sim is true or false during launch
    # if this launch file is called with an argument is_sim is false
    is_sim = LaunchConfiguration("is_sim")
    is_sim_arg = DeclareLaunchArgument(
        "is_sim", default_value="True")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ", os.path.join(get_package_share_directory(
                    "bot_desc"), 'urdf', 'bot.urdf.xacro'),
                " is_sim:=False"
            ]
        ),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"robot_description": robot_description}],
        condition=UnlessCondition(is_sim)
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{"robot_description": robot_description, "use_sim_time": is_sim}, os.path.join(
            get_package_share_directory("bot_controller"), "config", "controllers.yaml")],
        condition=UnlessCondition(is_sim)
    )

    joint_state_broadcast_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            # "--controller_manager","/controller_manager"
        ]
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            # "--controller_manager","/controller_manager"
        ]
    )

    handle_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "handle_controller",
            # "--controller_manager","/controller_manager"
        ]
    )

    return LaunchDescription([
        is_sim_arg,
        robot_state_publisher_node,
        controller_manager,
        joint_state_broadcast_spawner,
        arm_controller_spawner,
        handle_controller_spawner,
    ])
