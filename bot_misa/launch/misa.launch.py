import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    is_sim = LaunchConfiguration('is_sim')

    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='True'
    )
    kinematics_yaml_file = os.path.join(
        get_package_share_directory("bot_misa"), "config", "kinematics.yaml")

    moveit_config = (
        MoveItConfigsBuilder("proto_bot", package_name="bot_misa")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("bot_desc"),
            "urdf",
            "bot.urdf.xacro"
        )
        )
        .robot_description_semantic(file_path="config/proto_bot.srdf")

        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # declare_kinematics_arg = DeclareLaunchArgument(
    #     'kinematics_config',
    #     default_value=kinematics_yaml_file,
    #     description='Path to the kinematics configuration file'
    # )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    {'use_sim_time': is_sim},
                    {'publish_robot_description_semantic': True},
                    # LaunchConfiguration('kinematics_config')
                    ],
        arguments=["--ros-args", "--log-level", "info"],

    )

    rviz_config = os.path.join(get_package_share_directory(
        "bot_misa"), "config", "moveit_rviz.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits]
    )

    return LaunchDescription([
        is_sim_arg,
        # declare_kinematics_arg,
        move_group_node,
        rviz_node
    ])
