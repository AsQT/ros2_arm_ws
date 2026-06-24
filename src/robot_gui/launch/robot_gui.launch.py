from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("robot_gui"),
        "config",
        "config.yaml",
    ])

    embed_rviz_arg = DeclareLaunchArgument("embed_rviz", default_value="true")
    initial_page_arg = DeclareLaunchArgument("initial_page", default_value="-1")

    gui = Node(
        package="robot_gui",
        executable="robot_gui_node",
        output="screen",
        parameters=[
            config_file,
            {
                "embed_rviz": ParameterValue(LaunchConfiguration("embed_rviz"), value_type=bool),
                "initial_page": ParameterValue(LaunchConfiguration("initial_page"), value_type=int),
                "joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            },
        ],
    )

    return LaunchDescription([embed_rviz_arg, initial_page_arg, gui])
