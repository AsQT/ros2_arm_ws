from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    use_gui = LaunchConfiguration("use_gui")

    params_file = "{}/config/params.yaml".format(
        __import__("ament_index_python.packages").packages.get_package_share_directory("rs485_hardware_cpp")
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_gui", default_value="true"),

        Node(
            package="rs485_hardware_cpp",
            executable="rs485_hw_node",
            name="rs485_hw",
            output="screen",
            parameters=[params_file],
        ),

        Node(
            package="rs485_hardware_cpp",
            executable="rs485_hw_gui.py",
            name="rs485_hw_gui",
            output="screen",
            condition=IfCondition(use_gui),
            # optional: pass watch joints / axis ids here if you want
            # parameters=[{"watch_joints": ["joint1","joint2"], "axis_ids": [0,1]}],
        ),
    ])
