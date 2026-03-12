from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rs485_hardware_cpp",
            executable="rs485_hw_node",
            name="rs485_hw",
            output="screen",
            parameters=["{}/config/params.yaml".format(
                __import__('ament_index_python.packages').packages.get_package_share_directory('rs485_hardware_cpp')
            )]
        )
    ])
