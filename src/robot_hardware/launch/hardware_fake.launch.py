from launch             import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    fake_data_node = Node(
                    package     ="robot_hardware",
                    executable  ="fake_data_node",
                    name        ="fake_data_node",
                    output      ="screen" )

    return LaunchDescription([
        fake_data_node ])
