# bridge giữa URDF ↔ Controller ↔ Hardware/Sim
import os
from ament_index_python.packages    import get_package_share_directory
from launch                         import LaunchDescription
from launch.actions                 import DeclareLaunchArgument
from launch.conditions              import UnlessCondition
from launch.substitutions           import Command, LaunchConfiguration
from launch_ros.actions             import Node

def generate_launch_description():
    #________________ Tham số quan trọng __________________________
    is_sim_arg = DeclareLaunchArgument(
                "is_sim",
                default_value   ="False",
                description     ="True chay Gazebo, False chay Robot that" )
    is_sim = LaunchConfiguration("is_sim")

    moveit_pkg          = get_package_share_directory("robot_moveit")
    description_pkg     = get_package_share_directory("robot_description")

    xacro_file          = os.path.join(description_pkg, "urdf", "robot.urdf.xacro")
    controllers_file    = os.path.join(moveit_pkg, "config", "ros2_controllers.yaml")

    robot_description_content = Command([
                                "xacro ", xacro_file,
                                " use_sim:=", is_sim  ]) # chạy mô phỏng
    # load mô hình robot
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
                                    package     ="robot_state_publisher",
                                    executable  ="robot_state_publisher",
                                    output      ="screen",
                                    parameters  =[robot_description, {"use_sim_time": is_sim}], )

    node_ros2_control = Node(
                        package     ="controller_manager",
                        executable  ="ros2_control_node",
                        parameters  =[robot_description, controllers_file],
                        output      ="screen",
                        condition   =UnlessCondition(is_sim)  )

    spawn_jsb = Node(
                package     ="controller_manager",
                executable  ="spawner",
                arguments   =[
                            "joint_state_broadcaster",
                            "--controller-manager", 
                            "/controller_manager"],
                output      ="screen",)

    spawn_arm = Node(
                package     ="controller_manager",
                executable  ="spawner",
                arguments   =[
                            "arm_controller", 
                            "--controller-manager", 
                            "/controller_manager"],
                output      ="screen", )

    spawn_gripper = Node(
                    package     ="controller_manager",
                    executable  ="spawner",
                    arguments   =[
                                "gripper_controller", 
                                "--controller-manager", 
                                "/controller_manager"],
                    output      ="screen", )

    node_static_tf = Node(
                        package     ="tf2_ros",
                        executable  ="static_transform_publisher",
                        arguments   =[
                                    "0.0", "0.0", "0.0", 
                                    "0.0", "0.0", "0.0", 
                                    "world", "base_link"],
                        condition   =UnlessCondition(is_sim) )

    return LaunchDescription([
            is_sim_arg,
            node_static_tf,
            node_robot_state_publisher,
            node_ros2_control,
            spawn_jsb,
            spawn_arm,
            spawn_gripper, ])