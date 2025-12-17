import os
from launch                             import LaunchDescription
from launch.actions                     import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions                  import IfCondition
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch.substitutions               import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions                 import Node
from launch_ros.substitutions           import FindPackageShare

def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")

    declare_use_sim = DeclareLaunchArgument(
                        "use_sim", 
                        default_value   ="true",
                        description     ="true: Gazebo sim, false: real hardware"  )

    # robot_description from xacro
    xacro_file = PathJoinSubstitution([
                    FindPackageShare("robot_description"),
                    "urdf", 
                    "robot.urdf.xacro" ])

    robot_description = {
                        "robot_description": Command([
                                                "xacro ", xacro_file, 
                                                " use_sim:=", use_sim ])  }

    controllers_yaml = PathJoinSubstitution([
                        FindPackageShare("robot_controller"),
                        "config", "ros2_controllers.yaml" ])

    # ros2_control_node (works for both sim & real, hardware plugin chosen by xacro)
    control_node = Node(
                    package     ="controller_manager",
                    executable  ="ros2_control_node",
                    parameters  =[robot_description, controllers_yaml],
                    output      ="screen",  )

    # spawn controllers
    jsb = Node(
            package     ="controller_manager",
            executable  ="spawner",
            arguments   =[
                        "joint_state_broadcaster", 
                        "--controller-manager", 
                        "/controller_manager"],
            output      ="screen",  )

    arm = Node(
            package     ="controller_manager",
            executable  ="spawner",
            arguments   =[
                        "arm_controller", 
                        "-c", 
                        "/controller_manager"],
            output      ="screen", )

    # (optional) include gazebo when sim
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                                                FindPackageShare("ros_gz_sim"),
                                                "launch", "gz_sim.launch.py"  ])),
                condition=IfCondition(use_sim),  )

    return LaunchDescription([
            declare_use_sim,
            gazebo,
            control_node,
            jsb,
            arm, ])
