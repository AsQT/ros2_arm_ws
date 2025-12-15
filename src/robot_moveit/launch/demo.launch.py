# Chạy demo Moveit trong Rviz
import os
from launch                             import LaunchDescription
from launch.actions                     import IncludeLaunchDescription
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch_ros.actions                 import Node
from ament_index_python.packages        import get_package_share_directory
from moveit_configs_utils               import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config       = MoveItConfigsBuilder("robot", package_name="robot_moveit").to_moveit_configs()
    pkg_share           = get_package_share_directory("robot_moveit")
    rviz_config_file    = os.path.join(pkg_share, "config", "moveit.rviz")

    launch_control = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(get_package_share_directory("robot_moveit"), 
                                            "launch", 
                                            "control.launch.py") ),
                        launch_arguments={"is_sim": "False"}.items()
    )

    trajectory_execution = {
                            "moveit_manage_controllers": True,
                            "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                            "trajectory_execution.allowed_goal_duration_margin": 0.5,
                            "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    node_move_group = Node(
                    package     ="moveit_ros_move_group",
                    executable  ="move_group",
                    output      ="screen",
                    parameters  =[
                                moveit_config.to_dict(),
                                trajectory_execution,
                                {"use_sim_time": False},
                                ],
    )

    node_rviz = Node(
                package     ="rviz2",
                executable  ="rviz2",
                name        ="rviz2",
                output      ="log",
                arguments   =["-d", rviz_config_file],
                parameters  =[
                            moveit_config.robot_description,
                            moveit_config.robot_description_semantic,
                            moveit_config.planning_pipelines,
                            moveit_config.robot_description_kinematics,
                            ],
    )

    return LaunchDescription([
            launch_control,
            node_move_group,
            node_rviz,
    ])