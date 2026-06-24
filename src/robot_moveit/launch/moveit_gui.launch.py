import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock = LaunchConfiguration("use_mock")
    start_controller_manager = LaunchConfiguration("start_controller_manager")

    robot_description_pkg = get_package_share_directory("robot_description")
    robot_control_pkg = get_package_share_directory("robot_control")
    robot_gui_pkg = get_package_share_directory("robot_gui")

    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="robot_moveit")
        .robot_description(
            file_path=os.path.join(robot_description_pkg, "urdf", "robot.urdf.xacro"),
            mappings={
                "use_sim": use_sim_time,
                "use_mock_hardware": use_mock,
            },
        )
        .robot_description_semantic(file_path="config/robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    node_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_base",
        output="log",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "base_link",
        ],
    )

    node_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_control_pkg, "launch", "controllers.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_mock": use_mock,
        }.items(),
        condition=IfCondition(start_controller_manager),
    )

    gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_gui_pkg, "launch", "robot_gui.launch.py")
        ),
        launch_arguments={
            "embed_rviz": "true",
            "initial_page": "1",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time",
            ),
            DeclareLaunchArgument(
                "use_mock",
                default_value="true",
                description="Use mock hardware",
            ),
            DeclareLaunchArgument(
                "start_controller_manager",
                default_value="true",
                description="Start ros2_control controller manager from robot_control",
            ),
            node_static_tf,
            node_move_group,
            controller,
            gui,
        ]
    )
