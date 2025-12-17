# use_sim:=true/false

from launch                             import LaunchDescription
from launch.actions                     import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions                  import IfCondition, UnlessCondition
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch.substitutions               import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions           import FindPackageShare

def generate_launch_description():

    use_sim = LaunchConfiguration("use_sim")

    declare = [
        DeclareLaunchArgument("use_sim",    default_value="true"),
        DeclareLaunchArgument("port",       default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud",       default_value="115200"), ]

    # Start ros2_control + controllers (both sim/real)
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
                                        FindPackageShare("robot_controller"),
                                                        "launch", 
                                                        "control.launch.py" ])),
        launch_arguments={
                            "use_sim": use_sim,
                            "port": LaunchConfiguration("port"),
                            "baud": LaunchConfiguration("baud"), }.items()  )

    # Gazebo only when sim (tùy đổi sang ros_gz_sim + spawn_entity)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("robot_gazebo"), "launch", "sim.launch.py"   ])),
        condition=IfCondition(use_sim), )

    # (optional) MoveIt only when you want, có thể bật cả sim/real
    moveit = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare("robot_moveit"), "launch", "moveit.launch.py"  ])),  )

    return LaunchDescription(declare + [gazebo, control, moveit])
