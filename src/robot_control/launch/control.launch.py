# PKG Controllers + bringup ros2_control (dùng chung sim/real)
# start ros2_control_node + spawner
import os
from ament_index_python.packages    import get_package_share_directory
from launch                         import LaunchDescription
from launch.actions                 import DeclareLaunchArgument
from launch.conditions              import UnlessCondition
from launch.substitutions           import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions             import Node
from launch_ros.substitutions       import FindPackageShare

def generate_launch_description():

    use_sim = LaunchConfiguration("use_sim")
    port    = LaunchConfiguration("port")
    baud    = LaunchConfiguration("baud")

    declare = [
        DeclareLaunchArgument("use_sim", default_value="true"),
        DeclareLaunchArgument("port",    default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud",    default_value="115200"), ]

    is_sim_arg  = DeclareLaunchArgument(
                    "is_sim",
                    default_value   ="False",
                    description     ="True neu chay Gazebo, False neu chay Robot that" )
    
    is_sim = LaunchConfiguration("is_sim")

    use_sim_time_arg = DeclareLaunchArgument(
                        "use_sim_time",
                        default_value   ="False",  
                        description     ="Use simulation (Gazebo) clock if true" )

    use_sim_time = LaunchConfiguration("use_sim_time")

    moveit_pkg = get_package_share_directory("robot_moveit")
    description_pkg = get_package_share_directory("robot_description")
    
    xacro_file = os.path.join(description_pkg, "urdf", "robot.urdf.xacro")
    controllers_file = os.path.join(moveit_pkg, "config", "ros2_controllers.yaml")

    # ---  LOAD ROBOT DESCRIPTION ---
    robot_description = {
                        "robot_description": Command([
                                                "xacro ",       xacro_file,
                                                " use_sim:=",   use_sim,
                                                " port:=",      port,
                                                " baud:=",      baud, ]) }

    # ---  ROBOT STATE PUBLISHER (Dùng chung cho cả Sim và Real) ---
    controllers = PathJoinSubstitution([
                        FindPackageShare("robot_control"), 
                                        "config", "ros2_controllers.yaml" ])

    node_robot_state_publisher = Node(
                                    package     ="robot_state_publisher",
                                    executable  ="robot_state_publisher",
                                    outpu       ="screen",
                                    parameters  =[robot_description, {"use_sim_time": use_sim_time}], )

    # --- 5. ROS2 CONTROL NODE (CHỈ CHẠY KHI KHÔNG PHẢI SIM) ---
    # Nếu là Sim, Gazebo sẽ tự load plugin controller, ta không chạy node này
    node_ros2_control = Node(
                        package     ="controller_manager",
                        executable  ="ros2_control_node",
                        parameters  =[robot_description, controllers_file],
                        output      ="screen",
                        condition   =UnlessCondition(is_sim) )

    # --- 6. SPAWNERS (Dùng chung) ---
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
    
    # --- 7. STATIC TF (Optional: Nếu robot thật cần tf cố định ra world) ---
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
                use_sim_time_arg,
                node_static_tf,
                node_robot_state_publisher,
                node_ros2_control,
                spawn_jsb,
                spawn_arm,
                spawn_gripper, ])
    #return LaunchDescription(declare + [ros2_control_node, jsb, arm])
