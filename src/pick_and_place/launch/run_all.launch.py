import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # 1. Khai báo các Arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="True nếu dùng Gazebo, False nếu chạy Demo (Mock hardware)",
        )
    )
    
    # Lấy giá trị argument
    use_sim = LaunchConfiguration("use_sim")

    # 2. Xử lý Robot Description (Xacro)
    # Lưu ý: Truyền arg use_sim vào xacro để nó chọn plugin hardware đúng
    pkg_share = FindPackageShare("ten_package_cua_ban") # <--- THAY TÊN PACKAGE CỦA BẠN
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "robot.urdf.xacro"]) # <--- THAY ĐƯỜNG DẪN FILE XACRO
    
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            xacro_file, " ",
            "use_sim:=", use_sim,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 3. Node Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim}],
    )

    # 4. Cấu hình MoveIt (Load file moveit config)
    # Phần này khá dài nên thường load từ package moveit_config được tạo ra
    # Giả sử bạn đã có file moveit.launch.py riêng hoặc load trực tiếp ở đây.
    # Để đơn giản, ở đây mình ví dụ load file move_group.launch.py có sẵn
    
    # move_group = IncludeLaunchDescription( ... ) 
    # (Bạn nên giữ phần load move_group từ file demo cũ của bạn vào đây)

    # 5. Cấu hình Controller Manager & Spawner
    # Nếu chạy MOCK (Demo), ta cần tự chạy controller_manager. 
    # Nếu chạy GAZEBO, plugin của Gazebo sẽ tự chạy controller_manager.
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, PathJoinSubstitution([pkg_share, "config", "ros2_controllers.yaml"])],
        output="both",
        condition=UnlessCondition(use_sim) # Chỉ chạy khi KHÔNG dùng Gazebo
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"], # Thay arm_controller bằng tên controller của bạn
    )

    # 6. Cấu hình Gazebo (Chỉ chạy khi use_sim = true)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(), # Hoặc world của bạn
        condition=IfCondition(use_sim),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "my_robot",
            "-allow_renaming", "true",
        ],
        output="screen",
        condition=IfCondition(use_sim),
    )
    
    # 7. RViz (Chạy cả 2 trường hợp)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[robot_description, {"use_sim_time": use_sim}],
        # arguments=["-d", PathJoinSubstitution([pkg_share, "config", "config.rviz"])],
    )

    # --- TỔNG HỢP NODE ---
    nodes_to_start = [
        robot_state_publisher_node,
        ros2_control_node, 
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
        # move_group (thêm node move_group vào đây)
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
