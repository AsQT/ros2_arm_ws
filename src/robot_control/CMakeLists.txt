cmake_minimum_required(VERSION 3.8)
project(robot_control)

# Tìm dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 1. Cài đặt file Python (Sửa lỗi Exec format error)
install(PROGRAMS
  launch/slider_control.py  # <-- Đảm bảo đúng đường dẫn tới file code của ông
  DESTINATION lib/${PROJECT_NAME}
  RENAME slider_control
)

# 2. Cài đặt folder Config (Sửa lỗi Parameter file not found)
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()