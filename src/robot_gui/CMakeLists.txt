cmake_minimum_required(VERSION 3.16)
project(robot_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(robot_hardware_interface REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

add_executable(robot_gui_node
  include/robot_gui/main_window.hpp
  include/robot_gui/robot_gui_node.hpp
  include/robot_gui/rviz_panel.hpp
  src/main.cpp
  src/main_window.cpp
  src/robot_gui_node.cpp
  src/rviz_panel.cpp
  ui/robot_gui.ui
)

target_include_directories(robot_gui_node PRIVATE
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

ament_target_dependencies(robot_gui_node
  rclcpp
  sensor_msgs
  std_srvs
  trajectory_msgs
  robot_hardware_interface
  ament_index_cpp
  rviz_common
  rviz_rendering
  pluginlib
)

target_link_libraries(robot_gui_node Qt5::Widgets Qt5::Core Qt5::Gui)

install(TARGETS robot_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch ui config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
