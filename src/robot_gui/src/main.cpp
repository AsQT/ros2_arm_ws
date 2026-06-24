#include <memory>
#include <thread>

#include <QApplication>

#include "rclcpp/rclcpp.hpp"
#include "robot_gui/main_window.hpp"
#include "robot_gui/robot_gui_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto node = std::make_shared<robot_gui::RobotGuiNode>();
  robot_gui::MainWindow window(&app, node);
  window.show();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  const int result = app.exec();
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return result;
}
