#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "robot_hardware_interface/msg/flag_status.hpp"

namespace robot_gui
{

class RobotGuiNode : public rclcpp::Node
{
public:
  using ImageCallback = std::function<void(const std::string &, const sensor_msgs::msg::Image::SharedPtr &)>;
  using JointStateCallback = std::function<void(
    const std::vector<std::string> &, const std::vector<double> &, const std::vector<double> &)>;
  using FlagCallback = std::function<void(const std::vector<uint32_t> &)>;
  using LogCallback = std::function<void(const std::string &)>;

  explicit RobotGuiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void set_image_callback(ImageCallback callback);
  void set_joint_state_callback(JointStateCallback callback);
  void set_flag_callback(FlagCallback callback);
  void set_log_callback(LogCallback callback);

  bool embed_rviz() const;
  int initial_page() const;
  std::string rviz_config_package() const;
  std::string rviz_config_relative_path() const;
  const std::vector<std::string> & joint_names() const;
  const std::string & raw_image_topic() const;
  const std::string & detection_image_topic() const;
  const std::string & yolo_image_topic() const;

  void set_servo_all(bool enabled);

private:
  void subscribe_image(const std::string & panel, const std::string & topic);
  void log(const std::string & message) const;

  ImageCallback image_callback_;
  JointStateCallback joint_state_callback_;
  FlagCallback flag_callback_;
  LogCallback log_callback_;

  bool embed_rviz_{true};
  int initial_page_{-1};
  std::string rviz_config_package_{"robot_moveit"};
  std::string rviz_config_relative_path_{"config/moveit.rviz"};
  std::vector<std::string> joint_names_;
  std::string raw_image_topic_;
  std::string detection_image_topic_;
  std::string yolo_image_topic_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<robot_hardware_interface::msg::FlagStatus>::SharedPtr flags_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr servo_all_client_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
};

}  // namespace robot_gui
