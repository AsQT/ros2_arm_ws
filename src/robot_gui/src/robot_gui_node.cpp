#include "robot_gui/robot_gui_node.hpp"

#include <sstream>
#include <utility>

#include "rclcpp/qos.hpp"

namespace robot_gui
{

RobotGuiNode::RobotGuiNode(const rclcpp::NodeOptions & options)
: Node("robot_gui", options)
{
  joint_names_ = declare_parameter<std::vector<std::string>>(
    "joint_names", {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
  embed_rviz_ = declare_parameter<bool>("embed_rviz", true);
  initial_page_ = declare_parameter<int>("initial_page", -1);
  rviz_config_package_ = declare_parameter<std::string>("rviz_config_package", "robot_moveit");
  rviz_config_relative_path_ = declare_parameter<std::string>(
    "rviz_config_relative_path", "config/moveit.rviz");

  raw_image_topic_ = declare_parameter<std::string>(
    "image_topics.raw", "/camera/color/image_raw");
  detection_image_topic_ = declare_parameter<std::string>(
    "image_topics.detection", "/yolo/detection_image");
  yolo_image_topic_ = declare_parameter<std::string>(
    "image_topics.yolo", "/yolo/image");

  subscribe_image("raw", raw_image_topic_);
  subscribe_image("detection", detection_image_topic_);
  subscribe_image("yolo", yolo_image_topic_);

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      if (joint_state_callback_) {
        joint_state_callback_(msg->name, msg->position, msg->velocity);
      }
    });

  flags_sub_ = create_subscription<robot_hardware_interface::msg::FlagStatus>(
    "/robot_hw/flags", rclcpp::SensorDataQoS(),
    [this](const robot_hardware_interface::msg::FlagStatus::SharedPtr msg) {
      std::vector<uint32_t> flags;
      flags.reserve(msg->axes.size());
      for (const auto & axis : msg->axes) {
        flags.push_back(axis.status_f);
      }
      if (flag_callback_) {
        flag_callback_(flags);
      }
    });

  servo_all_client_ = create_client<std_srvs::srv::SetBool>("/robot_hw/servo_all");
  arm_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/arm_controller/joint_trajectory", 10);
  gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/gripper_controller/joint_trajectory", 10);
}

void RobotGuiNode::subscribe_image(const std::string & panel, const std::string & topic)
{
  if (topic.empty()) {
    RCLCPP_INFO(get_logger(), "Image topic for %s: not configured", panel.c_str());
    return;
  }
  auto sub = create_subscription<sensor_msgs::msg::Image>(
    topic, rclcpp::SensorDataQoS(),
    [this, panel](const sensor_msgs::msg::Image::SharedPtr msg) {
      if (image_callback_) {
        image_callback_(panel, msg);
      }
    });
  image_subs_.push_back(sub);
  RCLCPP_INFO(get_logger(), "Image topic for %s: %s", panel.c_str(), topic.c_str());
}

void RobotGuiNode::set_image_callback(ImageCallback callback) {image_callback_ = std::move(callback);}
void RobotGuiNode::set_joint_state_callback(JointStateCallback callback) {joint_state_callback_ = std::move(callback);}
void RobotGuiNode::set_flag_callback(FlagCallback callback) {flag_callback_ = std::move(callback);}
void RobotGuiNode::set_log_callback(LogCallback callback) {log_callback_ = std::move(callback);}

bool RobotGuiNode::embed_rviz() const {return embed_rviz_;}
int RobotGuiNode::initial_page() const {return initial_page_;}
std::string RobotGuiNode::rviz_config_package() const {return rviz_config_package_;}
std::string RobotGuiNode::rviz_config_relative_path() const {return rviz_config_relative_path_;}
const std::vector<std::string> & RobotGuiNode::joint_names() const {return joint_names_;}
const std::string & RobotGuiNode::raw_image_topic() const {return raw_image_topic_;}
const std::string & RobotGuiNode::detection_image_topic() const {return detection_image_topic_;}
const std::string & RobotGuiNode::yolo_image_topic() const {return yolo_image_topic_;}

void RobotGuiNode::set_servo_all(bool enabled)
{
  if (!servo_all_client_ || !servo_all_client_->service_is_ready()) {
    log("/robot_hw/servo_all service is not available");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  servo_all_client_->async_send_request(
    request,
    [this, enabled](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      try {
        const auto response = future.get();
        std::ostringstream stream;
        stream << "/robot_hw/servo_all " << (response->success ? "OK" : "FAILED")
               << " data=" << (enabled ? "true" : "false")
               << " " << response->message;
        log(stream.str());
      } catch (const std::exception & exc) {
        log(std::string("/robot_hw/servo_all response failed: ") + exc.what());
      }
    });
}

void RobotGuiNode::log(const std::string & message) const
{
  RCLCPP_INFO(get_logger(), "%s", message.c_str());
  if (log_callback_) {
    log_callback_(message);
  }
}

}  // namespace robot_gui
