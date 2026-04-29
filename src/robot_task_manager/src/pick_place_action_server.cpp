#include <memory>
#include <thread>
#include <future>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_task_manager/action/pick_place.hpp"
#include "robot_task_manager/moveit_executor.hpp"

#include "control_msgs/action/gripper_command.hpp"

using namespace std::chrono_literals;

class PickPlaceActionServer : public rclcpp::Node
{
public:
  using PickPlace = robot_task_manager::action::PickPlace;
  using GoalHandlePickPlace = rclcpp_action::ServerGoalHandle<PickPlace>;

  using GripperCommand = control_msgs::action::GripperCommand;
  using GripperGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

  PickPlaceActionServer()
  : Node("pick_place_action_server")
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "arm");
    base_frame_ = declare_parameter<std::string>("base_frame", "world");
    gripper_action_name_ = declare_parameter<std::string>(
      "gripper_action_name",
      "/gripper_controller/gripper_cmd");

    action_server_ = rclcpp_action::create_server<PickPlace>(
      this,
      "pick_place",
      std::bind(&PickPlaceActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PickPlaceActionServer::handle_accepted, this, std::placeholders::_1));

    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      this,
      gripper_action_name_);

    RCLCPP_INFO(get_logger(), "PickPlace action server ready");
  }

  void initialize_moveit()
  {
    executor_ = std::make_shared<robot_task_manager::MoveItExecutor>();
    executor_->initialize(shared_from_this(), planning_group_, base_frame_);
  }

private:
  std::string planning_group_;
  std::string base_frame_;
  std::string gripper_action_name_;

  std::shared_ptr<robot_task_manager::MoveItExecutor> executor_;

  rclcpp_action::Server<PickPlace>::SharedPtr action_server_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;

  static constexpr double GRIPPER_OPEN_MAX = 0.05;
  static constexpr double LIFT_HEIGHT = 0.10;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PickPlace::Goal> goal)
  {
    if (goal->velocity_scale <= 0.0 || goal->velocity_scale > 1.0) {
      RCLCPP_WARN(get_logger(), "Reject goal: velocity_scale must be in (0, 1]");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->gripper_close < 0.0 || goal->gripper_close > GRIPPER_OPEN_MAX) {
      RCLCPP_WARN(get_logger(), "Reject goal: gripper_close must be in [0.0, 0.05]");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePickPlace>)
  {
    RCLCPP_WARN(get_logger(), "Cancel pick_place received");

    if (executor_) {
      executor_->stop();
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<GoalHandlePickPlace> goal_handle)
  {
    std::thread(&PickPlaceActionServer::execute, this, goal_handle).detach();
  }

  void publish_step(
    const std::shared_ptr<GoalHandlePickPlace> & goal_handle,
    const std::string & stage,
    float progress)
  {
    auto feedback = std::make_shared<PickPlace::Feedback>();
    feedback->stage = stage;
    feedback->progress = progress;
    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(get_logger(), "[PickPlace] %s | %.1f%%", stage.c_str(), progress);
  }

  bool move_cartesian(
    const geometry_msgs::msg::Pose & pose,
    double velocity_scale,
    std::string & error_msg)
  {
    return executor_->moveToPoseCartesian(
      pose,
      error_msg,
      velocity_scale,
      0.3,
      5.0);
  }

  bool move_gripper(double position, std::string & error_msg)
  {
    if (!gripper_client_->wait_for_action_server(5s)) {
      error_msg = "Gripper action server not available: " + gripper_action_name_;
      return false;
    }

    GripperCommand::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = 50.0;

    auto send_goal_options =
      rclcpp_action::Client<GripperCommand>::SendGoalOptions();

    auto goal_handle_future = gripper_client_->async_send_goal(goal, send_goal_options);

    if (goal_handle_future.wait_for(5s) != std::future_status::ready) {
      error_msg = "Timeout while sending gripper goal";
      return false;
    }

    auto goal_handle = goal_handle_future.get();

    if (!goal_handle) {
      error_msg = "Gripper goal rejected";
      return false;
    }

    auto result_future = gripper_client_->async_get_result(goal_handle);

    if (result_future.wait_for(10s) != std::future_status::ready) {
      error_msg = "Timeout while waiting gripper result";
      return false;
    }

    auto result = result_future.get();

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      error_msg = "Gripper action failed";
      return false;
    }

    return true;
  }

  void execute(
    const std::shared_ptr<GoalHandlePickPlace> goal_handle)
  {
    auto result = std::make_shared<PickPlace::Result>();

    if (!executor_) {
      result->success = false;
      result->message = "MoveItExecutor not initialized";
      goal_handle->abort(result);
      return;
    }

    const auto goal = goal_handle->get_goal();

    std::string error_msg;

    geometry_msgs::msg::Pose current_pose = executor_->getCurrentPose();

    geometry_msgs::msg::Pose pick_xy_pose = goal->pick_pose;
    pick_xy_pose.position.z = current_pose.position.z;

    geometry_msgs::msg::Pose pick_pose = goal->pick_pose;

    geometry_msgs::msg::Pose lift_pose = goal->pick_pose;
    lift_pose.position.z += LIFT_HEIGHT;

    geometry_msgs::msg::Pose pre_place_pose = goal->place_pose;
    pre_place_pose.position.z += LIFT_HEIGHT;

    geometry_msgs::msg::Pose place_pose = goal->place_pose;

    publish_step(goal_handle, "Move XY to pick position, keep current Z", 10.0f);
    if (!move_cartesian(pick_xy_pose, goal->velocity_scale, error_msg)) {
      result->success = false;
      result->message = error_msg;
      goal_handle->abort(result);
      return;
    }

    publish_step(goal_handle, "Open gripper to 5cm", 25.0f);
    if (!move_gripper(GRIPPER_OPEN_MAX, error_msg)) {
      result->success = false;
      result->message = error_msg;
      goal_handle->abort(result);
      return;
    }

    publish_step(goal_handle, "Move down to pick pose", 40.0f);
    if (!move_cartesian(pick_pose, goal->velocity_scale, error_msg)) {
      result->success = false;
      result->message = error_msg;
      goal_handle->abort(result);
      return;
    }

    publish_step(goal_handle, "Close gripper", 50.0f);
    if (!move_gripper(goal->gripper_close, error_msg)) {
      result->success = false;
      result->message = error_msg;
      goal_handle->abort(result);
      return;
    }

    publish_step(goal_handle, "Lift object Z + 0.1m", 65.0f);
    if (!move_cartesian(lift_pose, goal->velocity_scale, error_msg)) {
      result->success = false;
      result->message = error_msg;
      goal_handle->abort(result);
      return;
    }

    publish_step(goal_handle, "Move to place position with Z + 0.1m", 80.0f);
    if (!move_cartesian(pre_place_pose, goal->velocity_scale, error_msg)) {
      result->success = false;
      result->message = error_msg;
      goal_handle->abort(result);
      return;
    }

    publish_step(goal_handle, "Move down to place pose", 90.0f);
    if (!move_cartesian(place_pose, goal->velocity_scale, error_msg)) {
      result->success = false;
      result->message = error_msg;
      goal_handle->abort(result);
      return;
    }

    publish_step(goal_handle, "Open gripper release object", 95.0f);
    if (!move_gripper(GRIPPER_OPEN_MAX, error_msg)) {
      result->success = false;
      result->message = error_msg;
      goal_handle->abort(result);
      return;
    }

    publish_step(goal_handle, "Pick and place done", 100.0f);

    result->success = true;
    result->message = "Pick and place executed successfully";
    goal_handle->succeed(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PickPlaceActionServer>();
  node->initialize_moveit();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}