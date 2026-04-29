#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_task_manager/action/gripper_move.hpp"
#include "robot_task_manager/gripper_executor.hpp"

class GripperActionServer : public rclcpp::Node
{
public:
  using GripperMove = robot_task_manager::action::GripperMove;
  using GoalHandleGripperMove =  rclcpp_action::ServerGoalHandle<GripperMove>;

    GripperActionServer()
    : Node("gripper_action_server")
    {
    action_server_ = rclcpp_action::create_server<GripperMove>(
                            this,
                            "gripper_move",
                            std::bind(&GripperActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                            std::bind(&GripperActionServer::handle_cancel, this, std::placeholders::_1),
                            std::bind(&GripperActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Action /gripper_move ready");
    }
    void initialize()
        {
        const std::string planning_group    =  declare_parameter<std::string>("planning_group", "gripper");
        const std::string base_frame        =  declare_parameter<std::string>("base_frame", "link_6");
        gripper_executor_                   =  std::make_shared<robot_task_manager::GripperExecutor>();
        gripper_executor_->initialize(
                            shared_from_this(),
                            planning_group,
                            base_frame);
        }

private:
  rclcpp_action::GoalResponse handle_goal(
                                const rclcpp_action::GoalUUID &,
                                std::shared_ptr<const GripperMove::Goal> goal)
  {
    if (goal->opening < 0.0 || goal->opening > 0.05) {
      RCLCPP_WARN(get_logger(), "Reject goal: opening out of range");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel( const std::shared_ptr<GoalHandleGripperMove>)
  {
    RCLCPP_WARN(get_logger(), "Cancel gripper action");

    if (gripper_executor_) {
      gripper_executor_->stop();
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGripperMove> goal_handle)
  {
    std::thread(
            &GripperActionServer::execute,
            this,
            goal_handle).detach();
  }

  void publish_feedback(
                const std::shared_ptr<GoalHandleGripperMove> & goal_handle,
                const std::string & stage,
                double target_opening)
  {
    auto feedback = std::make_shared<GripperMove::Feedback>();
    feedback->stage = stage;
    feedback->target_opening = target_opening;
    goal_handle->publish_feedback(feedback);
  }


  void execute(  const std::shared_ptr<GoalHandleGripperMove> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<GripperMove::Result>();

    double velocity_scale = goal->velocity_scale;
    double acceleration_scale = goal->acceleration_scale;

    if (velocity_scale <= 0.0) {
      velocity_scale = 0.5;
    }

    if (acceleration_scale <= 0.0) {
      acceleration_scale = 0.5;
    }

    std::string error_msg;

    publish_feedback(goal_handle, "Planning and executing gripper motion", goal->opening);

    const bool ok = gripper_executor_->moveToOpening(
      goal->opening,
      error_msg,
      velocity_scale,
      acceleration_scale);

    if (goal_handle->is_canceling()) {
      gripper_executor_->stop();

      result->success = false;
      result->message = "Gripper action canceled";
      goal_handle->canceled(result);
      return;
    }

    if (!ok) {
      result->success = false;
      result->message = error_msg;
      goal_handle->abort(result);
      return;
    }

    publish_feedback(goal_handle, "Done", goal->opening);

    result->success = true;
    result->message = "Gripper moved successfully";
    goal_handle->succeed(result);
  }

private:
  std::shared_ptr<robot_task_manager::GripperExecutor> gripper_executor_;

  rclcpp_action::Server<GripperMove>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GripperActionServer>();

  node->initialize();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}