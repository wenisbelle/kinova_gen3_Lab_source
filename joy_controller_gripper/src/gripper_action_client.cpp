#include <cstddef>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>

#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_result_kind.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/gripper_command.hpp"


class GripperJoyClient : public rclcpp::Node {
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

  explicit GripperJoyClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("gripper_joy_client", node_options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<GripperCommand>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "robotiq_gripper_controller");

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&MyActionClient::send_goal, this));
    
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&GripperJoyClient::joy_callback, this, std::placeholders::_1));
    
     joint_states_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&GripperJoyClient::joint_states_callback, this, std::placeholders::_1));
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = GripperCommand::Goal();
    // goal_msg =  ;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<GripperCommand>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&GripperJoyClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
        std::bind(&GripperJoyClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&GripperJoyClient::result_callback, this, _1);

    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<GripperCommand>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  bool goal_done_;

  void goal_response_callback(
      std::shared_future<GoalHandleGripperCommand::SharedPtr> & goal_handle) {
    //auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleGripperCommand::SharedPtr,
      const std::shared_ptr<const GripperCommand::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Feedback received: %f",
                feedback->current_total);
  }

  void result_callback(const GoalHandleGripperCommand::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    for (int i = 0; i < result.result->list_of_odoms.size(); ++i) {
      /*RCLCPP_INFO(this->get_logger(), "X: %f. Y: %f. Theta: %f",
                  result.result->list_of_odoms[i].x,
                  result.result->list_of_odoms[i].y,
                  result.result->list_of_odoms[i].z);*/

      std::cout << "Dado salvo: " << i + 1 << std::endl;
      std::cout << "X: . " << result.result->list_of_odoms[i].x << std::endl;
      std::cout << "Y: . " << result.result->list_of_odoms[i].y << std::endl;
      std::cout << "THETA: . " << result.result->list_of_odoms[i].z
                << std::endl;
    }
    rclcpp::shutdown();
  }
}; // class MyActionClient

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<MyActionClient>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_client);

  while (!action_client->is_goal_done()) {
    executor.spin();
  }

  rclcpp::shutdown();
  return 0;
}