// include/manymove_signals/signals.hpp

#pragma once

#include <memory>
#include <string>
#include <map>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Action Definitions
#include "manymove_signals/action/set_io.hpp"
#include "manymove_signals/action/get_io.hpp"
#include "manymove_signals/action/reset_robot_state.hpp"

// Service Definitions
#include "xarm_msgs/srv/set_digital_io.hpp"
#include "xarm_msgs/srv/get_digital_io.hpp"
#include "xarm_msgs/srv/set_int16.hpp"
#include "xarm_msgs/srv/set_int16_by_id.hpp"
#include "xarm_msgs/srv/call.hpp"

// Message Definitions
#include "xarm_msgs/msg/robot_msg.hpp"

namespace manymove_signals
{

class SignalsNode : public rclcpp::Node
{
public:
    using SetIO = manymove_signals::action::SetIO;
    using GetIO = manymove_signals::action::GetIO;
    using ResetRobotState = manymove_signals::action::ResetRobotState;

    SignalsNode();

private:
    // Member Variables
    std::string robot_model_;

    // Service Clients
    std::map<std::string, rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr> set_digital_io_clients_;
    std::map<std::string, rclcpp::Client<xarm_msgs::srv::GetDigitalIO>::SharedPtr> get_digital_io_clients_;
    std::map<std::string, rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr> set_int16_clients_;

    rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr clean_error_client_;
    rclcpp::Client<xarm_msgs::srv::SetInt16ById>::SharedPtr motion_enable_client_;

    // Action Servers
    rclcpp_action::Server<SetIO>::SharedPtr set_io_server_;
    rclcpp_action::Server<GetIO>::SharedPtr get_io_server_;
    rclcpp_action::Server<ResetRobotState>::SharedPtr reset_robot_state_server_;

    // Callback Groups
    rclcpp::CallbackGroup::SharedPtr action_callback_group_;
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;

    // Subscription
    rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr robot_state_subscription_;

    // Robot State
    xarm_msgs::msg::RobotMsg::SharedPtr current_robot_state_;
    std::mutex robot_state_mutex_;

    // Initialization Methods
    void initialize_service_clients();
    bool wait_for_services();
    void initialize_action_servers();
    void initialize_robot_state_subscription();

    // Callback Methods
    void robot_state_callback(const xarm_msgs::msg::RobotMsg::SharedPtr msg);

    // SetIO Action Callbacks
    rclcpp_action::GoalResponse handle_set_io_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const SetIO::Goal> goal);
    rclcpp_action::CancelResponse handle_set_io_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetIO>> goal_handle);
    void execute_set_io(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetIO>> goal_handle);

    // GetIO Action Callbacks
    rclcpp_action::GoalResponse handle_get_io_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const GetIO::Goal> goal);
    rclcpp_action::CancelResponse handle_get_io_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetIO>> goal_handle);
    void execute_get_io(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetIO>> goal_handle);

    // ResetRobotState Action Callbacks
    rclcpp_action::GoalResponse handle_reset_robot_state_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ResetRobotState::Goal> goal);
    rclcpp_action::CancelResponse handle_reset_robot_state_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle);
    void execute_reset_robot_state(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle);
};

} // namespace manymove_signals
