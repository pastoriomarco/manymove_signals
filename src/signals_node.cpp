#include "manymove_signals/signals.hpp"

namespace manymove_signals
{

    SignalsNode::SignalsNode()
        : Node("manymove_signals_node")
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("robot_model", "lite6"); // default to lite6
        this->get_parameter("robot_model", robot_model_);

        // Define Callback Groups
        action_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        subscription_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Initialize service clients for IO operations and ResetRobotState
        initialize_service_clients();

        // Wait for all service clients to be available
        if (!wait_for_services())
        {
            RCLCPP_ERROR(this->get_logger(), "Not all required services are available. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // Initialize Action Servers with the action callback group
        initialize_action_servers();

        // Initialize Subscription for Robot States
        initialize_robot_state_subscription();

        RCLCPP_INFO(this->get_logger(), "manymove_signals_node has been started.");
    }

    // Initialize all necessary service clients based on robot model
    void SignalsNode::initialize_service_clients()
    {
        std::string namespace_prefix;
        if (robot_model_ == "lite6" || robot_model_ == "uf850")
        {
            namespace_prefix = "/ufactory";
        }
        else if (robot_model_ == "xarm")
        {
            namespace_prefix = "/xarm";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unsupported robot model: %s", robot_model_.c_str());
            throw std::runtime_error("Unsupported robot model");
        }

        // Tool GPIO
        set_digital_io_clients_["tool"] = this->create_client<xarm_msgs::srv::SetDigitalIO>(
            namespace_prefix + "/set_tgpio_digital",
            rmw_qos_profile_services_default,
            service_callback_group_);
        get_digital_io_clients_["tool"] = this->create_client<xarm_msgs::srv::GetDigitalIO>(
            namespace_prefix + "/get_tgpio_digital",
            rmw_qos_profile_services_default,
            service_callback_group_);

        // Controller GPIO
        set_digital_io_clients_["controller"] = this->create_client<xarm_msgs::srv::SetDigitalIO>(
            namespace_prefix + "/set_cgpio_digital",
            rmw_qos_profile_services_default,
            service_callback_group_);
        get_digital_io_clients_["controller"] = this->create_client<xarm_msgs::srv::GetDigitalIO>(
            namespace_prefix + "/get_cgpio_digital",
            rmw_qos_profile_services_default,
            service_callback_group_);

        // Initialize SetInt16 clients for mode and state
        set_int16_clients_["mode"] = this->create_client<xarm_msgs::srv::SetInt16>(
            namespace_prefix + "/set_mode",
            rmw_qos_profile_services_default,
            service_callback_group_);
        set_int16_clients_["state"] = this->create_client<xarm_msgs::srv::SetInt16>(
            namespace_prefix + "/set_state",
            rmw_qos_profile_services_default,
            service_callback_group_);

        // Initialize additional service clients for ResetRobotState
        clean_error_client_ = this->create_client<xarm_msgs::srv::Call>(
            namespace_prefix + "/clean_error",
            rmw_qos_profile_services_default,
            service_callback_group_);

        motion_enable_client_ = this->create_client<xarm_msgs::srv::SetInt16ById>(
            namespace_prefix + "/motion_enable",
            rmw_qos_profile_services_default,
            service_callback_group_);
    }

    // Wait for all service clients to be available
    bool SignalsNode::wait_for_services()
    {
        bool all_services_available = true;

        // Wait for SetDigitalIO services
        for (const auto &[io_type, client] : set_digital_io_clients_)
        {
            if (!client->wait_for_service(std::chrono::seconds(5)))
            {
                RCLCPP_ERROR(this->get_logger(), "SetDigitalIO service for IO type '%s' not available.", io_type.c_str());
                all_services_available = false;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "SetDigitalIO service for IO type '%s' is available.", io_type.c_str());
            }
        }

        // Wait for GetDigitalIO services
        for (const auto &[io_type, client] : get_digital_io_clients_)
        {
            if (!client->wait_for_service(std::chrono::seconds(5)))
            {
                RCLCPP_ERROR(this->get_logger(), "GetDigitalIO service for IO type '%s' not available.", io_type.c_str());
                all_services_available = false;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "GetDigitalIO service for IO type '%s' is available.", io_type.c_str());
            }
        }

        // Wait for SetInt16 services (mode and state)
        for (const auto &[param, client] : set_int16_clients_)
        {
            if (!client->wait_for_service(std::chrono::seconds(5)))
            {
                RCLCPP_ERROR(this->get_logger(), "SetInt16 service for '%s' not available.", param.c_str());
                all_services_available = false;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "SetInt16 service for '%s' is available.", param.c_str());
            }
        }

        // Wait for additional ResetRobotState services
        if (!clean_error_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "CleanError service not available.");
            all_services_available = false;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "CleanError service is available.");
        }

        if (!motion_enable_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "MotionEnable service not available.");
            all_services_available = false;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "MotionEnable service is available.");
        }

        return all_services_available;
    }

    // Initialize all action servers
    void SignalsNode::initialize_action_servers()
    {
        auto serverOptions = rcl_action_server_get_default_options();
        serverOptions.result_service_qos = rmw_qos_profile_services_default;

        // Action Server for SetIO
        set_io_server_ = rclcpp_action::create_server<SetIO>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "set_io",
            [this](const rclcpp_action::GoalUUID &uuid,
                   std::shared_ptr<const SetIO::Goal> goal) -> rclcpp_action::GoalResponse
            {
                return this->handle_set_io_goal(uuid, goal);
            },
            [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<SetIO>> goal_handle) -> rclcpp_action::CancelResponse
            {
                return this->handle_set_io_cancel(goal_handle);
            },
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetIO>> goal_handle) -> void
            {
                this->execute_set_io(goal_handle);
            },
            serverOptions,
            action_callback_group_);

        // Action Server for GetIO
        get_io_server_ = rclcpp_action::create_server<GetIO>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "get_io",
            [this](const rclcpp_action::GoalUUID &uuid,
                   std::shared_ptr<const GetIO::Goal> goal) -> rclcpp_action::GoalResponse
            {
                return this->handle_get_io_goal(uuid, goal);
            },
            [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<GetIO>> goal_handle) -> rclcpp_action::CancelResponse
            {
                return this->handle_get_io_cancel(goal_handle);
            },
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetIO>> goal_handle) -> void
            {
                this->execute_get_io(goal_handle);
            },
            serverOptions,
            action_callback_group_);

        // Action Server for ResetRobotState
        reset_robot_state_server_ = rclcpp_action::create_server<ResetRobotState>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "reset_robot_state",
            [this](const rclcpp_action::GoalUUID &uuid,
                   std::shared_ptr<const ResetRobotState::Goal> goal) -> rclcpp_action::GoalResponse
            {
                return this->handle_reset_robot_state_goal(uuid, goal);
            },
            [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle) -> rclcpp_action::CancelResponse
            {
                return this->handle_reset_robot_state_cancel(goal_handle);
            },
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle) -> void
            {
                this->execute_reset_robot_state(goal_handle);
            },
            serverOptions,
            action_callback_group_);
    }

    // Initialize Subscription for Robot States
    void SignalsNode::initialize_robot_state_subscription()
    {
        std::string robot_states_topic;
        if (robot_model_ == "lite6" || robot_model_ == "uf850")
        {
            robot_states_topic = "/ufactory/robot_states";
        }
        else if (robot_model_ == "xarm")
        {
            robot_states_topic = "/xarm/robot_states";
        }

        rclcpp::SubscriptionOptions options;
        options.callback_group = subscription_callback_group_;

        robot_state_subscription_ = this->create_subscription<xarm_msgs::msg::RobotMsg>(
            robot_states_topic,
            10,
            std::bind(&SignalsNode::robot_state_callback, this, std::placeholders::_1),
            options);

        RCLCPP_INFO(this->get_logger(), "Subscribed to %s", robot_states_topic.c_str());
    }

    // Robot state callback
    void SignalsNode::robot_state_callback(const xarm_msgs::msg::RobotMsg::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(robot_state_mutex_);
        current_robot_state_ = msg;
    }

    // ----------- SetIO Action Callbacks -----------

    rclcpp_action::GoalResponse SignalsNode::handle_set_io_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const SetIO::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received SetIO goal: io_type=%s, ionum=%d, value=%d",
                    goal->io_type.c_str(), goal->ionum, goal->value);
        // Validate io_type
        if (goal->io_type != "tool" && goal->io_type != "controller")
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid io_type: %s. Must be 'tool' or 'controller'.", goal->io_type.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        // Optionally, validate ionum based on io_type
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse SignalsNode::handle_set_io_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetIO>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel SetIO goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void SignalsNode::execute_set_io(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetIO>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing SetIO goal");
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<SetIO::Result>();

        // Create the service request
        auto request = std::make_shared<xarm_msgs::srv::SetDigitalIO::Request>();
        request->ionum = goal->ionum;
        request->value = goal->value;

        // Determine the service client based on io_type
        auto client_it = set_digital_io_clients_.find(goal->io_type);
        if (client_it == set_digital_io_clients_.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid io_type: %s", goal->io_type.c_str());
            result->success = false;
            result->message = "Invalid io_type";
            goal_handle->abort(result);
            return;
        }
        auto client = client_it->second;

        // Call the service asynchronously and handle the response in a callback
        client->async_send_request(request,
                                   [this, goal_handle, result](rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedFuture future_response)
                                   {
                                       auto response = future_response.get();
                                       if (response->ret == 0)
                                       {
                                           RCLCPP_INFO(this->get_logger(), "SetIO succeeded for io_type=%s, ionum=%d",
                                                       goal_handle->get_goal()->io_type.c_str(), goal_handle->get_goal()->ionum);
                                           result->success = true;
                                           result->message = "SetIO succeeded";
                                           goal_handle->succeed(result);
                                       }
                                       else
                                       {
                                           RCLCPP_ERROR(this->get_logger(), "SetIO failed with ret=%d for io_type=%s, ionum=%d",
                                                        response->ret, goal_handle->get_goal()->io_type.c_str(), goal_handle->get_goal()->ionum);
                                           result->success = false;
                                           result->message = "SetIO failed with ret=" + std::to_string(response->ret);
                                           goal_handle->abort(result);
                                       }
                                   });
    }

    // ----------- GetIO Action Callbacks -----------

    rclcpp_action::GoalResponse SignalsNode::handle_get_io_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const GetIO::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received GetIO goal: io_type=%s, ionum=%d",
                    goal->io_type.c_str(), goal->ionum);
        // Validate io_type
        if (goal->io_type != "tool" && goal->io_type != "controller")
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid io_type: %s. Must be 'tool' or 'controller'.", goal->io_type.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        // Optionally, validate ionum based on io_type
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse SignalsNode::handle_get_io_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetIO>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel GetIO goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void SignalsNode::execute_get_io(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetIO>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing GetIO goal");
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<GetIO::Result>();

        // Create the service request (empty)
        auto request = std::make_shared<xarm_msgs::srv::GetDigitalIO::Request>();

        // Determine the service client based on io_type
        auto client_it = get_digital_io_clients_.find(goal->io_type);
        if (client_it == get_digital_io_clients_.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid io_type: %s", goal->io_type.c_str());
            result->success = false;
            result->message = "Invalid io_type";
            goal_handle->abort(result);
            return;
        }
        auto client = client_it->second;

        // Call the service asynchronously and handle the response in a callback
        client->async_send_request(request,
                                   [this, goal_handle, goal, result](rclcpp::Client<xarm_msgs::srv::GetDigitalIO>::SharedFuture future_response)
                                   {
                                       auto response = future_response.get();
                                       if (response->ret == 0)
                                       {
                                           // Validate ionum
                                           if (goal->ionum < 0 || goal->ionum >= static_cast<int16_t>(response->digitals.size()))
                                           {
                                               RCLCPP_ERROR(this->get_logger(), "GetIO failed: ionum=%d out of range", goal->ionum);
                                               result->success = false;
                                               result->message = "ionum out of range";
                                               goal_handle->abort(result);
                                               return;
                                           }

                                           // Extract the value from the digitals array
                                           int16_t raw_value = response->digitals[goal->ionum];
                                           int16_t value;

                                           // Inversion logic: 0 -> 1 (ON), else 0 (OFF)
                                           value = (raw_value == 0) ? 1 : 0;

                                           RCLCPP_INFO(this->get_logger(), "GetIO succeeded: io_type=%s, ionum=%d, value=%d",
                                                       goal->io_type.c_str(), goal->ionum, value);
                                           result->success = true;
                                           result->value = value;
                                           result->message = "GetIO succeeded";
                                           goal_handle->succeed(result);
                                       }
                                       else
                                       {
                                           RCLCPP_ERROR(this->get_logger(), "GetIO failed with ret=%d", response->ret);
                                           result->success = false;
                                           result->message = "GetIO failed with ret=" + std::to_string(response->ret);
                                           goal_handle->abort(result);
                                       }
                                   });
    }

    // ----------- ResetRobotState Action Callbacks -----------

    rclcpp_action::GoalResponse SignalsNode::handle_reset_robot_state_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ResetRobotState::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received ResetRobotState goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse SignalsNode::handle_reset_robot_state_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel ResetRobotState goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void SignalsNode::execute_reset_robot_state(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing ResetRobotState goal");
        auto result = std::make_shared<ResetRobotState::Result>();

        // Step 1: Clean Error
        auto clean_error_req = std::make_shared<xarm_msgs::srv::Call::Request>();

        clean_error_client_->async_send_request(clean_error_req,
                                                [this, goal_handle, result](rclcpp::Client<xarm_msgs::srv::Call>::SharedFuture future_response)
                                                {
                                                    auto response = future_response.get();
                                                    if (response->ret != 0)
                                                    {
                                                        RCLCPP_ERROR(this->get_logger(), "CleanError failed with ret=%d", response->ret);
                                                        result->success = false;
                                                        result->message = "CleanError failed with ret=" + std::to_string(response->ret);
                                                        goal_handle->abort(result);
                                                    }
                                                    else
                                                    {
                                                        RCLCPP_INFO(this->get_logger(), "CleanError succeeded");
                                                        // Proceed to Motion Enable
                                                        this->enable_motion(goal_handle, result);
                                                    }
                                                });
    }

    void SignalsNode::enable_motion(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle,
        std::shared_ptr<ResetRobotState::Result> result)
    {
        auto motion_enable_req = std::make_shared<xarm_msgs::srv::SetInt16ById::Request>();
        motion_enable_req->id = 8;
        motion_enable_req->data = 1;

        motion_enable_client_->async_send_request(motion_enable_req,
                                                  [this, goal_handle, result](rclcpp::Client<xarm_msgs::srv::SetInt16ById>::SharedFuture future_response)
                                                  {
                                                      auto response = future_response.get();
                                                      if (response->ret != 0)
                                                      {
                                                          RCLCPP_ERROR(this->get_logger(), "MotionEnable failed with ret=%d", response->ret);
                                                          result->success = false;
                                                          result->message = "MotionEnable failed with ret=" + std::to_string(response->ret);
                                                          goal_handle->abort(result);
                                                      }
                                                      else
                                                      {
                                                          RCLCPP_INFO(this->get_logger(), "MotionEnable succeeded");
                                                          // Proceed to Set Mode
                                                          this->set_mode(goal_handle, result);
                                                      }
                                                  });
    }

    void SignalsNode::set_mode(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle,
        std::shared_ptr<ResetRobotState::Result> result)
    {
        auto set_mode_req = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
        set_mode_req->data = 1; // As per the user's explanation

        set_int16_clients_["mode"]->async_send_request(set_mode_req,
                                                       [this, goal_handle, result](rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedFuture future_response)
                                                       {
                                                           auto response = future_response.get();
                                                           if (response->ret != 0)
                                                           {
                                                               RCLCPP_ERROR(this->get_logger(), "SetMode failed with ret=%d", response->ret);
                                                               result->success = false;
                                                               result->message = "SetMode failed with ret=" + std::to_string(response->ret);
                                                               goal_handle->abort(result);
                                                           }
                                                           else
                                                           {
                                                               RCLCPP_INFO(this->get_logger(), "SetMode succeeded");
                                                               // Proceed to Set State
                                                               this->set_state(goal_handle, result);
                                                           }
                                                       });
    }

    void SignalsNode::set_state(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle,
        std::shared_ptr<ResetRobotState::Result> result)
    {
        auto set_state_req = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
        set_state_req->data = 0;

        set_int16_clients_["state"]->async_send_request(set_state_req,
                                                        [this, goal_handle, result](rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedFuture future_response)
                                                        {
                                                            auto response = future_response.get();
                                                            if (response->ret != 0)
                                                            {
                                                                RCLCPP_ERROR(this->get_logger(), "SetState failed with ret=%d", response->ret);
                                                                result->success = false;
                                                                result->message = "SetState failed with ret=" + std::to_string(response->ret);
                                                                goal_handle->abort(result);
                                                            }
                                                            else
                                                            {
                                                                RCLCPP_INFO(this->get_logger(), "SetState succeeded");
                                                                // Proceed to Verification
                                                                this->verify_reset(goal_handle, result);
                                                            }
                                                        });
    }

    void SignalsNode::verify_reset(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle,
        std::shared_ptr<ResetRobotState::Result> result)
    {
        // Define the number of attempts and interval
        const int max_attempts = 3;
        const std::chrono::milliseconds interval(200);

        for (int attempt = 1; attempt <= max_attempts; ++attempt)
        {
            {
                std::lock_guard<std::mutex> lock(robot_state_mutex_);
                if (!current_robot_state_)
                {
                    RCLCPP_ERROR(this->get_logger(), "No robot state information available for verification.");
                    result->success = false;
                    result->message = "No robot state information available";
                    goal_handle->abort(result);
                    return;
                }

                // Check success criteria
                bool success = true;
                std::string failure_reason;

                if (current_robot_state_->err != 0)
                {
                    success = false;
                    failure_reason += "err != 0; ";
                }
                if (current_robot_state_->mode != 1)
                {
                    success = false;
                    failure_reason += "mode != 1; ";
                }
                if (current_robot_state_->state != 2)
                {
                    success = false;
                    failure_reason += "state != 2; ";
                }
                if (current_robot_state_->mt_brake == 0)
                {
                    success = false;
                    failure_reason += "mt_brake == 0; ";
                }
                if (current_robot_state_->mt_able == 0)
                {
                    success = false;
                    failure_reason += "mt_able == 0; ";
                }

                if (success)
                {
                    RCLCPP_INFO(this->get_logger(), "ResetRobotState succeeded");
                    result->success = true;
                    result->message = "ResetRobotState succeeded";
                    goal_handle->succeed(result);
                    return;
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Attempt %d: ResetRobotState verification failed: %s", attempt, failure_reason.c_str());
                }
            }

            // Wait for the specified interval before the next attempt
            std::this_thread::sleep_for(interval);
        }

        // After all attempts, if not successful, abort the action
        RCLCPP_ERROR(this->get_logger(), "ResetRobotState verification failed after %d attempts.", max_attempts);
        result->success = false;
        result->message = "ResetRobotState verification failed after " + std::to_string(max_attempts) + " attempts.";
        goal_handle->abort(result);
    }

} // namespace manymove_signals

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Use a MultiThreadedExecutor to handle multiple callback groups concurrently
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<manymove_signals::SignalsNode>();
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);

    rclcpp::shutdown();
    return 0;
}