#pragma once

#include "user_command_interface.h"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <atomic>
#include <cstring>

using namespace interface;
using namespace types;

class TwistInterface : public UserCommandInterface {
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    std::mutex cmd_mutex_;
    std::atomic<bool> running_{false};
    int startup_step_ = 0;

    float max_linear_x_ = 0.7f;
    float max_linear_y_ = 0.5f;
    float max_angular_z_ = 0.7f;

    void TwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        usr_cmd_->forward_vel_scale  = static_cast<float>(msg->linear.x / max_linear_x_);
        usr_cmd_->side_vel_scale     = static_cast<float>(msg->linear.y / max_linear_y_);
        usr_cmd_->turnning_vel_scale = static_cast<float>(msg->angular.z / max_angular_z_);

        // Clamp to [-1, 1]
        usr_cmd_->forward_vel_scale  = std::clamp(usr_cmd_->forward_vel_scale,  -1.0f, 1.0f);
        usr_cmd_->side_vel_scale     = std::clamp(usr_cmd_->side_vel_scale,     -1.0f, 1.0f);
        usr_cmd_->turnning_vel_scale = std::clamp(usr_cmd_->turnning_vel_scale, -1.0f, 1.0f);

        usr_cmd_->time_stamp = node_->get_clock()->now().seconds();
    }

    void StartupSequence() {
        startup_step_++;
        if (startup_step_ == 2) {
            RCLCPP_INFO(node_->get_logger(), "Auto-startup: StandingUp");
            usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
        } else if (startup_step_ == 5) {
            RCLCPP_INFO(node_->get_logger(), "Auto-startup: RLControlMode");
            usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);
        } else if (startup_step_ == 6) {
            RCLCPP_INFO(node_->get_logger(), "Auto-startup: ready for navigation");
            startup_timer_->cancel();
        }
    }

public:
    TwistInterface(RobotName robot_name, rclcpp::Node::SharedPtr node)
        : UserCommandInterface(robot_name), node_(node)
    {
        twist_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_smoothed", 10,
            std::bind(&TwistInterface::TwistCallback, this, std::placeholders::_1));

        startup_timer_ = node_->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TwistInterface::StartupSequence, this));

        std::memset(usr_cmd_, 0, sizeof(UserCommand));
        RCLCPP_INFO(node_->get_logger(), "TwistInterface: subscribed to /cmd_vel_smoothed");
    }

    ~TwistInterface() { Stop(); }

    void Start() override {
        if (running_) return;
        running_ = true;
        RCLCPP_INFO(node_->get_logger(), "TwistInterface started");
    }

    void Stop() override {
        if (!running_) return;
        running_ = false;
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        usr_cmd_->forward_vel_scale = 0.0f;
        usr_cmd_->side_vel_scale = 0.0f;
        usr_cmd_->turnning_vel_scale = 0.0f;
    }

    UserCommand* GetUserCommand() override { return usr_cmd_; }
    rclcpp::Node::SharedPtr get_node() { return node_; }
};
