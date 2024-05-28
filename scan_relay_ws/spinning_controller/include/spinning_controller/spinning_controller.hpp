/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#ifndef SPINNING_CONTROLLER__SPINNING_CONTROLLER_HPP_
#define SPINNING_CONTROLLER__SPINNING_CONTROLLER_HPP_

#include <vector>
#include <memory>
#include <algorithm>
#include <concepts>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "spin_interfaces/msg/spin_periodic_commands.hpp"

namespace spinning_controller
{

template <typename T>
concept is_nav2_controller = std::derived_from<T, nav2_core::Controller>;

template <is_nav2_controller T>
class SpinningController : public T
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    T::configure(parent, name, tf, costmap_ros);
    auto node = parent.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    last_spin_ = clock_->now().seconds();

    spin_config_sub_ = node->create_subscription<spin_interfaces::msg::SpinPeriodicCommands>(
        "/spin_config", rclcpp::SystemDefaultsQoS(), [this](spin_interfaces::msg::SpinPeriodicCommands::SharedPtr msg) {
          RCLCPP_INFO(logger_, "SpinningController::lambda");
          cmds_ = msg;
        });

    cmds_ = std::make_shared<spin_interfaces::msg::SpinPeriodicCommands>();
    cmds_->period = 5.0;

    RCLCPP_INFO(logger_, "SpinningController::configure");
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                           const geometry_msgs::msg::Twist& velocity,
                                                           nav2_core::GoalChecker* goal_checker) override
  {
    auto cmd_vel = T::computeVelocityCommands(pose, velocity, goal_checker);
    maybe_spin(cmd_vel);
    return cmd_vel;
  }

private:
  void maybe_spin(geometry_msgs::msg::TwistStamped& cmd_vel)
  {
    // TODO: Somewhere assert that sum of instruction durations is less than total duration
    // using LinearVec3 = decltype(cmd_vel.twist.linear);

    constexpr auto do_spin = [](auto& cmd_vel, auto omega) {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.linear.y = 0.0;
      cmd_vel.twist.linear.z = 0.0;
      cmd_vel.twist.angular.z = omega;
    };

    if (!cmds_ || cmds_->commands.empty())
    {
      RCLCPP_INFO(logger_, "no cmd");
      return;
    }

    auto now = clock_->now().seconds();
    auto delta = now - last_spin_;

    if (delta > cmds_->period)
    {
      // Start new period and start by doing that command
      last_spin_ = now;
      do_spin(cmd_vel, cmds_->commands[0].omega);
      RCLCPP_INFO(logger_, "new period");
      return;
    }

    // Find command to execute
    for (const auto& cmd : cmds_->commands)
    {
      if (delta < cmd.duration)
      {
        do_spin(cmd_vel, cmd.omega);
        RCLCPP_INFO(logger_, "Command: %f", cmd.omega);
        return;
      }
      delta -= cmd.duration;
    }

    // If we reach here, then do original command (no spin)
    RCLCPP_INFO(logger_, "orig command");
    return;
  }

  rclcpp::Logger logger_{ rclcpp::get_logger("SpinningController") };
  rclcpp::Clock::SharedPtr clock_;
  double last_spin_;
  spin_interfaces::msg::SpinPeriodicCommands::SharedPtr cmds_{};
  rclcpp::Subscription<spin_interfaces::msg::SpinPeriodicCommands>::SharedPtr spin_config_sub_;
};

}  // namespace spinning_controller

#endif  // SPINNING_CONTROLLER__SPINNING_CONTROLLER_HPP_