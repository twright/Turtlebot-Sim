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
#include "nav2_util/node_utils.hpp"

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
    plugin_name_ = name;

    // TODO: We are not using cmds_ for anything
    cmds_ = std::make_shared<spin_interfaces::msg::SpinPeriodicCommands>();
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".spin_period", rclcpp::ParameterValue(5.0));
    node->get_parameter(plugin_name_ + ".spin_period", cmds_->period);

    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".spin_commands", rclcpp::PARAMETER_DOUBLE_ARRAY);
    auto vec_d = node->get_parameter(plugin_name_ + ".spin_commands").as_double_array();
    vec_to_spins(vec_d);

    // TODO: Avoid copies???
    dyn_params_handler_ = node->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> params) { return dynamicParametersCallback(params); });

    RCLCPP_INFO(logger_, "SpinningController::configure");
  }

  void vec_to_spins(const std::vector<double>& vec)
  {
    // TODO: Assert even size
    cmds_->commands.clear();

    for (size_t i = 0; i < vec.size() / 2; i++)
    {
      spin_interfaces::msg::SpinCommand cmd;
      cmd.omega = vec[2 * i];
      cmd.duration = vec[2 * i + 1];
      cmds_->commands.emplace_back(cmd);
    }
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                           const geometry_msgs::msg::Twist& velocity,
                                                           nav2_core::GoalChecker* goal_checker) override
  {
    auto cmd_vel = T::computeVelocityCommands(pose, velocity, goal_checker);
    maybe_spin(cmd_vel);
    return cmd_vel;
  }

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
  {
    RCLCPP_INFO(logger_, "SpinningController::dynamicParametersCallback");

    rcl_interfaces::msg::SetParametersResult result;
    std::lock_guard<std::mutex> lock_reinit(param_mutex_);

    for (auto parameter : parameters)
    {
      const auto& type = parameter.get_type();
      const auto& name = parameter.get_name();

      if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
      {
        if (name == plugin_name_ + ".spin_period")
        {
          cmds_->period = parameter.as_double();
          RCLCPP_INFO(logger_, "New spin period: %f", cmds_->period);
        }
      }
      else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY)
      {
        if (name == plugin_name_ + ".spin_commands")
        {
          auto vec_d = parameter.as_double_array();
          vec_to_spins(vec_d);
          RCLCPP_INFO(logger_, "New spin commands:");
          for (const auto& cmd : cmds_->commands)
          {
            RCLCPP_INFO(logger_, "Omega: %f, Duration: %f", cmd.omega, cmd.duration);
          }
        }
      }
    }
    result.successful = true;
    return result;
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
      cmd_vel.twist.angular.x = 0.0;
      cmd_vel.twist.angular.y = 0.0;
      cmd_vel.twist.angular.z = omega;
    };

    if (!cmds_ || cmds_->commands.empty())
    {
      return;
    }

    auto now = clock_->now().seconds();
    auto delta = now - last_spin_;

    if (delta > cmds_->period)
    {
      // Start new period and start by doing that command
      last_spin_ = now;
      do_spin(cmd_vel, cmds_->commands[0].omega);
      // RCLCPP_INFO(logger_, "new period");
      return;
    }

    // Find command to execute
    for (const auto& cmd : cmds_->commands)
    {
      if (delta < cmd.duration)
      {
        do_spin(cmd_vel, cmd.omega);
        // RCLCPP_INFO(logger_, "Command: %f", cmd.omega);
        return;
      }
      delta -= cmd.duration;
    }

    // If we reach here, then do original command (no spin)
    // RCLCPP_INFO(logger_, "orig command");
    return;
  }

  rclcpp::Logger logger_{ rclcpp::get_logger("SpinningController") };
  rclcpp::Clock::SharedPtr clock_;
  double last_spin_;
  spin_interfaces::msg::SpinPeriodicCommands::SharedPtr cmds_{};
  std::string plugin_name_;

  std::mutex param_mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace spinning_controller

#endif  // SPINNING_CONTROLLER__SPINNING_CONTROLLER_HPP_