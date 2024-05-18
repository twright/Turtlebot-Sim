/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#include "spinning_controller/spinning_controller.hpp"
#include <algorithm>

namespace spinning_controller
{

void SpinningController::maybe_spin(geometry_msgs::msg::TwistStamped& cmd_vel)
{
  // TODO: Somewhere assert that sum of instruction durations is less than total duration
  // using LinearVec3 = decltype(cmd_vel.twist.linear);

  auto now = clock_->now().seconds();

  // Check if we need to spin
  for (size_t i = 0; i < instructions_.instructions.size(); i++)
  {
    auto end_it = instructions_.instructions.begin() + i + 1;
    auto time_delta = std::accumulate(instructions_.instructions.begin(), end_it, 0.0,
                                      [](auto res, auto instr) { return res + instr.duration; });
    if (last_spin_ < period_start_ + time_delta)
    {
      last_spin_ = now;
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.linear.y = 0.0;
      cmd_vel.twist.linear.z = 0.0;

      cmd_vel.twist.angular.z = instructions_.instructions[i].omega;
      RCLCPP_INFO(logger_, "Spinning: %f, %f, %f, %f", last_spin_, time_delta, period_start_,
                  instructions_.instructions[i].duration);
      return;
    }
    else
    {
      RCLCPP_INFO(logger_, "Not spinning: %f, %f, %f, %f", last_spin_, time_delta, period_start_,
                  instructions_.instructions[i].duration);
    }
  }
  if (period_start_ + instructions_.period < now)
  {
    RCLCPP_INFO(logger_, "New period");
    period_start_ = now;
    last_spin_ = now;
  }
}

void SpinningController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                                   std::shared_ptr<tf2_ros::Buffer> tf,
                                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  dwb_core::DWBLocalPlanner::configure(parent, name, tf, costmap_ros);
  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  last_spin_ = clock_->now().seconds();
  period_start_ = clock_->now().seconds();

  std::vector<SpinInstructions::Instruction> instrcts{ { { 1.57, 1.0 }, { -1.57, 1.0 } } };
  instructions_ = { std::move(instrcts), 5 };

  RCLCPP_INFO(logger_, "SpinningController::configure");
}

geometry_msgs::msg::TwistStamped SpinningController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* goal_checker)
{
  auto cmd_vel = dwb_core::DWBLocalPlanner::computeVelocityCommands(pose, velocity, goal_checker);
  maybe_spin(cmd_vel);
  return cmd_vel;
}

}  // namespace spinning_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(spinning_controller::SpinningController, nav2_core::Controller)