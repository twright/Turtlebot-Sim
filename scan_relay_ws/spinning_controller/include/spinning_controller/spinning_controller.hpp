/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#ifndef SPINNING_CONTROLLER__SPINNING_CONTROLLER_HPP_
#define SPINNING_CONTROLLER__SPINNING_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/dwb_local_planner.hpp"

namespace spinning_controller
{

struct SpinInstructions
{
  struct Instruction
  {
    double omega;
    double duration;
  };

  std::vector<Instruction> instructions;
  double period;
};

// TODO: Template class - base class Controller
class SpinningController : public dwb_core::DWBLocalPlanner
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                           const geometry_msgs::msg::Twist&,
                                                           nav2_core::GoalChecker*) override;

private:
  void maybe_spin(geometry_msgs::msg::TwistStamped& cmd_vel);

  rclcpp::Logger logger_{ rclcpp::get_logger("SpinningController") };
  rclcpp::Clock::SharedPtr clock_;
  double last_spin_;
  double period_start_;
  SpinInstructions instructions_{};
};

}  // namespace spinning_controller

#endif  // SPINNING_CONTROLLER__SPINNING_CONTROLLER_HPP_