/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#include "spinning_controller/spinning_controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

// Controller specializations
#include "dwb_core/dwb_local_planner.hpp"
// #include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"

namespace spinning_controller
{
using DWBSpinningController = spinning_controller::SpinningController<dwb_core::DWBLocalPlanner>;

// Alternative controller? Simply comment in code, change nav2_params and add CMake dependencies
// using RegulatedSpinningPurePursuitController =
//     spinning_controller::SpinningController<nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController>;

}  // namespace spinning_controller

// Register the controllers as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(spinning_controller::DWBSpinningController, nav2_core::Controller)
// PLUGINLIB_EXPORT_CLASS(spinning_controller::RegulatedSpinningPurePursuitController, nav2_core::Controller)