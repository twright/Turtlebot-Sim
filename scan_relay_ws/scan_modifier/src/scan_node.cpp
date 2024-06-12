#include "scan_modifier.hpp"
#include <sstream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

namespace scan
{

using namespace std::placeholders;
using ranges_t = ScanModifierNode::ranges_t;

// Copied from: /nav2_util/node_utils.hpp
template <typename NodeT>
void declare_parameter_if_not_declared(
    NodeT node, const std::string& param_name, const rclcpp::ParameterType& param_type,
    const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name))
  {
    node->declare_parameter(param_name, param_type, parameter_descriptor);
  }
}

void print_laser(const sensor_msgs::msg::LaserScan& msg, const rclcpp::Logger& logger)
{
  RCLCPP_DEBUG(logger,
               "Message: angle_min: %f, angle_max: %f, angle_increment: %f, time_increment: %f, scan_time: %f, "
               "range_min: %f, range_max: %f",
               msg.angle_min, msg.angle_max, msg.angle_increment, msg.time_increment, msg.scan_time, msg.range_min,
               msg.range_max);
  std::stringstream ss;
  for (const auto& v : msg.ranges)
  {
    ss << v;
    ss << ", ";
  }
  auto result = ss.str();
  // RCLCPP_DEBUG(logger, "Ranges: %s", result.c_str());
}

ScanModifierNode::ScanModifierNode() : Node("scan_modifier")
{
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), std::bind(&ScanModifierNode::scan_sub_callback, this, _1));
  publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan_safe", rclcpp::SensorDataQoS());
  config_sub_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/scan_config", rclcpp::SensorDataQoS(), std::bind(&ScanModifierNode::config_sub_callback, this, _1));

  constexpr auto param_name = "scan_ranges_size";
  this->declare_parameter(param_name, rclcpp::ParameterType::PARAMETER_INTEGER);
  lidar_filt_upper_ = this->get_parameter(param_name).as_int();

  RCLCPP_INFO(get_logger(), "Scan modifier node started!");
  RCLCPP_INFO(get_logger(), "Scan size: %d", lidar_filt_upper_);
}

void ScanModifierNode::config_sub_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr config)
{
  if (config->data.size() == 2)
  {
    lidar_filt_lower_ = config->data[0];
    lidar_filt_upper_ = config->data[1];
    RCLCPP_INFO(get_logger(), "Setting non-occluded area to %d - %d", lidar_filt_lower_, lidar_filt_upper_);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Recevied incorrectly formatted config data. Expected exactly two elements.");
  }
}

void ScanModifierNode::scan_sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg)
{
  constexpr auto between = [](const auto& val, const auto& lower, const auto& upper) {
    return val >= lower && val <= upper;
  };

  const auto& logger = get_logger();
  RCLCPP_DEBUG(get_logger(), "Data size: %ld. Scan config: %d, %d", laser_msg->ranges.size(), lidar_filt_lower_,
               lidar_filt_upper_);

  for (size_t i = 0; i < laser_msg->ranges.size(); i++)
  {
    if (!between(i, lidar_filt_lower_, lidar_filt_upper_))
    {
      laser_msg->ranges[i] = -std::numeric_limits<float>::infinity();
    }
  }

  publisher_->publish(*laser_msg);
}
}  // namespace scan

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<scan::ScanModifierNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}