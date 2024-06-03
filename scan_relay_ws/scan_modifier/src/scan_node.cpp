#include "scan_modifier.hpp"
#include <sstream>
#include <cmath>

namespace scan
{

using namespace std::placeholders;
using ranges_t = ScanModifierNode::ranges_t;

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
  RCLCPP_DEBUG(logger, "Ranges: %s", result.c_str());
}

ScanModifierNode::ScanModifierNode() : Node("scan_modifier")
{
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), std::bind(&ScanModifierNode::scan_sub_callback, this, _1));
  publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan_safe", rclcpp::SensorDataQoS());
  config_sub_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/scan_config", rclcpp::SensorDataQoS(), std::bind(&ScanModifierNode::config_sub_callback, this, _1));

  RCLCPP_INFO(get_logger(), "Scan modifier node started!");
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
  const auto& logger = get_logger();
  auto level = rcutils_logging_get_logger_effective_level(logger.get_name());

  RCLCPP_INFO(get_logger(), "Callback");

  if (level == RCUTILS_LOG_SEVERITY_DEBUG)
  {
    RCLCPP_DEBUG(get_logger(), "OG msg:");
    print_laser(*laser_msg, get_logger());
  }

  RCLCPP_INFO(get_logger(), "OG sizes: %ld, %ld", laser_msg->ranges.size(), laser_msg->intensities.size());

  auto laser_size = laser_msg->ranges.size();
  laser_msg->angle_min = lidar_filt_lower_ / laser_size * 2 * M_PI;
  laser_msg->angle_max = lidar_filt_upper_ / laser_size * 2 * M_PI;
  auto result = calculate_lidar_ranges(lidar_filt_lower_, lidar_filt_upper_, laser_msg->ranges);

  // TODO: Probably bug in the resizes...
  laser_msg->ranges.resize(result.size());
  laser_msg->intensities.resize(result.size());

  laser_msg->ranges = std::move(result);
  RCLCPP_INFO(get_logger(), "New sizes: %ld, %ld", laser_msg->ranges.size(), laser_msg->intensities.size());

  if (level == RCUTILS_LOG_SEVERITY_DEBUG)
  {
    RCLCPP_DEBUG(get_logger(), "New msg:");
    print_laser(*laser_msg, get_logger());
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