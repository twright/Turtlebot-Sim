#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <sstream>

using namespace std::placeholders;

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

class ScanModifierNode : public rclcpp::Node
{
public:
  ScanModifierNode() : Node("scan_modifier")
  {
    subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), std::bind(&ScanModifierNode::scan_sub_callback, this, _1));
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan_safe", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "Scan modifier node started!");
  }

private:
  void scan_sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg)
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
    // laser_msg->angle_max = laser_msg->angle_max / 2;

    laser_msg->angle_min = 3.14;

    decltype(laser_msg->ranges) result{};
    result.reserve(180);
    std::copy_n(laser_msg->ranges.begin() + 179, 180, std::back_inserter(result));

    laser_msg->ranges.resize(laser_msg->ranges.size() / 2);
    laser_msg->ranges = result;
    laser_msg->intensities.resize(laser_msg->intensities.size() / 2);
    RCLCPP_INFO(get_logger(), "New sizes: %ld, %ld", laser_msg->ranges.size(), laser_msg->intensities.size());

    if (level == RCUTILS_LOG_SEVERITY_DEBUG)
    {
      RCLCPP_DEBUG(get_logger(), "New msg:");
      print_laser(*laser_msg, get_logger());
    }

    publisher_->publish(*laser_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanModifierNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}