#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "math_help.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace scan
{

class ScanModifierNode : public rclcpp::Node
{
public:
  using ranges_t = sensor_msgs::msg::LaserScan::_ranges_type;

  ScanModifierNode();

private:
  void scan_sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg);
  void config_sub_callback(const std_msgs::msg::Float32MultiArray::SharedPtr config);

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr config_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

  float lidar_filt_upper_ = 2 * M_PI;
  float lidar_filt_lower_ = 0;
};

/// Takes `lidar_ranges` and returns a new vector including only the samples between `angle_min_rad` and `angle_max_rad`
template <bool safe_mode = true>
static ScanModifierNode::ranges_t calculate_lidar_ranges(float angle_min_rad, float angle_max_rad,
                                                         const auto& lidar_ranges)
{
  auto sample_begin = rad_to_degree<uint16_t>(angle_min_rad);
  auto sample_end = rad_to_degree<uint16_t>(angle_max_rad);
  auto size = sample_end - sample_begin;

  if constexpr (safe_mode)
    assert(lidar_ranges.size() >= sample_end);

  ScanModifierNode::ranges_t occluded_ranges{};
  occluded_ranges.reserve(size);
  std::copy_n(lidar_ranges.begin() + sample_begin, size, std::back_inserter(occluded_ranges));
  return occluded_ranges;
}

}  // namespace scan