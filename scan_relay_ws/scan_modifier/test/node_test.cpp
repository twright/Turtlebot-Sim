#include "scan_modifier.hpp"

#include <gtest/gtest.h>
#include <ranges>

#include <iostream>

TEST(scan_modifier, lidar_ranges)
{
  using namespace scan;
  // Create range with the data 0..359 (inclusive), mimicking that the lidar measures something from 0..359 meters away
  auto ranges = std::ranges::iota_view{ 0, 360 };
  for (auto i = 0; i < 361; i++)
  {
    for (auto j = i; j < 361; j++)
    {
      auto angle_rad_begin = degree_to_rad<float>(i) + degree_to_rad<float>(1) / 2;  // Avoid rounding errors
      auto angle_rad_end = degree_to_rad<float>(j) + degree_to_rad<float>(1) / 2;
      angle_rad_begin = std::clamp(angle_rad_begin, static_cast<float>(0), static_cast<float>(2 * M_PI));
      angle_rad_end = std::clamp(angle_rad_end, static_cast<float>(0), static_cast<float>(2 * M_PI));
      auto v = calculate_lidar_ranges(angle_rad_begin, angle_rad_end, ranges);
      assert(j - i == v.size());
      auto expected_data = std::ranges::iota_view{ i, j };
      assert(std::ranges::equal(v, expected_data));
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}