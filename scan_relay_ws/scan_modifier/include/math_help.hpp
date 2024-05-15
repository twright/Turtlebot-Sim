#pragma once
#include <cassert>
#include <cmath>
#include <limits>

namespace scan
{

template <typename ReturnT, bool safe_mode = true>
constexpr ReturnT rad_to_degree(auto angle_rad)
{
  // TODO: Introduce rounding function parameter
  if constexpr (safe_mode)
  {
    static_assert(std::numeric_limits<ReturnT>::max() >= 360, "Return type must support values up to 360");
    static_assert(std::numeric_limits<ReturnT>::lowest() <= 0, "Return type must support values down to 0");
    assert(angle_rad <= static_cast<decltype(angle_rad)>(2 * M_PI));
    assert(angle_rad >= 0);
  }
  return static_cast<ReturnT>(angle_rad * 180 / M_PI);
}

template <typename ReturnT, bool safe_mode = true>
constexpr ReturnT degree_to_rad(auto angle_degree)
{
  if constexpr (safe_mode)
  {
    static_assert(std::numeric_limits<ReturnT>::max() >= 2 * M_PI, "Return type must support values up to 2*PI");
    static_assert(std::numeric_limits<ReturnT>::lowest() <= 0, "Return type must support values down to 0");
    assert(angle_degree <= 360);
    assert(angle_degree >= 0);
  }
  return static_cast<ReturnT>(angle_degree * M_PI / 180);
}
}  // namespace scan