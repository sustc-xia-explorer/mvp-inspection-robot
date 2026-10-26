#pragma once

#include <cmath>
#include <stdexcept>

#include "neupan/types.hpp"

namespace neupan {

// Limits the command actually emitted, including startup and ordinary braking.
// dt is elapsed command time, not solver/ROS time spent doing computation.
inline Vec2 limitCommand(const Vec2& target, const Vec2& previous,
                         const Vec2& max_speed, const Vec2& max_acce,
                         double dt) {
  if (!target.allFinite() || !previous.allFinite() ||
      !max_speed.allFinite() || !max_acce.allFinite() ||
      (max_speed.array() <= 0).any() || (max_acce.array() <= 0).any() ||
      !std::isfinite(dt) || dt <= 0)
    throw std::invalid_argument("invalid command, limits or elapsed time");
  const Vec2 wanted = target.cwiseMax(-max_speed).cwiseMin(max_speed);
  const Vec2 change = max_acce * dt;
  return wanted.cwiseMax(previous - change).cwiseMin(previous + change);
}

}  // namespace neupan
