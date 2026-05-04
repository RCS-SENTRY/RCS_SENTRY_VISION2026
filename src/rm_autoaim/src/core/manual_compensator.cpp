#include "rm_autoaim/core/manual_compensator.hpp"

#include <algorithm>

namespace rm_autoaim
{

void ManualCompensator::configure(
  bool enabled,
  Lut pitch_lut,
  Lut yaw_lut,
  double clamp_min_distance,
  double clamp_max_distance)
{
  enabled_ = enabled;
  pitch_lut_ = std::move(pitch_lut);
  yaw_lut_ = std::move(yaw_lut);
  clamp_min_distance_ = std::min(clamp_min_distance, clamp_max_distance);
  clamp_max_distance_ = std::max(clamp_min_distance, clamp_max_distance);

  auto by_distance = [](const auto & a, const auto & b) {
    return a.first < b.first;
  };
  std::sort(pitch_lut_.begin(), pitch_lut_.end(), by_distance);
  std::sort(yaw_lut_.begin(), yaw_lut_.end(), by_distance);
}

double ManualCompensator::pitchOffset(double distance) const
{
  if (!enabled_) return 0.0;
  return interpolate(pitch_lut_, distance, clamp_min_distance_, clamp_max_distance_);
}

double ManualCompensator::yawOffset(double distance) const
{
  if (!enabled_) return 0.0;
  return interpolate(yaw_lut_, distance, clamp_min_distance_, clamp_max_distance_);
}

double ManualCompensator::interpolate(
  const Lut & lut,
  double distance,
  double clamp_min_distance,
  double clamp_max_distance)
{
  if (lut.empty()) return 0.0;
  if (lut.size() == 1) return lut.front().second;

  const double d = std::clamp(distance, clamp_min_distance, clamp_max_distance);
  if (d <= lut.front().first) return lut.front().second;
  if (d >= lut.back().first) return lut.back().second;

  for (std::size_t i = 1; i < lut.size(); ++i) {
    const auto & lo = lut[i - 1];
    const auto & hi = lut[i];
    if (d <= hi.first) {
      const double span = hi.first - lo.first;
      if (span <= 1e-9) return hi.second;
      const double t = (d - lo.first) / span;
      return lo.second + t * (hi.second - lo.second);
    }
  }

  return lut.back().second;
}

}  // namespace rm_autoaim
