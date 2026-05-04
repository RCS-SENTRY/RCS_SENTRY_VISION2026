// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// RCS V3 adaptation: distance LUT manual yaw/pitch compensator inspired by
// CSU-FYT-Vision/FYT2024_vision rm_utils::ManualCompensator, adapted to the
// RCS rm_autoaim parameter schema.

#ifndef RM_AUTOAIM__CORE__MANUAL_COMPENSATOR_HPP_
#define RM_AUTOAIM__CORE__MANUAL_COMPENSATOR_HPP_

#include <utility>
#include <vector>

namespace rm_autoaim
{

class ManualCompensator
{
public:
  using Lut = std::vector<std::pair<double, double>>;

  void configure(
    bool enabled,
    Lut pitch_lut,
    Lut yaw_lut,
    double clamp_min_distance,
    double clamp_max_distance);

  bool enabled() const { return enabled_; }

  double pitchOffset(double distance) const;
  double yawOffset(double distance) const;

private:
  static double interpolate(
    const Lut & lut,
    double distance,
    double clamp_min_distance,
    double clamp_max_distance);

  bool enabled_ = true;
  Lut pitch_lut_;
  Lut yaw_lut_;
  double clamp_min_distance_ = 0.5;
  double clamp_max_distance_ = 8.0;
};

}  // namespace rm_autoaim

#endif  // RM_AUTOAIM__CORE__MANUAL_COMPENSATOR_HPP_
