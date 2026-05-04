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
// RCS V3 adaptation: CSU-style armor-point tracking state machine. Unlike the
// legacy IMM backend, this production tracker outputs the shootable armor point.

#ifndef RM_AUTOAIM__CORE__CSU_ARMOR_TRACKER_HPP_
#define RM_AUTOAIM__CORE__CSU_ARMOR_TRACKER_HPP_

#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "rm_autoaim/core/aim_target.hpp"

namespace rm_autoaim
{

struct CsuArmorTrackerParams
{
  double max_match_distance = 0.5;
  double max_lost_time = 0.3;
  int min_detect_count = 2;
  int min_tracking_count_for_fire = 3;
  double process_noise_pos = 0.04;
  double process_noise_vel = 1.0;
  double measurement_noise = 0.05;
};

class CsuArmorTracker
{
public:
  explicit CsuArmorTracker(const CsuArmorTrackerParams & params = CsuArmorTrackerParams());

  void reset();
  void configure(const CsuArmorTrackerParams & params);

  std::optional<ArmorObservation> select(
    const std::vector<ArmorObservation> & observations) const;

  AimTarget update(
    const ArmorObservation & observation,
    double stamp_sec,
    double predict_delay_sec);

  AimTarget markLost(double stamp_sec, double predict_delay_sec);

  CsuTrackerState state() const { return state_; }
  int trackingFrames() const { return tracking_frames_; }
  int targetId() const { return target_id_; }
  std::string targetArmorType() const { return target_armor_type_; }
  Eigen::Vector3d lastSelectedPosition() const { return last_selected_position_; }
  Eigen::Vector3d predictedPosition(double stamp_sec, double predict_delay_sec) const;
  bool initialized() const { return initialized_; }

private:
  using StateVec = Eigen::Matrix<double, 6, 1>;
  using StateMat = Eigen::Matrix<double, 6, 6>;
  using MeasureMat = Eigen::Matrix<double, 3, 3>;

  void init(const ArmorObservation & observation, double stamp_sec);
  StateVec predictState(double stamp_sec, double predict_delay_sec) const;
  StateMat transition(double dt) const;
  void predictInPlace(double stamp_sec);
  bool finiteState(const StateVec & state) const;
  AimTarget makeTarget(const StateVec & state, double confidence, const std::string & source) const;

  CsuArmorTrackerParams params_;
  CsuTrackerState state_ = CsuTrackerState::LOST;
  bool initialized_ = false;
  StateVec x_ = StateVec::Zero();
  StateMat P_ = StateMat::Identity();
  double last_stamp_sec_ = 0.0;
  double last_detection_stamp_sec_ = 0.0;
  int detect_count_ = 0;
  int tracking_frames_ = 0;
  int target_id_ = -1;
  std::string target_label_;
  std::string target_armor_type_ = "small";
  Eigen::Vector3d last_selected_position_ = Eigen::Vector3d::Zero();
};

}  // namespace rm_autoaim

#endif  // RM_AUTOAIM__CORE__CSU_ARMOR_TRACKER_HPP_
