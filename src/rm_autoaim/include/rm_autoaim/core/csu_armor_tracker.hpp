// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under
// Apache License 2.0.
//
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
// RCS V3 adaptation of CSU-FYT armor_solver::Tracker. The FYT state layout and
// armor measurement model are preserved; this version uses a local Eigen EKF
// with numeric Jacobians to avoid importing FYT ROS messages/rm_utils/Ceres.

#ifndef RM_AUTOAIM__CORE__CSU_ARMOR_TRACKER_HPP_
#define RM_AUTOAIM__CORE__CSU_ARMOR_TRACKER_HPP_

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "rm_autoaim/core/aim_target.hpp"

namespace rm_autoaim
{

enum class ArmorsNum
{
  NORMAL_4 = 4,
  BALANCE_2 = 2,
  OUTPOST_3 = 3,
};

struct CsuArmorTrackerParams
{
  double max_match_distance = 0.5;
  double max_match_yaw_diff = 1.0;
  double max_lost_time = 0.3;
  int min_detect_count = 2;
  int min_tracking_count_for_fire = 3;
  double default_radius = 0.26;
  double min_radius = 0.12;
  double max_radius = 0.40;
  double q_xyz = 0.05;
  double q_yaw = 0.10;
  double q_radius = 0.02;
  double r_xyz = 0.05;
  double r_yaw = 0.15;
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

  CsuTrackerState state() const { return tracker_state_; }
  int trackingFrames() const { return tracking_frames_; }
  int targetId() const { return tracked_id_; }
  std::string targetArmorType() const { return tracked_armor_type_; }
  Eigen::Vector3d lastSelectedPosition() const { return last_selected_position_; }
  Eigen::Vector3d predictedPosition(double stamp_sec, double predict_delay_sec) const;
  bool initialized() const { return initialized_; }

private:
  static constexpr int X_N = 10;
  static constexpr int Z_N = 4;
  using StateVec = Eigen::Matrix<double, X_N, 1>;
  using StateMat = Eigen::Matrix<double, X_N, X_N>;
  using MeasureVec = Eigen::Matrix<double, Z_N, 1>;
  using MeasureMat = Eigen::Matrix<double, Z_N, Z_N>;

  void init(const ArmorObservation & observation, double stamp_sec);
  void predictInPlace(double stamp_sec);
  void updateEkf(const ArmorObservation & observation);
  void handleArmorJump(const ArmorObservation & observation);

  StateVec processModel(const StateVec & x, double dt) const;
  MeasureVec measureModel(const StateVec & x) const;
  StateMat numericalF(const StateVec & x, double dt) const;
  Eigen::Matrix<double, Z_N, X_N> numericalH(const StateVec & x) const;

  AimTarget makeTarget(double stamp_sec, double predict_delay_sec, double confidence,
    const std::string & source) const;
  Eigen::Vector3d armorPositionFromState(const StateVec & x) const;
  ArmorsNum inferArmorsNum(const ArmorObservation & observation) const;

  static double normalizeAngle(double angle);
  static double shortestAngularDistance(double from, double to);
  static bool finiteState(const StateVec & state);

  CsuArmorTrackerParams params_;
  CsuTrackerState tracker_state_ = CsuTrackerState::LOST;
  bool initialized_ = false;
  StateVec x_ = StateVec::Zero();
  StateMat P_ = StateMat::Identity();
  double last_stamp_sec_ = 0.0;
  double last_detection_stamp_sec_ = 0.0;
  int detect_count_ = 0;
  int lost_count_ = 0;
  int tracking_frames_ = 0;
  int tracked_id_ = -1;
  std::string tracked_label_;
  std::string tracked_armor_type_ = "small";
  ArmorsNum tracked_armors_num_ = ArmorsNum::NORMAL_4;
  double last_yaw_ = 0.0;
  double another_r_ = 0.26;
  double d_za_ = 0.0;
  double d_zc_ = 0.0;
  Eigen::Vector3d last_selected_position_ = Eigen::Vector3d::Zero();
};

}  // namespace rm_autoaim

#endif  // RM_AUTOAIM__CORE__CSU_ARMOR_TRACKER_HPP_
