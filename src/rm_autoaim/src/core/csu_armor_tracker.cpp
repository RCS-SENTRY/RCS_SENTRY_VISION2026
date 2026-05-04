#include "rm_autoaim/core/csu_armor_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rm_autoaim
{

CsuArmorTracker::CsuArmorTracker(const CsuArmorTrackerParams & params)
: params_(params)
{
  reset();
}

void CsuArmorTracker::configure(const CsuArmorTrackerParams & params)
{
  params_ = params;
}

void CsuArmorTracker::reset()
{
  state_ = CsuTrackerState::LOST;
  initialized_ = false;
  x_.setZero();
  P_.setIdentity();
  P_ *= 10.0;
  last_stamp_sec_ = 0.0;
  last_detection_stamp_sec_ = 0.0;
  detect_count_ = 0;
  tracking_frames_ = 0;
  target_id_ = -1;
  target_label_.clear();
  target_armor_type_ = "small";
  last_selected_position_.setZero();
}

std::optional<ArmorObservation> CsuArmorTracker::select(
  const std::vector<ArmorObservation> & observations) const
{
  if (observations.empty()) return std::nullopt;

  auto score_by_center = [](const ArmorObservation & a, const ArmorObservation & b) {
    if (std::abs(a.center_axis_error_px - b.center_axis_error_px) > 1e-3) {
      return a.center_axis_error_px < b.center_axis_error_px;
    }
    if (std::abs(a.confidence - b.confidence) > 1e-6) {
      return a.confidence > b.confidence;
    }
    return a.distance < b.distance;
  };

  if (!initialized_ || target_id_ < 0) {
    return *std::min_element(observations.begin(), observations.end(), score_by_center);
  }

  const Eigen::Vector3d predicted = x_.head<3>();
  const ArmorObservation * best_same_id = nullptr;
  double best_dist = std::numeric_limits<double>::max();
  for (const auto & obs : observations) {
    if (obs.class_id != target_id_) continue;
    const double d = (obs.position_gimbal - predicted).norm();
    if (d < best_dist) {
      best_dist = d;
      best_same_id = &obs;
    }
  }

  if (best_same_id && best_dist < params_.max_match_distance) {
    return *best_same_id;
  }

  return *std::min_element(observations.begin(), observations.end(), score_by_center);
}

AimTarget CsuArmorTracker::update(
  const ArmorObservation & observation,
  double stamp_sec,
  double predict_delay_sec)
{
  if (!initialized_) {
    init(observation, stamp_sec);
    return makeTarget(predictState(stamp_sec, predict_delay_sec), observation.confidence, "csu_tracker");
  }

  predictInPlace(stamp_sec);

  Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  const MeasureMat R =
    MeasureMat::Identity() * params_.measurement_noise * params_.measurement_noise;

  const Eigen::Vector3d z = observation.position_gimbal;
  const Eigen::Vector3d innovation = z - H * x_;
  const MeasureMat S = H * P_ * H.transpose() + R;
  const Eigen::Matrix<double, 6, 3> K = P_ * H.transpose() * S.inverse();
  x_ = x_ + K * innovation;
  P_ = (StateMat::Identity() - K * H) * P_;

  if (!finiteState(x_) || innovation.norm() > params_.max_match_distance * 3.0) {
    init(observation, stamp_sec);
  }

  target_id_ = observation.class_id;
  target_label_ = observation.label;
  target_armor_type_ = observation.armor_type;
  last_selected_position_ = observation.position_gimbal;
  last_detection_stamp_sec_ = stamp_sec;

  if (state_ == CsuTrackerState::LOST) {
    state_ = CsuTrackerState::DETECTING;
    detect_count_ = 1;
    tracking_frames_ = 1;
  } else if (state_ == CsuTrackerState::DETECTING) {
    detect_count_++;
    tracking_frames_++;
    if (detect_count_ >= params_.min_detect_count) {
      state_ = CsuTrackerState::TRACKING;
    }
  } else {
    state_ = CsuTrackerState::TRACKING;
    tracking_frames_++;
  }

  return makeTarget(predictState(stamp_sec, predict_delay_sec), observation.confidence, "csu_tracker");
}

AimTarget CsuArmorTracker::markLost(double stamp_sec, double predict_delay_sec)
{
  if (!initialized_) {
    reset();
    return {};
  }

  predictInPlace(stamp_sec);
  const double lost_time = stamp_sec - last_detection_stamp_sec_;
  if (lost_time <= params_.max_lost_time &&
      (state_ == CsuTrackerState::TRACKING || state_ == CsuTrackerState::TEMP_LOST))
  {
    state_ = CsuTrackerState::TEMP_LOST;
    return makeTarget(predictState(stamp_sec, predict_delay_sec), 0.0, "csu_tracker_temp_lost");
  }

  reset();
  return {};
}

void CsuArmorTracker::init(const ArmorObservation & observation, double stamp_sec)
{
  x_.setZero();
  x_.head<3>() = observation.position_gimbal;
  P_.setIdentity();
  P_.block<3, 3>(0, 0) *= 0.01;
  P_.block<3, 3>(3, 3) *= 1.0;
  initialized_ = true;
  last_stamp_sec_ = stamp_sec;
  last_detection_stamp_sec_ = stamp_sec;
  detect_count_ = 1;
  tracking_frames_ = 1;
  target_id_ = observation.class_id;
  target_label_ = observation.label;
  target_armor_type_ = observation.armor_type;
  last_selected_position_ = observation.position_gimbal;
  state_ = CsuTrackerState::DETECTING;
}

CsuArmorTracker::StateMat CsuArmorTracker::transition(double dt) const
{
  StateMat F = StateMat::Identity();
  F(0, 3) = dt;
  F(1, 4) = dt;
  F(2, 5) = dt;
  return F;
}

void CsuArmorTracker::predictInPlace(double stamp_sec)
{
  double dt = stamp_sec - last_stamp_sec_;
  if (dt <= 0.0) dt = 0.001;
  if (dt > 0.5) dt = 0.5;

  const StateMat F = transition(dt);
  StateMat Q = StateMat::Zero();
  Q.block<3, 3>(0, 0) =
    Eigen::Matrix3d::Identity() * params_.process_noise_pos * params_.process_noise_pos;
  Q.block<3, 3>(3, 3) =
    Eigen::Matrix3d::Identity() * params_.process_noise_vel * params_.process_noise_vel;

  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
  last_stamp_sec_ = stamp_sec;
}

CsuArmorTracker::StateVec CsuArmorTracker::predictState(
  double stamp_sec,
  double predict_delay_sec) const
{
  if (!initialized_) return StateVec::Zero();
  double dt = stamp_sec - last_stamp_sec_ + std::max(0.0, predict_delay_sec);
  if (dt < 0.0) dt = 0.0;
  if (dt > 0.8) dt = 0.8;
  return transition(dt) * x_;
}

Eigen::Vector3d CsuArmorTracker::predictedPosition(
  double stamp_sec,
  double predict_delay_sec) const
{
  return predictState(stamp_sec, predict_delay_sec).head<3>();
}

bool CsuArmorTracker::finiteState(const StateVec & state) const
{
  for (int i = 0; i < state.size(); ++i) {
    if (!std::isfinite(state(i))) return false;
  }
  return true;
}

AimTarget CsuArmorTracker::makeTarget(
  const StateVec & state,
  double confidence,
  const std::string & source) const
{
  AimTarget target;
  if (!finiteState(state)) return target;
  target.position_gimbal = state.head<3>();
  target.velocity_gimbal = state.tail<3>();
  target.distance = target.position_gimbal.norm();
  target.confidence = confidence;
  target.source = source;
  target.valid = target.distance > 0.05 && std::isfinite(target.distance);
  return target;
}

}  // namespace rm_autoaim
