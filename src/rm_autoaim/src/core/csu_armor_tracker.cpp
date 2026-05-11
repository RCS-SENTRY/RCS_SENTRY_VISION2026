#include "rm_autoaim/core/csu_armor_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cfloat>
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
  another_r_ = params_.default_radius;
}

void CsuArmorTracker::reset()
{
  tracker_state_ = CsuTrackerState::LOST;
  initialized_ = false;
  x_.setZero();
  P_.setIdentity();
  P_ *= 10.0;
  last_stamp_sec_ = 0.0;
  last_detection_stamp_sec_ = 0.0;
  detect_count_ = 0;
  lost_count_ = 0;
  tracking_frames_ = 0;
  tracked_id_ = -1;
  tracked_label_.clear();
  tracked_armor_type_ = "small";
  tracked_armors_num_ = ArmorsNum::NORMAL_4;
  last_yaw_ = 0.0;
  another_r_ = params_.default_radius;
  d_za_ = 0.0;
  d_zc_ = 0.0;
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

  if (!initialized_ || tracked_id_ < 0) {
    return *std::min_element(observations.begin(), observations.end(), score_by_center);
  }

  const Eigen::Vector3d predicted_position = armorPositionFromState(x_);
  const ArmorObservation * best_same_id = nullptr;
  double min_position_diff = DBL_MAX;
  double min_yaw_diff = DBL_MAX;

  for (const auto & obs : observations) {
    if (!obs.valid || obs.class_id != tracked_id_) continue;
    const double position_diff = (predicted_position - obs.position_gimbal).norm();
    const double yaw_diff = std::abs(shortestAngularDistance(x_(6), obs.yaw));
    if (position_diff < min_position_diff) {
      min_position_diff = position_diff;
      min_yaw_diff = yaw_diff;
      best_same_id = &obs;
    }
  }

  if (best_same_id) {
    // V3 runs the tracker in the current gimbal frame. During a fast yaw/pitch
    // correction a stationary armor can move noticeably in this frame, so a
    // hard gate here causes target switching or TEMP_LOST flicker. Keep the
    // same robot id sticky; update() will decide whether to snap or EKF update.
    return *best_same_id;
  }

  return *std::min_element(observations.begin(), observations.end(), score_by_center);
}

AimTarget CsuArmorTracker::update(
  const ArmorObservation & observation,
  double stamp_sec,
  double predict_delay_sec)
{
  if (!initialized_ || tracked_id_ != observation.class_id) {
    init(observation, stamp_sec);
    return makeTarget(stamp_sec, predict_delay_sec, observation.confidence, "csu_tracker");
  }

  predictInPlace(stamp_sec);

  const int prior_tracking_frames = tracking_frames_;
  const Eigen::Vector3d predicted_position = armorPositionFromState(x_);
  const double position_diff = (predicted_position - observation.position_gimbal).norm();
  const double yaw_diff = std::abs(shortestAngularDistance(x_(6), observation.yaw));
  bool matched =
    position_diff < params_.max_match_distance &&
    yaw_diff < params_.max_match_yaw_diff;

  if (matched) {
    updateEkf(observation);
  } else {
    handleArmorJump(observation);
    const Eigen::Vector3d inferred_position = armorPositionFromState(x_);
    matched = (inferred_position - observation.position_gimbal).norm() < params_.max_match_distance * 1.5;
    if (!matched && observation.class_id == tracked_id_ && observation.valid) {
      // Same-id observations are still useful even when the current-frame EKF
      // prediction lags behind cloud/gimbal motion. Snap to the observed
      // shootable armor point instead of declaring TEMP_LOST for one frame.
      init(observation, stamp_sec);
      tracker_state_ = CsuTrackerState::TRACKING;
      tracking_frames_ = std::max(
        prior_tracking_frames, params_.min_tracking_count_for_fire);
      matched = true;
    } else if (matched) {
      updateEkf(observation);
    }
  }

  x_(8) = std::clamp(std::abs(x_(8)), params_.min_radius, params_.max_radius);
  tracked_id_ = observation.class_id;
  tracked_label_ = observation.label;
  tracked_armor_type_ = observation.armor_type;
  tracked_armors_num_ = inferArmorsNum(observation);
  last_selected_position_ = observation.position_gimbal;
  last_detection_stamp_sec_ = stamp_sec;

  if (tracker_state_ == CsuTrackerState::DETECTING) {
    if (matched) {
      detect_count_++;
      tracking_frames_++;
      if (detect_count_ >= params_.min_detect_count) {
        detect_count_ = 0;
        tracker_state_ = CsuTrackerState::TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state_ = CsuTrackerState::LOST;
      initialized_ = false;
    }
  } else if (tracker_state_ == CsuTrackerState::TRACKING) {
    if (!matched) {
      tracker_state_ = CsuTrackerState::TEMP_LOST;
      lost_count_++;
    } else {
      tracking_frames_++;
      lost_count_ = 0;
    }
  } else if (tracker_state_ == CsuTrackerState::TEMP_LOST) {
    if (matched) {
      tracker_state_ = CsuTrackerState::TRACKING;
      lost_count_ = 0;
      tracking_frames_++;
    } else {
      lost_count_++;
    }
  } else {
    tracker_state_ = CsuTrackerState::DETECTING;
    detect_count_ = 1;
    tracking_frames_ = 1;
  }

  return makeTarget(stamp_sec, predict_delay_sec, observation.confidence, "csu_tracker");
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
      (tracker_state_ == CsuTrackerState::TRACKING ||
       tracker_state_ == CsuTrackerState::TEMP_LOST))
  {
    tracker_state_ = CsuTrackerState::TEMP_LOST;
    lost_count_++;
    return makeTarget(stamp_sec, predict_delay_sec, 0.0, "csu_tracker_temp_lost");
  }

  reset();
  return {};
}

void CsuArmorTracker::init(const ArmorObservation & observation, double stamp_sec)
{
  const double yaw = observation.yaw;
  const double r = params_.default_radius;
  x_.setZero();
  x_(0) = observation.position_gimbal.x() + r * std::cos(yaw);
  x_(1) = 0.0;
  x_(2) = observation.position_gimbal.y() + r * std::sin(yaw);
  x_(3) = 0.0;
  x_(4) = observation.position_gimbal.z();
  x_(5) = 0.0;
  x_(6) = yaw;
  x_(7) = 0.0;
  x_(8) = r;
  x_(9) = 0.0;

  P_.setIdentity();
  P_.block<3, 3>(0, 0) *= 0.05;
  P_.block<3, 3>(1, 1) *= 1.0;
  P_(6, 6) = 0.3;
  P_(7, 7) = 1.0;
  P_(8, 8) = 0.05;
  P_(9, 9) = 0.05;

  initialized_ = true;
  last_stamp_sec_ = stamp_sec;
  last_detection_stamp_sec_ = stamp_sec;
  tracker_state_ = CsuTrackerState::DETECTING;
  detect_count_ = 1;
  lost_count_ = 0;
  tracking_frames_ = 1;
  tracked_id_ = observation.class_id;
  tracked_label_ = observation.label;
  tracked_armor_type_ = observation.armor_type;
  tracked_armors_num_ = inferArmorsNum(observation);
  last_yaw_ = yaw;
  another_r_ = r;
  d_za_ = 0.0;
  d_zc_ = 0.0;
  last_selected_position_ = observation.position_gimbal;
}

void CsuArmorTracker::predictInPlace(double stamp_sec)
{
  double dt = stamp_sec - last_stamp_sec_;
  if (dt <= 0.0) dt = 0.001;
  if (dt > 0.5) dt = 0.5;

  const StateMat F = numericalF(x_, dt);
  StateMat Q = StateMat::Zero();
  Q(0, 0) = params_.q_xyz * params_.q_xyz;
  Q(2, 2) = params_.q_xyz * params_.q_xyz;
  Q(4, 4) = params_.q_xyz * params_.q_xyz;
  Q(1, 1) = params_.q_xyz * params_.q_xyz * 10.0;
  Q(3, 3) = params_.q_xyz * params_.q_xyz * 10.0;
  Q(5, 5) = params_.q_xyz * params_.q_xyz * 10.0;
  Q(6, 6) = params_.q_yaw * params_.q_yaw;
  Q(7, 7) = params_.q_yaw * params_.q_yaw * 10.0;
  Q(8, 8) = params_.q_radius * params_.q_radius;
  Q(9, 9) = params_.q_xyz * params_.q_xyz;

  x_ = processModel(x_, dt);
  P_ = F * P_ * F.transpose() + Q;
  last_stamp_sec_ = stamp_sec;
}

void CsuArmorTracker::updateEkf(const ArmorObservation & observation)
{
  MeasureVec z;
  z << observation.position_gimbal.x(),
    observation.position_gimbal.y(),
    observation.position_gimbal.z(),
    observation.yaw;

  const auto H = numericalH(x_);
  MeasureMat R = MeasureMat::Zero();
  R(0, 0) = params_.r_xyz * params_.r_xyz;
  R(1, 1) = params_.r_xyz * params_.r_xyz;
  R(2, 2) = params_.r_xyz * params_.r_xyz;
  R(3, 3) = params_.r_yaw * params_.r_yaw;

  MeasureVec innovation = z - measureModel(x_);
  innovation(3) = shortestAngularDistance(measureModel(x_)(3), z(3));

  const MeasureMat S = H * P_ * H.transpose() + R;
  const Eigen::Matrix<double, X_N, Z_N> K = P_ * H.transpose() * S.inverse();
  x_ = x_ + K * innovation;
  x_(6) = normalizeAngle(x_(6));
  P_ = (StateMat::Identity() - K * H) * P_;

  if (!finiteState(x_)) {
    x_.setZero();
    initialized_ = false;
    tracker_state_ = CsuTrackerState::LOST;
  }
}

void CsuArmorTracker::handleArmorJump(const ArmorObservation & observation)
{
  const double yaw = observation.yaw;
  if (std::abs(shortestAngularDistance(x_(6), yaw)) > 0.4) {
    x_(6) = yaw;
    if (tracked_armors_num_ == ArmorsNum::NORMAL_4) {
      d_za_ = x_(4) + x_(9) - observation.position_gimbal.z();
      std::swap(x_(8), another_r_);
      d_zc_ = (std::abs(d_zc_) < 1e-6) ? -d_za_ : 0.0;
      x_(9) = d_zc_;
    }
  }

  const Eigen::Vector3d inferred = armorPositionFromState(x_);
  if ((observation.position_gimbal - inferred).norm() > params_.max_match_distance) {
    d_zc_ = 0.0;
    const double r = std::clamp(std::abs(x_(8)), params_.min_radius, params_.max_radius);
    x_(0) = observation.position_gimbal.x() + r * std::cos(yaw);
    x_(1) = 0.0;
    x_(2) = observation.position_gimbal.y() + r * std::sin(yaw);
    x_(3) = 0.0;
    x_(4) = observation.position_gimbal.z();
    x_(5) = 0.0;
    x_(9) = d_zc_;
  }
}

CsuArmorTracker::StateVec CsuArmorTracker::processModel(const StateVec & x, double dt) const
{
  StateVec y = x;
  y(0) += x(1) * dt;
  y(2) += x(3) * dt;
  y(4) += x(5) * dt;
  y(6) = normalizeAngle(x(6) + x(7) * dt);
  return y;
}

CsuArmorTracker::MeasureVec CsuArmorTracker::measureModel(const StateVec & x) const
{
  MeasureVec z;
  z << x(0) - std::cos(x(6)) * x(8),
    x(2) - std::sin(x(6)) * x(8),
    x(4) + x(9),
    normalizeAngle(x(6));
  return z;
}

CsuArmorTracker::StateMat CsuArmorTracker::numericalF(const StateVec & x, double dt) const
{
  StateMat F = StateMat::Zero();
  const double eps = 1e-5;
  for (int i = 0; i < X_N; ++i) {
    StateVec xp = x;
    StateVec xm = x;
    xp(i) += eps;
    xm(i) -= eps;
    F.col(i) = (processModel(xp, dt) - processModel(xm, dt)) / (2.0 * eps);
  }
  return F;
}

Eigen::Matrix<double, CsuArmorTracker::Z_N, CsuArmorTracker::X_N>
CsuArmorTracker::numericalH(const StateVec & x) const
{
  Eigen::Matrix<double, Z_N, X_N> H = Eigen::Matrix<double, Z_N, X_N>::Zero();
  const double eps = 1e-5;
  for (int i = 0; i < X_N; ++i) {
    StateVec xp = x;
    StateVec xm = x;
    xp(i) += eps;
    xm(i) -= eps;
    H.col(i) = (measureModel(xp) - measureModel(xm)) / (2.0 * eps);
  }
  return H;
}

AimTarget CsuArmorTracker::makeTarget(
  double stamp_sec,
  double predict_delay_sec,
  double confidence,
  const std::string & source) const
{
  AimTarget target;
  if (!initialized_ || !finiteState(x_)) return target;

  double dt = stamp_sec - last_stamp_sec_ + std::max(0.0, predict_delay_sec);
  if (dt < 0.0) dt = 0.0;
  if (dt > 0.8) dt = 0.8;
  const StateVec pred = processModel(x_, dt);
  const StateVec pred_next = processModel(pred, 0.02);

  target.position_gimbal = armorPositionFromState(pred);
  target.velocity_gimbal = (armorPositionFromState(pred_next) - target.position_gimbal) / 0.02;
  target.distance = target.position_gimbal.norm();
  target.confidence = confidence;
  target.source = source;
  target.valid = target.position_gimbal.allFinite() &&
    target.velocity_gimbal.allFinite() &&
    target.distance > 0.05;
  return target;
}

Eigen::Vector3d CsuArmorTracker::armorPositionFromState(const StateVec & x) const
{
  const double xa = x(0) - std::cos(x(6)) * x(8);
  const double ya = x(2) - std::sin(x(6)) * x(8);
  const double za = x(4) + x(9);
  return Eigen::Vector3d(xa, ya, za);
}

Eigen::Vector3d CsuArmorTracker::predictedPosition(
  double stamp_sec,
  double predict_delay_sec) const
{
  return makeTarget(stamp_sec, predict_delay_sec, 0.0, "csu_tracker").position_gimbal;
}

ArmorsNum CsuArmorTracker::inferArmorsNum(const ArmorObservation & observation) const
{
  if (observation.label.find("outpost") != std::string::npos) {
    return ArmorsNum::OUTPOST_3;
  }
  if (observation.armor_type == "large" &&
      (observation.label.find("3") != std::string::npos ||
       observation.label.find("4") != std::string::npos ||
       observation.label.find("5") != std::string::npos))
  {
    return ArmorsNum::BALANCE_2;
  }
  return ArmorsNum::NORMAL_4;
}

double CsuArmorTracker::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double CsuArmorTracker::shortestAngularDistance(double from, double to)
{
  return normalizeAngle(to - from);
}

bool CsuArmorTracker::finiteState(const StateVec & state)
{
  for (int i = 0; i < state.size(); ++i) {
    if (!std::isfinite(state(i))) return false;
  }
  return true;
}

}  // namespace rm_autoaim
