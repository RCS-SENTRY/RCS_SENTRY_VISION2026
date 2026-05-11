// =============================================================================
// aimer.hpp — AimTarget inverse kinematics + fire window
// =============================================================================
// V3 production contract:
//   Aimer only accepts AimTarget, whose position_gimbal is the shootable armor
//   point in the current gimbal frame. ManualCompensator is the only production
//   yaw/pitch correction path.
// =============================================================================
#ifndef RM_AUTOAIM__AIMER_HPP_
#define RM_AUTOAIM__AIMER_HPP_

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Dense>

#include "rm_autoaim/core/aim_target.hpp"

namespace rm_autoaim
{

struct AimerParams
{
  double bullet_speed = 15.0;
  double fire_delay = 0.10;

  double yaw_offset = 0.0;
  bool pitch_invert = false;
  bool output_relative_command = false;

  double fire_tolerance = 0.03;
  double dynamic_lead_scale = 0.75;
  bool use_dynamic_fire_window = true;
  double shooting_range_width = 0.135;
  double shooting_range_height = 0.055;
  double min_fire_window_deg = 1.0;
  double max_fire_window_deg = 3.0;

  int fire_min_frames = 3;
  double fire_max_distance = 8.0;
};

class Aimer
{
public:
  explicit Aimer(const AimerParams & params = AimerParams())
  : params_(params)
  {
  }

  const AimerParams & getParams() const { return params_; }

  struct AimResult
  {
    double target_yaw = 0.0;
    double target_pitch = 0.0;
    double yaw_vel = 0.0;
    double pitch_vel = 0.0;
    bool alignment_ready = false;
    bool tracker_ready = false;
    bool can_fire = false;
    double t_flight = 0.0;
    double target_distance = 0.0;
    double yaw_window = 0.0;
    double pitch_window = 0.0;
    double manual_yaw_offset = 0.0;
    double manual_pitch_offset = 0.0;
    bool output_relative_command = false;
  };

  AimResult solve(
    const AimTarget & target,
    double current_yaw_deg,
    double current_pitch_deg,
    int tracker_frames,
    double bullet_speed = 0.0,
    bool input_in_current_gimbal_frame = true,
    double manual_yaw_offset = 0.0,
    double manual_pitch_offset = 0.0,
    double dynamic_lead_scale = -1.0) const
  {
    AimResult result;
    result.manual_yaw_offset = manual_yaw_offset;
    result.manual_pitch_offset = manual_pitch_offset;
    result.output_relative_command =
      input_in_current_gimbal_frame && params_.output_relative_command;
    result.yaw_window = params_.fire_tolerance;
    result.pitch_window = params_.fire_tolerance;

    if (!target.valid || !target.position_gimbal.allFinite() ||
        !target.velocity_gimbal.allFinite())
    {
      return result;
    }

    const double v_bullet = (bullet_speed > 2.0) ? bullet_speed : params_.bullet_speed;
    const double current_yaw = current_yaw_deg * M_PI / 180.0;
    const double current_pitch = current_pitch_deg * M_PI / 180.0;

    Eigen::Vector3d hit_pos = target.position_gimbal;
    const double lead_scale = std::clamp(
      dynamic_lead_scale >= 0.0 ? dynamic_lead_scale : params_.dynamic_lead_scale,
      0.0, 2.0);
    double t_flight = hit_pos.norm() / std::max(1e-3, v_bullet);
    for (int i = 0; i < 3; ++i) {
      hit_pos = target.position_gimbal +
        target.velocity_gimbal * ((params_.fire_delay + t_flight) * lead_scale);
      t_flight = hit_pos.norm() / std::max(1e-3, v_bullet);
    }
    result.t_flight = t_flight;
    result.target_distance = hit_pos.norm();

    double frame_yaw = 0.0;
    double frame_pitch = 0.0;
    inverseKinematics(hit_pos, frame_yaw, frame_pitch);

    const double command_yaw = frame_yaw + params_.yaw_offset + manual_yaw_offset;
    const double compensated_pitch = frame_pitch + manual_pitch_offset;
    const double command_pitch = params_.pitch_invert ? -compensated_pitch : compensated_pitch;

    const bool relative_output = result.output_relative_command;

    if (relative_output) {
      result.target_yaw = std::atan2(std::sin(command_yaw), std::cos(command_yaw));
      result.target_pitch = command_pitch;
    } else if (input_in_current_gimbal_frame) {
      result.target_yaw = current_yaw + command_yaw;
      result.target_pitch = current_pitch + command_pitch;
    } else {
      result.target_yaw = std::atan2(std::sin(command_yaw), std::cos(command_yaw));
      result.target_pitch = command_pitch;
    }

    const double x = hit_pos.x();
    const double y = hit_pos.y();
    const double vx = target.velocity_gimbal.x();
    const double vy = target.velocity_gimbal.y();
    const double r_sq = x * x + y * y;
    result.yaw_vel = (r_sq > 1e-4) ? ((x * vy - y * vx) / r_sq) : 0.0;
    result.pitch_vel = 0.0;

    auto windows = getFireWindows(result.target_distance);
    result.yaw_window = windows.first;
    result.pitch_window = windows.second;

    result.alignment_ready = relative_output ?
      (std::abs(result.target_yaw) < result.yaw_window &&
       std::abs(result.target_pitch) < result.pitch_window) :
      checkAlignmentReady(
        result.target_yaw, result.target_pitch,
        current_yaw_deg, current_pitch_deg,
        result.target_distance);
    result.tracker_ready =
      tracker_frames >= params_.fire_min_frames &&
      result.target_distance <= params_.fire_max_distance;
    result.can_fire = result.alignment_ready && result.tracker_ready;
    return result;
  }

  void inverseKinematics(const Eigen::Vector3d & hit_pos, double & yaw, double & pitch) const
  {
    yaw = std::atan2(hit_pos.y(), hit_pos.x());
    const double d_h = std::hypot(hit_pos.x(), hit_pos.y());
    pitch = std::atan2(-hit_pos.z(), d_h);
  }

  bool checkAlignmentReady(
    double target_yaw,
    double target_pitch,
    double current_yaw_deg,
    double current_pitch_deg,
    double distance) const
  {
    const double cur_yaw = current_yaw_deg * M_PI / 180.0;
    const double cur_pitch = current_pitch_deg * M_PI / 180.0;
    const double err_yaw = std::abs(std::atan2(
      std::sin(target_yaw - cur_yaw), std::cos(target_yaw - cur_yaw)));
    const double err_pitch = std::abs(target_pitch - cur_pitch);
    auto windows = getFireWindows(distance);
    return err_yaw < windows.first && err_pitch < windows.second;
  }

  std::pair<double, double> getFireWindows(double distance) const
  {
    if (!params_.use_dynamic_fire_window || distance <= 1e-3) {
      return {params_.fire_tolerance, params_.fire_tolerance};
    }

    double yaw_window = std::atan2(params_.shooting_range_width / 2.0, distance);
    double pitch_window = std::atan2(params_.shooting_range_height / 2.0, distance);

    double min_window = params_.min_fire_window_deg * M_PI / 180.0;
    double max_window = params_.max_fire_window_deg * M_PI / 180.0;
    if (min_window > max_window) std::swap(min_window, max_window);

    yaw_window = std::clamp(yaw_window, min_window, max_window);
    pitch_window = std::clamp(pitch_window, min_window, max_window);
    return {yaw_window, pitch_window};
  }

private:
  AimerParams params_;
};

}  // namespace rm_autoaim

#endif  // RM_AUTOAIM__AIMER_HPP_
