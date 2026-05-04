// =============================================================================
// aim_target.hpp — V3 production aim contract
// =============================================================================
#ifndef RM_AUTOAIM__CORE__AIM_TARGET_HPP_
#define RM_AUTOAIM__CORE__AIM_TARGET_HPP_

#include <string>

#include <Eigen/Dense>

namespace rm_autoaim
{

struct AimTarget
{
  Eigen::Vector3d position_gimbal = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_gimbal = Eigen::Vector3d::Zero();
  double distance = 0.0;
  double confidence = 0.0;
  std::string source = "none";
  bool valid = false;
};

struct ArmorObservation
{
  Eigen::Vector3d position_gimbal = Eigen::Vector3d::Zero();
  double distance = 0.0;
  double confidence = 0.0;
  int class_id = -1;
  std::string label;
  std::string armor_type = "small";
  double center_axis_error_px = 0.0;
};

enum class CsuTrackerState
{
  LOST,
  DETECTING,
  TRACKING,
  TEMP_LOST,
};

inline const char * tracker_state_name(CsuTrackerState state)
{
  switch (state) {
    case CsuTrackerState::LOST:
      return "LOST";
    case CsuTrackerState::DETECTING:
      return "DETECTING";
    case CsuTrackerState::TRACKING:
      return "TRACKING";
    case CsuTrackerState::TEMP_LOST:
      return "TEMP_LOST";
  }
  return "UNKNOWN";
}

}  // namespace rm_autoaim

#endif  // RM_AUTOAIM__CORE__AIM_TARGET_HPP_
