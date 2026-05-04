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
// RCS V3 adaptation of CSU-FYT ArmorPoseEstimator/PnPSolver ideas. It keeps
// RCS calibration parameters and message types, and only ports the PnP
// multi-solution selection and armor pose observation semantics.

#ifndef RM_AUTOAIM__CORE__ARMOR_POSE_ESTIMATOR_HPP_
#define RM_AUTOAIM__CORE__ARMOR_POSE_ESTIMATOR_HPP_

#include <optional>
#include <string>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <rm_interfaces/msg/armor_detection.hpp>

#include "rm_autoaim/core/aim_target.hpp"

namespace rm_autoaim
{

class ArmorPoseEstimator
{
public:
  void configure(
    const cv::Mat & camera_matrix,
    const cv::Mat & dist_coeffs,
    const cv::Matx33d & R_cam_to_gimbal,
    const cv::Vec3d & t_cam_to_gimbal,
    double armor_small_width,
    double armor_small_height,
    double armor_large_width,
    double armor_large_height,
    bool refine_iterative);

  std::optional<ArmorObservation> estimate(
    const rm_interfaces::msg::ArmorDetection & armor,
    const cv::Point2f & optical_axis_px) const;

  static std::string armorTypeFromDetection(
    const rm_interfaces::msg::ArmorDetection & armor);

private:
  bool solvePnP(
    const rm_interfaces::msg::ArmorDetection & armor,
    cv::Vec3d & rvec,
    cv::Vec3d & tvec,
    double & reprojection_error) const;

  int selectPnPSolution(
    const rm_interfaces::msg::ArmorDetection & armor,
    const std::vector<cv::Point3d> & object_points,
    const std::vector<cv::Point2d> & image_points,
    const std::vector<cv::Vec3d> & rvecs,
    const std::vector<cv::Vec3d> & tvecs) const;

  double calculateReprojectionError(
    const std::vector<cv::Point3d> & object_points,
    const std::vector<cv::Point2d> & image_points,
    const cv::Vec3d & rvec,
    const cv::Vec3d & tvec) const;

  Eigen::Vector3d rotationMatrixToRPY(const cv::Mat & R) const;
  double imageArmorTiltDeg(const rm_interfaces::msg::ArmorDetection & armor) const;
  double armorYawInGimbal(const cv::Vec3d & rvec) const;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Matx33d R_cam_to_gimbal_ = cv::Matx33d::eye();
  cv::Vec3d t_cam_to_gimbal_ = cv::Vec3d(0.0, 0.0, 0.0);
  double armor_small_width_ = 0.135;
  double armor_small_height_ = 0.055;
  double armor_large_width_ = 0.225;
  double armor_large_height_ = 0.055;
  bool refine_iterative_ = true;
};

}  // namespace rm_autoaim

#endif  // RM_AUTOAIM__CORE__ARMOR_POSE_ESTIMATOR_HPP_
