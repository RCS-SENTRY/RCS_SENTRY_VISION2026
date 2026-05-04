#include "rm_autoaim/core/armor_pose_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace rm_autoaim
{

void ArmorPoseEstimator::configure(
  const cv::Mat & camera_matrix,
  const cv::Mat & dist_coeffs,
  const cv::Matx33d & R_cam_to_gimbal,
  const cv::Vec3d & t_cam_to_gimbal,
  double armor_small_width,
  double armor_small_height,
  double armor_large_width,
  double armor_large_height,
  bool refine_iterative)
{
  camera_matrix_ = camera_matrix.clone();
  dist_coeffs_ = dist_coeffs.clone();
  R_cam_to_gimbal_ = R_cam_to_gimbal;
  t_cam_to_gimbal_ = t_cam_to_gimbal;
  armor_small_width_ = armor_small_width;
  armor_small_height_ = armor_small_height;
  armor_large_width_ = armor_large_width;
  armor_large_height_ = armor_large_height;
  refine_iterative_ = refine_iterative;
}

std::optional<ArmorObservation> ArmorPoseEstimator::estimate(
  const rm_interfaces::msg::ArmorDetection & armor,
  const cv::Point2f & optical_axis_px) const
{
  cv::Vec3d rvec;
  cv::Vec3d tvec;
  double reprojection_error = 0.0;
  if (!solvePnP(armor, rvec, tvec, reprojection_error)) {
    return std::nullopt;
  }

  cv::Vec3d p_gimbal = R_cam_to_gimbal_ * tvec + t_cam_to_gimbal_;

  cv::Point2f center_px(0.0f, 0.0f);
  for (const auto & apex : armor.apexes) {
    center_px.x += static_cast<float>(apex.x);
    center_px.y += static_cast<float>(apex.y);
  }
  center_px.x *= 0.25f;
  center_px.y *= 0.25f;

  ArmorObservation obs;
  obs.position_gimbal = Eigen::Vector3d(p_gimbal(0), p_gimbal(1), p_gimbal(2));
  obs.yaw = armorYawInGimbal(rvec);
  obs.distance = obs.position_gimbal.norm();
  obs.confidence = armor.confidence;
  obs.class_id = armor.class_id;
  obs.label = armor.label;
  obs.armor_type = armorTypeFromDetection(armor);
  obs.center_axis_error_px = cv::norm(center_px - optical_axis_px);
  obs.valid = obs.position_gimbal.allFinite() &&
    std::isfinite(obs.yaw) &&
    obs.distance > 0.05 &&
    reprojection_error < 12.0;
  return obs;
}

std::string ArmorPoseEstimator::armorTypeFromDetection(
  const rm_interfaces::msg::ArmorDetection & armor)
{
  if (armor.label.find("large") != std::string::npos ||
      armor.label.find("big") != std::string::npos)
  {
    return "large";
  }

  static const std::unordered_set<int> kBigArmorClassIds = {
    21, 22, 23, 24, 29, 30, 31, 32, 33, 34, 35, 36, 37,
  };
  return kBigArmorClassIds.count(armor.class_id) > 0 ? "large" : "small";
}

bool ArmorPoseEstimator::solvePnP(
  const rm_interfaces::msg::ArmorDetection & armor,
  cv::Vec3d & rvec,
  cv::Vec3d & tvec,
  double & reprojection_error) const
{
  const bool is_large = armorTypeFromDetection(armor) == "large";
  const double half_w = (is_large ? armor_large_width_ : armor_small_width_) / 2.0;
  const double half_h = (is_large ? armor_large_height_ : armor_small_height_) / 2.0;

  std::vector<cv::Point3d> object_points = {
    {-half_w,  half_h, 0.0},
    { half_w,  half_h, 0.0},
    { half_w, -half_h, 0.0},
    {-half_w, -half_h, 0.0},
  };

  std::vector<cv::Point2d> image_points;
  image_points.reserve(4);
  for (int i = 0; i < 4; ++i) {
    image_points.emplace_back(armor.apexes[i].x, armor.apexes[i].y);
  }

  std::vector<cv::Vec3d> rvecs;
  std::vector<cv::Vec3d> tvecs;
  const bool ok = cv::solvePnPGeneric(
    object_points, image_points,
    camera_matrix_, dist_coeffs_,
    rvecs, tvecs,
    false, cv::SOLVEPNP_IPPE);

  if (!ok || rvecs.empty()) return false;

  const int best_idx = selectPnPSolution(armor, object_points, image_points, rvecs, tvecs);
  rvec = rvecs[best_idx];
  tvec = tvecs[best_idx];

  if (refine_iterative_) {
    if (!cv::solvePnP(
        object_points, image_points,
        camera_matrix_, dist_coeffs_,
        rvec, tvec,
        true, cv::SOLVEPNP_ITERATIVE))
    {
      return false;
    }
  }

  reprojection_error = calculateReprojectionError(object_points, image_points, rvec, tvec);
  return std::isfinite(tvec[2]) && tvec[2] > 0.0;
}

int ArmorPoseEstimator::selectPnPSolution(
  const rm_interfaces::msg::ArmorDetection & armor,
  const std::vector<cv::Point3d> & object_points,
  const std::vector<cv::Point2d> & image_points,
  const std::vector<cv::Vec3d> & rvecs,
  const std::vector<cv::Vec3d> & tvecs) const
{
  if (rvecs.size() < 2 || tvecs.size() < 2) return 0;

  std::vector<double> errors(rvecs.size(), std::numeric_limits<double>::max());
  for (std::size_t i = 0; i < rvecs.size(); ++i) {
    if (std::isfinite(tvecs[i][2]) && tvecs[i][2] > 0.0) {
      errors[i] = calculateReprojectionError(object_points, image_points, rvecs[i], tvecs[i]);
    }
  }

  int reproj_best = static_cast<int>(
    std::min_element(errors.begin(), errors.end()) - errors.begin());

  const double error1 = errors[0];
  const double error2 = errors[1];
  if (!std::isfinite(error1) || !std::isfinite(error2)) return reproj_best;
  if (std::max(error1, error2) / std::max(1e-6, std::min(error1, error2)) > 3.0) {
    return reproj_best;
  }

  cv::Mat R1_cv;
  cv::Mat R2_cv;
  cv::Rodrigues(rvecs[0], R1_cv);
  cv::Rodrigues(rvecs[1], R2_cv);
  auto rpy1 = rotationMatrixToRPY(cv::Mat(R_cam_to_gimbal_) * R1_cv);
  auto rpy2 = rotationMatrixToRPY(cv::Mat(R_cam_to_gimbal_) * R2_cv);
  if (std::abs(rpy1.x()) > 10.0 * M_PI / 180.0 ||
      std::abs(rpy2.x()) > 10.0 * M_PI / 180.0)
  {
    return reproj_best;
  }

  double angle = imageArmorTiltDeg(armor);
  if (armor.label.find("outpost") != std::string::npos) {
    angle = -angle;
  }

  if ((angle > 0.0 && rpy1.z() > 0.0 && rpy2.z() < 0.0) ||
      (angle < 0.0 && rpy1.z() < 0.0 && rpy2.z() > 0.0))
  {
    return 1;
  }
  return 0;
}

double ArmorPoseEstimator::calculateReprojectionError(
  const std::vector<cv::Point3d> & object_points,
  const std::vector<cv::Point2d> & image_points,
  const cv::Vec3d & rvec,
  const cv::Vec3d & tvec) const
{
  std::vector<cv::Point2d> projected;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_, projected);
  double err = 0.0;
  for (std::size_t i = 0; i < projected.size(); ++i) {
    err += cv::norm(projected[i] - image_points[i]);
  }
  return err / std::max<std::size_t>(1, projected.size());
}

Eigen::Vector3d ArmorPoseEstimator::rotationMatrixToRPY(const cv::Mat & R_cv) const
{
  Eigen::Matrix3d R;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R(r, c) = R_cv.at<double>(r, c);
    }
  }
  return R.eulerAngles(0, 1, 2);
}

double ArmorPoseEstimator::imageArmorTiltDeg(
  const rm_interfaces::msg::ArmorDetection & armor) const
{
  const auto & lt = armor.apexes[0];
  const auto & rt = armor.apexes[1];
  const auto & rb = armor.apexes[2];
  const auto & lb = armor.apexes[3];
  const double l_angle = std::atan2(lb.y - lt.y, lb.x - lt.x) * 180.0 / M_PI;
  const double r_angle = std::atan2(rb.y - rt.y, rb.x - rt.x) * 180.0 / M_PI;
  return (l_angle + r_angle) * 0.5 + 90.0;
}

double ArmorPoseEstimator::armorYawInGimbal(const cv::Vec3d & rvec) const
{
  cv::Mat R_cv;
  cv::Rodrigues(rvec, R_cv);
  auto rpy = rotationMatrixToRPY(cv::Mat(R_cam_to_gimbal_) * R_cv);
  return rpy.z();
}

}  // namespace rm_autoaim
