// Copyright (c) 2024 FRC Team. All rights reserved.
// SPDX-License-Identifier: MIT

#include "ros_vision/pose_estimation.hpp"
#include "ros_vision/coordinate_transforms.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ros_vision {

// ---------------------------------------------------------------------------
// CameraIntrinsics helpers
// ---------------------------------------------------------------------------

cv::Mat CameraIntrinsics::cameraMatrixCV() const
{
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = fx;
  K.at<double>(1, 1) = fy;
  K.at<double>(0, 2) = cx;
  K.at<double>(1, 2) = cy;
  return K;
}

cv::Mat CameraIntrinsics::distCoeffsCV() const
{
  cv::Mat d(1, 5, CV_64F);
  for (int i = 0; i < 5; ++i) {
    d.at<double>(0, i) = distCoeffs[i];
  }
  return d;
}

// ---------------------------------------------------------------------------
// PoseEstimator constructor
// ---------------------------------------------------------------------------

PoseEstimator::PoseEstimator(
  const CameraIntrinsics & intrinsics,
  double tagSizeMetres,
  const Transform3d & camRobotTransform)
: intrinsics_(intrinsics),
  tagSizeMetres_(tagSizeMetres),
  camRobotTransform_(camRobotTransform)
{
  if (tagSizeMetres_ <= 0.0) {
    throw std::invalid_argument("PoseEstimator: tagSizeMetres must be positive");
  }
}

// ---------------------------------------------------------------------------
// tagObjectPoints
// ---------------------------------------------------------------------------
// Returns the four corners of the tag in the **detection tag frame**:
//   origin at centre
//   +x right (when looking at the tag)
//   +y down  (when looking at the tag)
//   +z toward the camera  (out of visible face)
//
// The corners are ordered to match the pixel-corner ordering produced by
// Isaac ROS AprilTag:  top-left, top-right, bottom-right, bottom-left
// (image-space order, starting at the corner with smallest pixel coords).
// ---------------------------------------------------------------------------
std::vector<cv::Point3f> PoseEstimator::tagObjectPoints() const
{
  const float s = static_cast<float>(tagSizeMetres_ / 2.0);
  // Tag plane is z=0; corners in tag frame:
  //   top-left     (-s, -s, 0)   — small x, small y in image
  //   top-right    (+s, -s, 0)
  //   bottom-right (+s, +s, 0)
  //   bottom-left  (-s, +s, 0)
  return {
    cv::Point3f(-s, -s, 0.0f),
    cv::Point3f(+s, -s, 0.0f),
    cv::Point3f(+s, +s, 0.0f),
    cv::Point3f(-s, +s, 0.0f),
  };
}

// ---------------------------------------------------------------------------
// estimateSingleTag
// ---------------------------------------------------------------------------
SingleTagResult PoseEstimator::estimateSingleTag(
  int tagId,
  const TagCorners & corners) const
{
  SingleTagResult result;
  result.tagId   = tagId;
  result.corners = corners;

  // Build image-point vector from the corner pixels.
  std::vector<cv::Point2f> imagePoints;
  imagePoints.reserve(4);
  double cx_sum = 0.0, cy_sum = 0.0;
  for (const auto & c : corners.pixels) {
    imagePoints.emplace_back(static_cast<float>(c[0]), static_cast<float>(c[1]));
    cx_sum += c[0];
    cy_sum += c[1];
  }
  result.center = {cx_sum / 4.0, cy_sum / 4.0};

  const cv::Mat K  = intrinsics_.cameraMatrixCV();
  const cv::Mat dc = intrinsics_.distCoeffsCV();
  const std::vector<cv::Point3f> objPts = tagObjectPoints();

  // IPPE_SQUARE returns both pose solutions sorted by reprojection error.
  // It is the recommended method for planar square targets such as AprilTags.
  std::vector<cv::Mat> rvecs, tvecs;
  const int nsols = cv::solvePnPGeneric(
    objPts, imagePoints, K, dc,
    rvecs, tvecs,
    false, cv::SOLVEPNP_IPPE_SQUARE);

  if (nsols < 1) {
    // No solution found — return result with invalid reprojection error.
    return result;
  }

  // Compute reprojection error for each solution.
  auto computeReprojError = [&](const cv::Mat & rvec, const cv::Mat & tvec) -> double {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(objPts, rvec, tvec, K, dc, projected);
    double total = 0.0;
    for (std::size_t i = 0; i < imagePoints.size(); ++i) {
      const double dx = projected[i].x - imagePoints[i].x;
      const double dy = projected[i].y - imagePoints[i].y;
      total += std::sqrt(dx * dx + dy * dy);
    }
    return total / static_cast<double>(imagePoints.size());
  };

  // Fill best solution (IPPE_SQUARE already sorts best first).
  {
    Eigen::Vector3d rvec_e, tvec_e;
    cv::cv2eigen(rvecs[0].reshape(1, 3), rvec_e);
    cv::cv2eigen(tvecs[0].reshape(1, 3), tvec_e);
    result.best.pose             = solvePnPToTransform(rvec_e, tvec_e);
    result.best.reprojectionError = computeReprojError(rvecs[0], tvecs[0]);
  }

  if (nsols >= 2) {
    Eigen::Vector3d rvec_e, tvec_e;
    cv::cv2eigen(rvecs[1].reshape(1, 3), rvec_e);
    cv::cv2eigen(tvecs[1].reshape(1, 3), tvec_e);
    PoseSolution alt;
    alt.pose             = solvePnPToTransform(rvec_e, tvec_e);
    alt.reprojectionError = computeReprojError(rvecs[1], tvecs[1]);
    result.alt = alt;

    // Ambiguity = best_error / alt_error ∈ [0, 1].
    // IPPE_SQUARE already returns solutions sorted by reprojection error
    // (best first), so e1 <= e2 is guaranteed.
    const double e1 = result.best.reprojectionError;
    const double e2 = alt.reprojectionError;
    if (e2 > 1e-6) {
      result.ambiguity = e1 / e2;
    }
  }

  return result;
}

// ---------------------------------------------------------------------------
// estimateMultiTag
// ---------------------------------------------------------------------------
//
// Multi-tag PnP: collect 3-D tag corner positions (field frame) and their
// corresponding 2-D image pixel positions for all detected tags that have
// known poses in the field layout.  Then call solvePnP once across all
// correspondences.
//
// Tag corner 3-D positions in field frame are derived from the tag centre pose
// in the layout and the known physical tag size.
// ---------------------------------------------------------------------------
MultiTagResult PoseEstimator::estimateMultiTag(
  const std::vector<SingleTagResult> & targets,
  const FieldLayout & layout) const
{
  MultiTagResult result;

  // Half side length of the tag.
  const double s = tagSizeMetres_ / 2.0;

  // Corners in WPILib/PhotonVision tag frame:
  //   +x  normal out of visible face, +y right, +z up
  //   All four corners lie in the x=0 plane of the tag frame.
  //   top-left     (0, -s, +s)
  //   top-right    (0, +s, +s)
  //   bottom-right (0, +s, -s)
  //   bottom-left  (0, -s, -s)
  const std::array<Eigen::Vector3d, 4> cornersTagFrame = {{
    {0.0, -s, +s},  // top-left
    {0.0, +s, +s},  // top-right
    {0.0, +s, -s},  // bottom-right
    {0.0, -s, -s},  // bottom-left
  }};

  std::vector<cv::Point3f> obj3d;
  std::vector<cv::Point2f> img2d;

  for (const auto & target : targets) {
    const auto maybeTag = getTag(layout, target.tagId);
    if (!maybeTag) {
      continue;  // Tag not in layout; skip.
    }
    const Transform3d & tagPose = maybeTag->pose;

    // Transform each corner from the tag frame to the field frame.
    for (std::size_t ci = 0; ci < 4; ++ci) {
      const Eigen::Vector3d fieldPt =
        tagPose.translation + tagPose.rotation * cornersTagFrame[ci];
      obj3d.emplace_back(
        static_cast<float>(fieldPt.x()),
        static_cast<float>(fieldPt.y()),
        static_cast<float>(fieldPt.z()));
      img2d.emplace_back(
        static_cast<float>(target.corners.pixels[ci][0]),
        static_cast<float>(target.corners.pixels[ci][1]));
    }
    result.usedTagIds.push_back(target.tagId);
  }

  // Deduplicate tag IDs (should already be unique if targets are unique).
  std::sort(result.usedTagIds.begin(), result.usedTagIds.end());
  result.usedTagIds.erase(
    std::unique(result.usedTagIds.begin(), result.usedTagIds.end()),
    result.usedTagIds.end());

  if (result.usedTagIds.size() < 2 || obj3d.size() < 8) {
    // Need at least 2 full tags (8 corner correspondences) for a reliable solve.
    result.isValid = false;
    return result;
  }

  const cv::Mat K  = intrinsics_.cameraMatrixCV();
  const cv::Mat dc = intrinsics_.distCoeffsCV();

  cv::Mat rvec, tvec;
  // SOLVEPNP_SQPNP is a robust, non-iterative solver for many-point problems.
  const bool ok = cv::solvePnP(obj3d, img2d, K, dc, rvec, tvec, false, cv::SOLVEPNP_SQPNP);
  if (!ok) {
    result.isValid = false;
    return result;
  }

  // Compute mean reprojection error.
  {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(obj3d, rvec, tvec, K, dc, projected);
    double total = 0.0;
    for (std::size_t i = 0; i < img2d.size(); ++i) {
      const double dx = projected[i].x - img2d[i].x;
      const double dy = projected[i].y - img2d[i].y;
      total += std::sqrt(dx * dx + dy * dy);
    }
    result.reprojectionError = total / static_cast<double>(img2d.size());
  }

  // The solvePnP result is: world (field) → camera, in optical frame.
  // Convert to WPILib camera frame.
  Eigen::Vector3d rvec_e, tvec_e;
  cv::cv2eigen(rvec.reshape(1, 3), rvec_e);
  cv::cv2eigen(tvec.reshape(1, 3), tvec_e);

  // t_field_camera: expresses camera pose in field frame.
  // solvePnP gives us R,t such that  p_camera = R * p_field + t
  // i.e., it is the camera-FROM-field transform (field → camera).
  // We store it as cameraFieldPose = field → camera.
  const Transform3d t_field_cam = solvePnPToTransform(rvec_e, tvec_e);
  result.cameraFieldPose = t_field_cam;
  result.robotFieldPose  = cameraPoseToRobotPose(t_field_cam, camRobotTransform_);
  result.isValid         = true;

  return result;
}

}  // namespace ros_vision
