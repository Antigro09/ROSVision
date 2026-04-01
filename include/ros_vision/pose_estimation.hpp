// Copyright (c) 2024 FRC Team. All rights reserved.
// SPDX-License-Identifier: MIT
//
// pose_estimation.hpp
// ==========================================================================
// Data structures and APIs for AprilTag-based pose estimation.
//
// Single-tag pipeline
// -------------------
//  For each detected tag:
//   1. Run solvePnP to obtain up to two candidate pose solutions.
//   2. Compute reprojection errors for both solutions.
//   3. Store the ambiguity ratio  =  min_error / max_error  ∈ [0, 1].
//      A low ratio (near 0) means one solution is clearly better.
//      A high ratio (near 1) means the two solutions are nearly equally good
//      and the result is ambiguous.
//   4. Return both solutions so the consumer can pick one (or discard
//      ambiguous results).
//
// Multi-tag pipeline
// ------------------
//  When ≥ 2 tags are visible:
//   1. Collect 3-D→2-D correspondences across all detected tags using the
//      known field layout.
//   2. Run a single solvePnP over all correspondences to get the camera pose
//      directly in the WPILib field frame.
//   3. Return the camera/robot field pose together with used tag IDs and a
//      quality/reprojection-error score.
// --------------------------------------------------------------------------
#pragma once

#include "ros_vision/coordinate_transforms.hpp"
#include "ros_vision/field_layout.hpp"

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <vector>
#include <optional>
#include <cstdint>

namespace ros_vision {

// ---------------------------------------------------------------------------
// Per-target result
// ---------------------------------------------------------------------------

/// Pixel coordinates of one detected tag's corners (image frame).
/// Order: [top-left, top-right, bottom-right, bottom-left]
struct TagCorners
{
  std::array<std::array<double, 2>, 4> pixels{};  ///< {{x0,y0}, {x1,y1}, ...}
};

/// One candidate pose solution from solvePnP.
struct PoseSolution
{
  Transform3d pose;              ///< Camera→tag transform in WPILib camera frame.
  double      reprojectionError{-1.0};  ///< Mean pixel reprojection error; -1 = invalid.
};

/// Full result for a single detected tag.
struct SingleTagResult
{
  int         tagId{-1};
  TagCorners  corners;
  std::array<double, 2> center{{0.0, 0.0}};  ///< Pixel centre {x, y}.

  PoseSolution            best;        ///< Lower-reprojection-error solution.
  std::optional<PoseSolution> alt;     ///< Higher-error (alternative) solution, if available.

  /// Ambiguity ∈ [0, 1] = best.reprojectionError / alt.reprojectionError.
  /// 0   → no ambiguity (only one solution / alternative has much higher error).
  /// 1   → fully ambiguous (both solutions equally reprojected).
  /// -1  → not computed (single solution only).
  double ambiguity{-1.0};

  bool isAmbiguous(double threshold = 0.2) const
  {
    return ambiguity >= 0.0 && ambiguity > threshold;
  }
};

// ---------------------------------------------------------------------------
// Multi-tag result
// ---------------------------------------------------------------------------

/// Result of a multi-tag PnP solve.
struct MultiTagResult
{
  Transform3d cameraFieldPose;   ///< Camera pose in field frame (field→camera).
  Transform3d robotFieldPose;    ///< Robot pose in field frame (field→robot).
  std::vector<int> usedTagIds;   ///< IDs of tags included in the solve.
  double reprojectionError{-1.0}; ///< Mean reprojection error across all used tags; -1 = invalid.
  bool   isValid{false};          ///< True if the solve succeeded.
};

// ---------------------------------------------------------------------------
// Pipeline result (one camera frame worth of detections)
// ---------------------------------------------------------------------------

struct PipelineResult
{
  double                    captureTimestampSec{0.0};
  double                    latencyMs{0.0};
  std::vector<SingleTagResult> targets;  ///< One entry per detected tag.
  std::optional<MultiTagResult> multiTagResult;  ///< Present when ≥ 2 tags visible.

  bool hasTargets() const { return !targets.empty(); }
};

// ---------------------------------------------------------------------------
// Camera intrinsics
// ---------------------------------------------------------------------------

struct CameraIntrinsics
{
  double fx{0.0}, fy{0.0};   ///< Focal lengths in pixels.
  double cx{0.0}, cy{0.0};   ///< Principal point in pixels.
  std::array<double, 5> distCoeffs{};  ///< k1,k2,p1,p2,k3 (OpenCV radtan model).

  cv::Mat cameraMatrixCV() const;
  cv::Mat distCoeffsCV()   const;
};

// ---------------------------------------------------------------------------
// Pose estimator
// ---------------------------------------------------------------------------

class PoseEstimator
{
public:
  /// @param intrinsics   Camera intrinsic parameters.
  /// @param tagSizeMetres  Physical side length of the AprilTag in metres.
  /// @param camRobotTransform  Camera→robot rigid transform.
  PoseEstimator(
    const CameraIntrinsics & intrinsics,
    double tagSizeMetres,
    const Transform3d & camRobotTransform = Transform3d{});

  /// Estimate the pose of a single tag from its corner pixels.
  /// Returns a SingleTagResult; result.best.reprojectionError == -1 if solve
  /// failed.
  SingleTagResult estimateSingleTag(
    int tagId,
    const TagCorners & corners) const;

  /// Run multi-tag PnP using all provided single-tag results and the known
  /// field layout.
  /// @param targets      Per-tag results (must include corner pixels).
  /// @param layout       Field layout for 3-D tag positions.
  /// @return MultiTagResult (isValid==false if < 2 matching tags found).
  MultiTagResult estimateMultiTag(
    const std::vector<SingleTagResult> & targets,
    const FieldLayout & layout) const;

  // Accessors
  const CameraIntrinsics & intrinsics()       const { return intrinsics_; }
  double                   tagSizeMetres()    const { return tagSizeMetres_; }
  const Transform3d &      camRobotTransform() const { return camRobotTransform_; }

private:
  CameraIntrinsics intrinsics_;
  double           tagSizeMetres_{0.1651};  // 6.5 inch default
  Transform3d      camRobotTransform_;

  /// 3-D tag corners in the detection tag frame (z toward camera).
  std::vector<cv::Point3f> tagObjectPoints() const;
};

}  // namespace ros_vision
