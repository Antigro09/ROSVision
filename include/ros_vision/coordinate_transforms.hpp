// Copyright (c) 2024 FRC Team. All rights reserved.
// SPDX-License-Identifier: MIT
//
// coordinate_transforms.hpp
// ==========================================================================
// Utilities for converting poses between the coordinate frames used in an
// FRC vision pipeline.
//
// Frame definitions
// -----------------
//
//  Camera-optical frame (ROS / OpenCV convention)
//    +x  right in image
//    +y  down  in image
//    +z  forward out of lens (into the scene)
//
//  PhotonVision camera frame  (NWU-style, lens-centric)
//    +x  forward out of lens  (same direction as optical +z)
//    +y  left   in image      (opposite of optical +x)
//    +z  up     in image      (opposite of optical +y)
//
//  WPILib robot / field frame  (NWU — North-West-Up)
//    +x  forward (robot-relative or down-field)
//    +y  left
//    +z  up
//
//  AprilTag field frame  (as stored in the WPILib field layout JSON)
//    +x  normal out of the *visible* face of the tag
//    +y  right  when looking at the tag
//    +z  up
//
//  AprilTag detection frame  (OpenCV solvePnP / Isaac ROS output)
//    Origin at tag centre
//    +x  right  when looking at the tag
//    +y  down   when looking at the tag
//    +z  normal *toward* the camera  (out of visible face toward viewer)
//
// --------------------------------------------------------------------------
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ros_vision {

// ---------------------------------------------------------------------------
// Rotation-only constants
// ---------------------------------------------------------------------------

/// Rotation matrix that maps camera-optical axes to PhotonVision camera axes.
/// PV_x = +optical_z,  PV_y = -optical_x,  PV_z = -optical_y
inline Eigen::Matrix3d rotOpticalToPhotonCamera()
{
  //        opt_x  opt_y  opt_z
  Eigen::Matrix3d R;
  R << 0.0, 0.0, 1.0,   // PV_x = optical_z
      -1.0, 0.0, 0.0,   // PV_y = -optical_x
       0.0, -1.0, 0.0;  // PV_z = -optical_y
  return R;
}

/// Rotation matrix that maps AprilTag detection frame to WPILib tag frame.
/// WPILib tag: +x normal out visible face, +y right, +z up
/// Detection:  +x right, +y down, +z toward camera (= out visible face)
///   WPILib_x = +det_z,  WPILib_y = +det_x,  WPILib_z = -det_y
inline Eigen::Matrix3d rotDetectionTagToWpilibTag()
{
  //         det_x  det_y  det_z
  Eigen::Matrix3d R;
  R << 0.0, 0.0, 1.0,   // WPILib_x = det_z
       1.0, 0.0, 0.0,   // WPILib_y = det_x
       0.0, -1.0, 0.0;  // WPILib_z = -det_y
  return R;
}

// ---------------------------------------------------------------------------
// Transform types
// ---------------------------------------------------------------------------

/// A 3-D rigid-body transform: rotation + translation.
struct Transform3d
{
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d    translation{Eigen::Vector3d::Zero()};

  Transform3d() = default;
  Transform3d(const Eigen::Quaterniond & rot, const Eigen::Vector3d & trans)
  : rotation(rot), translation(trans) {}

  /// Compose: returns the transform that first applies *this*, then *other*.
  Transform3d operator*(const Transform3d & other) const
  {
    Transform3d result;
    result.rotation    = rotation * other.rotation;
    result.translation = translation + rotation * other.translation;
    return result;
  }

  /// Invert: T^{-1} such that T * T^{-1} == Identity.
  Transform3d inverse() const
  {
    Transform3d inv;
    inv.rotation    = rotation.inverse();
    inv.translation = -(inv.rotation * translation);
    return inv;
  }

  /// Convert rotation to a 3×3 matrix.
  Eigen::Matrix3d rotationMatrix() const { return rotation.toRotationMatrix(); }
};

// ---------------------------------------------------------------------------
// Conversion helpers
// ---------------------------------------------------------------------------

/// Convert a pose expressed in the camera-optical frame to the PhotonVision
/// camera frame.
/// @param t_optical  Transform from optical origin to the object (e.g., a tag)
///                   expressed in the optical frame.
/// @return  The same transform expressed in the PhotonVision camera frame.
Transform3d opticalToPhotonCamera(const Transform3d & t_optical);

/// Convert a pose expressed in the camera-optical frame to the WPILib NWU
/// frame, given the camera-to-robot transform.
/// @param t_optical     Camera-optical→target transform.
/// @param t_cam_robot   Camera→robot transform (robot body frame).
/// @return  Target pose in robot-body frame (WPILib NWU).
Transform3d opticalToRobotFrame(
  const Transform3d & t_optical,
  const Transform3d & t_cam_robot);

/// Convert a camera pose in the field frame (from PnP solve) to the robot
/// pose in the field frame using the known camera-to-robot transform.
/// @param t_field_camera  Field→camera transform (camera pose in field).
/// @param t_cam_robot     Camera→robot transform.
/// @return  Robot pose in field frame (field→robot).
Transform3d cameraPoseToRobotPose(
  const Transform3d & t_field_camera,
  const Transform3d & t_cam_robot);

/// Convert a rotation+translation coming out of OpenCV's solvePnP (Rodrigues
/// rvec/tvec) into a Transform3d expressed in the WPILib camera frame.
/// @param rvec  3×1 Rodrigues rotation vector (double).
/// @param tvec  3×1 translation vector in metres (double).
Transform3d solvePnPToTransform(
  const Eigen::Vector3d & rvec,
  const Eigen::Vector3d & tvec);

}  // namespace ros_vision
