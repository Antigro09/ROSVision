// Copyright (c) 2024 FRC Team. All rights reserved.
// SPDX-License-Identifier: MIT

#include "ros_vision/coordinate_transforms.hpp"

#include <cmath>

namespace ros_vision {

// ---------------------------------------------------------------------------
// opticalToPhotonCamera
// ---------------------------------------------------------------------------
// Camera-optical frame (ROS/OpenCV):
//   +x right, +y down, +z forward (into the scene)
//
// PhotonVision camera frame (NWU-style, lens-centric):
//   +x forward (= optical +z)
//   +y left    (= -optical +x)
//   +z up      (= -optical +y)
//
// The fixed rotation R maps optical axes to PhotonVision axes:
//   PV_x =  optical_z
//   PV_y = -optical_x
//   PV_z = -optical_y
// ---------------------------------------------------------------------------
Transform3d opticalToPhotonCamera(const Transform3d & t_optical)
{
  static const Eigen::Matrix3d R = rotOpticalToPhotonCamera();
  static const Eigen::Quaterniond Q(R);

  Transform3d result;
  result.translation = R * t_optical.translation;
  result.rotation    = Q * t_optical.rotation;
  return result;
}

// ---------------------------------------------------------------------------
// opticalToRobotFrame
// ---------------------------------------------------------------------------
Transform3d opticalToRobotFrame(
  const Transform3d & t_optical,
  const Transform3d & t_cam_robot)
{
  // t_optical: camera-optical → target
  // t_cam_robot: camera-body → robot-body  (WPILib NWU for both)
  //
  // First convert optical → PhotonVision camera frame (NWU), then compose
  // with the camera→robot transform.
  Transform3d t_pv = opticalToPhotonCamera(t_optical);
  return t_cam_robot * t_pv;
}

// ---------------------------------------------------------------------------
// cameraPoseToRobotPose
// ---------------------------------------------------------------------------
Transform3d cameraPoseToRobotPose(
  const Transform3d & t_field_camera,
  const Transform3d & t_cam_robot)
{
  // t_field_camera:  field → camera  (i.e., camera pose in field)
  // t_cam_robot:     camera → robot
  //
  // field → robot = (field → camera) ∘ (camera → robot)
  return t_field_camera * t_cam_robot;
}

// ---------------------------------------------------------------------------
// solvePnPToTransform
// ---------------------------------------------------------------------------
// OpenCV solvePnP outputs (rvec, tvec) representing the transform from
// world (tag) to camera in the camera-optical frame.  We convert that to
// a Transform3d in the WPILib camera frame (PhotonVision convention).
// ---------------------------------------------------------------------------
Transform3d solvePnPToTransform(
  const Eigen::Vector3d & rvec,
  const Eigen::Vector3d & tvec)
{
  // Rodrigues rotation vector → rotation matrix
  const double angle = rvec.norm();
  Eigen::Matrix3d R_optical;
  if (angle < 1e-10) {
    R_optical = Eigen::Matrix3d::Identity();
  } else {
    Eigen::AngleAxisd aa(angle, rvec / angle);
    R_optical = aa.toRotationMatrix();
  }

  Transform3d t_optical;
  t_optical.rotation    = Eigen::Quaterniond(R_optical);
  t_optical.translation = tvec;

  return opticalToPhotonCamera(t_optical);
}

}  // namespace ros_vision
