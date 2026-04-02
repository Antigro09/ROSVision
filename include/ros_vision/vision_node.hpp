// Copyright (c) 2024 FRC Team. All rights reserved.
// SPDX-License-Identifier: MIT
//
// vision_node.hpp
// ==========================================================================
// ROS 2 node that ties together all vision pipeline components.
//
// Detection backend
// -----------------
// Uses NVIDIA Isaac ROS AprilTag (CUDA-accelerated) exclusively.
// Upstream: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag
//
// The isaac_ros_apriltag node must be running and publishing detections to
// the topic configured by the 'detection_topic' parameter (default:
// "/apriltag/detection_array").
//
// Subscriptions
// -------------
//  <detection_topic>  (isaac_ros_apriltag_interfaces/AprilTagDetectionArray)
//      Raw CUDA-detected AprilTag results from the Isaac ROS node.
//      The pose field in each detection is the tag pose expressed in the
//      camera optical frame (ROS convention: +x right, +y down, +z forward).
//
//  /camera_info       (sensor_msgs/CameraInfo)
//      Used to initialise the PoseEstimator with live camera intrinsics.
//
// Publications / side-effects
// ---------------------------
//  Publishes PipelineResult to NetworkTables via NtPublisher.
//  Optionally publishes geometry_msgs/PoseStamped for robot pose (RViz).
// --------------------------------------------------------------------------
#pragma once

#include "ros_vision/coordinate_transforms.hpp"
#include "ros_vision/field_layout.hpp"
#include "ros_vision/pose_estimation.hpp"
#include "ros_vision/nt_publisher.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Isaac ROS AprilTag interfaces — always required.
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection.hpp>

#include <memory>
#include <string>

namespace ros_vision {

class VisionNode : public rclcpp::Node
{
public:
  explicit VisionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  // -----------------------------------------------------------------------
  // ROS callbacks
  // -----------------------------------------------------------------------

  /// Called for every AprilTagDetectionArray published by the Isaac ROS node.
  void onDetections(
    const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::ConstSharedPtr & msg);

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  // -----------------------------------------------------------------------
  // Helpers
  // -----------------------------------------------------------------------
  void loadParameters();
  void initPublisher();

  /// Convert a single Isaac ROS detection into our SingleTagResult type.
  /// The Isaac ROS detection pose is in the camera optical frame; this helper
  /// applies the optical→WPILib camera frame transform and packages corner
  /// pixels in the expected order.
  SingleTagResult convertDetection(
    const isaac_ros_apriltag_interfaces::msg::AprilTagDetection & det) const;

  // -----------------------------------------------------------------------
  // Members
  // -----------------------------------------------------------------------

  // Subscriptions
  rclcpp::Subscription<
    isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Publisher for robot pose (optional, useful for RViz visualisation)
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_pub_;

  // Vision pipeline components
  std::unique_ptr<PoseEstimator> estimator_;
  std::unique_ptr<NtPublisher>   nt_publisher_;
  FieldLayout                    field_layout_;

  // State
  bool camera_info_received_{false};

  // Parameters
  std::string       field_layout_path_;
  std::string       detection_topic_{"/apriltag/detection_array"};
  bool              use_isaac_single_tag_pose_{true};
  double            tag_size_metres_{0.1651};
  Transform3d       cam_robot_transform_;
  NtPublisherConfig nt_config_;
};

}  // namespace ros_vision
