// Copyright (c) 2024 FRC Team. All rights reserved.
// SPDX-License-Identifier: MIT
//
// vision_node.cpp
// ==========================================================================
// Main ROS 2 node.
//
// Detection backend: NVIDIA Isaac ROS AprilTag (CUDA-accelerated).
//   https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag
//
// Pipeline per frame:
//  1. isaac_ros_apriltag detects tags on the GPU and publishes
//     AprilTagDetectionArray on <detection_topic>.
//  2. onDetections() converts each detection to a SingleTagResult:
//       - re-runs solvePnP (for ambiguity + reprojection error)
//       - converts tag pose from optical frame → WPILib camera frame
//  3. If ≥ 2 tags are visible and the field layout is loaded, a multi-tag
//     PnP solve is performed to get the robot's field pose.
//  4. Results are published to NetworkTables.
// ==========================================================================

#include "ros_vision/vision_node.hpp"
#include "ros_vision/coordinate_transforms.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace ros_vision {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

VisionNode::VisionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vision_node", options)
{
  loadParameters();

  // ---- Camera info subscription ----
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg) {
      onCameraInfo(msg);
    });

  // ---- Isaac ROS AprilTag detection subscription ----
  detection_sub_ = this->create_subscription<
    isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
    detection_topic_,
    rclcpp::SensorDataQoS(),
    [this](const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::ConstSharedPtr & msg) {
      onDetections(msg);
    });

  // ---- Optional: robot pose publisher for RViz ----
  robot_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/vision/robot_pose", 10);

  // ---- NT publisher ----
  initPublisher();

  RCLCPP_INFO(this->get_logger(),
    "VisionNode started. Listening for Isaac ROS AprilTag detections on '%s'",
    detection_topic_.c_str());
}

// ---------------------------------------------------------------------------
// loadParameters
// ---------------------------------------------------------------------------

void VisionNode::loadParameters()
{
  // Field layout path (required for pose estimation in field frame).
  this->declare_parameter<std::string>("field_layout_path", "");
  field_layout_path_ = this->get_parameter("field_layout_path").as_string();

  if (!field_layout_path_.empty()) {
    try {
      field_layout_ = loadFieldLayout(field_layout_path_);
      RCLCPP_INFO(this->get_logger(),
        "Loaded field layout from '%s' (%zu tags, %.2f x %.2f m)",
        field_layout_path_.c_str(),
        field_layout_.tags.size(),
        field_layout_.field.length,
        field_layout_.field.width);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to load field layout: %s — multi-tag pose estimation disabled",
        e.what());
    }
  } else {
    RCLCPP_WARN(this->get_logger(),
      "Parameter 'field_layout_path' not set — multi-tag pose estimation disabled");
  }

  // Tag physical size in metres (default 6.5 in = 0.1651 m for 2024 FRC).
  this->declare_parameter<double>("tag_size_metres", 0.1651);
  tag_size_metres_ = this->get_parameter("tag_size_metres").as_double();

  // Topic published by isaac_ros_apriltag.
  this->declare_parameter<std::string>("detection_topic", "/apriltag/detection_array");
  detection_topic_ = this->get_parameter("detection_topic").as_string();

  // Camera→robot transform (translation in metres, rotation as RPY in degrees).
  this->declare_parameter<double>("cam_robot.x",    0.0);
  this->declare_parameter<double>("cam_robot.y",    0.0);
  this->declare_parameter<double>("cam_robot.z",    0.0);
  this->declare_parameter<double>("cam_robot.roll",  0.0);
  this->declare_parameter<double>("cam_robot.pitch", 0.0);
  this->declare_parameter<double>("cam_robot.yaw",   0.0);

  const double cx  = this->get_parameter("cam_robot.x").as_double();
  const double cy  = this->get_parameter("cam_robot.y").as_double();
  const double cz  = this->get_parameter("cam_robot.z").as_double();
  const double cr  = this->get_parameter("cam_robot.roll").as_double()  * M_PI / 180.0;
  const double cp  = this->get_parameter("cam_robot.pitch").as_double() * M_PI / 180.0;
  const double cyw = this->get_parameter("cam_robot.yaw").as_double()   * M_PI / 180.0;

  cam_robot_transform_.translation = Eigen::Vector3d(cx, cy, cz);
  cam_robot_transform_.rotation    =
    Eigen::AngleAxisd(cyw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(cp,  Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(cr,  Eigen::Vector3d::UnitX());

  // NetworkTables parameters.
  this->declare_parameter<std::string>("nt.server_address", "");
  this->declare_parameter<int>("nt.team_number", 0);
  this->declare_parameter<std::string>("nt.client_name", "ROSVision");
  this->declare_parameter<int>("nt.port", 5810);
  this->declare_parameter<std::string>("nt.root_table", "/Vision");

  nt_config_.serverAddress = this->get_parameter("nt.server_address").as_string();
  nt_config_.teamNumber    = this->get_parameter("nt.team_number").as_int();
  nt_config_.clientName    = this->get_parameter("nt.client_name").as_string();
  nt_config_.port          = this->get_parameter("nt.port").as_int();
  nt_config_.rootTable     = this->get_parameter("nt.root_table").as_string();
}

// ---------------------------------------------------------------------------
// initPublisher
// ---------------------------------------------------------------------------

void VisionNode::initPublisher()
{
  nt_publisher_ = std::make_unique<NtPublisher>(nt_config_);
  RCLCPP_INFO(this->get_logger(),
    "NT publisher initialised (table='%s', connected=%s)",
    nt_config_.rootTable.c_str(),
    nt_publisher_->isConnected() ? "yes" : "no (stub/connecting)");
}

// ---------------------------------------------------------------------------
// onCameraInfo
// ---------------------------------------------------------------------------

void VisionNode::onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  if (camera_info_received_) {
    return;  // Intrinsics already set; camera info rarely changes.
  }

  // ROS CameraInfo K matrix is row-major 3×3:
  //   [ fx  0  cx ]
  //   [  0 fy  cy ]
  //   [  0  0   1 ]
  CameraIntrinsics intr;
  intr.fx = msg->k[0];
  intr.fy = msg->k[4];
  intr.cx = msg->k[2];
  intr.cy = msg->k[5];

  // Distortion coefficients (may be empty if undistorted).
  const auto & d = msg->d;
  for (std::size_t i = 0; i < std::min(d.size(), std::size_t{5}); ++i) {
    intr.distCoeffs[i] = d[i];
  }

  estimator_ = std::make_unique<PoseEstimator>(intr, tag_size_metres_, cam_robot_transform_);
  camera_info_received_ = true;

  RCLCPP_INFO(this->get_logger(),
    "Camera intrinsics received: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
    intr.fx, intr.fy, intr.cx, intr.cy);
}

// ---------------------------------------------------------------------------
// convertDetection
// ---------------------------------------------------------------------------
// Isaac ROS AprilTag detection message layout:
//   detection.id        — tag numeric ID
//   detection.family    — tag family string (e.g., "tag36h11")
//   detection.pose.pose.pose  — PoseWithCovarianceStamped.pose.pose
//       position: tag centre in camera-optical frame
//       orientation: quaternion (tag frame → camera-optical frame)
//   detection.corners   — float64[8]: [x0,y0, x1,y1, x2,y2, x3,y3]
//       corners in pixel coordinates, order: top-left, top-right,
//       bottom-right, bottom-left (counter-clockwise when viewed from front)
//   detection.center    — Point: pixel centre of the detected tag
// ---------------------------------------------------------------------------

SingleTagResult VisionNode::convertDetection(
  const isaac_ros_apriltag_interfaces::msg::AprilTagDetection & det) const
{
  SingleTagResult result;
  result.tagId = det.id;

  // ---- Corner pixels ----
  // Isaac ROS corners array is [x0,y0, x1,y1, x2,y2, x3,y3].
  for (std::size_t i = 0; i < 4; ++i) {
    result.corners.pixels[i][0] = det.corners[2 * i];
    result.corners.pixels[i][1] = det.corners[2 * i + 1];
  }

  // ---- Centre pixel ----
  result.center = {det.center.x, det.center.y};

  if (!estimator_) {
    // Camera intrinsics not yet received; return result with no pose.
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Camera info not yet received; skipping solvePnP for tag %d", det.id);
    return result;
  }

  // ---- Re-run solvePnP for ambiguity and reprojection error ----
  // Isaac ROS already provides a pose estimate via its internal GPU solver,
  // but running IPPE_SQUARE via our PoseEstimator gives us:
  //   - Ambiguity metric (ratio of the two IPPE solution errors).
  //   - Both candidate pose solutions for the consumer to select.
  //   - Consistent WPILib camera-frame output.
  result = estimator_->estimateSingleTag(det.id, result.corners);
  // Override center with Isaac ROS GPU-detected centroid rather than the
  // corner-average computed inside estimateSingleTag; the GPU-detected
  // centroid is typically sub-pixel accurate.
  result.center = {det.center.x, det.center.y};

  return result;
}

// ---------------------------------------------------------------------------
// onDetections  (Isaac ROS AprilTag callback)
// ---------------------------------------------------------------------------

void VisionNode::onDetections(
  const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::ConstSharedPtr & msg)
{
  const auto recv_time = this->now();

  // ---- Build per-target results ----
  PipelineResult pipeline;
  pipeline.captureTimestampSec = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;
  pipeline.latencyMs =
    (recv_time.nanoseconds() -
     rclcpp::Time(msg->header.stamp).nanoseconds()) * 1e-6;

  for (const auto & det : msg->detections) {
    pipeline.targets.push_back(convertDetection(det));
  }

  // ---- Multi-tag solve ----
  if (estimator_ && pipeline.targets.size() >= 2 && !field_layout_.tags.empty()) {
    pipeline.multiTagResult = estimator_->estimateMultiTag(
      pipeline.targets, field_layout_);

    if (pipeline.multiTagResult->isValid) {
      // Publish robot pose to ROS for RViz.
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp    = msg->header.stamp;
      pose_msg.header.frame_id = "map";

      const auto & rt = pipeline.multiTagResult->robotFieldPose.translation;
      const auto & rq = pipeline.multiTagResult->robotFieldPose.rotation;
      pose_msg.pose.position.x    = rt.x();
      pose_msg.pose.position.y    = rt.y();
      pose_msg.pose.position.z    = rt.z();
      pose_msg.pose.orientation.w = rq.w();
      pose_msg.pose.orientation.x = rq.x();
      pose_msg.pose.orientation.y = rq.y();
      pose_msg.pose.orientation.z = rq.z();

      robot_pose_pub_->publish(pose_msg);
    }
  }

  // ---- Publish to NetworkTables ----
  nt_publisher_->publish(pipeline);

  if (!pipeline.targets.empty()) {
    RCLCPP_DEBUG(this->get_logger(),
      "Processed %zu tag(s), latency=%.1f ms, multiTag=%s",
      pipeline.targets.size(),
      pipeline.latencyMs,
      (pipeline.multiTagResult && pipeline.multiTagResult->isValid) ? "ok" : "no");
  }
}

}  // namespace ros_vision

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros_vision::VisionNode>());
  rclcpp::shutdown();
  return 0;
}
