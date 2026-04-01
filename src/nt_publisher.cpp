// Copyright (c) 2024 FRC Team. All rights reserved.
// SPDX-License-Identifier: MIT

#include "ros_vision/nt_publisher.hpp"

#include <nlohmann/json.hpp>
#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <sstream>

#ifdef HAVE_NTCORE
// WPILib ntcore headers
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/StringTopic.h>
#endif

namespace ros_vision {

// ---------------------------------------------------------------------------
// JSON serialisation helpers
// ---------------------------------------------------------------------------

static nlohmann::json transform3dToJson(const Transform3d & t)
{
  const auto & tr = t.translation;
  const auto & q  = t.rotation;
  // Quaternion keys use uppercase W,X,Y,Z to match the WPILib field layout
  // JSON schema (see config/field_layout.json) and the Rotation3d convention
  // used by WPILib's AprilTagFieldLayout serialiser.
  return {
    {"translation", {{"x", tr.x()}, {"y", tr.y()}, {"z", tr.z()}}},
    {"rotation",    {{"W", q.w()}, {"X", q.x()}, {"Y", q.y()}, {"Z", q.z()}}}
  };
}

static nlohmann::json pipelineResultToJson(const PipelineResult & r)
{
  nlohmann::json j;
  j["timestamp"]  = r.captureTimestampSec;
  j["latencyMs"]  = r.latencyMs;
  j["hasTargets"] = r.hasTargets();

  nlohmann::json targets_arr = nlohmann::json::array();
  for (const auto & t : r.targets) {
    nlohmann::json tj;
    tj["tagId"]     = t.tagId;
    tj["ambiguity"] = t.ambiguity;
    tj["center"]    = {{"x", t.center[0]}, {"y", t.center[1]}};
    tj["bestPose"]  = transform3dToJson(t.best.pose);
    tj["bestReprojError"] = t.best.reprojectionError;
    if (t.alt) {
      tj["altPose"]         = transform3dToJson(t.alt->pose);
      tj["altReprojError"]  = t.alt->reprojectionError;
    }
    targets_arr.push_back(std::move(tj));
  }
  j["targets"] = std::move(targets_arr);

  if (r.multiTagResult) {
    const auto & mt = *r.multiTagResult;
    j["multiTag"] = {
      {"isValid",         mt.isValid},
      {"reprojError",     mt.reprojectionError},
      {"usedTagIds",      mt.usedTagIds},
      {"cameraFieldPose", transform3dToJson(mt.cameraFieldPose)},
      {"robotFieldPose",  transform3dToJson(mt.robotFieldPose)},
    };
  }

  return j;
}

/// Extract yaw (rotation around world-Z) from a quaternion in degrees.
static double quatToYawDeg(const Eigen::Quaterniond & q)
{
  // Standard ZYX Euler extraction for the yaw angle.
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp) * (180.0 / M_PI);
}

/// Target yaw (horizontal angle to tag) from the camera-to-tag translation
/// vector in the WPILib camera frame (+x forward, +y left).
static double targetYawDeg(const Eigen::Vector3d & t_cam_to_tag)
{
  // Positive yaw → tag is to the left.
  return std::atan2(t_cam_to_tag.y(), t_cam_to_tag.x()) * (180.0 / M_PI);
}

/// Target pitch (vertical angle to tag).
static double targetPitchDeg(const Eigen::Vector3d & t_cam_to_tag)
{
  const double horiz = std::sqrt(
    t_cam_to_tag.x() * t_cam_to_tag.x() +
    t_cam_to_tag.y() * t_cam_to_tag.y());
  return std::atan2(t_cam_to_tag.z(), horiz) * (180.0 / M_PI);
}

// ---------------------------------------------------------------------------
// Impl — real ntcore or stub
// ---------------------------------------------------------------------------

struct NtPublisher::Impl
{
  NtPublisherConfig config;

#ifdef HAVE_NTCORE
  nt::NetworkTableInstance inst;
  std::shared_ptr<nt::NetworkTable> table;
  std::shared_ptr<nt::NetworkTable> mt_table;

  nt::DoublePublisher  pub_timestamp;
  nt::DoublePublisher  pub_latency;
  nt::BooleanPublisher pub_has_target;
  nt::IntegerPublisher pub_num_targets;
  nt::DoublePublisher  pub_yaw;
  nt::DoublePublisher  pub_pitch;
  nt::DoublePublisher  pub_skew;
  nt::DoublePublisher  pub_area;
  nt::StringPublisher  pub_json;

  // Multi-tag sub-table publishers
  nt::DoublePublisher  pub_mt_x;
  nt::DoublePublisher  pub_mt_y;
  nt::DoublePublisher  pub_mt_z;
  nt::DoublePublisher  pub_mt_yaw;
  nt::DoublePublisher  pub_mt_err;
  nt::IntegerPublisher pub_mt_num_tags;
#endif

  explicit Impl(const NtPublisherConfig & cfg) : config(cfg)
  {
#ifdef HAVE_NTCORE
    inst = nt::NetworkTableInstance::GetDefault();

    if (!config.serverAddress.empty()) {
      // Connect to an explicit server address.
      inst.StartClient4(config.clientName);
      inst.SetServer(config.serverAddress.c_str(), config.port);
    } else if (config.teamNumber > 0) {
      // Connect to RoboRIO via team number.
      inst.StartClient4(config.clientName);
      inst.SetServerTeam(config.teamNumber, config.port);
    } else {
      // Run as NT server (useful for desktop testing).
      inst.StartServer();
    }

    table    = inst.GetTable(config.rootTable);
    mt_table = table->GetSubTable("multiTag");

    // Initialise all publishers once.
    pub_timestamp   = table->GetDoubleTopic("timestamp").Publish();
    pub_latency     = table->GetDoubleTopic("latencyMs").Publish();
    pub_has_target  = table->GetBooleanTopic("hasTarget").Publish();
    pub_num_targets = table->GetIntegerTopic("numTargets").Publish();
    pub_yaw         = table->GetDoubleTopic("targetYaw").Publish();
    pub_pitch       = table->GetDoubleTopic("targetPitch").Publish();
    pub_skew        = table->GetDoubleTopic("targetSkew").Publish();
    pub_area        = table->GetDoubleTopic("targetArea").Publish();
    pub_json        = table->GetStringTopic("pipelineResultJson").Publish();

    pub_mt_x        = mt_table->GetDoubleTopic("robotX").Publish();
    pub_mt_y        = mt_table->GetDoubleTopic("robotY").Publish();
    pub_mt_z        = mt_table->GetDoubleTopic("robotZ").Publish();
    pub_mt_yaw      = mt_table->GetDoubleTopic("robotYaw").Publish();
    pub_mt_err      = mt_table->GetDoubleTopic("reprojErr").Publish();
    pub_mt_num_tags = mt_table->GetIntegerTopic("numTags").Publish();
#endif
  }

  bool connected() const
  {
#ifdef HAVE_NTCORE
    return inst.IsConnected();
#else
    return false;
#endif
  }

  void publish(const PipelineResult & result)
  {
    const std::string json_str = pipelineResultToJson(result).dump();

    // ---- Best target metrics ----
    double best_yaw   = 0.0;
    double best_pitch = 0.0;
    double best_skew  = 0.0;

    if (result.hasTargets()) {
      const auto & best = result.targets.front();
      const Eigen::Vector3d & t = best.best.pose.translation;
      best_yaw   = targetYawDeg(t);
      best_pitch = targetPitchDeg(t);
      // Skew: angle of the tag's x-axis projected onto the image plane (stub).
      best_skew = quatToYawDeg(best.best.pose.rotation);
    }

#ifdef HAVE_NTCORE
    pub_timestamp.Set(result.captureTimestampSec);
    pub_latency.Set(result.latencyMs);
    pub_has_target.Set(result.hasTargets());
    pub_num_targets.Set(static_cast<int64_t>(result.targets.size()));
    pub_yaw.Set(best_yaw);
    pub_pitch.Set(best_pitch);
    pub_skew.Set(best_skew);
    pub_area.Set(0.0);  // TODO: compute bounding-box area as % of image
    pub_json.Set(json_str);

    if (result.multiTagResult && result.multiTagResult->isValid) {
      const auto & mt = *result.multiTagResult;
      const auto & tr = mt.robotFieldPose.translation;
      pub_mt_x.Set(tr.x());
      pub_mt_y.Set(tr.y());
      pub_mt_z.Set(tr.z());
      pub_mt_yaw.Set(quatToYawDeg(mt.robotFieldPose.rotation));
      pub_mt_err.Set(mt.reprojectionError);
      pub_mt_num_tags.Set(static_cast<int64_t>(mt.usedTagIds.size()));
    }
#else
    // Stub: print to stdout so the pipeline can be verified without ntcore.
    std::cout << "[NT-STUB] "
              << "ts=" << result.captureTimestampSec
              << " latency=" << result.latencyMs << "ms"
              << " hasTarget=" << result.hasTargets()
              << " numTargets=" << result.targets.size()
              << " yaw=" << best_yaw
              << " pitch=" << best_pitch;
    if (result.multiTagResult && result.multiTagResult->isValid) {
      const auto & tr = result.multiTagResult->robotFieldPose.translation;
      std::cout << " robot=(" << tr.x() << "," << tr.y() << "," << tr.z() << ")";
    }
    std::cout << "\n";
#endif
  }
};

// ---------------------------------------------------------------------------
// NtPublisher public API
// ---------------------------------------------------------------------------

NtPublisher::NtPublisher(const NtPublisherConfig & config)
: impl_(std::make_unique<Impl>(config))
{}

NtPublisher::~NtPublisher() = default;

void NtPublisher::publish(const PipelineResult & result)
{
  impl_->publish(result);
}

bool NtPublisher::isConnected() const
{
  return impl_->connected();
}

}  // namespace ros_vision
