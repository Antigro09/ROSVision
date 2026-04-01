// Copyright (c) 2024 FRC Team. All rights reserved.
// SPDX-License-Identifier: MIT
//
// nt_publisher.hpp
// ==========================================================================
// NetworkTables 4 (NT4) publisher module.
//
// Publishes vision pipeline results under the "/Vision/" table hierarchy
// using WPILib-compatible types and naming conventions.
//
// Published entries
// -----------------
//   /Vision/timestamp          double   Capture timestamp [seconds, FPGA epoch]
//   /Vision/latencyMs          double   Pipeline latency [ms]
//   /Vision/hasTarget          boolean  True if any tag was detected
//   /Vision/numTargets         int      Number of detected tags
//   /Vision/targetYaw          double   Yaw to best target [degrees, +left]
//   /Vision/targetPitch        double   Pitch to best target [degrees, +up]
//   /Vision/targetSkew         double   Skew/rotation of best target [degrees]
//   /Vision/targetArea         double   Bounding-box area as % of image (stub)
//   /Vision/pipelineResultJson string   Full result serialised as JSON
//
//   Multi-tag (when a multi-tag solve succeeded):
//   /Vision/multiTag/robotX    double   Robot X in field frame [m]
//   /Vision/multiTag/robotY    double   Robot Y in field frame [m]
//   /Vision/multiTag/robotZ    double   Robot Z in field frame [m]
//   /Vision/multiTag/robotYaw  double   Robot yaw in field frame [deg]
//   /Vision/multiTag/reprojErr double   Mean reprojection error [px]
//   /Vision/multiTag/numTags   int      Number of tags used in the solve
// --------------------------------------------------------------------------
#pragma once

#include "ros_vision/pose_estimation.hpp"

#include <string>
#include <memory>

namespace ros_vision {

/// Configuration for the NT4 publisher.
struct NtPublisherConfig
{
  /// NT server hostname or IP address.  If empty, acts as a server itself.
  std::string serverAddress;

  /// WPILib team number — used to derive the default server address when
  /// serverAddress is empty (e.g., team 1234 → "roborio-1234-frc.local").
  int teamNumber{0};

  /// NT client identity string (shown in the DS).
  std::string clientName{"ROSVision"};

  /// NT4 server port (default 5810).
  int port{5810};

  /// Root table path (default "/Vision").
  std::string rootTable{"/Vision"};
};

/// NT4 publisher.  When ntcore is not available at compile-time the
/// implementation falls back to printing values to stdout so the rest of the
/// pipeline can still run.
class NtPublisher
{
public:
  explicit NtPublisher(const NtPublisherConfig & config);
  ~NtPublisher();

  // Non-copyable, movable.
  NtPublisher(const NtPublisher &)            = delete;
  NtPublisher & operator=(const NtPublisher &) = delete;
  NtPublisher(NtPublisher &&)                  = default;
  NtPublisher & operator=(NtPublisher &&)      = default;

  /// Publish a complete pipeline result.
  void publish(const PipelineResult & result);

  /// Return true if the underlying NT connection is available.
  bool isConnected() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ros_vision
