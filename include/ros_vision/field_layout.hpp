// Copyright (c) 2024 FRC Team. All rights reserved.
// SPDX-License-Identifier: MIT
//
// field_layout.hpp
// ==========================================================================
// Loads and validates a WPILib-format field layout JSON file.
//
// Expected JSON schema
// --------------------
//   {
//     "tags": [
//       {
//         "ID": <int>,
//         "pose": {
//           "translation": { "x": <m>, "y": <m>, "z": <m> },
//           "rotation": {
//             "quaternion": { "W": <>, "X": <>, "Y": <>, "Z": <> }
//           }
//         }
//       },
//       ...
//     ],
//     "field": { "length": <m>, "width": <m> }
//   }
//
// All poses are in the WPILib NWU field frame.
// --------------------------------------------------------------------------
#pragma once

#include "ros_vision/coordinate_transforms.hpp"

#include <string>
#include <unordered_map>
#include <optional>
#include <stdexcept>

namespace ros_vision {

/// A single AprilTag entry from the field layout file.
struct FieldTag
{
  int        id{0};
  Transform3d pose;  ///< Tag pose in the WPILib field frame.
};

/// Dimensions of the playing field in metres.
struct FieldDimensions
{
  double length{0.0};  ///< Along the x axis (red←→blue alliance wall direction).
  double width{0.0};   ///< Along the y axis.
};

/// Parsed field layout.
struct FieldLayout
{
  std::unordered_map<int, FieldTag> tags;  ///< Keyed by tag ID.
  FieldDimensions                   field;
};

// ---------------------------------------------------------------------------
// Parsing
// ---------------------------------------------------------------------------

/// Load a field layout from a JSON file at @p path.
/// @throws std::runtime_error with a descriptive message on any parse error
///         or schema violation.
FieldLayout loadFieldLayout(const std::string & path);

/// Load a field layout from a JSON string already in memory.
/// @throws std::runtime_error on parse/validation errors.
FieldLayout parseFieldLayout(const std::string & json_text);

/// Look up a tag by ID; returns nullopt if not present.
inline std::optional<FieldTag> getTag(const FieldLayout & layout, int id)
{
  auto it = layout.tags.find(id);
  if (it == layout.tags.end()) {
    return std::nullopt;
  }
  return it->second;
}

}  // namespace ros_vision
