// Copyright (c) 2024 FRC Team. All rights reserved.
// SPDX-License-Identifier: MIT

#include "ros_vision/field_layout.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>

namespace ros_vision {

// ---------------------------------------------------------------------------
// Internal validation helpers
// ---------------------------------------------------------------------------

static void requireKey(const nlohmann::json & j, const std::string & key, const std::string & ctx)
{
  if (!j.contains(key)) {
    throw std::runtime_error(
      "Field layout parse error: missing key '" + key + "' in " + ctx);
  }
}

static void requireArray(const nlohmann::json & j, const std::string & key, const std::string & ctx)
{
  requireKey(j, key, ctx);
  if (!j[key].is_array()) {
    throw std::runtime_error(
      "Field layout parse error: '" + key + "' in " + ctx + " must be an array");
  }
}

static double requireDouble(const nlohmann::json & j, const std::string & key, const std::string & ctx)
{
  requireKey(j, key, ctx);
  if (!j[key].is_number()) {
    throw std::runtime_error(
      "Field layout parse error: '" + key + "' in " + ctx + " must be a number");
  }
  return j[key].get<double>();
}

// ---------------------------------------------------------------------------
// parseFieldLayout
// ---------------------------------------------------------------------------
FieldLayout parseFieldLayout(const std::string & json_text)
{
  nlohmann::json root;
  try {
    root = nlohmann::json::parse(json_text);
  } catch (const nlohmann::json::parse_error & e) {
    throw std::runtime_error(
      std::string("Field layout JSON parse error: ") + e.what());
  }

  // ---- Top-level keys ----
  requireArray(root, "tags", "root");
  requireKey(root, "field", "root");

  FieldLayout layout;

  // ---- Field dimensions ----
  {
    const auto & fj = root["field"];
    const std::string ctx = "root.field";
    layout.field.length = requireDouble(fj, "length", ctx);
    layout.field.width  = requireDouble(fj, "width",  ctx);

    if (layout.field.length <= 0.0) {
      throw std::runtime_error("Field layout: field.length must be positive");
    }
    if (layout.field.width <= 0.0) {
      throw std::runtime_error("Field layout: field.width must be positive");
    }
  }

  // ---- Tags ----
  for (std::size_t i = 0; i < root["tags"].size(); ++i) {
    const auto & tj  = root["tags"][i];
    const std::string ctx = "tags[" + std::to_string(i) + "]";

    requireKey(tj, "ID", ctx);
    if (!tj["ID"].is_number_integer()) {
      throw std::runtime_error("Field layout: " + ctx + ".ID must be an integer");
    }
    const int id = tj["ID"].get<int>();
    if (id < 0) {
      throw std::runtime_error(
        "Field layout: " + ctx + ".ID must be non-negative (got " + std::to_string(id) + ")");
    }
    if (layout.tags.count(id)) {
      throw std::runtime_error(
        "Field layout: duplicate tag ID " + std::to_string(id));
    }

    requireKey(tj, "pose", ctx);
    const auto & pj  = tj["pose"];
    const std::string pctx = ctx + ".pose";

    requireKey(pj, "translation", pctx);
    const auto & trj = pj["translation"];
    const std::string tctx = pctx + ".translation";

    Eigen::Vector3d trans;
    trans.x() = requireDouble(trj, "x", tctx);
    trans.y() = requireDouble(trj, "y", tctx);
    trans.z() = requireDouble(trj, "z", tctx);

    requireKey(pj, "rotation", pctx);
    const auto & rj  = pj["rotation"];
    const std::string rctx = pctx + ".rotation";

    requireKey(rj, "quaternion", rctx);
    const auto & qj  = rj["quaternion"];
    const std::string qctx = rctx + ".quaternion";

    const double qw = requireDouble(qj, "W", qctx);
    const double qx = requireDouble(qj, "X", qctx);
    const double qy = requireDouble(qj, "Y", qctx);
    const double qz = requireDouble(qj, "Z", qctx);

    Eigen::Quaterniond quat(qw, qx, qy, qz);
    const double norm = quat.norm();
    if (norm < 1e-6) {
      throw std::runtime_error(
        "Field layout: " + qctx + " quaternion has near-zero norm for tag ID " +
        std::to_string(id));
    }
    quat.normalize();

    FieldTag tag;
    tag.id               = id;
    tag.pose.translation = trans;
    tag.pose.rotation    = quat;

    layout.tags.emplace(id, tag);
  }

  if (layout.tags.empty()) {
    throw std::runtime_error("Field layout: 'tags' array is empty");
  }

  return layout;
}

// ---------------------------------------------------------------------------
// loadFieldLayout
// ---------------------------------------------------------------------------
FieldLayout loadFieldLayout(const std::string & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error(
      "Field layout: cannot open file '" + path + "'");
  }

  std::ostringstream ss;
  ss << file.rdbuf();
  if (file.bad()) {
    throw std::runtime_error(
      "Field layout: I/O error reading file '" + path + "'");
  }

  return parseFieldLayout(ss.str());
}

}  // namespace ros_vision
