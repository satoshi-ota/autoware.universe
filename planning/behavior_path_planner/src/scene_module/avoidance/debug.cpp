// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "behavior_path_planner/scene_module/avoidance/debug.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <tf2/utils.h>

#include <string>
#include <vector>

namespace marker_utils::avoidance_marker
{

using behavior_path_planner::AvoidPoint;
using behavior_path_planner::util::shiftPose;
using motion_utils::createDeadLineVirtualWallMarker;
using motion_utils::createSlowDownVirtualWallMarker;
using motion_utils::createStopVirtualWallMarker;
using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::getPose;
using tier4_autoware_utils::rad2deg;
using tier4_autoware_utils::toMsg;

using geometry_msgs::msg::Vector3;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;

namespace
{

int32_t uuidToInt32(const unique_identifier_msgs::msg::UUID & uuid)
{
  int32_t ret = 0;

  for (size_t i = 0; i < sizeof(int32_t) / sizeof(int8_t); ++i) {
    ret <<= sizeof(int8_t);
    ret |= uuid.uuid.at(i);
  }

  return ret;
}

MarkerArray createObjectsCubeMarkerArray(
  const ObjectDataArray & objects, std::string && ns, const Vector3 & scale,
  const ColorRGBA & color)
{
  MarkerArray msg;

  auto marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::CUBE, scale, color);
  for (const auto & object : objects) {
    marker.id = uuidToInt32(object.object.object_id);
    marker.pose = object.object.kinematics.initial_pose_with_covariance.pose;
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createObjectInfoMarkerArray(const ObjectDataArray & objects, std::string && ns)
{
  MarkerArray msg;

  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::TEXT_VIEW_FACING,
    createMarkerScale(0.5, 0.5, 0.5), createMarkerColor(1.0, 1.0, 0.0, 1.0));

  for (const auto & object : objects) {
    {
      marker.id = uuidToInt32(object.object.object_id);
      marker.pose = object.object.kinematics.initial_pose_with_covariance.pose;
      std::ostringstream string_stream;
      string_stream << std::fixed << std::setprecision(2) << std::boolalpha;
      string_stream << "ratio:" << object.offset_ratio << " [-]\n"
                    << "lateral:" << object.lateral << " [m]\n"
                    << "intersection:" << object.to_intersection_distance << " [m]\n"
                    << "move_time:" << object.move_time << " [s]\n"
                    << "stop_time:" << object.stop_time << " [s]\n"
                    << "avoid_req:" << object.avoid_required;
      marker.text = string_stream.str();
      msg.markers.push_back(marker);
    }

    {
      marker.id = uuidToInt32(object.object.object_id);
      marker.pose.position.z += 2.0;
      std::ostringstream string_stream;
      string_stream << object.reason;
      marker.text = string_stream.str();
      marker.color = createMarkerColor(1.0, 1.0, 1.0, 0.999);
      marker.scale = createMarkerScale(0.6, 0.6, 0.6);
      marker.ns = ns + "_reason";
      msg.markers.push_back(marker);
    }
  }

  return msg;
}

}  // namespace

MarkerArray createAvoidPointMarkerArray(
  const AvoidPointArray & shift_points, std::string && ns, const float & r, const float & g,
  const float & b, const double & w)
{
  AvoidPointArray shift_points_local = shift_points;
  if (shift_points_local.empty()) {
    shift_points_local.push_back(AvoidPoint());
  }

  int32_t id{0};
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  for (const auto & sp : shift_points_local) {
    // ROS_ERROR("sp: s = (%f, %f), g = (%f, %f)", sp.start.x, sp.start.y, sp.end.x, sp.end.y);
    Marker basic_marker = createDefaultMarker(
      "map", current_time, ns, 0L, Marker::CUBE, createMarkerScale(0.5, 0.5, 0.5),
      createMarkerColor(r, g, b, 0.9));
    basic_marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    {
      // start point
      auto marker_s = basic_marker;
      marker_s.id = id++;
      marker_s.pose = sp.start;
      // shiftPose(&marker_s.pose, current_shift);  // old
      shiftPose(&marker_s.pose, sp.start_length);
      msg.markers.push_back(marker_s);

      // end point
      auto marker_e = basic_marker;
      marker_e.id = id++;
      marker_e.pose = sp.end;
      shiftPose(&marker_e.pose, sp.length);
      msg.markers.push_back(marker_e);

      // start-to-end line
      auto marker_l = basic_marker;
      marker_l.id = id++;
      marker_l.type = Marker::LINE_STRIP;
      marker_l.scale = tier4_autoware_utils::createMarkerScale(w, 0.0, 0.0);
      marker_l.points.push_back(marker_s.pose.position);
      marker_l.points.push_back(marker_e.pose.position);
      msg.markers.push_back(marker_l);
    }
    // current_shift = sp.length;
  }

  return msg;
}

MarkerArray createTargetObjectsMarkerArray(const ObjectDataArray & objects, std::string && ns)
{
  MarkerArray msg;

  {
    const auto filtered_objects = [&objects]() {
      ObjectDataArray ret{};
      for (const auto & o : objects) {
        if (!o.avoid_required) {
          continue;
        }
        ret.push_back(o);
      }

      return ret;
    }();

    appendMarkerArray(
      createObjectsCubeMarkerArray(
        filtered_objects, ns + "_cube", createMarkerScale(3.0, 1.5, 1.5),
        createMarkerColor(1.0, 0.0, 0.0, 0.8)),
      &msg);
    appendMarkerArray(createObjectInfoMarkerArray(filtered_objects, ns + "_info"), &msg);
  }

  {
    const auto filtered_objects = [&objects]() {
      ObjectDataArray ret{};
      for (const auto & o : objects) {
        if (o.avoid_required) {
          continue;
        }
        ret.push_back(o);
      }

      return ret;
    }();

    appendMarkerArray(
      createObjectsCubeMarkerArray(
        filtered_objects, ns + "_cube", createMarkerScale(3.0, 1.5, 1.5),
        createMarkerColor(0.8, 0.8, 0.0, 0.8)),
      &msg);
    appendMarkerArray(createObjectInfoMarkerArray(filtered_objects, ns + "_info"), &msg);
  }

  {
    for (const auto & object : objects) {
      const auto pos = object.object.kinematics.initial_pose_with_covariance.pose.position;

      {
        Marker marker = createDefaultMarker(
          "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns + "_envelope_polygon", 0L,
          Marker::LINE_STRIP, createMarkerScale(0.1, 0.0, 0.0),
          createMarkerColor(1.0, 1.0, 1.0, 0.999));

        for (const auto & p : object.envelope_poly.outer()) {
          marker.points.push_back(createPoint(p.x(), p.y(), pos.z));
        }

        marker.points.push_back(marker.points.front());
        marker.id = uuidToInt32(object.object.object_id);
        msg.markers.push_back(marker);
      }

      {
        Marker marker = createDefaultMarker(
          "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns + "_centroid", 0, Marker::POINTS,
          createMarkerScale(0.6, 0.6, 0.0), createMarkerColor(1.0, 1.0, 0.0, 0.999));

        marker.points.push_back(toMsg(object.centroid.to_3d(pos.z)));
        msg.markers.push_back(marker);
      }
    }
  }

  return msg;
}

MarkerArray createOtherObjectsMarkerArray(
  const ObjectDataArray & objects, const std::string & reason)
{
  const auto filtered_objects = [&objects, &reason]() {
    ObjectDataArray ret{};
    for (const auto & o : objects) {
      if (o.reason != reason) {
        continue;
      }
      ret.push_back(o);
    }

    return ret;
  }();

  MarkerArray msg;

  appendMarkerArray(
    createObjectsCubeMarkerArray(
      filtered_objects, "others_" + reason + "_cube", createMarkerScale(3.0, 1.5, 1.5),
      createMarkerColor(0.0, 1.0, 0.0, 0.8)),
    &msg);
  appendMarkerArray(
    createObjectInfoMarkerArray(filtered_objects, "others_" + reason + "_info"), &msg);

  return msg;
}

MarkerArray createUnsafeObjectsMarkerArray(const ObjectDataArray & objects, std::string && ns)
{
  return createObjectsCubeMarkerArray(
    objects, ns + "_cube", createMarkerScale(3.2, 1.7, 2.0), createMarkerColor(0.0, 0.0, 1.0, 0.8));
}

MarkerArray createUnavoidableObjectsMarkerArray(const ObjectDataArray & objects, std::string && ns)
{
  MarkerArray msg;

  appendMarkerArray(
    createObjectsCubeMarkerArray(
      objects, ns + "_cube", createMarkerScale(3.2, 1.7, 2.0),
      createMarkerColor(1.0, 0.0, 1.0, 0.9)),
    &msg);
  appendMarkerArray(createObjectInfoMarkerArray(objects, ns + "_info"), &msg);

  return msg;
}

MarkerArray createPredictedVehiclePositions(const PathWithLaneId & path, std::string && ns)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  auto p_marker = createDefaultMarker(
    "map", current_time, ns, 0L, Marker::POINTS, createMarkerScale(0.4, 0.4, 0.0),
    createMarkerColor(1.0, 0.0, 0.0, 0.999));

  const auto pushPointMarker = [&](const Pose & p, const double t) {
    const auto r = t > 10.0 ? 1.0 : t / 10.0;
    p_marker.points.push_back(p.position);
    p_marker.colors.push_back(createMarkerColor(r, 1.0 - r, 0.0, 0.999));
  };

  auto t_marker = createDefaultMarker(
    "map", current_time, ns + "_text", 0L, Marker::TEXT_VIEW_FACING,
    createMarkerScale(0.3, 0.3, 0.3), createMarkerColor(1.0, 1.0, 0.0, 1.0));

  const auto pushTextMarker = [&](const Pose & p, const double t, const double d, const double v) {
    t_marker.id++;
    t_marker.pose = p;
    std::ostringstream string_stream;
    string_stream << std::fixed << std::setprecision(2);
    string_stream << "t[s]: " << t << "\n"
                  << "d[m]: " << d << "\n"
                  << "v[m/s]: " << v;
    t_marker.text = string_stream.str();
    msg.markers.push_back(t_marker);
  };

  constexpr double dt_save = 0.5;
  double t_save = 0.0;
  double t_sum = 0.0;
  double d_sum = 0.0;

  if (path.points.empty()) {
    return msg;
  }

  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto & p1 = path.points.at(i - 1);
    const auto & p2 = path.points.at(i);
    const auto ds = calcDistance2d(p1, p2);

    if (t_save < t_sum + 1e-3) {
      pushPointMarker(getPose(p1), t_sum);
      pushTextMarker(getPose(p1), t_sum, d_sum, p1.point.longitudinal_velocity_mps);
      t_save += dt_save;
    }

    const auto v = std::max(p1.point.longitudinal_velocity_mps, float{1.0});

    t_sum += ds / v;
    d_sum += ds;
  }

  msg.markers.push_back(p_marker);

  return msg;
}

MarkerArray createOverhangFurthestLineStringMarkerArray(
  const lanelet::ConstLineStrings3d & linestrings, std::string && ns, const float & r,
  const float & g, const float & b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  for (const auto & linestring : linestrings) {
    const auto id = static_cast<int>(linestring.id());
    Marker marker = createDefaultMarker(
      "map", current_time, ns, id, Marker::LINE_STRIP, createMarkerScale(0.4, 0.0, 0.0),
      createMarkerColor(r, g, b, 0.999));

    marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    for (const auto & p : linestring.basicLineString()) {
      marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
    }
    msg.markers.push_back(marker);

    Marker marker_linestring_id = createDefaultMarker(
      "map", current_time, "linestring_id", id, Marker::TEXT_VIEW_FACING,
      createMarkerScale(1.5, 1.5, 1.5), createMarkerColor(1.0, 1.0, 1.0, 0.8));
    Pose text_id_pose;
    text_id_pose.position.x = linestring.front().x();
    text_id_pose.position.y = linestring.front().y();
    text_id_pose.position.z = linestring.front().z();
    marker_linestring_id.pose = text_id_pose;
    std::ostringstream ss;
    ss << "(ID : " << id << ") ";
    marker_linestring_id.text = ss.str();
    msg.markers.push_back(marker_linestring_id);
  }

  return msg;
}

MarkerArray createYieldAlertMarkerArray(
  const AvoidanceState & state, const Pose & pose, const MarginDataArray & margin_data_array,
  const bool exist_adjacent_objects)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  if (exist_adjacent_objects) {
    auto marker = createDefaultMarker(
      "map", current_time, "yield_alert", 0L, Marker::CYLINDER, createMarkerScale(0.2, 0.2, 2.0),
      createMarkerColor(1.0, 1.0, 0.0, 0.8));

    marker.color = state == AvoidanceState::YIELD ? createMarkerColor(1.0, 0.0, 0.0, 0.8)
                                                  : createMarkerColor(1.0, 1.0, 0.0, 0.8);

    marker.pose = calcOffsetPose(pose, 0.0, 1.5, 1.0);
    msg.markers.push_back(marker);

    marker.pose = calcOffsetPose(pose, 0.0, -1.5, 1.0);
    marker.id++;
    msg.markers.push_back(marker);
  }

  if (state == AvoidanceState::YIELD) {
    return msg;
  }

  {
    auto marker = createDefaultMarker(
      "map", current_time, "longitudinal_margin", 0L, Marker::CUBE,
      createMarkerScale(3.0, 1.5, 1.5), createMarkerColor(1.0, 1.0, 1.0, 0.1));

    for (const auto & m : margin_data_array) {
      if (m.enough_lateral_margin) {
        continue;
      }

      constexpr double max_x = 10.0;

      const auto offset = 0.5 * (m.base_link2front + m.base_link2rear) - m.base_link2rear;
      const auto diff = m.longitudinal_distance - m.longitudinal_margin;
      const auto scale_x = std::min(max_x, 2.0 * (m.base_link2front + m.base_link2rear + diff));

      const auto ratio = std::clamp(diff / max_x, 0.0, 1.0);

      marker.pose = calcOffsetPose(m.pose, offset, 0.0, 0.0);
      marker.pose.position.z += 1.0;
      marker.scale = createMarkerScale(scale_x, 2.0 * m.vehicle_width, 2.0);
      marker.color = createMarkerColor(1.0 - ratio, ratio, 0.0, 0.1);
      marker.id++;
      msg.markers.push_back(marker);
    }
  }

  return msg;
}

MarkerArray createEgoStatusMarkerArray(
  const AvoidancePlanningData & data, const Pose & p_ego, std::string && ns)
{
  MarkerArray msg;

  auto marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::TEXT_VIEW_FACING,
    createMarkerScale(0.5, 0.5, 0.5), createMarkerColor(1.0, 1.0, 1.0, 0.999));
  marker.pose = p_ego;

  {
    std::ostringstream string_stream;
    string_stream << std::fixed << std::setprecision(2) << std::boolalpha;
    string_stream << "avoid_vel:" << data.safe_velocity << "[m/s],"
                  << "jerk_req:" << data.required_jerk << "[m/sss],\n"
                  << "length_variable:" << data.lateral_margin_variable << "[m],"
                  << "shift_length:" << data.shift_length << "[m]\n"
                  << "avoid_now:" << data.avoiding_now << ","
                  << "avoid_req:" << data.avoid_required << ","
                  << "yield_req:" << data.yield_required << ","
                  << "safe:" << data.safe << "\n"
                  << "d_variable:" << data.avoid_variable_distance << "[m],"
                  << "d_constant:" << data.avoid_constant_distance << "[m]\n"
                  << "d_total:" << data.avoid_constant_distance + data.avoid_variable_distance
                  << "[m],"
                  << "d_front_object:" << data.front_object_longitudinal << "[m]\n"
                  << "sp(raw,new,safe):(" << data.unapproved_raw_sp.size() << ","
                  << data.unapproved_new_sp.size() << "," << data.safe_new_sp.size() << ")";
    marker.text = string_stream.str();

    msg.markers.push_back(marker);
  }

  {
    std::ostringstream string_stream;
    string_stream << "ego_state:";
    switch (data.state) {
      case AvoidanceState::NOT_AVOID:
        string_stream << "NOT_AVOID";
        break;
      case AvoidanceState::AVOID_PATH_NOT_READY:
        string_stream << "AVOID_PATH_NOT_READY";
        marker.color = createMarkerColor(1.0, 0.0, 0.0, 0.999);
        break;
      case AvoidanceState::YIELD:
        string_stream << "YIELD";
        marker.color = createMarkerColor(1.0, 1.0, 0.0, 0.999);
        break;
      case AvoidanceState::AVOID_PATH_READY:
        string_stream << "AVOID_PATH_READY";
        marker.color = createMarkerColor(0.0, 1.0, 0.0, 0.999);
        break;
      case AvoidanceState::AVOID_EXECUTE:
        string_stream << "AVOID_EXECUTE";
        marker.color = createMarkerColor(0.0, 1.0, 0.0, 0.999);
        break;
      default:
        throw std::domain_error("invalid behaivor");
    }
    marker.text = string_stream.str();
    marker.pose.position.z += 2.0;
    marker.id++;

    msg.markers.push_back(marker);
  }

  return msg;
}
}  // namespace marker_utils::avoidance_marker

std::string toStrInfo(const behavior_path_planner::ShiftPointArray & sp_arr)
{
  if (sp_arr.empty()) {
    return "point is empty";
  }
  std::stringstream ss;
  for (const auto & sp : sp_arr) {
    ss << std::endl << toStrInfo(sp);
  }
  return ss.str();
}

std::string toStrInfo(const behavior_path_planner::ShiftPoint & sp)
{
  const auto & ps = sp.start.position;
  const auto & pe = sp.end.position;
  std::stringstream ss;
  ss << "shift length: " << sp.length << ", start_idx: " << sp.start_idx
     << ", end_idx: " << sp.end_idx << ", start: (" << ps.x << ", " << ps.y << "), end: (" << pe.x
     << ", " << pe.y << ")";
  return ss.str();
}

std::string toStrInfo(const behavior_path_planner::AvoidPointArray & ap_arr)
{
  if (ap_arr.empty()) {
    return "point is empty";
  }
  std::stringstream ss;
  for (const auto & ap : ap_arr) {
    ss << std::endl << toStrInfo(ap);
  }
  return ss.str();
}
std::string toStrInfo(const behavior_path_planner::AvoidPoint & ap)
{
  std::stringstream pids;
  for (const auto pid : ap.parent_ids) {
    pids << pid << ", ";
  }
  const auto & ps = ap.start.position;
  const auto & pe = ap.end.position;
  std::stringstream ss;
  ss << "id = " << ap.id << ", shift length: " << ap.length << ", start_idx: " << ap.start_idx
     << ", end_idx: " << ap.end_idx << ", start_dist = " << ap.start_longitudinal
     << ", end_dist = " << ap.end_longitudinal << ", start_length: " << ap.start_length
     << ", start: (" << ps.x << ", " << ps.y << "), end: (" << pe.x << ", " << pe.y
     << "), relative_length: " << ap.getRelativeLength() << ", grad = " << ap.getGradient()
     << ", parent_ids = [" << pids.str() << "]";
  return ss.str();
}
