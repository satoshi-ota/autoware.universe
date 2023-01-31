// Copyright 2022 TIER IV, Inc.
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

#include "behavior_path_planner/scene_module/avoidance/manager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

AvoidanceModuleManager::AvoidanceModuleManager(
  rclcpp::Node * node, const std::string & name, const size_t max_module_num, const size_t priority,
  const bool enable_simultaneous_execution)
: SceneModuleManagerInterface(node, name, max_module_num, priority, enable_simultaneous_execution)
{
  rtc_interface_left_ = std::make_shared<RTCInterface>(node, name + "_left");
  rtc_interface_right_ = std::make_shared<RTCInterface>(node, name + "_right");
  getModuleParams(node);
}

void AvoidanceModuleManager::updateModuleParams(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto p = parameters_;

  std::string ns = "avoidance.";
  updateParam<bool>(parameters, ns + "enable_safety_check", p->enable_safety_check);
  updateParam<bool>(parameters, ns + "publish_debug_marker", p->publish_debug_marker);
  updateParam<bool>(parameters, ns + "print_debug_info", p->print_debug_info);

  std::for_each(registered_modules_.begin(), registered_modules_.end(), [&p](const auto & m) {
    m.second->updateModuleParams(p);
  });
}

void AvoidanceModuleManager::getModuleParams(rclcpp::Node * node)
{
  AvoidanceParameters p{};
  // general params
  {
    std::string ns = "avoidance.";
    p.resample_interval_for_planning =
      node->declare_parameter<double>(ns + "resample_interval_for_planning");
    p.resample_interval_for_output =
      node->declare_parameter<double>(ns + "resample_interval_for_output");
    p.detection_area_right_expand_dist =
      node->declare_parameter<double>(ns + "detection_area_right_expand_dist");
    p.detection_area_left_expand_dist =
      node->declare_parameter<double>(ns + "detection_area_left_expand_dist");
    p.drivable_area_right_bound_offset =
      node->declare_parameter<double>(ns + "drivable_area_right_bound_offset");
    p.drivable_area_left_bound_offset =
      node->declare_parameter<double>(ns + "drivable_area_left_bound_offset");
    p.drivable_area_types_to_skip =
      node->declare_parameter<std::vector<std::string>>(ns + "drivable_area_types_to_skip");
    p.object_envelope_buffer = node->declare_parameter<double>(ns + "object_envelope_buffer");
    p.enable_bound_clipping = node->declare_parameter<bool>(ns + "enable_bound_clipping");
    p.enable_avoidance_over_same_direction =
      node->declare_parameter<bool>(ns + "enable_avoidance_over_same_direction");
    p.enable_avoidance_over_opposite_direction =
      node->declare_parameter<bool>(ns + "enable_avoidance_over_opposite_direction");
    p.enable_update_path_when_object_is_gone =
      node->declare_parameter<bool>(ns + "enable_update_path_when_object_is_gone");
    p.enable_safety_check = node->declare_parameter<bool>(ns + "enable_safety_check");
    p.enable_yield_maneuver = node->declare_parameter<bool>(ns + "enable_yield_maneuver");
    p.publish_debug_marker = node->declare_parameter<bool>(ns + "publish_debug_marker");
    p.print_debug_info = node->declare_parameter<bool>(ns + "print_debug_info");
  }

  // target object
  {
    std::string ns = "avoidance.target_object.";
    p.avoid_car = node->declare_parameter<bool>(ns + "car");
    p.avoid_truck = node->declare_parameter<bool>(ns + "truck");
    p.avoid_bus = node->declare_parameter<bool>(ns + "bus");
    p.avoid_trailer = node->declare_parameter<bool>(ns + "trailer");
    p.avoid_unknown = node->declare_parameter<bool>(ns + "unknown");
    p.avoid_bicycle = node->declare_parameter<bool>(ns + "bicycle");
    p.avoid_motorcycle = node->declare_parameter<bool>(ns + "motorcycle");
    p.avoid_pedestrian = node->declare_parameter<bool>(ns + "pedestrian");
  }

  // target filtering
  {
    std::string ns = "avoidance.target_filtering.";
    p.threshold_speed_object_is_stopped =
      node->declare_parameter<double>(ns + "threshold_speed_object_is_stopped");
    p.threshold_time_object_is_moving =
      node->declare_parameter<double>(ns + "threshold_time_object_is_moving");
    p.object_check_forward_distance =
      node->declare_parameter<double>(ns + "object_check_forward_distance");
    p.object_check_backward_distance =
      node->declare_parameter<double>(ns + "object_check_backward_distance");
    p.object_check_goal_distance =
      node->declare_parameter<double>(ns + "object_check_goal_distance");
    p.threshold_distance_object_is_on_center =
      node->declare_parameter<double>(ns + "threshold_distance_object_is_on_center");
    p.object_check_shiftable_ratio =
      node->declare_parameter<double>(ns + "object_check_shiftable_ratio");
    p.object_check_min_road_shoulder_width =
      node->declare_parameter<double>(ns + "object_check_min_road_shoulder_width");
    p.object_last_seen_threshold =
      node->declare_parameter<double>(ns + "object_last_seen_threshold");
  }

  // safety check
  {
    std::string ns = "avoidance.safety_check.";
    p.safety_check_backward_distance =
      node->declare_parameter<double>(ns + "safety_check_backward_distance");
    p.safety_check_time_horizon = node->declare_parameter<double>(ns + "safety_check_time_horizon");
    p.safety_check_idling_time = node->declare_parameter<double>(ns + "safety_check_idling_time");
    p.safety_check_accel_for_rss =
      node->declare_parameter<double>(ns + "safety_check_accel_for_rss");
    p.safety_check_hysteresis_factor =
      node->declare_parameter<double>(ns + "safety_check_hysteresis_factor");
  }

  // avoidance maneuver (lateral)
  {
    std::string ns = "avoidance.avoidance.lateral.";
    p.lateral_collision_margin = node->declare_parameter<double>(ns + "lateral_collision_margin");
    p.lateral_collision_safety_buffer =
      node->declare_parameter<double>(ns + "lateral_collision_safety_buffer");
    p.lateral_passable_safety_buffer =
      node->declare_parameter<double>(ns + "lateral_passable_safety_buffer");
    p.road_shoulder_safety_margin =
      node->declare_parameter<double>(ns + "road_shoulder_safety_margin");
    p.avoidance_execution_lateral_threshold =
      node->declare_parameter<double>(ns + "avoidance_execution_lateral_threshold");
    p.max_right_shift_length = node->declare_parameter<double>(ns + "max_right_shift_length");
    p.max_left_shift_length = node->declare_parameter<double>(ns + "max_left_shift_length");
  }

  // avoidance maneuver (longitudinal)
  {
    std::string ns = "avoidance.avoidance.longitudinal.";
    p.prepare_time = node->declare_parameter<double>(ns + "prepare_time");
    p.longitudinal_collision_safety_buffer =
      node->declare_parameter<double>(ns + "longitudinal_collision_safety_buffer");
    p.min_prepare_distance = node->declare_parameter<double>(ns + "min_prepare_distance");
    p.min_avoidance_distance = node->declare_parameter<double>(ns + "min_avoidance_distance");
    p.min_nominal_avoidance_speed =
      node->declare_parameter<double>(ns + "min_nominal_avoidance_speed");
    p.min_sharp_avoidance_speed = node->declare_parameter<double>(ns + "min_sharp_avoidance_speed");
  }

  // yield
  {
    std::string ns = "avoidance.yield.";
    p.yield_velocity = node->declare_parameter<double>(ns + "yield_velocity");
  }

  // stop
  {
    std::string ns = "avoidance.stop.";
    p.stop_min_distance = node->declare_parameter<double>(ns + "min_distance");
    p.stop_max_distance = node->declare_parameter<double>(ns + "max_distance");
  }

  // constraints
  {
    std::string ns = "avoidance.constraints.";
    p.use_constraints_for_decel = node->declare_parameter<bool>(ns + "use_constraints_for_decel");
  }

  // constraints (longitudinal)
  {
    std::string ns = "avoidance.constraints.longitudinal.";
    p.nominal_deceleration = node->declare_parameter<double>(ns + "nominal_deceleration");
    p.nominal_jerk = node->declare_parameter<double>(ns + "nominal_jerk");
    p.max_deceleration = node->declare_parameter<double>(ns + "max_deceleration");
    p.max_jerk = node->declare_parameter<double>(ns + "max_jerk");
    p.min_avoidance_speed_for_acc_prevention =
      node->declare_parameter<double>(ns + "min_avoidance_speed_for_acc_prevention");
    p.max_avoidance_acceleration =
      node->declare_parameter<double>(ns + "max_avoidance_acceleration");
  }

  // constraints (lateral)
  {
    std::string ns = "avoidance.constraints.lateral.";
    p.nominal_lateral_jerk = node->declare_parameter<double>(ns + "nominal_lateral_jerk");
    p.max_lateral_jerk = node->declare_parameter<double>(ns + "max_lateral_jerk");
  }

  // velocity matrix
  {
    std::string ns = "avoidance.target_velocity_matrix.";
    p.col_size = node->declare_parameter<int>(ns + "col_size");
    p.target_velocity_matrix = node->declare_parameter<std::vector<double>>(ns + "matrix");
  }

  parameters_ = std::make_shared<AvoidanceParameters>(p);
}
}  // namespace behavior_path_planner
