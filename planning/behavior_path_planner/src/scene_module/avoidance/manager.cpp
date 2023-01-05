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
  rclcpp::Node * node, const std::string & name, const size_t max_module_num)
: SceneModuleManagerInterface(node, name, max_module_num)
{
  rtc_interface_left_ = std::make_shared<RTCInterface>(node, name + "_left");
  rtc_interface_right_ = std::make_shared<RTCInterface>(node, name + "_right");
  getModuleParams(node);
}

void AvoidanceModuleManager::updateModuleParams(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto p = module_params_;

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
  const auto dp = [this, &node](const std::string & str, auto & value, auto def_val) {
    std::string name = "avoidance." + str;

    if (node->has_parameter(name)) {
      node->get_parameter<decltype(def_val)>(name, value);
    } else {
      value = node->declare_parameter(name, def_val);
    }
  };

  AvoidanceParameters p{};
  dp("resample_interval_for_planning", p.resample_interval_for_planning, 0.3);
  dp("resample_interval_for_output", p.resample_interval_for_output, 3.0);
  dp("detection_area_right_expand_dist", p.detection_area_right_expand_dist, 0.0);
  dp("detection_area_left_expand_dist", p.detection_area_left_expand_dist, 1.0);
  dp("enable_avoidance_over_same_direction", p.enable_avoidance_over_same_direction, true);
  dp("enable_avoidance_over_opposite_direction", p.enable_avoidance_over_opposite_direction, true);
  dp("enable_update_path_when_object_is_gone", p.enable_update_path_when_object_is_gone, false);
  dp("enable_safety_check", p.enable_safety_check, false);

  dp("threshold_distance_object_is_on_center", p.threshold_distance_object_is_on_center, 1.0);
  dp("threshold_speed_object_is_stopped", p.threshold_speed_object_is_stopped, 1.0);
  dp("threshold_time_object_is_moving", p.threshold_time_object_is_moving, 1.0);
  dp("object_check_forward_distance", p.object_check_forward_distance, 150.0);
  dp("object_check_backward_distance", p.object_check_backward_distance, 2.0);
  dp("object_check_shiftable_ratio", p.object_check_shiftable_ratio, 1.0);
  dp("object_check_min_road_shoulder_width", p.object_check_min_road_shoulder_width, 0.5);
  dp("object_envelope_buffer", p.object_envelope_buffer, 0.1);
  dp("object_last_seen_threshold", p.object_last_seen_threshold, 2.0);

  dp("target_object.car", p.avoid_car, true);
  dp("target_object.truck", p.avoid_truck, true);
  dp("target_object.bus", p.avoid_bus, true);
  dp("target_object.trailer", p.avoid_trailer, true);
  dp("target_object.unknown", p.avoid_unknown, false);
  dp("target_object.bicycle", p.avoid_bicycle, false);
  dp("target_object.motorcycle", p.avoid_motorcycle, false);
  dp("target_object.pedestrian", p.avoid_pedestrian, false);

  dp("enable_bound_clipping", p.enable_bound_clipping, false);

  dp("lateral_collision_margin", p.lateral_collision_margin, 2.0);
  dp("lateral_collision_safety_buffer", p.lateral_collision_safety_buffer, 0.5);
  dp("longitudinal_collision_safety_buffer", p.longitudinal_collision_safety_buffer, 0.0);

  dp("safety_check_min_longitudinal_margin", p.safety_check_min_longitudinal_margin, 0.0);
  dp("safety_check_backward_distance", p.safety_check_backward_distance, 0.0);
  dp("safety_check_time_horizon", p.safety_check_time_horizon, 10.0);
  dp("safety_check_idling_time", p.safety_check_idling_time, 1.5);
  dp("safety_check_accel_for_rss", p.safety_check_accel_for_rss, 2.5);

  dp("prepare_time", p.prepare_time, 3.0);
  dp("min_prepare_distance", p.min_prepare_distance, 10.0);
  dp("min_avoidance_distance", p.min_avoidance_distance, 10.0);

  dp("min_nominal_avoidance_speed", p.min_nominal_avoidance_speed, 5.0);
  dp("min_sharp_avoidance_speed", p.min_sharp_avoidance_speed, 1.0);

  dp("road_shoulder_safety_margin", p.road_shoulder_safety_margin, 0.0);

  dp("max_right_shift_length", p.max_right_shift_length, 1.5);
  dp("max_left_shift_length", p.max_left_shift_length, 1.5);

  dp("nominal_lateral_jerk", p.nominal_lateral_jerk, 0.3);
  dp("max_lateral_jerk", p.max_lateral_jerk, 2.0);

  dp(
    "longitudinal_collision_margin_min_distance", p.longitudinal_collision_margin_min_distance,
    0.0);
  dp("longitudinal_collision_margin_time", p.longitudinal_collision_margin_time, 0.0);

  dp("min_avoidance_speed_for_acc_prevention", p.min_avoidance_speed_for_acc_prevention, 3.0);
  dp("max_avoidance_acceleration", p.max_avoidance_acceleration, 0.5);

  dp("publish_debug_marker", p.publish_debug_marker, false);
  dp("print_debug_info", p.print_debug_info, false);

  // velocity matrix
  {
    std::string ns = "avoidance.target_velocity_matrix.";
    p.col_size = node->declare_parameter<int>(ns + "col_size");
    p.target_velocity_matrix = node->declare_parameter<std::vector<double>>(ns + "matrix");
  }

  dp("drivable_area_right_bound_offset", p.drivable_area_right_bound_offset, 0.0);
  dp("drivable_area_left_bound_offset", p.drivable_area_left_bound_offset, 0.0);

  dp("enable_bound_clipping", p.enable_bound_clipping, false);

  dp("avoidance_execution_lateral_threshold", p.avoidance_execution_lateral_threshold, 0.499);

  module_params_ = std::make_shared<AvoidanceParameters>(p);
}
}  // namespace behavior_path_planner
