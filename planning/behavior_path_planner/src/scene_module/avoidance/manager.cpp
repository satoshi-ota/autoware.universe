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
  const auto dp = [this, &node](const std::string & str, auto def_val) {
    std::string name = name_ + "." + str;
    return node->declare_parameter(name, def_val);
  };

  AvoidanceParameters p{};
  p.resample_interval_for_planning = dp("resample_interval_for_planning", 0.3);
  p.resample_interval_for_output = dp("resample_interval_for_output", 3.0);
  p.detection_area_right_expand_dist = dp("detection_area_right_expand_dist", 0.0);
  p.detection_area_left_expand_dist = dp("detection_area_left_expand_dist", 1.0);
  p.enable_avoidance_over_same_direction = dp("enable_avoidance_over_same_direction", true);
  p.enable_avoidance_over_opposite_direction = dp("enable_avoidance_over_opposite_direction", true);
  p.enable_update_path_when_object_is_gone = dp("enable_update_path_when_object_is_gone", false);
  p.enable_safety_check = dp("enable_safety_check", false);

  p.threshold_distance_object_is_on_center = dp("threshold_distance_object_is_on_center", 1.0);
  p.threshold_speed_object_is_stopped = dp("threshold_speed_object_is_stopped", 1.0);
  p.threshold_time_object_is_moving = dp("threshold_time_object_is_moving", 1.0);
  p.object_check_forward_distance = dp("object_check_forward_distance", 150.0);
  p.object_check_backward_distance = dp("object_check_backward_distance", 2.0);
  p.object_check_shiftable_ratio = dp("object_check_shiftable_ratio", 1.0);
  p.object_check_min_road_shoulder_width = dp("object_check_min_road_shoulder_width", 0.5);
  p.object_envelope_buffer = dp("object_envelope_buffer", 0.1);
  p.lateral_collision_margin = dp("lateral_collision_margin", 2.0);
  p.lateral_collision_safety_buffer = dp("lateral_collision_safety_buffer", 0.5);
  p.longitudinal_collision_safety_buffer = dp("longitudinal_collision_safety_buffer", 0.0);

  p.safety_check_min_longitudinal_margin = dp("safety_check_min_longitudinal_margin", 0.0);
  p.safety_check_backward_distance = dp("safety_check_backward_distance", 0.0);
  p.safety_check_time_horizon = dp("safety_check_time_horizon", 10.0);
  p.safety_check_idling_time = dp("safety_check_idling_time", 1.5);
  p.safety_check_accel_for_rss = dp("safety_check_accel_for_rss", 2.5);

  p.prepare_time = dp("prepare_time", 3.0);
  p.min_prepare_distance = dp("min_prepare_distance", 10.0);
  p.min_avoidance_distance = dp("min_avoidance_distance", 10.0);

  p.min_nominal_avoidance_speed = dp("min_nominal_avoidance_speed", 5.0);
  p.min_sharp_avoidance_speed = dp("min_sharp_avoidance_speed", 1.0);

  p.road_shoulder_safety_margin = dp("road_shoulder_safety_margin", 0.0);

  p.max_right_shift_length = dp("max_right_shift_length", 1.5);
  p.max_left_shift_length = dp("max_left_shift_length", 1.5);

  p.nominal_lateral_jerk = dp("nominal_lateral_jerk", 0.3);
  p.max_lateral_jerk = dp("max_lateral_jerk", 2.0);

  p.longitudinal_collision_margin_min_distance =
    dp("longitudinal_collision_margin_min_distance", 0.0);
  p.longitudinal_collision_margin_time = dp("longitudinal_collision_margin_time", 0.0);

  p.object_last_seen_threshold = dp("object_last_seen_threshold", 2.0);

  p.min_avoidance_speed_for_acc_prevention = dp("min_avoidance_speed_for_acc_prevention", 3.0);
  p.max_avoidance_acceleration = dp("max_avoidance_acceleration", 0.5);

  p.publish_debug_marker = dp("publish_debug_marker", false);
  p.print_debug_info = dp("print_debug_info", false);

  // velocity matrix
  {
    std::string ns = "avoidance.target_velocity_matrix.";
    p.col_size = node->declare_parameter<int>(ns + "col_size");
    p.target_velocity_matrix = node->declare_parameter<std::vector<double>>(ns + "matrix");
  }

  p.avoid_car = dp("target_object.car", true);
  p.avoid_truck = dp("target_object.truck", true);
  p.avoid_bus = dp("target_object.bus", true);
  p.avoid_trailer = dp("target_object.trailer", true);
  p.avoid_unknown = dp("target_object.unknown", false);
  p.avoid_bicycle = dp("target_object.bicycle", false);
  p.avoid_motorcycle = dp("target_object.motorcycle", false);
  p.avoid_pedestrian = dp("target_object.pedestrian", false);

  p.drivable_area_right_bound_offset = dp("drivable_area_right_bound_offset", 0.0);
  p.drivable_area_left_bound_offset = dp("drivable_area_left_bound_offset", 0.0);

  p.enable_bound_clipping = dp("enable_bound_clipping", false);

  p.avoidance_execution_lateral_threshold = dp("avoidance_execution_lateral_threshold", 0.499);

  module_params_ = std::make_shared<AvoidanceParameters>(p);
}
}  // namespace behavior_path_planner
