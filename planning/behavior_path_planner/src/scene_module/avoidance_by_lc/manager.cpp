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

#include "behavior_path_planner/scene_module/avoidance_by_lc/manager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

AvoidanceByLCModuleManager::AvoidanceByLCModuleManager(
  rclcpp::Node * node, const std::string & name, const size_t max_module_num, const size_t priority)
: SceneModuleManagerInterface(node, name, max_module_num, priority)
{
  rtc_interface_left_ = std::make_shared<RTCInterface>(node, name + "_left");
  rtc_interface_right_ = std::make_shared<RTCInterface>(node, name + "_right");
  getModuleParams(node);
}

void AvoidanceByLCModuleManager::updateModuleParams(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto p = module_params_;

  std::string ns = name_ + ".";
  updateParam<bool>(parameters, ns + "publish_debug_marker", p->publish_debug_marker);

  std::for_each(registered_modules_.begin(), registered_modules_.end(), [&p](const auto & m) {
    m.second->updateModuleParams(p);
  });
}

void AvoidanceByLCModuleManager::getModuleParams(rclcpp::Node * node)
{
  using tier4_autoware_utils::deg2rad;

  AvoidanceByLCParameters p{};

  // avoidance parameters
  {
    const auto dp = [this, &node](const std::string & str, auto & value, auto def_val) {
      std::string name = "avoidance." + str;

      if (node->has_parameter(name)) {
        node->get_parameter<decltype(def_val)>(name, value);
      } else {
        value = node->declare_parameter(name, def_val);
      }
    };

    dp("resample_interval_for_planning", p.resample_interval_for_planning, 0.3);
    dp("resample_interval_for_output", p.resample_interval_for_output, 3.0);
    dp("detection_area_right_expand_dist", p.detection_area_right_expand_dist, 0.0);
    dp("detection_area_left_expand_dist", p.detection_area_left_expand_dist, 1.0);
    dp("enable_avoidance_over_same_direction", p.enable_avoidance_over_same_direction, true);
    dp(
      "enable_avoidance_over_opposite_direction", p.enable_avoidance_over_opposite_direction, true);
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
  }

  // lane change parameters
  {
    const auto dp = [this, &node](const std::string & str, auto & value, auto def_val) {
      std::string name = "lane_change." + str;

      if (node->has_parameter(name)) {
        node->get_parameter<decltype(def_val)>(name, value);
      } else {
        value = node->declare_parameter(name, def_val);
      }
    };

    dp("lane_change_prepare_duration", p.lane_change_prepare_duration, 2.0);
    dp("lane_changing_safety_check_duration", p.lane_changing_safety_check_duration, 4.0);
    dp("lane_changing_lateral_jerk", p.lane_changing_lateral_jerk, 0.5);
    dp("lane_changing_lateral_acc", p.lane_changing_lateral_acc, 0.5);
    dp("lane_change_finish_judge_buffer", p.lane_change_finish_judge_buffer, 3.0);
    dp("minimum_lane_change_velocity", p.minimum_lane_change_velocity, 5.6);
    dp("prediction_time_resolution", p.prediction_time_resolution, 0.5);
    dp("maximum_deceleration", p.maximum_deceleration, 1.0);
    dp("lane_change_sampling_num", p.lane_change_sampling_num, 10);
    // dp("abort_lane_change_velocity_thresh", p.abort_lane_change_velocity_thresh, 0.5);
    // dp("abort_lane_change_angle_thresh", p.abort_lane_change_angle_thresh, deg2rad(10.0));
    // dp("abort_lane_change_distance_thresh", p.abort_lane_change_distance_thresh, 0.3);
    dp("prepare_phase_ignore_target_speed_thresh", p.prepare_phase_ignore_target_speed_thresh, 0.1);
    dp("enable_abort_lane_change", p.enable_abort_lane_change, true);
    dp("enable_collision_check_at_prepare_phase", p.enable_collision_check_at_prepare_phase, true);
    dp("use_predicted_path_outside_lanelet", p.use_predicted_path_outside_lanelet, true);
    dp("use_all_predicted_path", p.use_all_predicted_path, true);
    dp("publish_debug_marker", p.publish_debug_marker, false);
    dp("drivable_area_right_bound_offset", p.drivable_area_right_bound_offset, 0.0);
    dp("drivable_area_left_bound_offset", p.drivable_area_left_bound_offset, 0.0);

    // validation of parameters
    if (p.lane_change_sampling_num < 1) {
      RCLCPP_FATAL_STREAM(
        logger_, "lane_change_sampling_num must be positive integer. Given parameter: "
                   << p.lane_change_sampling_num << std::endl
                   << "Terminating the program...");
      exit(EXIT_FAILURE);
    }
    if (p.maximum_deceleration < 0.0) {
      RCLCPP_FATAL_STREAM(
        logger_, "maximum_deceleration cannot be negative value. Given parameter: "
                   << p.maximum_deceleration << std::endl
                   << "Terminating the program...");
      exit(EXIT_FAILURE);
    }
  }

  module_params_ = std::make_shared<AvoidanceByLCParameters>(p);
}
}  // namespace behavior_path_planner
