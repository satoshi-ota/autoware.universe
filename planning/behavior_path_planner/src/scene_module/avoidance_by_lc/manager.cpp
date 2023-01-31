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
  rclcpp::Node * node, const std::string & name, const size_t max_module_num, const size_t priority,
  const bool enable_simultaneous_execution)
: SceneModuleManagerInterface(node, name, max_module_num, priority, enable_simultaneous_execution)
{
  rtc_interface_left_ = std::make_shared<RTCInterface>(node, name + "_left");
  rtc_interface_right_ = std::make_shared<RTCInterface>(node, name + "_right");
  getModuleParams(node);
}

void AvoidanceByLCModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto p = parameters_;

  [[maybe_unused]] std::string ns = name_ + ".";

  std::for_each(registered_modules_.begin(), registered_modules_.end(), [&p](const auto & m) {
    m.second->updateModuleParams(p);
  });
}

void AvoidanceByLCModuleManager::getModuleParams(rclcpp::Node * node)
{
  parameters_ = std::make_shared<AvoidanceByLCParameters>();

  // avoidance parameters
  {
    AvoidanceParameters p{};
    // general params
    {
      std::string ns = "avoidance_by_lc.";
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
      std::string ns = "avoidance_by_lc.target_object.";
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
      std::string ns = "avoidance_by_lc.target_filtering.";
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
      std::string ns = "avoidance_by_lc.safety_check.";
      p.safety_check_backward_distance =
        node->declare_parameter<double>(ns + "safety_check_backward_distance");
      p.safety_check_time_horizon =
        node->declare_parameter<double>(ns + "safety_check_time_horizon");
      p.safety_check_idling_time = node->declare_parameter<double>(ns + "safety_check_idling_time");
      p.safety_check_accel_for_rss =
        node->declare_parameter<double>(ns + "safety_check_accel_for_rss");
      p.safety_check_hysteresis_factor =
        node->declare_parameter<double>(ns + "safety_check_hysteresis_factor");
    }

    // avoidance maneuver (lateral)
    {
      std::string ns = "avoidance_by_lc.avoidance.lateral.";
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
      std::string ns = "avoidance_by_lc.avoidance.longitudinal.";
      p.prepare_time = node->declare_parameter<double>(ns + "prepare_time");
      p.longitudinal_collision_safety_buffer =
        node->declare_parameter<double>(ns + "longitudinal_collision_safety_buffer");
      p.min_prepare_distance = node->declare_parameter<double>(ns + "min_prepare_distance");
      p.min_avoidance_distance = node->declare_parameter<double>(ns + "min_avoidance_distance");
      p.min_nominal_avoidance_speed =
        node->declare_parameter<double>(ns + "min_nominal_avoidance_speed");
      p.min_sharp_avoidance_speed =
        node->declare_parameter<double>(ns + "min_sharp_avoidance_speed");
    }

    // yield
    {
      std::string ns = "avoidance_by_lc.yield.";
      p.yield_velocity = node->declare_parameter<double>(ns + "yield_velocity");
    }

    // stop
    {
      std::string ns = "avoidance_by_lc.stop.";
      p.stop_min_distance = node->declare_parameter<double>(ns + "min_distance");
      p.stop_max_distance = node->declare_parameter<double>(ns + "max_distance");
    }

    // constraints
    {
      std::string ns = "avoidance_by_lc.constraints.";
      p.use_constraints_for_decel = node->declare_parameter<bool>(ns + "use_constraints_for_decel");
    }

    // constraints (longitudinal)
    {
      std::string ns = "avoidance_by_lc.constraints.longitudinal.";
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
      std::string ns = "avoidance_by_lc.constraints.lateral.";
      p.nominal_lateral_jerk = node->declare_parameter<double>(ns + "nominal_lateral_jerk");
      p.max_lateral_jerk = node->declare_parameter<double>(ns + "max_lateral_jerk");
    }

    // velocity matrix
    {
      std::string ns = "avoidance_by_lc.target_velocity_matrix.";
      p.col_size = node->declare_parameter<int>(ns + "col_size");
      p.target_velocity_matrix = node->declare_parameter<std::vector<double>>(ns + "matrix");
    }

    parameters_->avoidance = std::make_shared<AvoidanceParameters>(p);
  }

  // lane change parameters
  {
    const auto dp = [this, &node](const std::string & str, auto def_val) {
      std::string name = "avoidance_by_lc." + str;
      return node->declare_parameter(name, def_val);
    };

    LaneChangeParameters p{};

    // trajectory generation
    p.lane_change_prepare_duration = dp("lane_change_prepare_duration", 2.0);
    p.lane_changing_safety_check_duration = dp("lane_changing_safety_check_duration", 4.0);
    p.lane_changing_lateral_jerk = dp("lane_changing_lateral_jerk", 0.5);
    p.lane_changing_lateral_acc = dp("lane_changing_lateral_acc", 0.5);
    p.lane_change_finish_judge_buffer = dp("lane_change_finish_judge_buffer", 3.0);
    p.minimum_lane_change_velocity = dp("minimum_lane_change_velocity", 5.6);
    p.prediction_time_resolution = dp("prediction_time_resolution", 0.5);
    p.maximum_deceleration = dp("maximum_deceleration", 1.0);
    p.lane_change_sampling_num = dp("lane_change_sampling_num", 10);

    // collision check
    p.enable_collision_check_at_prepare_phase = dp("enable_collision_check_at_prepare_phase", true);
    p.prepare_phase_ignore_target_speed_thresh =
      dp("prepare_phase_ignore_target_speed_thresh", 0.1);
    p.use_predicted_path_outside_lanelet = dp("use_predicted_path_outside_lanelet", true);
    p.use_all_predicted_path = dp("use_all_predicted_path", true);

    // abort
    p.enable_cancel_lane_change = dp("enable_cancel_lane_change", true);
    p.enable_abort_lane_change = dp("enable_abort_lane_change", false);

    p.abort_delta_time = dp("abort_delta_time", 3.0);
    p.abort_max_lateral_jerk = dp("abort_max_lateral_jerk", 10.0);

    // drivable area expansion
    p.drivable_area_right_bound_offset = parameters_->avoidance->drivable_area_right_bound_offset;
    p.drivable_area_left_bound_offset = parameters_->avoidance->drivable_area_left_bound_offset;
    p.drivable_area_types_to_skip = parameters_->avoidance->drivable_area_types_to_skip;

    // debug marker
    p.publish_debug_marker = parameters_->avoidance->publish_debug_marker;

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

    if (p.abort_delta_time < 1.0) {
      RCLCPP_FATAL_STREAM(
        logger_, "abort_delta_time: " << p.abort_delta_time << ", is too short.\n"
                                      << "Terminating the program...");
      exit(EXIT_FAILURE);
    }

    const auto lc_buffer =
      node->get_parameter("lane_change.backward_length_buffer_for_end_of_lane").get_value<double>();
    if (lc_buffer < p.lane_change_finish_judge_buffer + 1.0) {
      p.lane_change_finish_judge_buffer = lc_buffer - 1;
      RCLCPP_WARN_STREAM(
        logger_, "lane change buffer is less than finish buffer. Modifying the value to "
                   << p.lane_change_finish_judge_buffer << "....");
    }

    parameters_->lane_change = std::make_shared<LaneChangeParameters>(p);
  }
}
}  // namespace behavior_path_planner
