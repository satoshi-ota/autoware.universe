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

#include "behavior_path_planner/scene_module/lane_change/manager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

LaneChangeModuleManager::LaneChangeModuleManager(
  rclcpp::Node * node, const std::string & name, const size_t max_module_num, const size_t priority,
  const bool enable_simultaneous_execution)
: SceneModuleManagerInterface(node, name, max_module_num, priority, enable_simultaneous_execution)
{
  rtc_interface_left_ = std::make_shared<RTCInterface>(node, name + "_left");
  rtc_interface_right_ = std::make_shared<RTCInterface>(node, name + "_right");
  getModuleParams(node);
}

void LaneChangeModuleManager::updateModuleParams(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto p = parameters_;

  std::string ns = name_ + ".";
  updateParam<bool>(parameters, ns + "publish_debug_marker", p->publish_debug_marker);

  std::for_each(registered_modules_.begin(), registered_modules_.end(), [&p](const auto & m) {
    m.second->updateModuleParams(p);
  });
}

void LaneChangeModuleManager::getModuleParams(rclcpp::Node * node)
{
  const auto dp = [this, &node](const std::string & str, auto def_val) {
    std::string name = name_ + "." + str;
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
  p.prepare_phase_ignore_target_speed_thresh = dp("prepare_phase_ignore_target_speed_thresh", 0.1);
  p.use_predicted_path_outside_lanelet = dp("use_predicted_path_outside_lanelet", true);
  p.use_all_predicted_path = dp("use_all_predicted_path", true);

  // abort
  p.enable_cancel_lane_change = dp("enable_cancel_lane_change", true);
  p.enable_abort_lane_change = dp("enable_abort_lane_change", false);

  p.abort_delta_time = dp("abort_delta_time", 3.0);
  p.abort_max_lateral_jerk = dp("abort_max_lateral_jerk", 10.0);

  // drivable area expansion
  p.drivable_area_right_bound_offset = dp("drivable_area_right_bound_offset", 0.0);
  p.drivable_area_left_bound_offset = dp("drivable_area_left_bound_offset", 0.0);
  p.drivable_area_types_to_skip =
    dp("drivable_area_types_to_skip", std::vector<std::string>({"road_border"}));

  // debug marker
  p.publish_debug_marker = dp("publish_debug_marker", false);

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

  parameters_ = std::make_shared<LaneChangeParameters>(p);
}
}  // namespace behavior_path_planner
