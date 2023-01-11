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
  rclcpp::Node * node, const std::string & name, const size_t max_module_num, const size_t priority)
: SceneModuleManagerInterface(node, name, max_module_num, priority)
{
  rtc_interface_left_ = std::make_shared<RTCInterface>(node, name + "_left");
  rtc_interface_right_ = std::make_shared<RTCInterface>(node, name + "_right");
  getModuleParams(node);
}

void LaneChangeModuleManager::updateModuleParams(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto p = module_params_;

  std::string ns = name_ + ".";
  updateParam<bool>(parameters, ns + "publish_debug_marker", p->publish_debug_marker);

  std::for_each(registered_modules_.begin(), registered_modules_.end(), [&p](const auto & m) {
    m.second->updateModuleParams(p);
  });
}

void LaneChangeModuleManager::getModuleParams(rclcpp::Node * node)
{
  using tier4_autoware_utils::deg2rad;

  const auto dp = [this, &node](const std::string & str, auto & value, auto def_val) {
    std::string name = name_ + "." + str;

    if (node->has_parameter(name)) {
      node->get_parameter<decltype(def_val)>(name, value);
    } else {
      value = node->declare_parameter(name, def_val);
    }
  };

  LaneChangeParameters p{};
  dp("lane_change_prepare_duration", p.lane_change_prepare_duration, 2.0);
  dp("lane_changing_safety_check_duration", p.lane_changing_safety_check_duration, 4.0);
  dp("lane_changing_lateral_jerk", p.lane_changing_lateral_jerk, 0.5);
  dp("lane_changing_lateral_acc", p.lane_changing_lateral_acc, 0.5);
  dp("lane_change_finish_judge_buffer", p.lane_change_finish_judge_buffer, 3.0);
  dp("minimum_lane_change_velocity", p.minimum_lane_change_velocity, 5.6);
  dp("prediction_time_resolution", p.prediction_time_resolution, 0.5);
  dp("maximum_deceleration", p.maximum_deceleration, 1.0);
  dp("lane_change_sampling_num", p.lane_change_sampling_num, 10);
  dp("abort_lane_change_velocity_thresh", p.abort_lane_change_velocity_thresh, 0.5);
  dp("abort_lane_change_angle_thresh", p.abort_lane_change_angle_thresh, deg2rad(10.0));
  dp("abort_lane_change_distance_thresh", p.abort_lane_change_distance_thresh, 0.3);
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

  module_params_ = std::make_shared<LaneChangeParameters>(p);
}
}  // namespace behavior_path_planner
