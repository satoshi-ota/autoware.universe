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

#include "behavior_path_planner/scene_module/pull_over/manager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

PullOverModuleManager::PullOverModuleManager(
  rclcpp::Node * node, const std::string & name, const size_t max_module_num)
: SceneModuleManagerInterface(node, name, max_module_num)
{
  rtc_interface_ = std::make_shared<RTCInterface>(node, name);
  getModuleParams(node);
}

void PullOverModuleManager::updateModuleParams(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto p = module_params_;

  std::string ns = name_ + ".";
  updateParam<bool>(parameters, ns + "print_debug_info", p.print_debug_info);

  std::for_each(registered_modules_.begin(), registered_modules_.end(), [&p](const auto & m) {
    m.second->updateModuleParams(p);
  });
}

void PullOverModuleManager::getModuleParams(rclcpp::Node * node)
{
  const auto dp = [this, &node](const std::string & str, auto def_val) {
    std::string name = name_ + "." + str;
    return node->declare_parameter(name, def_val);
  };

  auto & p = module_params_;

  p.request_length = dp("request_length", 200.0);
  p.th_stopped_velocity = dp("th_stopped_velocity", 0.01);
  p.th_arrived_distance = dp("th_arrived_distance", 0.3);
  p.th_stopped_time = dp("th_stopped_time", 2.0);
  p.margin_from_boundary = dp("margin_from_boundary", 0.3);
  p.decide_path_distance = dp("decide_path_distance", 10.0);
  p.maximum_deceleration = dp("maximum_deceleration", 1.0);
  // goal research
  p.enable_goal_research = dp("enable_goal_research", true);
  p.search_priority = dp("search_priority", "efficient_path");
  p.forward_goal_search_length = dp("forward_goal_search_length", 20.0);
  p.backward_goal_search_length = dp("backward_goal_search_length", 20.0);
  p.goal_search_interval = dp("goal_search_interval", 5.0);
  p.longitudinal_margin = dp("longitudinal_margin", 3.0);
  p.max_lateral_offset = dp("max_lateral_offset", 1.0);
  p.lateral_offset_interval = dp("lateral_offset_interval", 0.25);
  // occupancy grid map
  p.use_occupancy_grid = dp("use_occupancy_grid", true);
  p.use_occupancy_grid_for_longitudinal_margin =
    dp("use_occupancy_grid_for_longitudinal_margin", false);
  p.occupancy_grid_collision_check_margin = dp("occupancy_grid_collision_check_margin", 0.0);
  p.theta_size = dp("theta_size", 360);
  p.obstacle_threshold = dp("obstacle_threshold", 90);
  // object recognition
  p.use_object_recognition = dp("use_object_recognition", true);
  p.object_recognition_collision_check_margin =
    dp("object_recognition_collision_check_margin", 1.0);
  // shift path
  p.enable_shift_parking = dp("enable_shift_parking", true);
  p.pull_over_sampling_num = dp("pull_over_sampling_num", 4);
  p.maximum_lateral_jerk = dp("maximum_lateral_jerk", 3.0);
  p.minimum_lateral_jerk = dp("minimum_lateral_jerk", 1.0);
  p.deceleration_interval = dp("deceleration_interval", 10.0);
  p.pull_over_velocity = dp("pull_over_velocity", 8.3);
  p.pull_over_minimum_velocity = dp("pull_over_minimum_velocity", 0.3);
  p.after_pull_over_straight_distance = dp("after_pull_over_straight_distance", 3.0);
  // parallel parking
  p.enable_arc_forward_parking = dp("enable_arc_forward_parking", true);
  p.enable_arc_backward_parking = dp("enable_arc_backward_parking", true);
  p.after_forward_parking_straight_distance = dp("after_forward_parking_straight_distance", 0.5);
  p.after_backward_parking_straight_distance = dp("after_backward_parking_straight_distance", 0.5);
  p.forward_parking_velocity = dp("forward_parking_velocity", 1.0);
  p.backward_parking_velocity = dp("backward_parking_velocity", -0.5);
  p.forward_parking_lane_departure_margin = dp("forward_parking_lane_departure_margin", 0.0);
  p.backward_parking_lane_departure_margin = dp("backward_parking_lane_departure_margin", 0.0);
  p.arc_path_interval = dp("arc_path_interval", 1.0);
  p.pull_over_max_steer_angle = dp("pull_over_max_steer_angle", 0.35);  // 20deg
  // hazard
  p.hazard_on_threshold_distance = dp("hazard_on_threshold_distance", 1.0);
  p.hazard_on_threshold_velocity = dp("hazard_on_threshold_velocity", 0.5);
  // safety with dynamic objects. Not used now.
  p.pull_over_duration = dp("pull_over_duration", 4.0);
  p.pull_over_prepare_duration = dp("pull_over_prepare_duration", 2.0);
  p.min_stop_distance = dp("min_stop_distance", 5.0);
  p.stop_time = dp("stop_time", 2.0);
  p.hysteresis_buffer_distance = dp("hysteresis_buffer_distance", 2.0);
  p.prediction_time_resolution = dp("prediction_time_resolution", 0.5);
  p.enable_collision_check_at_prepare_phase = dp("enable_collision_check_at_prepare_phase", true);
  p.use_predicted_path_outside_lanelet = dp("use_predicted_path_outside_lanelet", true);
  p.use_all_predicted_path = dp("use_all_predicted_path", false);
  // drivable area
  p.drivable_area_right_bound_offset = dp("drivable_area_right_bound_offset", 0.0);
  p.drivable_area_left_bound_offset = dp("drivable_area_left_bound_offset", 0.0);
  // debug
  p.print_debug_info = dp("print_debug_info", false);

  // validation of parameters
  if (p.pull_over_sampling_num < 1) {
    RCLCPP_FATAL_STREAM(
      logger_, "pull_over_sampling_num must be positive integer. Given parameter: "
                 << p.pull_over_sampling_num << std::endl
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
}  // namespace behavior_path_planner
