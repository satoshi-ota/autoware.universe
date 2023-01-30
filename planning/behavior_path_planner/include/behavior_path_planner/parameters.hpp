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

#ifndef BEHAVIOR_PATH_PLANNER__PARAMETERS_HPP_
#define BEHAVIOR_PATH_PLANNER__PARAMETERS_HPP_

#include <vehicle_info_util/vehicle_info_util.hpp>

struct BehaviorPathPlannerParameters
{
  bool launch_avoidance_by_lc{false};
  bool launch_avoidance{false};
  bool launch_lane_change{false};
  bool launch_external_lane_change{false};
  bool launch_pull_out{false};
  bool launch_pull_over{false};
  bool launch_side_shift{false};

  size_t priority_avoidance_by_lc;
  size_t priority_avoidance;
  size_t priority_lane_change;
  size_t priority_extrenal_lane_change;
  size_t priority_pull_out;
  size_t priority_pull_over;

  bool enable_simultaneous_execution_avoidance_by_lc{false};
  bool enable_simultaneous_execution_avoidance{false};
  bool enable_simultaneous_execution_lane_change{false};
  bool enable_simultaneous_execution_external_lane_change{false};
  bool enable_simultaneous_execution_side_shift{false};
  bool enable_simultaneous_execution_pull_over{false};
  bool enable_simultaneous_execution_pull_out{false};

  bool verbose{false};

  double backward_path_length;
  double forward_path_length;
  double backward_length_buffer_for_end_of_lane;
  double backward_length_buffer_for_end_of_pull_over;
  double backward_length_buffer_for_end_of_pull_out;
  double minimum_lane_change_length;
  double minimum_lane_change_prepare_distance;

  double minimum_pull_over_length;
  double minimum_pull_out_length;
  double drivable_area_resolution;

  double refine_goal_search_radius_range;

  double turn_signal_intersection_search_distance;
  double turn_signal_intersection_angle_threshold_deg;
  double turn_signal_search_time;
  double turn_signal_minimum_search_distance;
  double turn_signal_shift_length_threshold;
  bool turn_signal_on_swerving;

  double path_interval;

  double ego_nearest_dist_threshold;
  double ego_nearest_yaw_threshold;

  // vehicle info
  vehicle_info_util::VehicleInfo vehicle_info;
  double wheel_base;
  double front_overhang;
  double rear_overhang;
  double vehicle_width;
  double vehicle_length;
  double wheel_tread;
  double left_over_hang;
  double right_over_hang;
  double base_link2front;
  double base_link2rear;

  // maximum drivable area visualization
  bool visualize_maximum_drivable_area;

  // collision check
  double lateral_distance_max_threshold;
  double longitudinal_distance_min_threshold;

  double expected_front_deceleration;  // brake parameter under normal lane change
  double expected_rear_deceleration;   // brake parameter under normal lane change

  double expected_front_deceleration_for_abort;  // hard brake parameter for abort
  double expected_rear_deceleration_for_abort;   // hard brake parameter for abort

  double rear_vehicle_reaction_time;
  double rear_vehicle_safety_time_margin;
};

#endif  // BEHAVIOR_PATH_PLANNER__PARAMETERS_HPP_
