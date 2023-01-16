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
#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE_BY_LC__AVOIDANCE_BY_LC_MODULE_DATA_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE_BY_LC__AVOIDANCE_BY_LC_MODULE_DATA_HPP_

#include "behavior_path_planner/scene_module/lane_change/lane_change_path.hpp"
#include "lanelet2_core/geometry/Lanelet.h"

#include "autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp"

#include <vector>

namespace behavior_path_planner
{

struct AvoidanceByLCParameters
{
  // avoidance

  // path resample interval for avoidance planning path.
  double resample_interval_for_planning = 0.3;

  // path resample interval for output path. Too short interval increases
  // computational cost for latter modules.
  double resample_interval_for_output = 3.0;

  // lanelet expand length for right side to find avoidance target vehicles
  double detection_area_right_expand_dist = 0.0;

  // lanelet expand length for left side to find avoidance target vehicles
  double detection_area_left_expand_dist = 1.0;

  // enable avoidance to be perform only in lane with same direction
  bool enable_avoidance_over_same_direction{true};

  // enable avoidance to be perform in opposite lane direction
  // to use this, enable_avoidance_over_same_direction need to be set to true.
  bool enable_avoidance_over_opposite_direction{true};

  // enable avoidance for all parking vehicle
  bool enable_avoidance_all_parking_vehicle{false};

  // enable abort maneuver
  bool enable_avoidance_yield{false};

  // enable slow down
  bool enable_slow_down{false};

  // constrains
  bool hard_constraints{false};

  // yield velocity
  double yield_velocity;

  // yield max deceleration
  double max_deceleration;

  // yield max jerk
  double max_jerk;

  // max deceleration
  double nominal_deceleration;

  // max jerk
  double nominal_jerk;

  // Vehicles whose distance to the center of the path is
  // less than this will not be considered for avoidance.
  double threshold_distance_object_is_on_center;

  // vehicles with speed greater than this will not be avoided
  double threshold_speed_object_is_stopped;

  // vehicles which is moving more than this parameter will not be avoided
  double threshold_time_object_is_moving;

  // force avoidance
  double threshold_time_force_avoidance;

  // force avoidance
  double threshold_intersection_force_avoidance;

  // distance to avoid object detection
  double object_check_forward_distance;

  // continue to detect backward vehicles as avoidance targets until they are this distance away
  double object_check_backward_distance;

  // find adjacent lane vehicles
  double adjacent_lane_check_backward_distance;

  // overhang threshold to judge whether object merges ego lane
  double object_check_overhang;

  // yaw threshold to detect merging vehicles
  double object_check_yaw;

  // ratio between object width and distance to shoulder
  double object_check_road_shoulder_ratio;

  // we want to keep this lateral margin when avoiding
  // double lateral_collision_margin;

  // we must keep this lateral margin when avoiding
  // double minimum_collision_margin;

  // a buffer in case lateral_collision_margin is set to 0. Will throw error
  // don't ever set this value to 0
  double lateral_collision_safety_buffer{0.5};

  double lateral_passable_safety_buffer{0.5};

  // margin between object back and end point of avoid shift point
  double longitudinal_collision_safety_buffer_front;

  // margin between object front and start point of return shift point
  double longitudinal_collision_safety_buffer_back;

  // when complete avoidance motion, there is a distance margin with the object
  // for longitudinal direction
  double longitudinal_collision_margin_min_distance;

  // when complete avoidance motion, there is a time margin with the object
  // for longitudinal direction
  double longitudinal_collision_margin_time;

  double safety_check_hysteresis_factor;

  // start avoidance after this time to avoid sudden path change
  double prepare_time;

  // Even if the vehicle speed is zero, avoidance will start after a distance of this much.
  double min_prepare_distance;

  // minimum distance while avoiding TODO(Horibe): will be changed to jerk constraint later
  double min_avoidance_distance;

  // minimum stop distance
  double stop_min_distance;

  // maximum stop distance
  double stop_max_distance;

  // minimum longitudinal margin for vehicles in adjacent lane
  double min_longitudinal_margin_for_moving_object;

  // minimum speed for jerk calculation in a nominal situation, i.e. there is an enough
  // distance for avoidance, and the object is very far from ego. In that case, the
  // vehicle speed is unknown passing along the object. Then use this speed as a minimum.
  // Note: This parameter is needed because we have to plan an avoidance path in advance
  //       without knowing the speed of the distant path.
  double min_nominal_avoidance_speed;

  // avoidance speed step for safety velocity profile searchin g
  float avoidance_speed_step;

  // minimum speed for jerk calculation in a tight situation, i.e. there is NOT an enough
  // distance for avoidance. Need a sharp avoidance path to avoid the object.
  double min_sharp_avoidance_speed;

  // The margin is configured so that the generated avoidance trajectory does not come near to the
  // road shoulder.
  double road_shoulder_safety_margin{1.0};

  // Even if the obstacle is very large, it will not avoid more than this length for right direction
  double max_right_shift_length;

  // Even if the obstacle is very large, it will not avoid more than this length for left direction
  double max_left_shift_length;

  // Avoidance path is generated with this jerk.
  // If there is no margin, the jerk increases up to max lateral jerk.
  double nominal_lateral_jerk;

  // if the avoidance path exceeds this lateral jerk, it will be not used anymore.
  double max_lateral_jerk;

  // For the compensation of the detection lost. Once an object is observed, it is registered and
  // will be used for planning from the next time. If the object is not observed, it counts up the
  // lost_count and the registered object will be removed when the count exceeds this max count.
  double object_last_seen_threshold;

  // For object's enveloped polygon
  double object_envelope_buffer;

  // For velocity planning to avoid acceleration during avoidance.
  // Speeds smaller than this are not inserted.
  double min_avoidance_speed_for_acc_prevention;

  // To prevent large acceleration while avoidance. The max velocity is limited with this
  // acceleration.
  double max_avoidance_acceleration;

  // The avoidance path generation is performed when the shift distance of the
  // avoidance points is greater than this threshold.
  // In multiple targets case: if there are multiple vehicles in a row to be avoided, no new
  // avoidance path will be generated unless their lateral margin difference exceeds this value.
  double avoidance_execution_lateral_threshold;

  // minimum road shoulder width. maybe 0.5 [m]
  double minimum_road_shoulder_width;

  double min_executable_distance;

  // target velocity matrix
  std::vector<double> target_velocity_matrix;

  // matrix col size
  size_t col_size;

  // true by default
  bool avoid_car{true};      // avoidance is performed for type object car
  bool avoid_truck{true};    // avoidance is performed for type object truck
  bool avoid_bus{true};      // avoidance is performed for type object bus
  bool avoid_trailer{true};  // avoidance is performed for type object trailer

  // false by default
  bool avoid_unknown{false};     // avoidance is performed for type object unknown
  bool avoid_bicycle{false};     // avoidance is performed for type object bicycle
  bool avoid_motorcycle{false};  // avoidance is performed for type object motorbike
  bool avoid_pedestrian{false};  // avoidance is performed for type object pedestrian

  // debug
  bool publish_debug_marker = false;
  bool print_debug_info = false;
  bool print_processing_time = false;

  // lane change

  double min_stop_distance;
  double stop_time;
  double hysteresis_buffer_distance;
  double lane_change_prepare_duration;
  double lane_changing_duration;
  double lane_change_finish_judge_buffer;
  double minimum_lane_change_velocity;
  double prediction_duration;
  double prediction_time_resolution;
  double static_obstacle_velocity_thresh;
  double maximum_deceleration;
  int lane_change_sampling_num;
  double abort_lane_change_velocity_thresh;
  double abort_lane_change_angle_thresh;
  double abort_lane_change_distance_thresh;
  bool enable_abort_lane_change;
  bool enable_collision_check_at_prepare_phase;
  bool use_predicted_path_outside_lanelet;
  bool use_all_predicted_path;
  bool enable_blocked_by_obstacle;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE_BY_LC__AVOIDANCE_BY_LC_MODULE_DATA_HPP_
