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

  // enable update path when if detected objects on planner data is gone.
  bool enable_update_path_when_object_is_gone{false};

  // enable safety check. if avoidance path is NOT safe, the ego will execute yield maneuver
  bool enable_safety_check{false};

  // Vehicles whose distance to the center of the path is
  // less than this will not be considered for avoidance.
  double threshold_distance_object_is_on_center;

  // vehicles with speed greater than this will not be avoided
  double threshold_speed_object_is_stopped;

  // distance to avoid object detection
  double object_check_forward_distance;

  // continue to detect backward vehicles as avoidance targets until they are this distance away
  double object_check_backward_distance;

  // use in judge whether the vehicle is parking object on road shoulder
  double object_check_shiftable_ratio;

  // minimum road shoulder width. maybe 0.5 [m]
  double object_check_min_road_shoulder_width;

  // object's enveloped polygon
  double object_envelope_buffer;

  // vehicles which is moving more than this parameter will not be avoided
  double threshold_time_object_is_moving;

  // For the compensation of the detection lost. Once an object is observed, it is registered and
  // will be used for planning from the next time. If the object is not observed, it counts up the
  // lost_count and the registered object will be removed when the count exceeds this max count.
  double object_last_seen_threshold;

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

  // clip left and right bounds for objects
  bool enable_bound_clipping{false};

  // lane change
  double lane_change_prepare_duration{2.0};
  double lane_changing_safety_check_duration{4.0};
  double lane_changing_lateral_jerk{0.5};
  double lane_changing_lateral_acc{0.5};
  double lane_change_finish_judge_buffer{3.0};
  double minimum_lane_change_velocity{5.6};
  double prediction_time_resolution{0.5};
  double maximum_deceleration{1.0};
  int lane_change_sampling_num{10};
  double abort_lane_change_velocity_thresh{0.5};
  double abort_lane_change_angle_thresh{0.174533};
  double abort_lane_change_distance_thresh{0.3};
  double prepare_phase_ignore_target_speed_thresh{0.1};
  bool enable_abort_lane_change{true};
  bool enable_collision_check_at_prepare_phase{true};
  bool use_predicted_path_outside_lanelet{false};
  bool use_all_predicted_path{false};
  bool publish_debug_marker{false};
  // drivable area expansion
  double drivable_area_right_bound_offset{0.0};
  double drivable_area_left_bound_offset{0.0};
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE_BY_LC__AVOIDANCE_BY_LC_MODULE_DATA_HPP_
