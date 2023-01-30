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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE_BY_LC__AVOIDANCE_BY_LC_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE_BY_LC__AVOIDANCE_BY_LC_MODULE_HPP_

#include "behavior_path_planner/scene_module/avoidance/avoidance_module_data.hpp"
#include "behavior_path_planner/scene_module/avoidance_by_lc/avoidance_by_lc_module_data.hpp"
#include "behavior_path_planner/scene_module/lane_change/debug.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_module_data.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_path.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"

#include <rclcpp/rclcpp.hpp>

#include "tier4_planning_msgs/msg/detail/lane_change_debug_msg_array__struct.hpp"
#include "tier4_planning_msgs/msg/lane_change_debug_msg.hpp"
#include "tier4_planning_msgs/msg/lane_change_debug_msg_array.hpp"
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2/utils.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using marker_utils::CollisionCheckDebug;
using tier4_planning_msgs::msg::LaneChangeDebugMsg;
using tier4_planning_msgs::msg::LaneChangeDebugMsgArray;

class AvoidanceByLCModule : public SceneModuleInterface
{
public:
  AvoidanceByLCModule(
    const std::string & name, rclcpp::Node & node,
    std::shared_ptr<AvoidanceByLCParameters> parameters,
    std::shared_ptr<RTCInterface> & rtc_interface_left,
    std::shared_ptr<RTCInterface> & rtc_interface_right);

  BehaviorModuleOutput run() override;

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  ModuleStatus updateState() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;
  void onEntry() override;
  void onExit() override;
  void updateData() override;

  std::shared_ptr<LaneChangeDebugMsgArray> get_debug_msg_array() const;
  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override;

  void updateModuleParams(const std::shared_ptr<AvoidanceByLCParameters> & parameters)
  {
    // avoidance parameters
    {
      AvoidanceParameters p{};

      p.resample_interval_for_planning = parameters->resample_interval_for_planning;
      p.resample_interval_for_output = parameters->resample_interval_for_output;
      p.detection_area_right_expand_dist = parameters->detection_area_right_expand_dist;
      p.detection_area_left_expand_dist = parameters->detection_area_left_expand_dist;
      p.enable_avoidance_over_same_direction = parameters->enable_avoidance_over_same_direction;
      p.enable_avoidance_over_opposite_direction =
        parameters->enable_avoidance_over_opposite_direction;
      p.enable_update_path_when_object_is_gone = parameters->enable_update_path_when_object_is_gone;
      p.enable_safety_check = parameters->enable_safety_check;

      p.threshold_distance_object_is_on_center = parameters->threshold_distance_object_is_on_center;
      p.threshold_speed_object_is_stopped = parameters->threshold_speed_object_is_stopped;
      p.threshold_time_object_is_moving = parameters->threshold_time_object_is_moving;
      p.object_check_forward_distance = parameters->object_check_forward_distance;
      p.object_check_backward_distance = parameters->object_check_backward_distance;
      p.object_check_shiftable_ratio = parameters->object_check_shiftable_ratio;
      p.object_check_min_road_shoulder_width = parameters->object_check_min_road_shoulder_width;
      p.object_envelope_buffer = parameters->object_envelope_buffer;
      p.object_last_seen_threshold = parameters->object_last_seen_threshold;

      p.avoid_car = parameters->avoid_car;
      p.avoid_truck = parameters->avoid_truck;
      p.avoid_bus = parameters->avoid_bus;
      p.avoid_trailer = parameters->avoid_trailer;
      p.avoid_unknown = parameters->avoid_unknown;
      p.avoid_bicycle = parameters->avoid_bicycle;
      p.avoid_motorcycle = parameters->avoid_motorcycle;
      p.avoid_pedestrian = parameters->avoid_pedestrian;

      parameters_avoidance_ = std::make_shared<AvoidanceParameters>(p);
    }

    // lane change parameters
    {
      LaneChangeParameters p{};

      p.lane_change_prepare_duration = parameters->lane_change_prepare_duration;
      p.lane_changing_safety_check_duration = parameters->lane_changing_safety_check_duration;
      p.lane_changing_lateral_jerk = parameters->lane_changing_lateral_jerk;
      p.lane_changing_lateral_acc = parameters->lane_changing_lateral_acc;
      p.lane_change_finish_judge_buffer = parameters->lane_change_finish_judge_buffer;
      p.minimum_lane_change_velocity = parameters->minimum_lane_change_velocity;
      p.prediction_time_resolution = parameters->prediction_time_resolution;
      p.maximum_deceleration = parameters->maximum_deceleration;
      p.lane_change_sampling_num = parameters->lane_change_sampling_num;
      // p.abort_lane_change_velocity_thresh = parameters->abort_lane_change_velocity_thresh;
      // p.abort_lane_change_angle_thresh = parameters->abort_lane_change_angle_thresh;
      // p.abort_lane_change_distance_thresh = parameters->abort_lane_change_distance_thresh;
      p.prepare_phase_ignore_target_speed_thresh =
        parameters->prepare_phase_ignore_target_speed_thresh;
      p.enable_abort_lane_change = parameters->enable_abort_lane_change;
      p.enable_collision_check_at_prepare_phase =
        parameters->enable_collision_check_at_prepare_phase;
      p.use_predicted_path_outside_lanelet = parameters->use_predicted_path_outside_lanelet;
      p.use_all_predicted_path = parameters->use_all_predicted_path;
      p.publish_debug_marker = parameters->publish_debug_marker;
      p.drivable_area_right_bound_offset = parameters->drivable_area_right_bound_offset;
      p.drivable_area_left_bound_offset = parameters->drivable_area_left_bound_offset;

      parameters_lane_change_ = std::make_shared<LaneChangeParameters>(p);
    }
  }

  void publishRTCStatus() override
  {
    rtc_interface_left_->publishCooperateStatus(clock_->now());
    rtc_interface_right_->publishCooperateStatus(clock_->now());
  }

  bool isActivated() override
  {
    if (rtc_interface_left_->isRegistered(uuid_left_)) {
      return rtc_interface_left_->isActivated(uuid_left_);
    }
    if (rtc_interface_right_->isRegistered(uuid_right_)) {
      return rtc_interface_right_->isActivated(uuid_right_);
    }
    return false;
  }

  void lockRTCCommand() override
  {
    rtc_interface_left_->lockCommandUpdate();
    rtc_interface_right_->lockCommandUpdate();
  }

  void unlockRTCCommand() override
  {
    rtc_interface_left_->unlockCommandUpdate();
    rtc_interface_right_->unlockCommandUpdate();
  }

private:
  double lane_change_lane_length_{200.0};
  double check_distance_{100.0};

  std::shared_ptr<AvoidanceParameters> parameters_avoidance_;
  std::shared_ptr<LaneChangeParameters> parameters_lane_change_;

  std::shared_ptr<RTCInterface> rtc_interface_left_;
  std::shared_ptr<RTCInterface> rtc_interface_right_;
  UUID uuid_left_;
  UUID uuid_right_;

  LaneChangeStatus status_;
  PathShifter path_shifter_;
  mutable LaneChangeDebugMsgArray lane_change_debug_msg_array_;

  bool is_activated_ = false;

  void waitApprovalLeft(const double start_distance, const double finish_distance)
  {
    rtc_interface_left_->updateCooperateStatus(
      uuid_left_, isExecutionReady(), start_distance, finish_distance, clock_->now());
    is_waiting_approval_ = true;
  }

  void waitApprovalRight(const double start_distance, const double finish_distance)
  {
    rtc_interface_right_->updateCooperateStatus(
      uuid_right_, isExecutionReady(), start_distance, finish_distance, clock_->now());
    is_waiting_approval_ = true;
  }

  void updateRTCStatus(const CandidateOutput & candidate)
  {
    if (candidate.lateral_shift > 0.0) {
      rtc_interface_left_->updateCooperateStatus(
        uuid_left_, isExecutionReady(), candidate.start_distance_to_path_change,
        candidate.finish_distance_to_path_change, clock_->now());
      return;
    }
    if (candidate.lateral_shift < 0.0) {
      rtc_interface_right_->updateCooperateStatus(
        uuid_right_, isExecutionReady(), candidate.start_distance_to_path_change,
        candidate.finish_distance_to_path_change, clock_->now());
      return;
    }

    RCLCPP_WARN_STREAM(
      getLogger(),
      "Direction is UNKNOWN, start_distance = " << candidate.start_distance_to_path_change);
  }

  void removeRTCStatus() override
  {
    rtc_interface_left_->clearCooperateStatus();
    rtc_interface_right_->clearCooperateStatus();
  }

  lanelet::ConstLanelets getCurrentLanes(const PathWithLaneId & path) const
  {
    const auto & route_handler = planner_data_->route_handler;
    const auto & common_parameters = planner_data_->parameters;

    lanelet::ConstLanelets reference_lanes{};
    for (const auto & p : path.points) {
      reference_lanes.push_back(
        planner_data_->route_handler->getLaneletsFromId(p.lane_ids.front()));
    }

    lanelet::ConstLanelet current_lane;
    lanelet::utils::query::getClosestLanelet(reference_lanes, getEgoPose(), &current_lane);

    return route_handler->getLaneletSequence(
      current_lane, getEgoPose(), common_parameters.backward_path_length,
      common_parameters.forward_path_length);
  }

  AvoidancePlanningData calcAvoidancePlanningData(DebugData & debug) const;
  AvoidancePlanningData avoidance_data_;
  mutable DebugData debug_data_;

  ObjectDataArray registered_objects_;
  mutable ObjectDataArray stopped_objects_;

  void fillAvoidanceTargetObjects(AvoidancePlanningData & data, DebugData & debug) const;
  void fillObjectEnvelopePolygon(const Pose & closest_pose, ObjectData & object_data) const;
  void fillObjectMovingTime(ObjectData & object_data) const;
  void updateRegisteredObject(const ObjectDataArray & objects);
  void compensateDetectionLost(
    ObjectDataArray & target_objects, ObjectDataArray & other_objects) const;
  bool isTargetObjectType(const PredictedObject & object) const;

  PathWithLaneId getReferencePath() const;
  lanelet::ConstLanelets getLaneChangeLanes(
    const lanelet::ConstLanelets & current_lanes, const double lane_change_lane_length) const;
  std::pair<bool, bool> getSafePath(
    const lanelet::ConstLanelets & lane_change_lanes, const double check_distance,
    LaneChangePath & safe_path) const;

  void updateLaneChangeStatus();
  void generateExtendedDrivableArea(PathWithLaneId & path);
  void updateOutputTurnSignal(BehaviorModuleOutput & output);
  void updateSteeringFactorPtr(const BehaviorModuleOutput & output);

  void updateSteeringFactorPtr(
    const CandidateOutput & output, const LaneChangePath & selected_path) const;
  bool isSafe() const;
  bool isValidPath(const PathWithLaneId & path) const;
  bool isNearEndOfLane() const;
  bool isCurrentSpeedLow() const;
  bool isAbortConditionSatisfied();
  bool hasFinishedLaneChange() const;
  void resetParameters();

  // getter
  Point getEgoPosition() const { return planner_data_->self_odometry->pose.pose.position; }

  Pose getEgoPose() const { return planner_data_->self_odometry->pose.pose; }

  Twist getEgoTwist() const;
  std_msgs::msg::Header getRouteHeader() const;

  // debug
  mutable std::unordered_map<std::string, CollisionCheckDebug> object_debug_;
  mutable std::vector<LaneChangePath> debug_valid_path_;

  void setObjectDebugVisualization() const;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE_BY_LC__AVOIDANCE_BY_LC_MODULE_HPP_
