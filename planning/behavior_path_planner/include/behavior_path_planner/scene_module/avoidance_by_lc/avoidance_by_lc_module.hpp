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
  void acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const override;

  void updateModuleParams(const std::shared_ptr<AvoidanceByLCParameters> & parameters)
  {
    parameters_ = parameters;
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
  std::shared_ptr<AvoidanceByLCParameters> parameters_;
  LaneChangeStatus status_;
  PathShifter path_shifter_;
  mutable LaneChangeDebugMsgArray lane_change_debug_msg_array_;
  LaneChangeStates current_lane_change_state_;
  std::shared_ptr<LaneChangePath> abort_path_;
  PathWithLaneId prev_approved_path_;

  double lane_change_lane_length_{200.0};
  double check_distance_{100.0};
  bool is_abort_path_approved_{false};
  bool is_abort_approval_requested_{false};
  bool is_abort_condition_satisfied_{false};
  bool is_activated_ = false;

  std::shared_ptr<RTCInterface> rtc_interface_left_;
  std::shared_ptr<RTCInterface> rtc_interface_right_;
  UUID uuid_left_;
  UUID uuid_right_;

  void resetParameters();

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

  void removePreviousRTCStatusLeft()
  {
    if (rtc_interface_left_->isRegistered(uuid_left_)) {
      rtc_interface_left_->removeCooperateStatus(uuid_left_);
    }
  }

  void removePreviousRTCStatusRight()
  {
    if (rtc_interface_right_->isRegistered(uuid_right_)) {
      rtc_interface_right_->removeCooperateStatus(uuid_right_);
    }
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

  lanelet::ConstLanelets get_original_lanes() const;
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
  bool isApprovedPathSafe(Pose & ego_pose_before_collision) const;
  void calcTurnSignalInfo();

  void updateSteeringFactorPtr(
    const CandidateOutput & output, const LaneChangePath & selected_path) const;
  bool isSafe() const;
  bool isValidPath() const;
  bool isValidPath(const PathWithLaneId & path) const;
  bool isNearEndOfLane() const;
  bool isCurrentSpeedLow() const;
  bool isAbortConditionSatisfied();
  bool hasFinishedLaneChange() const;
  bool isAvoidancePlanRunning() const;
  bool isAbortState() const;

  // getter
  Point getEgoPosition() const { return planner_data_->self_odometry->pose.pose.position; }
  Pose getEgoPose() const { return planner_data_->self_odometry->pose.pose; }
  Twist getEgoTwist() const;
  std_msgs::msg::Header getRouteHeader() const;
  void resetPathIfAbort();

  // debug
  void setObjectDebugVisualization() const;
  mutable std::unordered_map<std::string, CollisionCheckDebug> object_debug_;
  mutable std::vector<LaneChangePath> debug_valid_path_;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE_BY_LC__AVOIDANCE_BY_LC_MODULE_HPP_
