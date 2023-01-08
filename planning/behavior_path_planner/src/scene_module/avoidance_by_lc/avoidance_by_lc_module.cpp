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

#include "behavior_path_planner/scene_module/avoidance_by_lc/avoidance_by_lc_module.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_utils.hpp"
#include "behavior_path_planner/scene_module/lane_change/util.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include "tier4_planning_msgs/msg/detail/lane_change_debug_msg_array__struct.hpp"
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{

using autoware_auto_perception_msgs::msg::ObjectClassification;
using motion_utils::calcSignedArcLength;
using motion_utils::findNearestIndex;
using motion_utils::findNearestSegmentIndex;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcLateralDeviation;

AvoidanceByLCModule::AvoidanceByLCModule(
  const std::string & name, rclcpp::Node & node,
  std::shared_ptr<AvoidanceByLCParameters> parameters,
  std::shared_ptr<RTCInterface> & rtc_interface_left,
  std::shared_ptr<RTCInterface> & rtc_interface_right)
: SceneModuleInterface{name, node},
  rtc_interface_left_{rtc_interface_left},
  rtc_interface_right_{rtc_interface_right},
  uuid_left_{generateUUID()},
  uuid_right_{generateUUID()}
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
    p.abort_lane_change_velocity_thresh = parameters->abort_lane_change_velocity_thresh;
    p.abort_lane_change_angle_thresh = parameters->abort_lane_change_angle_thresh;
    p.abort_lane_change_distance_thresh = parameters->abort_lane_change_distance_thresh;
    p.prepare_phase_ignore_target_speed_thresh =
      parameters->prepare_phase_ignore_target_speed_thresh;
    p.enable_abort_lane_change = parameters->enable_abort_lane_change;
    p.enable_collision_check_at_prepare_phase = parameters->enable_collision_check_at_prepare_phase;
    p.use_predicted_path_outside_lanelet = parameters->use_predicted_path_outside_lanelet;
    p.use_all_predicted_path = parameters->use_all_predicted_path;
    p.publish_debug_marker = parameters->publish_debug_marker;
    p.drivable_area_right_bound_offset = parameters->drivable_area_right_bound_offset;
    p.drivable_area_left_bound_offset = parameters->drivable_area_left_bound_offset;

    parameters_lane_change_ = std::make_shared<LaneChangeParameters>(p);
  }
}

BehaviorModuleOutput AvoidanceByLCModule::run()
{
  RCLCPP_DEBUG(getLogger(), "Was waiting approval, and now approved. Do plan().");
  current_state_ = ModuleStatus::RUNNING;
  is_activated_ = isActivated();

  if (!isActivated()) {
    return planWaitingApproval();
  }

  auto output = plan();

  if (!isSafe()) {
    current_state_ = ModuleStatus::SUCCESS;  // for breaking loop
    return output;
  }

  // updateSteeringFactorPtr(output);

  clearWaitingApproval();

  return output;
}

void AvoidanceByLCModule::onEntry()
{
  RCLCPP_DEBUG(getLogger(), "LANE_CHANGE onEntry");
  current_state_ = ModuleStatus::IDLE;
  updateLaneChangeStatus();
}

void AvoidanceByLCModule::onExit()
{
  resetParameters();
  current_state_ = ModuleStatus::SUCCESS;
  *previous_module_output_.path = PathWithLaneId();
  publishReferencePath();
  RCLCPP_DEBUG(getLogger(), "LANE_CHANGE onExit");
}

bool AvoidanceByLCModule::isExecutionRequested() const
{
  const auto avoid_data_ = calcAvoidancePlanningData(debug_data_);

  if (current_state_ == ModuleStatus::IDLE) {
    return !avoid_data_.target_objects.empty();
  }

  if (avoid_data_.target_objects.empty()) {
    return !hasFinishedLaneChange();
  }

  if (current_state_ == ModuleStatus::RUNNING) {
    return true;
  }

  // const auto current_lanes = util::getCurrentLanes(planner_data_);
  const auto current_lanes = getCurrentLanes(*previous_module_output_.path);
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);

  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] =
    getSafePath(lane_change_lanes, check_distance_, selected_path);

  return found_valid_path;
}

bool AvoidanceByLCModule::isExecutionReady() const
{
  if (current_state_ == ModuleStatus::RUNNING) {
    return true;
  }

  // const auto current_lanes = util::getCurrentLanes(planner_data_);
  const auto current_lanes = getCurrentLanes(*previous_module_output_.path);
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);

  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] =
    getSafePath(lane_change_lanes, check_distance_, selected_path);

  return found_safe_path;
}

void AvoidanceByLCModule::updateData()
{
  debug_data_ = DebugData();
  avoidance_data_ = calcAvoidancePlanningData(debug_data_);

  // TODO(Horibe): this is not tested yet, disable now.
  updateRegisteredObject(avoidance_data_.target_objects);
  compensateDetectionLost(avoidance_data_.target_objects, avoidance_data_.other_objects);

  std::sort(
    avoidance_data_.target_objects.begin(), avoidance_data_.target_objects.end(),
    [](auto a, auto b) { return a.longitudinal < b.longitudinal; });
}

AvoidancePlanningData AvoidanceByLCModule::calcAvoidancePlanningData(DebugData & debug) const
{
  AvoidancePlanningData data;

  // reference pose
  const auto reference_pose = getEgoPose();
  data.reference_pose = reference_pose;

  data.reference_path = util::resamplePathWithSpline(
    *previous_module_output_.path, parameters_avoidance_->resample_interval_for_planning);

  const size_t nearest_segment_index =
    findNearestSegmentIndex(data.reference_path.points, data.reference_pose.position);
  data.ego_closest_path_index =
    std::min(nearest_segment_index + 1, data.reference_path.points.size() - 1);

  // arclength from ego pose (used in many functions)
  data.arclength_from_ego = util::calcPathArcLengthArray(
    data.reference_path, 0, data.reference_path.points.size(),
    calcSignedArcLength(data.reference_path.points, getEgoPosition(), 0));

  // lanelet info
  data.current_lanelets = util::calcLaneAroundPose(
    planner_data_->route_handler, reference_pose, planner_data_->parameters.forward_path_length,
    planner_data_->parameters.backward_path_length);

  // target objects for avoidance
  fillAvoidanceTargetObjects(data, debug);

  // DEBUG_PRINT("target object size = %lu", data.target_objects.size());

  return data;
}

void AvoidanceByLCModule::fillAvoidanceTargetObjects(
  AvoidancePlanningData & data, [[maybe_unused]] DebugData & debug) const
{
  using boost::geometry::return_centroid;
  using boost::geometry::within;
  using lanelet::geometry::distance2d;
  using lanelet::geometry::toArcCoordinates;
  using lanelet::utils::getId;
  using lanelet::utils::to2D;

  const auto & path_points = data.reference_path.points;
  const auto & ego_pos = getEgoPosition();

  // detection area filter
  // when expanding lanelets, right_offset must be minus.
  // This is because y axis is positive on the left.
  const auto expanded_lanelets = lanelet::utils::getExpandedLanelets(
    data.current_lanelets, parameters_avoidance_->detection_area_left_expand_dist,
    parameters_avoidance_->detection_area_right_expand_dist * (-1.0));

  const auto [object_within_target_lane, object_outside_target_lane] =
    util::separateObjectsByLanelets(*planner_data_->dynamic_object, expanded_lanelets);

  for (const auto & object : object_outside_target_lane.objects) {
    ObjectData other_object;
    other_object.object = object;
    other_object.reason = "OutOfTargetArea";
    data.other_objects.push_back(other_object);
  }

  // DEBUG_PRINT("dynamic_objects size = %lu", planner_data_->dynamic_object->objects.size());
  // DEBUG_PRINT("lane_filtered_objects size = %lu", object_within_target_lane.objects.size());

  // for goal
  const auto & rh = planner_data_->route_handler;
  const auto dist_to_goal =
    rh->isInGoalRouteSection(expanded_lanelets.back())
      ? calcSignedArcLength(path_points, ego_pos, rh->getGoalPose().position)
      : std::numeric_limits<double>::max();

  lanelet::ConstLineStrings3d debug_linestring;
  debug_linestring.clear();
  // for filtered objects
  ObjectDataArray target_objects;
  // std::vector<AvoidanceDebugMsg> avoidance_debug_msg_array;
  for (const auto & object : object_within_target_lane.objects) {
    const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
    // AvoidanceDebugMsg avoidance_debug_msg;
    // const auto avoidance_debug_array_false_and_push_back =
    //   [&avoidance_debug_msg, &avoidance_debug_msg_array](const std::string & failed_reason) {
    //     avoidance_debug_msg.allow_avoidance = false;
    //     avoidance_debug_msg.failed_reason = failed_reason;
    //     avoidance_debug_msg_array.push_back(avoidance_debug_msg);
    //   };

    ObjectData object_data;
    object_data.object = object;
    // avoidance_debug_msg.object_id = getUuidStr(object_data);

    if (!isTargetObjectType(object)) {
      // avoidance_debug_array_false_and_push_back(AvoidanceDebugFactor::OBJECT_IS_NOT_TYPE);
      // object_data.reason = AvoidanceDebugFactor::OBJECT_IS_NOT_TYPE;
      data.other_objects.push_back(object_data);
      continue;
    }

    const auto object_closest_index = findNearestIndex(path_points, object_pose.position);
    const auto object_closest_pose = path_points.at(object_closest_index).point.pose;

    // Calc envelop polygon.
    fillObjectEnvelopePolygon(object_closest_pose, object_data);

    // calc object centroid.
    object_data.centroid = return_centroid<Point2d>(object_data.envelope_poly);

    // calc longitudinal distance from ego to closest target object footprint point.
    fillLongitudinalAndLengthByClosestEnvelopeFootprint(data.reference_path, ego_pos, object_data);
    // avoidance_debug_msg.longitudinal_distance = object_data.longitudinal;

    // Calc moving time.
    fillObjectMovingTime(object_data);

    if (object_data.move_time > parameters_avoidance_->threshold_time_object_is_moving) {
      // avoidance_debug_array_false_and_push_back(AvoidanceDebugFactor::MOVING_OBJECT);
      // object_data.reason = AvoidanceDebugFactor::MOVING_OBJECT;
      data.other_objects.push_back(object_data);
      continue;
    }

    // object is behind ego or too far.
    if (object_data.longitudinal < -parameters_avoidance_->object_check_backward_distance) {
      // avoidance_debug_array_false_and_push_back(AvoidanceDebugFactor::OBJECT_IS_BEHIND_THRESHOLD);
      // object_data.reason = AvoidanceDebugFactor::OBJECT_BEHIND_PATH_GOAL;
      data.other_objects.push_back(object_data);
      continue;
    }
    if (object_data.longitudinal > parameters_avoidance_->object_check_forward_distance) {
      // avoidance_debug_array_false_and_push_back(AvoidanceDebugFactor::OBJECT_IS_IN_FRONT_THRESHOLD);
      // object_data.reason = AvoidanceDebugFactor::OBJECT_IS_IN_FRONT_THRESHOLD;
      data.other_objects.push_back(object_data);
      continue;
    }

    // Target object is behind the path goal -> ignore.
    if (object_data.longitudinal > dist_to_goal) {
      // avoidance_debug_array_false_and_push_back(AvoidanceDebugFactor::OBJECT_BEHIND_PATH_GOAL);
      // object_data.reason = AvoidanceDebugFactor::OBJECT_BEHIND_PATH_GOAL;
      data.other_objects.push_back(object_data);
      continue;
    }

    // Calc lateral deviation from path to target object.
    object_data.lateral = calcLateralDeviation(object_closest_pose, object_pose.position);
    // avoidance_debug_msg.lateral_distance_from_centerline = object_data.lateral;

    // Find the footprint point closest to the path, set to object_data.overhang_distance.
    object_data.overhang_dist = calcEnvelopeOverhangDistance(
      object_data, object_closest_pose, object_data.overhang_pose.position);

    lanelet::ConstLanelet overhang_lanelet;
    if (!rh->getClosestLaneletWithinRoute(object_closest_pose, &overhang_lanelet)) {
      continue;
    }

    if (overhang_lanelet.id()) {
      object_data.overhang_lanelet = overhang_lanelet;
      lanelet::BasicPoint3d overhang_basic_pose(
        object_data.overhang_pose.position.x, object_data.overhang_pose.position.y,
        object_data.overhang_pose.position.z);
      const bool get_left =
        isOnRight(object_data) && parameters_avoidance_->enable_avoidance_over_same_direction;
      const bool get_right =
        !isOnRight(object_data) && parameters_avoidance_->enable_avoidance_over_same_direction;

      const auto target_lines = rh->getFurthestLinestring(
        overhang_lanelet, get_right, get_left,
        parameters_avoidance_->enable_avoidance_over_opposite_direction);

      if (isOnRight(object_data)) {
        object_data.to_road_shoulder_distance =
          distance2d(to2D(overhang_basic_pose), to2D(target_lines.back().basicLineString()));
        debug_linestring.push_back(target_lines.back());
      } else {
        object_data.to_road_shoulder_distance =
          distance2d(to2D(overhang_basic_pose), to2D(target_lines.front().basicLineString()));
        debug_linestring.push_back(target_lines.front());
      }
    }

    // DEBUG_PRINT(
    //   "set object_data: longitudinal = %f, lateral = %f, largest_overhang = %f,"
    //   "to_road_shoulder_distance = %f",
    //   object_data.longitudinal, object_data.lateral, object_data.overhang_dist,
    //   object_data.to_road_shoulder_distance);

    // Object is on center line -> ignore.
    // avoidance_debug_msg.lateral_distance_from_centerline = object_data.lateral;
    if (
      std::abs(object_data.lateral) <
      parameters_avoidance_->threshold_distance_object_is_on_center) {
      // avoidance_debug_array_false_and_push_back(AvoidanceDebugFactor::TOO_NEAR_TO_CENTERLINE);
      // object_data.reason = AvoidanceDebugFactor::TOO_NEAR_TO_CENTERLINE;
      data.other_objects.push_back(object_data);
      continue;
    }

    lanelet::ConstLanelet object_closest_lanelet;
    const auto lanelet_map = rh->getLaneletMapPtr();
    if (!lanelet::utils::query::getClosestLanelet(
          lanelet::utils::query::laneletLayer(lanelet_map), object_pose, &object_closest_lanelet)) {
      continue;
    }

    lanelet::BasicPoint2d object_centroid(object_data.centroid.x(), object_data.centroid.y());

    /**
     * Is not object in adjacent lane?
     *   - Yes -> Is parking object?
     *     - Yes -> the object is avoidance target.
     *     - No -> ignore this object.
     *   - No -> the object is avoidance target no matter whether it is parking object or not.
     */
    const auto is_in_ego_lane =
      within(object_centroid, overhang_lanelet.polygon2d().basicPolygon());
    if (is_in_ego_lane) {
      /**
       * TODO(Satoshi Ota) use intersection area
       * under the assumption that there is no parking vehicle inside intersection,
       * ignore all objects that is in the ego lane as not parking objects.
       */
      std::string turn_direction = overhang_lanelet.attributeOr("turn_direction", "else");
      if (turn_direction == "right" || turn_direction == "left" || turn_direction == "straight") {
        // avoidance_debug_array_false_and_push_back(AvoidanceDebugFactor::NOT_PARKING_OBJECT);
        // object_data.reason = AvoidanceDebugFactor::NOT_PARKING_OBJECT;
        data.other_objects.push_back(object_data);
        continue;
      }

      const auto centerline_pose =
        lanelet::utils::getClosestCenterPose(object_closest_lanelet, object_pose.position);
      lanelet::BasicPoint3d centerline_point(
        centerline_pose.position.x, centerline_pose.position.y, centerline_pose.position.z);

      // ============================================ <- most_left_lanelet.leftBound()
      // y              road shoulder
      // ^ ------------------------------------------
      // |   x                                +
      // +---> --- object closest lanelet --- o ----- <- object_closest_lanelet.centerline()
      //
      // --------------------------------------------
      // +: object position
      // o: nearest point on centerline

      const auto most_left_road_lanelet = rh->getMostLeftLanelet(object_closest_lanelet);
      const auto most_left_lanelet_candidates =
        rh->getLaneletMapPtr()->laneletLayer.findUsages(most_left_road_lanelet.leftBound());

      lanelet::ConstLanelet most_left_lanelet = most_left_road_lanelet;

      for (const auto & ll : most_left_lanelet_candidates) {
        const lanelet::Attribute sub_type = ll.attribute(lanelet::AttributeName::Subtype);
        if (sub_type.value() == "road_shoulder") {
          most_left_lanelet = ll;
        }
      }

      const auto center_to_left_boundary =
        distance2d(to2D(most_left_lanelet.leftBound().basicLineString()), to2D(centerline_point));
      double object_shiftable_distance = center_to_left_boundary - 0.5 * object.shape.dimensions.y;

      const lanelet::Attribute sub_type =
        most_left_lanelet.attribute(lanelet::AttributeName::Subtype);
      if (sub_type.value() != "road_shoulder") {
        object_shiftable_distance += parameters_avoidance_->object_check_min_road_shoulder_width;
      }

      const auto arc_coordinates = toArcCoordinates(
        to2D(object_closest_lanelet.centerline().basicLineString()), object_centroid);
      object_data.shiftable_ratio = arc_coordinates.distance / object_shiftable_distance;

      const auto is_parking_object =
        object_data.shiftable_ratio > parameters_avoidance_->object_check_shiftable_ratio;

      if (!is_parking_object) {
        // avoidance_debug_array_false_and_push_back(AvoidanceDebugFactor::NOT_PARKING_OBJECT);
        // object_data.reason = AvoidanceDebugFactor::NOT_PARKING_OBJECT;
        data.other_objects.push_back(object_data);
        continue;
      }
    }

    object_data.last_seen = clock_->now();

    // set data
    data.target_objects.push_back(object_data);
  }

  // // debug
  // {
  //   updateAvoidanceDebugData(avoidance_debug_msg_array);
  //   debug.farthest_linestring_from_overhang =
  //     std::make_shared<lanelet::ConstLineStrings3d>(debug_linestring);
  //   debug.current_lanelets = std::make_shared<lanelet::ConstLanelets>(data.current_lanelets);
  //   debug.expanded_lanelets = std::make_shared<lanelet::ConstLanelets>(expanded_lanelets);
  // }
}

void AvoidanceByLCModule::fillObjectEnvelopePolygon(
  const Pose & closest_pose, ObjectData & object_data) const
{
  using boost::geometry::within;

  const auto id = object_data.object.object_id;
  const auto same_id_obj = std::find_if(
    registered_objects_.begin(), registered_objects_.end(),
    [&id](const auto & o) { return o.object.object_id == id; });

  if (same_id_obj == registered_objects_.end()) {
    object_data.envelope_poly = createEnvelopePolygon(
      object_data, closest_pose, parameters_avoidance_->object_envelope_buffer);
    return;
  }

  Polygon2d object_polygon{};
  util::calcObjectPolygon(object_data.object, &object_polygon);

  if (!within(object_polygon, same_id_obj->envelope_poly)) {
    object_data.envelope_poly = createEnvelopePolygon(
      object_data, closest_pose, parameters_avoidance_->object_envelope_buffer);
    return;
  }

  object_data.envelope_poly = same_id_obj->envelope_poly;
}

void AvoidanceByLCModule::fillObjectMovingTime(ObjectData & object_data) const
{
  const auto & object_vel =
    object_data.object.kinematics.initial_twist_with_covariance.twist.linear.x;
  const auto is_faster_than_threshold =
    object_vel > parameters_avoidance_->threshold_speed_object_is_stopped;

  const auto id = object_data.object.object_id;
  const auto same_id_obj = std::find_if(
    stopped_objects_.begin(), stopped_objects_.end(),
    [&id](const auto & o) { return o.object.object_id == id; });

  const auto is_new_object = same_id_obj == stopped_objects_.end();

  if (!is_faster_than_threshold) {
    object_data.last_stop = clock_->now();
    object_data.move_time = 0.0;
    if (is_new_object) {
      stopped_objects_.push_back(object_data);
    } else {
      same_id_obj->last_stop = clock_->now();
      same_id_obj->move_time = 0.0;
    }
    return;
  }

  if (is_new_object) {
    object_data.move_time = std::numeric_limits<double>::max();
    return;
  }

  object_data.last_stop = same_id_obj->last_stop;
  object_data.move_time = (clock_->now() - same_id_obj->last_stop).seconds();

  if (object_data.move_time > parameters_avoidance_->threshold_time_object_is_moving) {
    stopped_objects_.erase(same_id_obj);
  }
}

void AvoidanceByLCModule::updateRegisteredObject(const ObjectDataArray & now_objects)
{
  const auto updateIfDetectedNow = [&now_objects, this](auto & registered_object) {
    const auto & n = now_objects;
    const auto r_id = registered_object.object.object_id;
    const auto same_id_obj = std::find_if(
      n.begin(), n.end(), [&r_id](const auto & o) { return o.object.object_id == r_id; });

    // same id object is detected. update registered.
    if (same_id_obj != n.end()) {
      registered_object = *same_id_obj;
      return true;
    }

    constexpr auto POS_THR = 1.5;
    const auto r_pos = registered_object.object.kinematics.initial_pose_with_covariance.pose;
    const auto similar_pos_obj = std::find_if(n.begin(), n.end(), [&](const auto & o) {
      return calcDistance2d(r_pos, o.object.kinematics.initial_pose_with_covariance.pose) < POS_THR;
    });

    // same id object is not detected, but object is found around registered. update registered.
    if (similar_pos_obj != n.end()) {
      registered_object = *similar_pos_obj;
      return true;
    }

    // Same ID nor similar position object does not found.
    return false;
  };

  // -- check registered_objects, remove if lost_count exceeds limit. --
  for (int i = static_cast<int>(registered_objects_.size()) - 1; i >= 0; --i) {
    auto & r = registered_objects_.at(i);
    const std::string s = getUuidStr(r);

    // registered object is not detected this time. lost count up.
    if (!updateIfDetectedNow(r)) {
      r.lost_time = (clock_->now() - r.last_seen).seconds();
    } else {
      r.last_seen = clock_->now();
      r.lost_time = 0.0;
    }

    // lost count exceeds threshold. remove object from register.
    if (r.lost_time > parameters_avoidance_->object_last_seen_threshold) {
      registered_objects_.erase(registered_objects_.begin() + i);
    }
  }

  const auto isAlreadyRegistered = [this](const auto & n_id) {
    const auto & r = registered_objects_;
    return std::any_of(
      r.begin(), r.end(), [&n_id](const auto & o) { return o.object.object_id == n_id; });
  };

  // -- check now_objects, add it if it has new object id --
  for (const auto & now_obj : now_objects) {
    if (!isAlreadyRegistered(now_obj.object.object_id)) {
      registered_objects_.push_back(now_obj);
    }
  }
}

void AvoidanceByLCModule::compensateDetectionLost(
  ObjectDataArray & now_objects, ObjectDataArray & other_objects) const
{
  // const auto old_size = now_objects.size();  // for debug

  const auto isDetectedNow = [&](const auto & r_id) {
    const auto & n = now_objects;
    return std::any_of(
      n.begin(), n.end(), [&r_id](const auto & o) { return o.object.object_id == r_id; });
  };

  const auto isIgnoreObject = [&](const auto & r_id) {
    const auto & n = other_objects;
    return std::any_of(
      n.begin(), n.end(), [&r_id](const auto & o) { return o.object.object_id == r_id; });
  };

  for (const auto & registered : registered_objects_) {
    if (
      !isDetectedNow(registered.object.object_id) && !isIgnoreObject(registered.object.object_id)) {
      now_objects.push_back(registered);
    }
  }
  // DEBUG_PRINT("object size: %lu -> %lu", old_size, now_objects.size());
}

ModuleStatus AvoidanceByLCModule::updateState()
{
  RCLCPP_DEBUG(getLogger(), "LANE_CHANGE updateState");
  if (current_state_ == ModuleStatus::IDLE) {
    return current_state_;
  }

  if (!isSafe()) {
    current_state_ = ModuleStatus::SUCCESS;
    return current_state_;
  }

  if (isAbortConditionSatisfied()) {
    if (isNearEndOfLane() && isCurrentSpeedLow()) {
      current_state_ = ModuleStatus::RUNNING;
      return current_state_;
    }
    // cancel lane change path
    current_state_ = ModuleStatus::FAILURE;
    return current_state_;
  }

  if (hasFinishedLaneChange()) {
    current_state_ = ModuleStatus::SUCCESS;
    return current_state_;
  }
  current_state_ = ModuleStatus::RUNNING;
  return current_state_;
}

BehaviorModuleOutput AvoidanceByLCModule::plan()
{
  resetPathCandidate();

  auto path = status_.lane_change_path.path;

  if (!isValidPath(path)) {
    status_.is_safe = false;
    return BehaviorModuleOutput{};
  }
  generateExtendedDrivableArea(path);

  if (isAbortConditionSatisfied()) {
    if (isNearEndOfLane() && isCurrentSpeedLow()) {
      const auto stop_point = util::insertStopPoint(0.1, &path);
    }
  }

  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(path);
  updateOutputTurnSignal(output);

  return output;
}

CandidateOutput AvoidanceByLCModule::planCandidate() const
{
  CandidateOutput output;

  // Get lane change lanes
  // const auto current_lanes = util::getCurrentLanes(planner_data_);
  const auto current_lanes = getCurrentLanes(*previous_module_output_.path);
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);

  LaneChangePath selected_path;
  [[maybe_unused]] const auto [found_valid_path, found_safe_path] =
    getSafePath(lane_change_lanes, check_distance_, selected_path);
  selected_path.path.header = planner_data_->route_handler->getRouteHeader();

  if (selected_path.path.points.empty()) {
    return output;
  }

  const auto & start_idx = selected_path.shift_line.start_idx;
  const auto & end_idx = selected_path.shift_line.end_idx;

  output.path_candidate = selected_path.path;
  output.lateral_shift = selected_path.shifted_path.shift_length.at(end_idx) -
                         selected_path.shifted_path.shift_length.at(start_idx);
  output.start_distance_to_path_change = motion_utils::calcSignedArcLength(
    selected_path.path.points, getEgoPose().position, selected_path.shift_line.start.position);
  output.finish_distance_to_path_change = motion_utils::calcSignedArcLength(
    selected_path.path.points, getEgoPose().position, selected_path.shift_line.end.position);

  // updateSteeringFactorPtr(output, selected_path);

  return output;
}

BehaviorModuleOutput AvoidanceByLCModule::planWaitingApproval()
{
  BehaviorModuleOutput out;
  updateLaneChangeStatus();
  out.path = std::make_shared<PathWithLaneId>(*previous_module_output_.path);
  // out.path = std::make_shared<PathWithLaneId>(getReferencePath());
  const auto candidate = planCandidate();
  path_candidate_ = std::make_shared<PathWithLaneId>(candidate.path_candidate);
  // updateRTCStatus(candidate);
  // waitApproval();
  constexpr double threshold_to_update_status = -1.0e-03;
  if (candidate.start_distance_to_path_change > threshold_to_update_status) {
    updateRTCStatus(candidate);
    waitApproval();
  } else {
    clearWaitingApproval();
    removeRTCStatus();
    current_state_ = ModuleStatus::IDLE;
  }
  return out;
}

void AvoidanceByLCModule::updateLaneChangeStatus()
{
  // status_.current_lanes = util::getCurrentLanes(planner_data_);
  status_.current_lanes = getCurrentLanes(*previous_module_output_.path);
  status_.lane_change_lanes = getLaneChangeLanes(status_.current_lanes, lane_change_lane_length_);

  // Find lane change path
  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] =
    getSafePath(status_.lane_change_lanes, check_distance_, selected_path);

  // Update status
  status_.is_safe = found_safe_path;
  status_.lane_change_path = selected_path;
  status_.lane_follow_lane_ids = util::getIds(status_.current_lanes);
  status_.lane_change_lane_ids = util::getIds(status_.lane_change_lanes);

  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, getEgoPose());
  status_.start_distance = arclength_start.length;
  status_.lane_change_path.path.header = getRouteHeader();
}

PathWithLaneId AvoidanceByLCModule::getReferencePath() const
{
  PathWithLaneId reference_path;

  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = getEgoPose();
  const auto & common_parameters = planner_data_->parameters;

  // Set header
  reference_path.header = getRouteHeader();

  // const auto current_lanes = util::getCurrentLanes(planner_data_);
  const auto current_lanes = getCurrentLanes(*previous_module_output_.path);

  if (current_lanes.empty()) {
    return reference_path;
  }

  if (reference_path.points.empty()) {
    reference_path = util::getCenterLinePath(
      *route_handler, current_lanes, current_pose, common_parameters.backward_path_length,
      common_parameters.forward_path_length, common_parameters);
  }

  const int num_lane_change =
    std::abs(route_handler->getNumLaneToPreferredLane(current_lanes.back()));
  double optional_lengths{0.0};
  const auto isInIntersection = util::checkLaneIsInIntersection(
    *route_handler, reference_path, current_lanes, common_parameters, num_lane_change,
    optional_lengths);
  if (isInIntersection) {
    reference_path = util::getCenterLinePath(
      *route_handler, current_lanes, current_pose, common_parameters.backward_path_length,
      common_parameters.forward_path_length, common_parameters, optional_lengths);
  }

  const double lane_change_buffer =
    util::calcLaneChangeBuffer(common_parameters, num_lane_change, optional_lengths);

  reference_path = util::setDecelerationVelocity(
    *route_handler, reference_path, current_lanes,
    parameters_lane_change_->lane_change_prepare_duration, lane_change_buffer);

  const auto drivable_lanes = util::generateDrivableLanes(current_lanes);
  const auto shorten_lanes = util::cutOverlappedLanes(reference_path, drivable_lanes);
  const auto expanded_lanes = util::expandLanelets(
    shorten_lanes, parameters_lane_change_->drivable_area_left_bound_offset,
    parameters_lane_change_->drivable_area_right_bound_offset);
  util::generateDrivableArea(
    reference_path, expanded_lanes, common_parameters.vehicle_length, planner_data_);

  return reference_path;
}

lanelet::ConstLanelets AvoidanceByLCModule::getLaneChangeLanes(
  const lanelet::ConstLanelets & current_lanes, const double lane_change_lane_length) const
{
  lanelet::ConstLanelets lane_change_lanes;
  const auto & route_handler = planner_data_->route_handler;
  const auto minimum_lane_change_length = planner_data_->parameters.minimum_lane_change_length;
  const auto lane_change_prepare_duration = parameters_lane_change_->lane_change_prepare_duration;
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();

  if (current_lanes.empty()) {
    return lane_change_lanes;
  }

  if (avoidance_data_.target_objects.empty()) {
    return lane_change_lanes;
  }

  const auto o_front = avoidance_data_.target_objects.front();

  // Get lane change lanes
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(current_lanes, current_pose, &current_lane);
  const double lane_change_prepare_length =
    std::max(current_twist.linear.x * lane_change_prepare_duration, minimum_lane_change_length);
  lanelet::ConstLanelets current_check_lanes =
    route_handler->getLaneletSequence(current_lane, current_pose, 0.0, lane_change_prepare_length);
  lanelet::ConstLanelet lane_change_lane;

  // if (route_handler->getLaneChangeTarget(current_check_lanes, &lane_change_lane)) {
  if (isOnRight(o_front)) {
    for (const auto & lanelet : current_check_lanes) {
      const auto & left_lane = route_handler->getRoutingGraphPtr()->left(lanelet);
      if (left_lane) {
        lane_change_lanes = route_handler->getLaneletSequence(
          left_lane.get(), current_pose, lane_change_lane_length, lane_change_lane_length);
        break;
      }
    }
  } else {
    for (const auto & lanelet : current_check_lanes) {
      const auto & right_lane = route_handler->getRoutingGraphPtr()->right(lanelet);
      if (right_lane) {
        lane_change_lanes = route_handler->getLaneletSequence(
          right_lane.get(), current_pose, lane_change_lane_length, lane_change_lane_length);
        break;
      }
    }
  }

  // if (route_handler->getRightLaneletWithinRoute(current_check_lanes, &lane_change_lane)) {
  //   lane_change_lanes = route_handler->getLaneletSequence(
  //     lane_change_lane, current_pose, lane_change_lane_length, lane_change_lane_length);
  // } else {
  //   lane_change_lanes.clear();
  // }

  return lane_change_lanes;
}

std::pair<bool, bool> AvoidanceByLCModule::getSafePath(
  const lanelet::ConstLanelets & lane_change_lanes, const double check_distance,
  LaneChangePath & safe_path) const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();
  const auto & common_parameters = planner_data_->parameters;

  // const auto current_lanes = util::getCurrentLanes(planner_data_);
  const auto current_lanes = getCurrentLanes(*previous_module_output_.path);

  if (!lane_change_lanes.empty()) {
    // find candidate paths
    const auto lane_change_paths = lane_change_utils::getLaneChangePaths(
      *route_handler, current_lanes, lane_change_lanes, current_pose, current_twist,
      common_parameters, *parameters_lane_change_, *previous_module_output_.path);

    // get lanes used for detection
    lanelet::ConstLanelets check_lanes;
    if (!lane_change_paths.empty()) {
      const auto & longest_path = lane_change_paths.front();
      // we want to see check_distance [m] behind vehicle so add lane changing length
      const double check_distance_with_path =
        check_distance + longest_path.preparation_length + longest_path.lane_change_length;
      check_lanes = route_handler->getCheckTargetLanesFromPath(
        longest_path.path, lane_change_lanes, check_distance_with_path);
    }

    // select valid path
    const LaneChangePaths valid_paths = lane_change_utils::selectValidPaths(
      lane_change_paths, current_lanes, check_lanes, *route_handler->getOverallGraphPtr(),
      current_pose, route_handler->isInGoalRouteSection(current_lanes.back()),
      route_handler->getGoalPose());

    if (valid_paths.empty()) {
      return std::make_pair(false, false);
    }
    debug_valid_path_ = valid_paths;

    // select safe path
    const bool found_safe_path = lane_change_utils::selectSafePath(
      valid_paths, current_lanes, check_lanes, planner_data_->dynamic_object, current_pose,
      current_twist, common_parameters, *parameters_lane_change_, &safe_path, object_debug_);

    if (parameters_lane_change_->publish_debug_marker) {
      setObjectDebugVisualization();
    } else {
      debug_marker_.markers.clear();
    }

    return std::make_pair(true, found_safe_path);
  }

  return std::make_pair(false, false);
}

bool AvoidanceByLCModule::isSafe() const { return status_.is_safe; }

bool AvoidanceByLCModule::isValidPath(const PathWithLaneId & path) const
{
  const auto & route_handler = planner_data_->route_handler;

  // check lane departure
  const auto drivable_lanes = lane_change_utils::generateDrivableLanes(
    *route_handler, util::extendLanes(route_handler, status_.current_lanes),
    util::extendLanes(route_handler, status_.lane_change_lanes));
  const auto expanded_lanes = util::expandLanelets(
    drivable_lanes, parameters_lane_change_->drivable_area_left_bound_offset,
    parameters_lane_change_->drivable_area_right_bound_offset);
  const auto lanelets = util::transformToLanelets(expanded_lanes);

  // check path points are in any lanelets
  for (const auto & point : path.points) {
    bool is_in_lanelet = false;
    for (const auto & lanelet : lanelets) {
      if (lanelet::utils::isInLanelet(point.point.pose, lanelet)) {
        is_in_lanelet = true;
        break;
      }
    }
    if (!is_in_lanelet) {
      RCLCPP_WARN_STREAM_THROTTLE(getLogger(), *clock_, 1000, "path is out of lanes");
      return false;
    }
  }

  // check relative angle
  if (!util::checkPathRelativeAngle(path, M_PI)) {
    RCLCPP_WARN_STREAM_THROTTLE(getLogger(), *clock_, 1000, "path relative angle is invalid");
    return false;
  }

  return true;
}

bool AvoidanceByLCModule::isNearEndOfLane() const
{
  const auto & current_pose = getEgoPose();
  const double threshold = util::calcTotalLaneChangeDistance(planner_data_->parameters);

  return std::max(0.0, util::getDistanceToEndOfLane(current_pose, status_.current_lanes)) <
         threshold;
}

bool AvoidanceByLCModule::isCurrentSpeedLow() const
{
  constexpr double threshold_ms = 10.0 * 1000 / 3600;
  return util::l2Norm(getEgoTwist().linear) < threshold_ms;
}

bool AvoidanceByLCModule::isAbortConditionSatisfied() const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();
  const auto & dynamic_objects = planner_data_->dynamic_object;
  const auto & common_parameters = planner_data_->parameters;

  const auto & current_lanes = status_.current_lanes;

  // check abort enable flag
  if (!parameters_lane_change_->enable_abort_lane_change) {
    return false;
  }

  if (!is_activated_) {
    return false;
  }

  // find closest lanelet in original lane
  lanelet::ConstLanelet closest_lanelet{};
  auto clock{rclcpp::Clock{RCL_ROS_TIME}};
  if (!lanelet::utils::query::getClosestLanelet(current_lanes, current_pose, &closest_lanelet)) {
    RCLCPP_ERROR_THROTTLE(
      getLogger(), clock, 1000,
      "Failed to find closest lane! Lane change aborting function is not working!");
    return false;
  }

  // check if lane change path is still safe
  const bool is_path_safe = std::invoke([this, &route_handler, &dynamic_objects, &current_lanes,
                                         &current_pose, &current_twist, &common_parameters]() {
    constexpr double check_distance = 100.0;
    // get lanes used for detection
    const auto & path = status_.lane_change_path;
    const double check_distance_with_path =
      check_distance + path.preparation_length + path.lane_change_length;
    const auto check_lanes = route_handler->getCheckTargetLanesFromPath(
      path.path, status_.lane_change_lanes, check_distance_with_path);

    std::unordered_map<std::string, CollisionCheckDebug> debug_data;

    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.path.points, current_pose, common_parameters.ego_nearest_dist_threshold,
      common_parameters.ego_nearest_yaw_threshold);
    return lane_change_utils::isLaneChangePathSafe(
      path.path, current_lanes, check_lanes, dynamic_objects, current_pose, current_seg_idx,
      current_twist, common_parameters, *parameters_lane_change_,
      common_parameters.expected_front_deceleration_for_abort,
      common_parameters.expected_rear_deceleration_for_abort, debug_data, false,
      status_.lane_change_path.acceleration);
  });

  // abort only if velocity is low or vehicle pose is close enough
  if (!is_path_safe) {
    // check vehicle velocity thresh
    const bool is_velocity_low = util::l2Norm(current_twist.linear) <
                                 parameters_lane_change_->abort_lane_change_velocity_thresh;
    const bool is_within_original_lane =
      lane_change_utils::isEgoWithinOriginalLane(current_lanes, current_pose, common_parameters);
    if (is_velocity_low && is_within_original_lane) {
      return true;
    }

    const bool is_distance_small = lane_change_utils::isEgoDistanceNearToCenterline(
      closest_lanelet, current_pose, *parameters_lane_change_);

    // check angle thresh from original lane
    const bool is_angle_diff_small = lane_change_utils::isEgoHeadingAngleLessThanThreshold(
      closest_lanelet, current_pose, *parameters_lane_change_);

    if (is_distance_small && is_angle_diff_small) {
      return true;
    }
    auto clock{rclcpp::Clock{RCL_ROS_TIME}};
    RCLCPP_WARN_STREAM_THROTTLE(
      getLogger(), clock, 1000,
      "DANGER!!! Path is not safe anymore, but it is too late to abort! Please be cautious");
  }

  return false;
}

bool AvoidanceByLCModule::hasFinishedLaneChange() const
{
  const auto & current_pose = getEgoPose();
  const auto arclength_current =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, current_pose);
  const double travel_distance = arclength_current.length - status_.start_distance;
  const double finish_distance = status_.lane_change_path.preparation_length +
                                 status_.lane_change_path.lane_change_length +
                                 parameters_lane_change_->lane_change_finish_judge_buffer;
  return travel_distance > finish_distance;
}

void AvoidanceByLCModule::setObjectDebugVisualization() const
{
  using marker_utils::lane_change_markers::showAllValidLaneChangePath;
  using marker_utils::lane_change_markers::showLerpedPose;
  using marker_utils::lane_change_markers::showObjectInfo;
  using marker_utils::lane_change_markers::showPolygon;
  using marker_utils::lane_change_markers::showPolygonPose;

  debug_marker_.markers.clear();
  const auto add = [this](const MarkerArray & added) {
    tier4_autoware_utils::appendMarkerArray(added, &debug_marker_);
  };

  add(showObjectInfo(object_debug_, "object_debug_info"));
  add(showLerpedPose(object_debug_, "lerp_pose_before_true"));
  add(showPolygonPose(object_debug_, "expected_pose"));
  add(showPolygon(object_debug_, "lerped_polygon"));
  add(showAllValidLaneChangePath(debug_valid_path_, "lane_change_valid_paths"));
}

std::shared_ptr<LaneChangeDebugMsgArray> AvoidanceByLCModule::get_debug_msg_array() const
{
  LaneChangeDebugMsgArray debug_msg_array;
  debug_msg_array.lane_change_info.reserve(object_debug_.size());
  for (const auto & [uuid, debug_data] : object_debug_) {
    LaneChangeDebugMsg debug_msg;
    debug_msg.object_id = uuid;
    debug_msg.allow_lane_change = debug_data.allow_lane_change;
    debug_msg.is_front = debug_data.is_front;
    debug_msg.relative_distance = debug_data.relative_to_ego;
    debug_msg.failed_reason = debug_data.failed_reason;
    debug_msg.velocity = util::l2Norm(debug_data.object_twist.linear);
    debug_msg_array.lane_change_info.push_back(debug_msg);
  }
  lane_change_debug_msg_array_ = debug_msg_array;

  lane_change_debug_msg_array_.header.stamp = clock_->now();
  return std::make_shared<LaneChangeDebugMsgArray>(lane_change_debug_msg_array_);
}

void AvoidanceByLCModule::updateSteeringFactorPtr(const BehaviorModuleOutput & output)
{
  const auto turn_signal_info = output.turn_signal_info;
  const auto current_pose = getEgoPose();
  const double start_distance = motion_utils::calcSignedArcLength(
    output.path->points, current_pose.position, status_.lane_change_path.shift_line.start.position);
  const double finish_distance = motion_utils::calcSignedArcLength(
    output.path->points, current_pose.position, status_.lane_change_path.shift_line.end.position);

  const uint16_t steering_factor_direction =
    std::invoke([this, &start_distance, &finish_distance, &turn_signal_info]() {
      if (turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT) {
        waitApprovalLeft(start_distance, finish_distance);
        return SteeringFactor::LEFT;
      }
      if (turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
        waitApprovalRight(start_distance, finish_distance);
        return SteeringFactor::RIGHT;
      }
      return SteeringFactor::UNKNOWN;
    });

  // TODO(tkhmy) add handle status TRYING
  steering_factor_interface_ptr_->updateSteeringFactor(
    {status_.lane_change_path.shift_line.start, status_.lane_change_path.shift_line.end},
    {start_distance, finish_distance}, SteeringFactor::LANE_CHANGE, steering_factor_direction,
    SteeringFactor::TURNING, "");
}

void AvoidanceByLCModule::updateSteeringFactorPtr(
  const CandidateOutput & output, const LaneChangePath & selected_path) const
{
  const uint16_t steering_factor_direction = std::invoke([&output]() {
    if (output.lateral_shift > 0.0) {
      return SteeringFactor::LEFT;
    }
    return SteeringFactor::RIGHT;
  });

  steering_factor_interface_ptr_->updateSteeringFactor(
    {selected_path.shift_line.start, selected_path.shift_line.end},
    {output.start_distance_to_path_change, output.finish_distance_to_path_change},
    SteeringFactor::LANE_CHANGE, steering_factor_direction, SteeringFactor::APPROACHING, "");
}

Twist AvoidanceByLCModule::getEgoTwist() const { return planner_data_->self_odometry->twist.twist; }

std_msgs::msg::Header AvoidanceByLCModule::getRouteHeader() const
{
  return planner_data_->route_handler->getRouteHeader();
}

void AvoidanceByLCModule::generateExtendedDrivableArea(PathWithLaneId & path)
{
  const auto & common_parameters = planner_data_->parameters;
  const auto & route_handler = planner_data_->route_handler;
  const auto drivable_lanes = lane_change_utils::generateDrivableLanes(
    *route_handler, status_.current_lanes, status_.lane_change_lanes);
  const auto shorten_lanes = util::cutOverlappedLanes(path, drivable_lanes);
  const auto expanded_lanes = util::expandLanelets(
    shorten_lanes, parameters_lane_change_->drivable_area_left_bound_offset,
    parameters_lane_change_->drivable_area_right_bound_offset);
  util::generateDrivableArea(path, expanded_lanes, common_parameters.vehicle_length, planner_data_);
}

void AvoidanceByLCModule::updateOutputTurnSignal(BehaviorModuleOutput & output)
{
  const auto turn_signal_info = util::getPathTurnSignal(
    status_.current_lanes, status_.lane_change_path.shifted_path,
    status_.lane_change_path.shift_line, getEgoPose(), getEgoTwist().linear.x,
    planner_data_->parameters);
  output.turn_signal_info.turn_signal.command = turn_signal_info.first.command;

  lane_change_utils::get_turn_signal_info(status_.lane_change_path, &output.turn_signal_info);
}

void AvoidanceByLCModule::resetParameters()
{
  clearWaitingApproval();
  removeRTCStatus();
  // steering_factor_interface_ptr_->clearSteeringFactors();
  object_debug_.clear();
  debug_marker_.markers.clear();
  resetPathCandidate();
}

bool AvoidanceByLCModule::isTargetObjectType(const PredictedObject & object) const
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  const auto t = util::getHighestProbLabel(object.classification);
  const auto is_object_type =
    ((t == ObjectClassification::CAR && parameters_avoidance_->avoid_car) ||
     (t == ObjectClassification::TRUCK && parameters_avoidance_->avoid_truck) ||
     (t == ObjectClassification::BUS && parameters_avoidance_->avoid_bus) ||
     (t == ObjectClassification::TRAILER && parameters_avoidance_->avoid_trailer) ||
     (t == ObjectClassification::UNKNOWN && parameters_avoidance_->avoid_unknown) ||
     (t == ObjectClassification::BICYCLE && parameters_avoidance_->avoid_bicycle) ||
     (t == ObjectClassification::MOTORCYCLE && parameters_avoidance_->avoid_motorcycle) ||
     (t == ObjectClassification::PEDESTRIAN && parameters_avoidance_->avoid_pedestrian));
  return is_object_type;
}

void AvoidanceByLCModule::acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const
{
  if (visitor) {
    visitor->visitAvoidanceByLCModule(this);
  }
}

void SceneModuleVisitor::visitAvoidanceByLCModule(
  [[maybe_unused]] const AvoidanceByLCModule * module) const
{
  // lane_change_visitor_ = module->get_debug_msg_array();
}
}  // namespace behavior_path_planner
