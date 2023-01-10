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

#include "behavior_path_planner/behavior_path_planner_node.hpp"

#include "behavior_path_planner/debug_utilities.hpp"
#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/avoidance/manager.hpp"
#include "behavior_path_planner/scene_module/avoidance_by_lc/manager.hpp"
#include "behavior_path_planner/scene_module/lane_change/manager.hpp"
#include "behavior_path_planner/scene_module/pull_over/manager.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <tier4_planning_msgs/msg/path_change_module_id.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
rclcpp::SubscriptionOptions createSubscriptionOptions(rclcpp::Node * node_ptr)
{
  rclcpp::CallbackGroup::SharedPtr callback_group =
    node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group;

  return sub_opt;
}
}  // namespace

namespace behavior_path_planner
{
using tier4_planning_msgs::msg::PathChangeModuleId;
using vehicle_info_util::VehicleInfoUtil;

BehaviorPathPlannerNode::BehaviorPathPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("behavior_path_planner", node_options)
{
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  // data_manager
  {
    planner_data_ = std::make_shared<PlannerData>();
    planner_data_->parameters = getCommonParam();
  }

  // publisher
  path_publisher_ = create_publisher<PathWithLaneId>("~/output/path", 1);
  turn_signal_publisher_ =
    create_publisher<TurnIndicatorsCommand>("~/output/turn_indicators_cmd", 1);
  hazard_signal_publisher_ = create_publisher<HazardLightsCommand>("~/output/hazard_lights_cmd", 1);
  modified_goal_publisher_ = create_publisher<PoseWithUuidStamped>("~/output/modified_goal", 1);
  // debug_avoidance_msg_array_publisher_ =
  //   create_publisher<AvoidanceDebugMsgArray>("~/debug/avoidance_debug_message_array", 1);
  // debug_lane_change_msg_array_publisher_ =
  //   create_publisher<LaneChangeDebugMsgArray>("~/debug/lane_change_debug_message_array", 1);

  if (planner_data_->parameters.visualize_maximum_drivable_area) {
    debug_maximum_drivable_area_publisher_ =
      create_publisher<MarkerArray>("~/maximum_drivable_area", 1);
  }

  bound_publisher_ = create_publisher<MarkerArray>("~/debug/bound", 1);

  // subscriber
  velocity_subscriber_ = create_subscription<Odometry>(
    "~/input/odometry", 1, std::bind(&BehaviorPathPlannerNode::onOdometry, this, _1),
    createSubscriptionOptions(this));
  acceleration_subscriber_ = create_subscription<AccelWithCovarianceStamped>(
    "~/input/accel", 1, std::bind(&BehaviorPathPlannerNode::onAcceleration, this, _1),
    createSubscriptionOptions(this));
  perception_subscriber_ = create_subscription<PredictedObjects>(
    "~/input/perception", 1, std::bind(&BehaviorPathPlannerNode::onPerception, this, _1),
    createSubscriptionOptions(this));
  occupancy_grid_subscriber_ = create_subscription<OccupancyGrid>(
    "~/input/occupancy_grid_map", 1, std::bind(&BehaviorPathPlannerNode::onOccupancyGrid, this, _1),
    createSubscriptionOptions(this));
  scenario_subscriber_ = create_subscription<Scenario>(
    "~/input/scenario", 1,
    [this](const Scenario::ConstSharedPtr msg) {
      current_scenario_ = std::make_shared<Scenario>(*msg);
    },
    createSubscriptionOptions(this));

  // route_handler
  auto qos_transient_local = rclcpp::QoS{1}.transient_local();
  vector_map_subscriber_ = create_subscription<HADMapBin>(
    "~/input/vector_map", qos_transient_local, std::bind(&BehaviorPathPlannerNode::onMap, this, _1),
    createSubscriptionOptions(this));
  route_subscriber_ = create_subscription<LaneletRoute>(
    "~/input/route", qos_transient_local, std::bind(&BehaviorPathPlannerNode::onRoute, this, _1),
    createSubscriptionOptions(this));

  // behavior tree manager
  {
    const std::string path_candidate_name_space = "/planning/path_candidate/";
    mutex_bt_.lock();

    const auto & p = planner_data_->parameters;
    planner_manager_ = std::make_shared<PlannerManager>(
      *this, p.enable_simultaneous_execution_of_multiple_modules, p.verbose);

    if (p.launch_pull_over) {
      auto manager = std::make_shared<PullOverModuleManager>(this, "pull_over", 1);
      planner_manager_->registerSceneModuleManager(manager);
      path_candidate_publishers_.emplace(
        "pull_over", create_publisher<Path>(path_candidate_name_space + "pull_over", 1));
    }

    if (p.launch_avoidance_by_lc) {
      auto manager = std::make_shared<AvoidanceByLCModuleManager>(this, "avoidance_by_lc", 1);
      planner_manager_->registerSceneModuleManager(manager);
      path_candidate_publishers_.emplace(
        "avoidance_by_lc",
        create_publisher<Path>(path_candidate_name_space + "avoidance_by_lc", 1));
    }

    if (p.launch_avoidance) {
      auto manager = std::make_shared<AvoidanceModuleManager>(this, "avoidance", 1);
      planner_manager_->registerSceneModuleManager(manager);
      path_candidate_publishers_.emplace(
        "avoidance", create_publisher<Path>(path_candidate_name_space + "avoidance", 1));
    }

    if (p.launch_lane_change) {
      auto manager = std::make_shared<LaneChangeModuleManager>(this, "lane_change", 1);
      planner_manager_->registerSceneModuleManager(manager);
      path_candidate_publishers_.emplace(
        "lane_change", create_publisher<Path>(path_candidate_name_space + "lane_change", 1));
    }

    // if (p.launch_pull_out) {
    //   auto manager = std::make_shared<PullOutModuleManager>(this, "pull_out", 1);
    //   planner_manager_->registerSceneModuleManager(manager);
    //   path_candidate_publishers_.emplace(
    //     "pull_out",
    //     create_publisher<Path>(path_candidate_name_space + "pull_out", 1));
    // }

    mutex_bt_.unlock();
  }

  // turn signal decider
  {
    const double turn_signal_intersection_search_distance =
      planner_data_->parameters.turn_signal_intersection_search_distance;
    const double turn_signal_intersection_angle_threshold_deg =
      planner_data_->parameters.turn_signal_intersection_angle_threshold_deg;
    const double turn_signal_search_time = planner_data_->parameters.turn_signal_search_time;
    turn_signal_decider_.setParameters(
      planner_data_->parameters.base_link2front, turn_signal_intersection_search_distance,
      turn_signal_search_time, turn_signal_intersection_angle_threshold_deg);
  }

  steering_factor_interface_ptr_ = std::make_unique<SteeringFactorInterface>(this, "intersection");

  // Start timer
  {
    const auto planning_hz = declare_parameter("planning_hz", 10.0);
    const auto period_ns = rclcpp::Rate(planning_hz).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&BehaviorPathPlannerNode::run, this));
  }

  m_set_param_res = this->add_on_set_parameters_callback(
    std::bind(&BehaviorPathPlannerNode::onSetParam, this, std::placeholders::_1));
}

BehaviorPathPlannerParameters BehaviorPathPlannerNode::getCommonParam()
{
  BehaviorPathPlannerParameters p{};

  // vehicle info
  const auto vehicle_info = VehicleInfoUtil(*this).getVehicleInfo();
  p.vehicle_info = vehicle_info;
  p.vehicle_width = vehicle_info.vehicle_width_m;
  p.vehicle_length = vehicle_info.vehicle_length_m;
  p.wheel_tread = vehicle_info.wheel_tread_m;
  p.wheel_base = vehicle_info.wheel_base_m;
  p.front_overhang = vehicle_info.front_overhang_m;
  p.rear_overhang = vehicle_info.rear_overhang_m;
  p.left_over_hang = vehicle_info.left_overhang_m;
  p.right_over_hang = vehicle_info.right_overhang_m;
  p.base_link2front = vehicle_info.max_longitudinal_offset_m;
  p.base_link2rear = p.rear_overhang;

  // NOTE: backward_path_length is used not only calculating path length but also calculating the
  // size of a drivable area.
  //       The drivable area has to cover not the base link but the vehicle itself. Therefore
  //       rear_overhang must be added to backward_path_length. In addition, because of the
  //       calculation of the drivable area in the obstacle_avoidance_planner package, the drivable
  //       area has to be a little longer than the backward_path_length parameter by adding
  //       min_backward_offset.
  constexpr double min_backward_offset = 1.0;
  const double backward_offset = vehicle_info.rear_overhang_m + min_backward_offset;

  p.launch_pull_over = declare_parameter<bool>("launch_pull_over");
  p.launch_avoidance_by_lc = declare_parameter<bool>("launch_avoidance_by_lc");
  p.launch_avoidance = declare_parameter<bool>("launch_avoidance");
  p.launch_lane_change = declare_parameter<bool>("launch_lane_change");

  p.enable_simultaneous_execution_of_multiple_modules =
    declare_parameter<bool>("enable_simultaneous_execution_of_multiple_modules");
  p.verbose = declare_parameter<bool>("verbose");

  // ROS parameters
  p.backward_path_length = declare_parameter("backward_path_length", 5.0) + backward_offset;
  p.forward_path_length = declare_parameter("forward_path_length", 100.0);
  p.backward_length_buffer_for_end_of_lane =
    declare_parameter("lane_change.backward_length_buffer_for_end_of_lane", 5.0);
  p.backward_length_buffer_for_end_of_pull_over =
    declare_parameter("backward_length_buffer_for_end_of_pull_over", 5.0);
  p.backward_length_buffer_for_end_of_pull_out =
    declare_parameter("backward_length_buffer_for_end_of_pull_out", 5.0);
  p.minimum_lane_change_length = declare_parameter("lane_change.minimum_lane_change_length", 8.0);
  p.minimum_lane_change_prepare_distance =
    declare_parameter("lane_change.minimum_lane_change_prepare_distance", 2.0);

  p.minimum_pull_over_length = declare_parameter("minimum_pull_over_length", 15.0);
  p.refine_goal_search_radius_range = declare_parameter("refine_goal_search_radius_range", 7.5);
  p.turn_signal_intersection_search_distance =
    declare_parameter("turn_signal_intersection_search_distance", 30.0);
  p.turn_signal_intersection_angle_threshold_deg =
    declare_parameter("turn_signal_intersection_angle_threshold_deg", 15.0);
  p.turn_signal_minimum_search_distance =
    declare_parameter("turn_signal_minimum_search_distance", 10.0);
  p.turn_signal_search_time = declare_parameter("turn_signal_search_time", 3.0);
  p.turn_signal_shift_length_threshold =
    declare_parameter("turn_signal_shift_length_threshold", 0.3);
  p.turn_signal_on_swerving = declare_parameter("turn_signal_on_swerving", true);

  p.path_interval = declare_parameter<double>("path_interval");
  p.visualize_maximum_drivable_area = declare_parameter("visualize_maximum_drivable_area", true);
  p.ego_nearest_dist_threshold = declare_parameter<double>("ego_nearest_dist_threshold");
  p.ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");

  p.lateral_distance_max_threshold = declare_parameter("lateral_distance_max_threshold", 2.0);
  p.longitudinal_distance_min_threshold =
    declare_parameter("longitudinal_distance_min_threshold", 3.0);

  p.expected_front_deceleration = declare_parameter("expected_front_deceleration", -0.5);
  p.expected_rear_deceleration = declare_parameter("expected_rear_deceleration", -1.0);

  p.expected_front_deceleration_for_abort =
    declare_parameter("expected_front_deceleration_for_abort", -2.0);
  p.expected_rear_deceleration_for_abort =
    declare_parameter("expected_rear_deceleration_for_abort", -2.5);

  p.rear_vehicle_reaction_time = declare_parameter("rear_vehicle_reaction_time", 2.0);
  p.rear_vehicle_safety_time_margin = declare_parameter("rear_vehicle_safety_time_margin", 2.0);

  if (p.backward_length_buffer_for_end_of_lane < 1.0) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Lane change buffer must be more than 1 meter. Modifying the buffer.");
  }
  return p;
}

// wait until mandatory data is ready
bool BehaviorPathPlannerNode::isDataReady()
{
  const auto missing = [this](const auto & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", name);
    return false;
  };

  if (!current_scenario_) {
    return missing("scenario_topic");
  }

  if (!route_ptr_) {
    return missing("route");
  }

  if (!map_ptr_) {
    return missing("map");
  }

  if (!planner_data_->dynamic_object) {
    return missing("dynamic_object");
  }

  if (!planner_data_->self_odometry) {
    return missing("self_odometry");
  }

  if (!planner_data_->self_acceleration) {
    return missing("self_acceleration");
  }

  return true;
}

std::shared_ptr<PlannerData> BehaviorPathPlannerNode::createLatestPlannerData()
{
  const std::lock_guard<std::mutex> lock(mutex_pd_);

  // update map
  if (has_received_map_) {
    planner_data_->route_handler->setMap(*map_ptr_);
    has_received_map_ = false;
  }

  // update route
  const bool is_first_time = !(planner_data_->route_handler->isHandlerReady());
  if (has_received_route_) {
    planner_data_->route_handler->setRoute(*route_ptr_);
    // Reset behavior tree when new route is received,
    // so that the each modules do not have to care about the "route jump".
    if (!is_first_time) {
      RCLCPP_DEBUG(get_logger(), "new route is received. reset behavior tree.");
      planner_manager_->reset();
    }

    has_received_route_ = false;
  }

  return std::make_shared<PlannerData>(*planner_data_);
}

void BehaviorPathPlannerNode::run()
{
  if (!isDataReady()) {
    return;
  }

  RCLCPP_DEBUG(get_logger(), "----- BehaviorPathPlannerNode start -----");
  mutex_bt_.lock();  // for bt_manager_

  // behavior_path_planner runs only in LANE DRIVING scenario.
  if (current_scenario_->current_scenario != Scenario::LANEDRIVING) {
    mutex_bt_.unlock();  // for bt_manager_
    return;
  }

  // create latest planner data
  const auto planner_data = createLatestPlannerData();

  // run behavior planner
  const auto output = planner_manager_->run(planner_data);

  // path handling
  const auto path = getPath(output, planner_data);

  // update planner data
  planner_data_->prev_output_path = path;

  // compute turn signal
  computeTurnSignal(planner_data, *path, output);

  // publish drivable bounds
  publish_bounds(*path);

  const size_t target_idx = findEgoIndex(path->points);
  util::clipPathLength(*path, target_idx, planner_data_->parameters);

  if (!path->points.empty()) {
    path_publisher_->publish(*path);
  } else {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "behavior path output is empty! Stop publish.");
  }

  publishPathCandidate(planner_manager_->getSceneModuleManagers());

  // publishSceneModuleDebugMsg();

  if (output.modified_goal) {
    PoseWithUuidStamped modified_goal = *(output.modified_goal);
    modified_goal.header.stamp = path->header.stamp;
    modified_goal_publisher_->publish(modified_goal);
  }

  if (planner_data->parameters.visualize_maximum_drivable_area) {
    const auto maximum_drivable_area =
      marker_utils::createFurthestLineStringMarkerArray(util::getMaximumDrivableArea(planner_data));
    debug_maximum_drivable_area_publisher_->publish(maximum_drivable_area);
  }

  planner_manager_->print();

  mutex_bt_.unlock();
  RCLCPP_DEBUG(get_logger(), "----- behavior path planner end -----\n\n");
}

void BehaviorPathPlannerNode::computeTurnSignal(
  const std::shared_ptr<PlannerData> planner_data, const PathWithLaneId & path,
  const BehaviorModuleOutput & output)
{
  TurnIndicatorsCommand turn_signal;
  HazardLightsCommand hazard_signal;
  if (output.turn_signal_info.hazard_signal.command == HazardLightsCommand::ENABLE) {
    turn_signal.command = TurnIndicatorsCommand::DISABLE;
    hazard_signal.command = output.turn_signal_info.hazard_signal.command;
  } else {
    turn_signal = turn_signal_decider_.getTurnSignal(planner_data, path, output.turn_signal_info);
    hazard_signal.command = HazardLightsCommand::DISABLE;
  }
  turn_signal.stamp = get_clock()->now();
  hazard_signal.stamp = get_clock()->now();
  turn_signal_publisher_->publish(turn_signal);
  hazard_signal_publisher_->publish(hazard_signal);

  publish_steering_factor(turn_signal);
}

void BehaviorPathPlannerNode::publish_steering_factor(const TurnIndicatorsCommand & turn_signal)
{
  const auto [intersection_flag, approaching_intersection_flag] =
    turn_signal_decider_.getIntersectionTurnSignalFlag();
  if (intersection_flag || approaching_intersection_flag) {
    const uint16_t steering_factor_direction = std::invoke([&turn_signal]() {
      if (turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT) {
        return SteeringFactor::LEFT;
      }
      return SteeringFactor::RIGHT;
    });

    const auto [intersection_pose, intersection_distance] =
      turn_signal_decider_.getIntersectionPoseAndDistance();
    const uint16_t steering_factor_state = std::invoke([&intersection_flag]() {
      if (intersection_flag) {
        return SteeringFactor::TURNING;
      }
      return SteeringFactor::TRYING;
    });

    steering_factor_interface_ptr_->updateSteeringFactor(
      {intersection_pose, intersection_pose}, {intersection_distance, intersection_distance},
      SteeringFactor::INTERSECTION, steering_factor_direction, steering_factor_state, "");
  } else {
    steering_factor_interface_ptr_->clearSteeringFactors();
  }
  steering_factor_interface_ptr_->publishSteeringFactor(get_clock()->now());
}

void BehaviorPathPlannerNode::publish_bounds(const PathWithLaneId & path)
{
  constexpr double scale_x = 0.2;
  constexpr double scale_y = 0.2;
  constexpr double scale_z = 0.2;
  constexpr double color_r = 0.0 / 256.0;
  constexpr double color_g = 148.0 / 256.0;
  constexpr double color_b = 205.0 / 256.0;
  constexpr double color_a = 0.999;

  const auto current_time = path.header.stamp;
  auto left_marker = tier4_autoware_utils::createDefaultMarker(
    "map", current_time, "left_bound", 0L, Marker::LINE_STRIP,
    tier4_autoware_utils::createMarkerScale(scale_x, scale_y, scale_z),
    tier4_autoware_utils::createMarkerColor(color_r, color_g, color_b, color_a));
  for (const auto lb : path.left_bound) {
    left_marker.points.push_back(lb);
  }

  auto right_marker = tier4_autoware_utils::createDefaultMarker(
    "map", current_time, "right_bound", 0L, Marker::LINE_STRIP,
    tier4_autoware_utils::createMarkerScale(scale_x, scale_y, scale_z),
    tier4_autoware_utils::createMarkerColor(color_r, color_g, color_b, color_a));
  for (const auto rb : path.right_bound) {
    right_marker.points.push_back(rb);
  }

  MarkerArray msg;
  msg.markers.push_back(left_marker);
  msg.markers.push_back(right_marker);
  bound_publisher_->publish(msg);
}

// void BehaviorPathPlannerNode::publishSceneModuleDebugMsg()
// {
//   {
//     const auto debug_messages_data_ptr = bt_manager_->getAllSceneModuleDebugMsgData();
//     const auto avoidance_debug_message = debug_messages_data_ptr->getAvoidanceModuleDebugMsg();
//     if (avoidance_debug_message) {
//       debug_avoidance_msg_array_publisher_->publish(*avoidance_debug_message);
//     }

//     const auto lane_change_debug_message =
//     debug_messages_data_ptr->getLaneChangeModuleDebugMsg(); if (lane_change_debug_message) {
//       debug_lane_change_msg_array_publisher_->publish(*lane_change_debug_message);
//     }
//   }
// }

void BehaviorPathPlannerNode::publishPathCandidate(
  const std::vector<std::shared_ptr<SceneModuleManagerInterface>> & managers)
{
  for (auto & manager : managers) {
    for (auto & module : manager->getSceneModules()) {
      if (path_candidate_publishers_.count(module->name()) != 0) {
        path_candidate_publishers_.at(module->name())
          ->publish(convertToPath(module->getPathCandidate(), module->isExecutionReady()));
      }
    }
  }
}

Path BehaviorPathPlannerNode::convertToPath(
  const std::shared_ptr<PathWithLaneId> & path_candidate_ptr, const bool is_ready)
{
  Path output;
  output.header = planner_data_->route_handler->getRouteHeader();
  output.header.stamp = this->now();

  if (!path_candidate_ptr) {
    return output;
  }

  output = util::toPath(*path_candidate_ptr);
  // header is replaced by the input one, so it is substituted again
  output.header = planner_data_->route_handler->getRouteHeader();
  output.header.stamp = this->now();

  if (!is_ready) {
    for (auto & point : output.points) {
      point.longitudinal_velocity_mps = 0.0;
    }
  }

  return output;
}

PathWithLaneId::SharedPtr BehaviorPathPlannerNode::getPath(
  const BehaviorModuleOutput & bt_output, const std::shared_ptr<PlannerData> planner_data)
{
  // TODO(Horibe) do some error handling when path is not available.

  auto path = bt_output.path ? bt_output.path : planner_data->prev_output_path;
  path->header = planner_data->route_handler->getRouteHeader();
  path->header.stamp = this->now();
  RCLCPP_DEBUG(
    get_logger(), "BehaviorTreeManager: output is %s.", bt_output.path ? "FOUND" : "NOT FOUND");

  PathWithLaneId connected_path;
  // const auto module_status_ptr_vec = bt_manager_->getModulesStatus();
  if (skipSmoothGoalConnection(planner_manager_->getModulesStatus())) {
    connected_path = *path;
  } else {
    connected_path = modifyPathForSmoothGoalConnection(*path);
  }

  const auto resampled_path =
    util::resamplePathWithSpline(connected_path, planner_data_->parameters.path_interval, false);
  // keepInputPoints(module_status_ptr_vec));
  return std::make_shared<PathWithLaneId>(resampled_path);
}

bool BehaviorPathPlannerNode::skipSmoothGoalConnection(
  const std::vector<std::shared_ptr<SceneModuleStatus>> & statuses) const
{
  const auto target_module = "PullOver";

  for (auto & status : statuses) {
    if (status->is_waiting_approval || status->status == ModuleStatus::RUNNING) {
      if (target_module == status->module_name) {
        return true;
      }
    }
  }
  return false;
}

// This is a temporary process until motion planning can take the terminal pose into account
// bool BehaviorPathPlannerNode::keepInputPoints(
//   const std::vector<std::shared_ptr<SceneModuleStatus>> & statuses) const
// {
//   const auto target_module = "PullOver";

//   for (auto & status : statuses) {
//     if (status->is_waiting_approval || status->status == BT::NodeStatus::RUNNING) {
//       if (target_module == status->module_name) {
//         return true;
//       }
//     }
//   }
//   return false;
// }

void BehaviorPathPlannerNode::onOdometry(const Odometry::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_pd_);
  planner_data_->self_odometry = msg;
}
void BehaviorPathPlannerNode::onAcceleration(const AccelWithCovarianceStamped::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_pd_);
  planner_data_->self_acceleration = msg;
}
void BehaviorPathPlannerNode::onPerception(const PredictedObjects::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_pd_);
  planner_data_->dynamic_object = msg;
}
void BehaviorPathPlannerNode::onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_pd_);
  planner_data_->occupancy_grid = msg;
}
void BehaviorPathPlannerNode::onExternalApproval(const ApprovalMsg::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_pd_);
  planner_data_->approval.is_approved.data = msg->approval;
  // TODO(wep21): Replace msg stamp after {stamp: now} is implemented in ros2 topic pub
  planner_data_->approval.is_approved.stamp = this->now();
}
void BehaviorPathPlannerNode::onForceApproval(const PathChangeModule::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_pd_);
  auto getModuleName = [](PathChangeModuleId module) {
    if (module.type == PathChangeModuleId::FORCE_LANE_CHANGE) {
      return "ForceLaneChange";
    } else {
      return "NONE";
    }
  };
  planner_data_->approval.is_force_approved.module_name = getModuleName(msg->module);
  planner_data_->approval.is_force_approved.stamp = msg->header.stamp;
}
void BehaviorPathPlannerNode::onMap(const HADMapBin::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_pd_);
  map_ptr_ = msg;
  has_received_map_ = true;
}
void BehaviorPathPlannerNode::onRoute(const LaneletRoute::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_pd_);
  route_ptr_ = msg;
  has_received_route_ = true;
}

SetParametersResult BehaviorPathPlannerNode::onSetParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  if (planner_manager_->getSceneModuleManagers().empty()) {
    result.successful = false;
    result.reason = "module not initialized";
    return result;
  }

  result.successful = true;
  result.reason = "success";

  try {
    planner_manager_->updateModuleParams(parameters);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

PathWithLaneId BehaviorPathPlannerNode::modifyPathForSmoothGoalConnection(
  const PathWithLaneId & path) const
{
  const auto goal = planner_data_->route_handler->getGoalPose();
  const auto goal_lane_id = planner_data_->route_handler->getGoalLaneId();

  Pose refined_goal{};
  {
    lanelet::ConstLanelet goal_lanelet;
    if (planner_data_->route_handler->getGoalLanelet(&goal_lanelet)) {
      refined_goal = util::refineGoal(goal, goal_lanelet);
    } else {
      refined_goal = goal;
    }
  }

  auto refined_path = util::refinePathForGoal(
    planner_data_->parameters.refine_goal_search_radius_range, M_PI * 0.5, path, refined_goal,
    goal_lane_id);
  refined_path.header.frame_id = "map";
  refined_path.header.stamp = this->now();

  return refined_path;
}
}  // namespace behavior_path_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(behavior_path_planner::BehaviorPathPlannerNode)
