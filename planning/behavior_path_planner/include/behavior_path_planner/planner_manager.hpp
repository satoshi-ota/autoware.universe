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

#ifndef BEHAVIOR_PATH_PLANNER__PLANNER_MANAGER_HPP_
#define BEHAVIOR_PATH_PLANNER__PLANNER_MANAGER_HPP_

#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/scene_module/scene_module_manager_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{

using autoware_auto_planning_msgs::msg::PathWithLaneId;
using unique_identifier_msgs::msg::UUID;
using ModuleID = std::pair<std::shared_ptr<SceneModuleManagerInterface>, UUID>;

class PlannerManager
{
public:
  PlannerManager(rclcpp::Node & node, const bool enable_simultaneous_execution, const bool verbose);

  BehaviorModuleOutput run(const std::shared_ptr<PlannerData> & data);

  void registerSceneModuleManager(
    const std::shared_ptr<SceneModuleManagerInterface> & scene_module_manager_ptr)
  {
    RCLCPP_INFO(logger_, "register %s module", scene_module_manager_ptr->getModuleName().c_str());
    scene_manager_ptrs_.push_back(scene_module_manager_ptr);
  }

  void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters)
  {
    std::for_each(
      scene_manager_ptrs_.begin(), scene_manager_ptrs_.end(),
      [&parameters](const auto & m) { m->updateModuleParams(parameters); });
  }

  void reset()
  {
    approved_modules_.clear();
    candidate_module_id_ = boost::none;
    start_lanelet_ = boost::none;
    std::for_each(
      scene_manager_ptrs_.begin(), scene_manager_ptrs_.end(), [](const auto & m) { m->reset(); });
  }

  void print()
  {
    if (!verbose_) {
      return;
    }

    std::ostringstream string_stream;
    string_stream << "\n";
    string_stream << "***********************************************************\n";
    string_stream << "                  planner manager status\n";
    string_stream << "-----------------------------------------------------------\n";
    string_stream << "registered modules: ";
    for (const auto & m : scene_manager_ptrs_) {
      string_stream << "[" << m->getModuleName() << "]";
    }

    string_stream << "\n";
    string_stream << "approved modules  : ";
    for (const auto & id : approved_modules_) {
      const auto & manager = id.first;
      string_stream << "[" << manager->getModuleName() << "]->";
    }

    string_stream << "\n";
    string_stream << "candidate module  : ";
    if (!!candidate_module_id_) {
      const auto & manager = candidate_module_id_.get().first;
      string_stream << "[" << manager->getModuleName() << "]";
    }

    RCLCPP_INFO_STREAM(logger_, string_stream.str());
  }

  std::vector<std::shared_ptr<SceneModuleManagerInterface>> getSceneModuleManagers()
  {
    return scene_manager_ptrs_;
  }

private:
  BehaviorModuleOutput run(
    const ModuleID & id, const BehaviorModuleOutput & previous_module_output) const
  {
    const auto & manager = id.first;
    const auto & uuid = id.second;

    if (manager == nullptr) {
      return {};
    }

    if (!manager->exist(uuid)) {
      return {};
    }

    return manager->run(uuid, previous_module_output);
  }

  bool isExecutionRequested(const ModuleID & id) const
  {
    const auto & manager = id.first;
    const auto & uuid = id.second;

    if (manager == nullptr) {
      return false;
    }

    if (!manager->exist(uuid)) {
      return false;
    }

    return manager->isExecutionRequested(uuid);
  }

  bool isWaitingApproval(const ModuleID & id) const
  {
    const auto & manager = id.first;
    const auto & uuid = id.second;

    if (manager == nullptr) {
      return false;
    }

    if (!manager->exist(uuid)) {
      return false;
    }

    return manager->isWaitingApproval(uuid);
  }

  bool isRunning(const ModuleID & id) const
  {
    const auto & manager = id.first;
    const auto & uuid = id.second;

    if (manager == nullptr) {
      return false;
    }

    if (!manager->exist(uuid)) {
      return false;
    }

    return manager->getCurrentStatus(uuid) == ModuleStatus::RUNNING;
  }

  void deleteExpiredModules(const ModuleID & id) const
  {
    const auto & manager = id.first;
    const auto & uuid = id.second;

    if (manager == nullptr) {
      return;
    }

    if (!manager->exist(uuid)) {
      return;
    }

    manager->deleteModules(uuid);
  }

  void addApprovedModule(const ModuleID & id) { approved_modules_.push_back(id); }

  void updateStartLanelet(const std::shared_ptr<PlannerData> & data)
  {
    start_lanelet_ = lanelet::ConstLanelet();
    data->route_handler->getClosestLaneletWithinRoute(
      data->self_pose->pose, start_lanelet_.get_ptr());
    RCLCPP_WARN(logger_, "update start lanelet. id:%ld", start_lanelet_.get().id());
  }

  BehaviorModuleOutput update(const std::shared_ptr<PlannerData> & data);

  BehaviorModuleOutput getReferencePath(const std::shared_ptr<PlannerData> & data) const;

  boost::optional<ModuleID> getCandidateModuleID(
    const BehaviorModuleOutput & previous_module_output) const;

  boost::optional<ModuleID> candidate_module_id_{boost::none};

  boost::optional<lanelet::ConstLanelet> start_lanelet_{boost::none};

  std::vector<std::shared_ptr<SceneModuleManagerInterface>> scene_manager_ptrs_;

  std::vector<ModuleID> approved_modules_;

  rclcpp::Logger logger_;

  rclcpp::Clock clock_;

  bool enable_simultaneous_execution_{false};

  bool verbose_{false};
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__PLANNER_MANAGER_HPP_
