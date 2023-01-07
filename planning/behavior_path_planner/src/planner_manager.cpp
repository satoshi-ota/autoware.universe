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

#include "behavior_path_planner/planner_manager.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/utilities.hpp>

#include <boost/format.hpp>

#include <memory>
#include <string>

namespace behavior_path_planner
{

PlannerManager::PlannerManager(
  rclcpp::Node & node, const bool enable_simultaneous_execution, const bool verbose)
: logger_(node.get_logger().get_child("planner_manager")),
  clock_(*node.get_clock()),
  enable_simultaneous_execution_{enable_simultaneous_execution},
  verbose_{verbose}
{
}

BehaviorModuleOutput PlannerManager::run(const std::shared_ptr<PlannerData> & data)
{
  if (!start_lanelet_) {
    updateStartLanelet(data);
  }

  std::for_each(scene_manager_ptrs_.begin(), scene_manager_ptrs_.end(), [&data](const auto & m) {
    m->setData(data);
  });

  while (rclcpp::ok()) {
    const auto approved_path_data = update(data);

    const auto candidate_module_id = getCandidateModuleID(approved_path_data);

    if (!candidate_module_id) {
      candidate_module_id_ = boost::none;
      return approved_path_data;
    }

    const auto result = run(candidate_module_id.get(), approved_path_data);

    if (isWaitingApproval(candidate_module_id.get())) {
      candidate_module_id_ = candidate_module_id.get();
      return result;
    }

    candidate_module_id_ = boost::none;
    addApprovedModule(candidate_module_id.get());
  }

  return {};
}

boost::optional<ModuleID> PlannerManager::getCandidateModuleID(
  const BehaviorModuleOutput & path_data) const
{
  const auto module_running = !approved_modules_.empty();
  if (!enable_simultaneous_execution_ && module_running) {
    return {};
  }

  for (const auto & m : scene_manager_ptrs_) {
    /**
     * CASE1: there is no candidate module
     */
    if (!candidate_module_id_) {
      if (m->isExecutionRequested(path_data) && m->canLaunchNewModule()) {
        // launch new candidate module
        return std::make_pair(m, m->launchNewModule(path_data));
      } else {
        // candidate module is not needed
        continue;
      }
    }

    // candidate module already exist
    const auto & manager = candidate_module_id_.get().first;

    /**
     * CASE2: same name module already launched as candidate
     */
    if (manager->getModuleName() == m->getModuleName()) {
      if (isExecutionRequested(candidate_module_id_.get())) {
        // keep current candidate module
        return candidate_module_id_.get();
      } else {
        // candidate module is no longer needed
        deleteExpiredModules(candidate_module_id_.get());
        continue;
      }
    }

    /**
     * CASE3: different name module already launched as candidate
     */

    // don't launch new module as candidate
    if (!m->isExecutionRequested(path_data) || !m->canLaunchNewModule()) {
      continue;
    }

    // delete current candidate module from manager
    deleteExpiredModules(candidate_module_id_.get());

    // override candidate module
    return std::make_pair(m, m->launchNewModule(path_data));
  }

  return {};
}

BehaviorModuleOutput PlannerManager::update(const std::shared_ptr<PlannerData> & data)
{
  BehaviorModuleOutput output = getReferencePath(data);

  bool remove_after_module = false;

  for (auto itr = approved_modules_.begin(); itr != approved_modules_.end();) {
    if (remove_after_module) {
      itr = approved_modules_.erase(itr);
      continue;
    }

    const auto result = run(*itr, output);

    if (!isExecutionRequested(*itr)) {
      if (itr == approved_modules_.begin()) {
        const auto & manager = itr->first;
        const auto & name = manager->getModuleName();
        if (name == "lane_change" || name == "avoidance_by_lc") {
          updateStartLanelet(data);
        }
        deleteExpiredModules(*itr);
        itr = approved_modules_.erase(itr);
        output = result;
        continue;
      }
    }

    if (isWaitingApproval(*itr)) {
      candidate_module_id_ = *itr;
      itr = approved_modules_.erase(itr);
      remove_after_module = true;
      continue;
    }

    output = result;
    itr++;
  }

  return output;
}

BehaviorModuleOutput PlannerManager::getReferencePath(
  const std::shared_ptr<PlannerData> & data) const
{
  PathWithLaneId reference_path{};

  const auto & route_handler = data->route_handler;
  const auto & pose = data->self_pose->pose;
  const auto p = data->parameters;

  // Set header
  reference_path.header = route_handler->getRouteHeader();

  const auto current_lanes = data->route_handler->getLaneletSequence(
    start_lanelet_.get(), pose, p.backward_path_length, p.forward_path_length);
  const auto drivable_lanes = util::generateDrivableLanes(current_lanes);

  // calculate path with backward margin to avoid end points' instability by spline interpolation
  constexpr double extra_margin = 50.0;
  const double backward_length =
    std::max(p.backward_path_length, p.backward_path_length + extra_margin);

  const auto current_lanes_with_backward_margin = route_handler->getLaneletSequence(
    start_lanelet_.get(), pose, backward_length, p.forward_path_length);

  reference_path = util::getCenterLinePath(
    *route_handler, current_lanes_with_backward_margin, pose, backward_length,
    p.forward_path_length, p);

  // clip backward length
  const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    reference_path.points, pose, 3.0, 1.0);
  util::clipPathLength(reference_path, current_seg_idx, p.forward_path_length, backward_length);

  const auto expanded_lanes = util::expandLanelets(drivable_lanes, 0.0, 0.0);
  util::generateDrivableArea(reference_path, expanded_lanes, p.vehicle_length, data);

  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(reference_path);
  output.reference_path = std::make_shared<PathWithLaneId>(reference_path);

  return output;
}

}  // namespace behavior_path_planner
