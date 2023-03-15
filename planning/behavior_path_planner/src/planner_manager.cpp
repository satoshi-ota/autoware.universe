// Copyright 2023 TIER IV, Inc.
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
  rclcpp::Node & node, const std::shared_ptr<LaneFollowingParameters> & parameters,
  const bool verbose)
: parameters_{parameters},
  logger_(node.get_logger().get_child("planner_manager")),
  clock_(*node.get_clock()),
  verbose_{verbose}
{
  processing_time_.emplace("total_time", 0.0);
}

BehaviorModuleOutput PlannerManager::run(const std::shared_ptr<PlannerData> & data)
{
  resetProcessingTime();
  stop_watch_.tic("total_time");

  if (!root_lanelet_) {
    root_lanelet_ = updateRootLanelet(data);
  }

  std::for_each(
    manager_ptrs_.begin(), manager_ptrs_.end(), [&data](const auto & m) { m->setData(data); });

  while (rclcpp::ok()) {
    /**
     * STEP1: get approved modules' output
     */
    const auto approved_path_data = update(data);

    /**
     * STEP2: check modules that need to be launched
     */
    const auto candidate_modules = getCandidateModules(approved_path_data);

    /**
     * STEP3: if there is no module that need to be launched, return approved modules' output
     */
    if (candidate_modules.empty()) {
      processing_time_.at("total_time") = stop_watch_.toc("total_time", true);
      return approved_path_data;
    }

    /**
     * STEP4: if there is module that should be launched, execute the module
     */
    const auto [highest_priority_module, result] =
      getHighestPriorityModule(candidate_modules, data, approved_path_data);

    updateCandidateModules(candidate_modules, highest_priority_module);

    /**
     * STEP5: if the candidate module's modification is NOT approved yet, return the result.
     * NOTE: the result is output of the candidate module, but the output path don't contains path
     * shape modification that needs approval. On the other hand, it could include velocity profile
     * modification.
     */
    if (highest_priority_module->isWaitingApproval()) {
      processing_time_.at("total_time") = stop_watch_.toc("total_time", true);
      return result;
    }

    /**
     * STEP6: if the candidate module is approved, push the module into approved_module_ptrs_
     */
    addApprovedModule(highest_priority_module);
  }

  processing_time_.at("total_time") = stop_watch_.toc("total_time", true);

  return {};
}

std::vector<SceneModulePtr> PlannerManager::getCandidateModules(
  const BehaviorModuleOutput & previous_module_output) const
{
  std::vector<SceneModulePtr> request_modules{};

  const auto block_simultaneous_execution = [this]() {
    for (const auto & module_ptr : approved_module_ptrs_) {
      const auto itr = std::find_if(
        manager_ptrs_.begin(), manager_ptrs_.end(),
        [&module_ptr](const auto & m) { return m->getModuleName() == module_ptr->name(); });
      if (itr == manager_ptrs_.end()) {
        return true;
      }
      if (!(*itr)->isSimultaneousExecutable()) {
        return true;
      }
    }
    return false;
  }();

  // pickup execution requested modules
  for (const auto & manager_ptr : manager_ptrs_) {
    stop_watch_.tic(manager_ptr->getModuleName());

    const auto toc = [this, &manager_ptr]() {
      const auto name = manager_ptr->getModuleName();
      processing_time_.at(name) += stop_watch_.toc(name, true);
    };

    // already exist the modules that don't support simultaneous execution. -> DO NOTHING.
    if (block_simultaneous_execution) {
      toc();
      continue;
    }

    // the module doesn't support simultaneous execution. -> DO NOTHING.
    if (!approved_module_ptrs_.empty() && !manager_ptr->isSimultaneousExecutable()) {
      toc();
      continue;
    }

    // check there is a launched module which is waiting approval as candidate.
    if (candidate_module_ptrs_.empty()) {
      // launched module num reach limit. -> CAN'T LAUNCH NEW MODULE. DO NOTHING.
      if (!manager_ptr->canLaunchNewModule()) {
        toc();
        continue;
      }

      // the module requests it to be launch. -> CAN LAUNCH THE MODULE. PUSH BACK AS REQUEST
      // MODULES.
      const auto new_module_ptr = manager_ptr->getNewModule();
      if (manager_ptr->isExecutionRequested(new_module_ptr, previous_module_output)) {
        request_modules.emplace_back(new_module_ptr);
      }

      toc();
      continue;
    }

    for (const auto & module_ptr : candidate_module_ptrs_) {
      const auto & name = module_ptr->name();
      const auto itr = std::find_if(
        manager_ptrs_.begin(), manager_ptrs_.end(),
        [&name](const auto & m) { return m->getModuleName() == name; });

      if (itr == manager_ptrs_.end()) {
        continue;
      }

      // when the approved module throw another approval request, block all. -> CLEAR REQUEST
      // MODULES AND PUSH BACK.
      if (module_ptr->isLockedNewModuleLaunch()) {
        request_modules.clear();
        request_modules.emplace_back(module_ptr);
        toc();
        return request_modules;
      }
    }

    {
      const auto & name = manager_ptr->getModuleName();
      const auto itr = std::find_if(
        candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(),
        [&name](const auto & m) { return m->name() == name; });

      // same name module already launched as candidate.
      if (itr != candidate_module_ptrs_.end()) {
        // the module launched as candidate and is running now. the module hasn't thrown any
        // approval yet. -> PUSH BACK AS REQUEST MODULES.
        if ((*itr)->getCurrentStatus() == ModuleStatus::RUNNING) {
          request_modules.emplace_back(*itr);
        } else {
          // TODO(Satoshi OTA) this line is no longer needed? think later.
          manager_ptr->deleteModules(*itr);
        }

        toc();
        continue;
      }
    }

    // different name module already launched as candidate.
    // launched module num reach limit. -> CAN'T LAUNCH NEW MODULE. DO NOTHING.
    if (!manager_ptr->canLaunchNewModule()) {
      toc();
      continue;
    }

    // the module requests it to be launch. -> CAN LAUNCH THE MODULE. PUSH BACK AS REQUEST MODULES.
    const auto new_module_ptr = manager_ptr->getNewModule();
    if (!manager_ptr->isExecutionRequested(new_module_ptr, previous_module_output)) {
      toc();
      continue;
    }

    toc();
    request_modules.emplace_back(new_module_ptr);
  }

  return request_modules;
}

SceneModulePtr PlannerManager::selectHighestPriorityModule(
  std::vector<SceneModulePtr> & request_modules) const
{
  if (request_modules.empty()) {
    return {};
  }

  // TODO(someone) enhance this priority decision method.
  std::sort(request_modules.begin(), request_modules.end(), [this](auto a, auto b) {
    return getManager(a)->getPriority() < getManager(b)->getPriority();
  });

  return request_modules.front();
}

std::pair<SceneModulePtr, BehaviorModuleOutput> PlannerManager::getHighestPriorityModule(
  const std::vector<SceneModulePtr> & request_modules, const std::shared_ptr<PlannerData> & data,
  const BehaviorModuleOutput & previous_module_output)
{
  std::vector<SceneModulePtr> waiting_approved_modules;
  std::vector<SceneModulePtr> already_approved_modules;
  std::unordered_map<std::string, BehaviorModuleOutput> results;

  for (const auto & module_ptr : request_modules) {
    const auto & manager_ptr = getManager(module_ptr);

    if (!manager_ptr->exist(module_ptr)) {
      manager_ptr->registerNewModule(module_ptr, previous_module_output);
    }

    const auto result = run(module_ptr, data, previous_module_output);
    results.emplace(module_ptr->name(), result);

    if (module_ptr->isWaitingApproval()) {
      waiting_approved_modules.push_back(module_ptr);
    } else {
      already_approved_modules.push_back(module_ptr);
    }
  }

  // select one module to run as candidate module.
  const auto module_ptr = [&]() {
    if (!already_approved_modules.empty()) {
      return selectHighestPriorityModule(already_approved_modules);
    }

    if (!waiting_approved_modules.empty()) {
      return selectHighestPriorityModule(waiting_approved_modules);
    }

    return SceneModulePtr();
  }();

  return std::make_pair(module_ptr, results.at(module_ptr->name()));
}

BehaviorModuleOutput PlannerManager::update(const std::shared_ptr<PlannerData> & data)
{
  BehaviorModuleOutput output = getReferencePath(data);  // generate root reference path.

  bool remove_after_module = false;

  for (auto itr = approved_module_ptrs_.begin(); itr != approved_module_ptrs_.end();) {
    const auto & name = (*itr)->name();

    stop_watch_.tic(name);

    // if one of the approved modules changes to waiting approval, remove all behind modules.
    if (remove_after_module) {
      itr = approved_module_ptrs_.erase(itr);
      processing_time_.at(name) += stop_watch_.toc(name, true);
      continue;
    }

    const auto result = run(*itr, data, output);  // execute approved module planning.

    // check the module is necessary or not.
    if ((*itr)->getCurrentStatus() != ModuleStatus::RUNNING) {
      if (itr == approved_module_ptrs_.begin()) {
        // update root lanelet when the lane change is done.
        if (name == "lane_change") {
          root_lanelet_ = updateRootLanelet(data);
        }

        deleteExpiredModules(*itr);  // unregister the expired module from manager.
        itr = approved_module_ptrs_.erase(itr);
        output = result;
        processing_time_.at(name) += stop_watch_.toc(name, true);
        continue;
      }
    }

    // if one of the approved modules is waiting approval, the module is popped as candidate
    // module again.
    if ((*itr)->isWaitingApproval()) {
      // for (const auto & m : candidate_module_ptrs_) {
      //   deleteExpiredModules(m);
      // }
      // candidate_module_ptrs_.clear();
      candidate_module_ptrs_.push_back(*itr);
      // if (!!candidate_module_opt_) {
      //   deleteExpiredModules(candidate_module_opt_.get());
      // }
      // candidate_module_opt_ = *itr;
      itr = approved_module_ptrs_.erase(itr);
      remove_after_module = true;
      processing_time_.at(name) += stop_watch_.toc(name, true);
      continue;
    }

    processing_time_.at(name) += stop_watch_.toc(name, true);
    output = result;
    itr++;
  }

  return output;
}

BehaviorModuleOutput PlannerManager::getReferencePath(
  const std::shared_ptr<PlannerData> & data) const
{
  PathWithLaneId reference_path{};

  constexpr double extra_margin = 10.0;

  const auto & route_handler = data->route_handler;
  const auto & pose = data->self_odometry->pose.pose;
  const auto p = data->parameters;

  reference_path.header = route_handler->getRouteHeader();

  const auto backward_length =
    std::max(p.backward_path_length, p.backward_path_length + extra_margin);

  const auto lanelet_sequence = route_handler->getLaneletSequence(
    root_lanelet_.get(), pose, backward_length, std::numeric_limits<double>::max());

  lanelet::ConstLanelet closest_lane{};
  if (!lanelet::utils::query::getClosestLanelet(lanelet_sequence, pose, &closest_lane)) {
    return {};
  }

  const auto current_lanes =
    route_handler->getLaneletSequence(closest_lane, pose, backward_length, p.forward_path_length);

  reference_path = util::getCenterLinePath(
    *route_handler, current_lanes, pose, backward_length, p.forward_path_length, p);

  // clip backward length
  const size_t current_seg_idx = data->findEgoSegmentIndex(reference_path.points);
  util::clipPathLength(
    reference_path, current_seg_idx, p.forward_path_length, p.backward_path_length);
  const auto drivable_lanelets = util::getLaneletsFromPath(reference_path, route_handler);
  const auto drivable_lanes = util::generateDrivableLanes(drivable_lanelets);

  {
    const int num_lane_change =
      std::abs(route_handler->getNumLaneToPreferredLane(current_lanes.back()));

    const double lane_change_buffer = util::calcLaneChangeBuffer(p, num_lane_change);

    reference_path = util::setDecelerationVelocity(
      *route_handler, reference_path, current_lanes, parameters_->lane_change_prepare_duration,
      lane_change_buffer);
  }

  const auto shorten_lanes = util::cutOverlappedLanes(reference_path, drivable_lanes);

  const auto expanded_lanes = util::expandLanelets(
    shorten_lanes, parameters_->drivable_area_left_bound_offset,
    parameters_->drivable_area_right_bound_offset, parameters_->drivable_area_types_to_skip);

  util::generateDrivableArea(reference_path, expanded_lanes, p.vehicle_length, data);

  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(reference_path);
  output.reference_path = std::make_shared<PathWithLaneId>(reference_path);
  output.drivable_lanes = drivable_lanes;

  return output;
}

void PlannerManager::updateCandidateModules(
  const std::vector<SceneModulePtr> & candidate_modules,
  const SceneModulePtr & highest_priority_module)
{
  {
    const auto already_exist = [&](const auto & module_ptr) {
      const auto itr = std::find_if(
        candidate_modules.begin(), candidate_modules.end(),
        [&module_ptr](const auto & m) { return m->name() == module_ptr->name(); });

      return itr != candidate_modules.end();
    };

    for (auto itr = candidate_module_ptrs_.begin(); itr != candidate_module_ptrs_.end();) {
      if (!already_exist(*itr)) {
        itr = candidate_module_ptrs_.erase(itr);
        continue;
      }

      if (
        (*itr)->name() == highest_priority_module->name() &&
        !highest_priority_module->isWaitingApproval()) {
        itr = candidate_module_ptrs_.erase(itr);
        continue;
      }

      itr++;
    }
  }

  {
    const auto already_exist = [&](const auto & module_ptr) {
      const auto itr = std::find_if(
        candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(),
        [&module_ptr](const auto & m) { return m->name() == module_ptr->name(); });

      return itr != candidate_module_ptrs_.end();
    };

    for (const auto & m : candidate_modules) {
      if (
        m->name() == highest_priority_module->name() &&
        !highest_priority_module->isWaitingApproval()) {
        continue;
      }

      if (!already_exist(m)) {
        candidate_module_ptrs_.push_back(m);
      }
    }
  }

  std::sort(candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), [this](auto a, auto b) {
    return getManager(a)->getPriority() < getManager(b)->getPriority();
  });

  // for (const auto & m : candidate_module_ptrs_) {
  //   std::cout << "can1:" << m->name() << std::endl;
  // }

  // {
  //   std::cout << "can2:" << highest_priority_module->name() << std::endl;
  // }

  // for (const auto & m : candidate_module_ptrs_) {
  //   std::cout << "can3:" << m->name() << std::endl;
  // }
}

void PlannerManager::resetRootLanelet(const std::shared_ptr<PlannerData> & data)
{
  if (!root_lanelet_) {
    root_lanelet_ = updateRootLanelet(data);
    return;
  }

  const auto root_lanelet = updateRootLanelet(data);

  // check ego is in same lane
  if (root_lanelet_.get().id() == root_lanelet.id()) {
    return;
  }

  const auto route_handler = data->route_handler;
  const auto next_lanelets = route_handler->getRoutingGraphPtr()->following(root_lanelet_.get());

  // check ego is in next lane
  for (const auto & next : next_lanelets) {
    if (next.id() == root_lanelet_.get().id()) {
      return;
    }
  }

  if (!approved_module_ptrs_.empty()) {
    return;
  }

  for (const auto & m : candidate_module_ptrs_) {
    if (m->isLockedNewModuleLaunch()) {
      return;
    }
  }

  reset();

  RCLCPP_INFO_EXPRESSION(logger_, verbose_, "change ego's following lane. reset.");
}

void PlannerManager::print() const
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
  for (const auto & m : manager_ptrs_) {
    string_stream << "[" << m->getModuleName() << "]";
  }

  string_stream << "\n";
  string_stream << "approved modules  : ";
  for (const auto & m : approved_module_ptrs_) {
    string_stream << "[" << m->name() << "]->";
  }

  string_stream << "\n";
  string_stream << "candidate module  : ";
  for (const auto & m : candidate_module_ptrs_) {
    string_stream << "[" << m->name() << "]->";
  }

  string_stream << "\n" << std::fixed << std::setprecision(1);
  string_stream << "processing time   : ";
  for (const auto & t : processing_time_) {
    string_stream << std::right << "[" << std::setw(16) << std::left << t.first << ":"
                  << std::setw(4) << std::right << t.second << "ms]\n"
                  << std::setw(21);
  }

  RCLCPP_INFO_STREAM(logger_, string_stream.str());
}

}  // namespace behavior_path_planner
