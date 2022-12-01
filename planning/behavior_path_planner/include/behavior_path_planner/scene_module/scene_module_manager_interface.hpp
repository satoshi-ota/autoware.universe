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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_MANAGER_INTERFACE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_MANAGER_INTERFACE_HPP_

#include "behavior_path_planner/scene_module/scene_module_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner
{

using unique_identifier_msgs::msg::UUID;

class SceneModuleManagerInterface
{
public:
  SceneModuleManagerInterface(
    rclcpp::Node * node, const std::string & name, const size_t max_module_num)
  : node_(node),
    clock_(*node->get_clock()),
    logger_(node->get_logger().get_child(name)),
    name_(name),
    max_module_num_(max_module_num)
  {
  }

  virtual ~SceneModuleManagerInterface() = default;

  bool isExecutionRequested(const BehaviorModuleOutput & path_data)
  {
    for (const auto & m : registered_modules_) {
      if (!m.second->isWaitingApproval()) {
        return false;
      }
    }

    const auto m = createNewSceneModuleInstance();

    m->setData(planner_data_);
    m->setPath(path_data);
    m->onEntry();
    m->updateData();

    return m->isExecutionRequested();
  }

  bool isExecutionRequested(const UUID & uuid) const
  {
    const auto m = findSceneModule(uuid);

    if (m == nullptr) {
      return false;
    }

    return m->isExecutionRequested();
  }

  bool isWaitingApproval(const UUID & uuid) const
  {
    const auto m = findSceneModule(uuid);

    if (m == nullptr) {
      return false;
    }

    if (!m->isActivated()) {
      return m->isWaitingApproval();
    }

    return false;
  }

  ModuleStatus getCurrentStatus(const UUID & uuid) const
  {
    const auto m = findSceneModule(uuid);

    if (m == nullptr) {
      return ModuleStatus::IDLE;
    }

    return m->getCurrentStatus();
  }

  BehaviorModuleOutput run(const UUID & uuid, const BehaviorModuleOutput & path_data) const
  {
    const auto m = findSceneModule(uuid);

    if (m == nullptr) {
      return {};
    }

    m->setData(planner_data_);
    m->setPath(path_data);
    m->updateData();

    m->lockRTCCommand();
    const auto result = m->run();
    m->unlockRTCCommand();

    m->updateState();

    m->publishRTCStatus();
    m->publishDebugMarker();
    m->publishReferencePath();

    return result;
  }

  UUID launchNewModule(const BehaviorModuleOutput & path_data)
  {
    const auto uuid = generateUUID();
    const auto m = createNewSceneModuleInstance();

    m->setData(planner_data_);
    m->setPath(path_data);
    m->onEntry();

    registered_modules_.insert(std::make_pair(toHexString(uuid), m));

    return uuid;
  }

  void deleteModules(const UUID & uuid)
  {
    const auto m = findSceneModule(uuid);

    if (m == nullptr) {
      return;
    }

    m->onExit();
    m->publishRTCStatus();

    registered_modules_.erase(toHexString(uuid));
  }

  bool exist(const UUID & uuid) const { return registered_modules_.count(toHexString(uuid)) > 0; }

  bool canLaunchNewModule() const { return registered_modules_.size() < max_module_num_; }

  void setData(const std::shared_ptr<PlannerData> & planner_data) { planner_data_ = planner_data; }

  void reset()
  {
    std::for_each(registered_modules_.begin(), registered_modules_.end(), [](const auto & m) {
      m.second->onExit();
    });
    registered_modules_.clear();
  }

  std::string getModuleName() const { return name_; }

  std::vector<std::shared_ptr<SceneModuleInterface>> getSceneModules()
  {
    std::vector<std::shared_ptr<SceneModuleInterface>> modules;
    for (const auto & m : registered_modules_) {
      modules.push_back(m.second);
    }

    return modules;
  }

  virtual void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters) = 0;

protected:
  virtual void getModuleParams(rclcpp::Node * node) = 0;

  virtual std::shared_ptr<SceneModuleInterface> createNewSceneModuleInstance() = 0;

  rclcpp::Node * node_;

  rclcpp::Clock clock_;

  rclcpp::Logger logger_;

  std::string name_;

  std::shared_ptr<PlannerData> planner_data_;

  std::unordered_map<std::string, std::shared_ptr<SceneModuleInterface>> registered_modules_;

private:
  std::shared_ptr<SceneModuleInterface> findSceneModule(const UUID & uuid) const
  {
    const auto itr = registered_modules_.find(toHexString(uuid));
    if (itr == registered_modules_.end()) {
      return nullptr;
    }

    return registered_modules_.at(toHexString(uuid));
  }

  static UUID generateUUID()
  {
    // Generate random number
    UUID uuid;
    std::mt19937 gen(std::random_device{}());
    std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
    std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);

    return uuid;
  }

  static std::string toHexString(const unique_identifier_msgs::msg::UUID & uuid)
  {
    std::stringstream ss;
    for (auto i = 0; i < 16; ++i) {
      ss << std::hex << std::setfill('0') << std::setw(2) << +uuid.uuid[i];
    }
    return ss.str();
  }

  uint16_t max_module_num_;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_MANAGER_INTERFACE_HPP_
