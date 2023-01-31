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

#include "behavior_path_planner/scene_module/avoidance/avoidance_module_data.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_module_data.hpp"

#include <memory>

namespace behavior_path_planner
{

struct AvoidanceByLCParameters
{
  // avoidance
  std::shared_ptr<AvoidanceParameters> avoidance;

  // lane change
  std::shared_ptr<LaneChangeParameters> lane_change;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE_BY_LC__AVOIDANCE_BY_LC_MODULE_DATA_HPP_
