//  Copyright 2022 Sakai Hibiki
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#pragma once

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/empty.hpp"

#include "vl53l5cx/ranging_helper.hpp"
#include "vl53l5cx/vl53l5cx.hpp"

#include "vl53l5cx_params.hpp"

namespace vl53l5cx
{
class RangingHelper;

class VL53L5CXNode : public rclcpp_lifecycle::LifecycleNode
{
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::vector<rclcpp::Service<std_srvs::srv::Empty>::SharedPtr> services_;

  std::vector<std::shared_ptr<VL53L5CX>> sensors_;
  std::unique_ptr<RangingHelper> ranging_helper_;

public:
  using LifecycleNode::LifecycleNode;
  ~VL53L5CXNode();

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

private:
  void initialize();
  void apply_parameters();
  void calibrate_xtalk();
  static std::vector<VL53L5CX::Config> parse_parameters(const Params & params);
};

}  // namespace vl53l5cx
