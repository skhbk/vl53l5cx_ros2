//  Copyright 2023 Sakai Hibiki
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

#include <future>

#include "vl53l5cx/vl53l5cx_node.hpp"

#include "multiline_params.hpp"

class MultilineManager : public rclcpp_lifecycle::LifecycleNode
{
public:
  std::vector<std::shared_ptr<vl53l5cx::VL53L5CXNode>> nodes;

  using LifecycleNode::LifecycleNode;
  ~MultilineManager() = default;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    for (auto & node : nodes) {
      node->configure();
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    for (auto & node : nodes) {
      node->cleanup();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    for (auto & node : nodes) {
      node->shutdown();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    std::vector<std::future<void>> futures;

    for (auto & node : nodes) {
      auto future = std::async(std::launch::async, [&] { node->activate(); });
      futures.push_back(std::move(future));
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    for (auto & node : nodes) {
      node->deactivate();
    }
    return CallbackReturn::SUCCESS;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const auto manager = std::make_shared<MultilineManager>("vl53l5cx");
  auto param_listener =
    std::make_shared<multiline::ParamListener>(manager->get_node_parameters_interface());
  const auto params = param_listener->get_params();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(manager->get_node_base_interface());

  for (const auto & name : params.nodes) {
    auto node = std::make_shared<vl53l5cx::VL53L5CXNode>(name);
    executor.add_node(node->get_node_base_interface());
    manager->nodes.push_back(node);
  }

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
