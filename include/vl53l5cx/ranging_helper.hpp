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

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vl53l5cx/vl53l5cx.hpp"
#include "vl53l5cx/vl53l5cx_node.hpp"

namespace vl53l5cx
{
class VL53L5CXNode;

class RangingHelper
{
protected:
  VL53L5CXNode & node_;
  std::vector<std::shared_ptr<VL53L5CX>> sensors_;

  std::map<Address, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pubs_distance_;
  std::map<Address, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> pubs_camera_info_;

public:
  RangingHelper(VL53L5CXNode & node, std::vector<std::shared_ptr<VL53L5CX>> sensors);
  virtual void start() = 0;
  virtual void stop() = 0;

protected:
  void publish(std::shared_ptr<VL53L5CX> sensor);
  static sensor_msgs::msg::CameraInfo get_camera_info(const VL53L5CX::Config & config);

  template <class T>
  static sensor_msgs::msg::Image convert_to_image_msg(
    const std::vector<T> & src, const VL53L5CX::Config & config);
};

class PollI2C : public RangingHelper
{
  rclcpp::TimerBase::SharedPtr timer_;

public:
  using RangingHelper::RangingHelper;
  void start() override;
  void stop() override;
};

}  // namespace vl53l5cx