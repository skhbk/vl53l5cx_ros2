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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/empty.hpp"
#include "vl53l5cx/vl53l5cx.hpp"

namespace vl53l5cx
{
class VL53L5CXNode : public rclcpp::Node
{
  std::map<ID, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pubs_distance_;
  std::map<ID, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> pubs_camera_info_;

  std::vector<rclcpp::Service<std_srvs::srv::Empty>::SharedPtr> services_;

  OnSetParametersCallbackHandle::SharedPtr on_parameters_callback_handle_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::unique_ptr<VL53L5CX>> sensors_;

  std::vector<int64_t> addresses_, gpios_lpn_, gpios_int_;
  uint8_t resolution_, frequency_, ranging_mode_;
  bool have_parameters_changed_ = false;

public:
  explicit VL53L5CXNode(const std::string & node_name);
  ~VL53L5CXNode();

  void initialize();
  void apply_parameters();
  void start_ranging();
  void stop_ranging();

private:
  void configure_parameters();
  sensor_msgs::msg::CameraInfo get_camera_info() const;

  template <class T>
  sensor_msgs::msg::Image convert_to_image_msg(const std::vector<T> & src) const;
};

}  // namespace vl53l5cx
