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

#include "sensor_msgs/image_encodings.hpp"
#include "vl53l5cx/ranging_helper.hpp"

using namespace std::chrono_literals;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using std_msgs::msg::Header;
namespace encodings = sensor_msgs::image_encodings;

namespace vl53l5cx
{
RangingHelper::RangingHelper(VL53L5CXNode & node, std::vector<std::shared_ptr<VL53L5CX>> sensors)
: node_(node), sensors_(sensors)
{
  for (const auto & e : sensors_) {
    const auto address = e->get_config().address;
    const auto prefix = "~/x" + get_hex(address);

    // Create publishers
    pubs_distance_[address] =
      node_.create_publisher<Image>(prefix + "/image", rclcpp::SensorDataQoS());
    pubs_camera_info_[address] =
      node_.create_publisher<CameraInfo>(prefix + "/camera_info", rclcpp::SensorDataQoS());
  }
}

void RangingHelper::publish(std::shared_ptr<VL53L5CX> sensor)
{
  const auto & config = sensor->get_config();

  Header header;
  header.frame_id = config.frame_id;
  header.stamp = node_.now();

  try {
    const auto & ranging_results = sensor->get_results();
    auto image = this->convert_to_image_msg(ranging_results.distance, config);
    auto camera_info = this->get_camera_info(config);
    image.header = header;
    camera_info.header = header;

    const auto address = config.address;
    pubs_distance_.at(address)->publish(image);
    pubs_camera_info_.at(address)->publish(camera_info);
  } catch (const DeviceError & e) {
    this->stop();
    RCLCPP_ERROR(node_.get_logger(), e.what());
  }
}

CameraInfo RangingHelper::get_camera_info(const VL53L5CX::Config & config)
{
  CameraInfo msg;

  const auto resolution = static_cast<int>(config.resolution);
  msg.height = resolution;
  msg.width = resolution;

  msg.distortion_model = "plumb_bob";
  msg.d = {0, 0, 0, 0, 0};  // No distortion

  double fov;
  if (config.part_number == PartNumber::VL53L7CX) {
    fov = 60;
  } else {
    fov = 45;
  }

  const double c = (msg.width - 1) / 2.;                // Optical center
  const double f = c / std::tan(fov * M_PI / 180 / 2);  // Focal length
  msg.k = {f, 0, c, 0, f, c, 0, 0, 1};
  msg.p = {f, 0, c, 0, 0, f, c, 0, 0, 0, 1, 0};
  msg.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

  return msg;
}

template <class T>
Image RangingHelper::convert_to_image_msg(
  const std::vector<T> & src, const VL53L5CX::Config & config)
{
  using std::is_same;
  static_assert(
    is_same<T, float>::value || is_same<T, uint8_t>::value || is_same<T, uint16_t>::value,
    "Only float, uint8_t or uint16_t is available for image encoding");

  Image msg;

  const auto resolution = static_cast<int>(config.resolution);
  msg.width = resolution;
  msg.height = resolution;
  assert(msg.width * msg.height == src.size());

  if (is_same<T, float>::value) {
    msg.encoding = encodings::TYPE_32FC1;
  } else if (is_same<T, uint8_t>::value) {
    msg.encoding = encodings::TYPE_8UC1;
  } else {
    msg.encoding = encodings::TYPE_16UC1;
  }

  msg.step = msg.width * sizeof(T);
  auto img_ptr = reinterpret_cast<const uint8_t *>(src.data());
  msg.data = std::vector<uint8_t>(img_ptr, img_ptr + msg.step * msg.height);

  return msg;
}

void PollI2C::start()
{
  // Set polling
  timer_ = node_.create_wall_timer(3ms, [this] {
    try {
      for (auto & e : sensors_) {
        if (e->check_data_ready()) {
          this->publish(e);
        }
      }
    } catch (const DeviceError & e) {
      this->stop();
      RCLCPP_ERROR(node_.get_logger(), e.what());
    }
  });
}

void PollI2C::stop()
{
  if (timer_) timer_->cancel();
}

void DetectInterrupt::start()
{
  // Clear (maybe) pseudo interrupts
  constexpr int timeout = 100;
  for (int i = 0; i < timeout; ++i) {
    try {
      for (auto & e : sensors_) {
        e->check_interrupt();  // This function throws DeviceError if pseudo interrupt is detected
        this->publish(e);
      }
      break;
    } catch (DeviceError &) {
      std::this_thread::sleep_for(1ms);
    }
  }

  // Set polling
  timer_ = node_.create_wall_timer(100us, [this] {
    for (auto & e : sensors_)
      if (e->check_interrupt()) this->publish(e);
  });
}

void DetectInterrupt::stop()
{
  if (timer_) timer_->cancel();
}

}  // namespace vl53l5cx
