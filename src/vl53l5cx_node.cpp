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

#include "vl53l5cx/types.hpp"
#include "vl53l5cx/vl53l5cx_node.hpp"

using namespace std::chrono_literals;
using sensor_msgs::msg::Image;
using std_srvs::srv::Empty;
namespace encodings = sensor_msgs::image_encodings;

namespace vl53l5cx
{
static auto get_integer_range(int64_t from_value, int64_t to_value, int64_t step = 1)
{
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = from_value;
  range.to_value = to_value;
  range.step = step;
  return range;
}

VL53L5CXNode::VL53L5CXNode(const std::string & node_name) : Node(node_name)
{
  // Parameter settings
  {  // Address
    rcl_interfaces::msg::ParameterDescriptor d;
    d.name = "address";
    d.description = "7-bit I2C device addresses";
    d.read_only = true;
    d.type = rclcpp::PARAMETER_INTEGER_ARRAY;
    d.additional_constraints = "not 0x29";
    d.integer_range.emplace_back(get_integer_range(0x08, 0x77));
    this->declare_parameter<std::vector<int64_t>>("address", {0x29}, d);
  }
  {  // GPIO
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;
    d.type = rclcpp::PARAMETER_INTEGER_ARRAY;
    d.integer_range.emplace_back(get_integer_range(2, 27));
    auto d_lpn = d, d_int = d;
    d_lpn.description = "LPn gpio pins";
    d_int.description = "INT gpio pins";
    this->declare_parameter<std::vector<int64_t>>("gpio_lpn", {}, d_lpn);
    this->declare_parameter<std::vector<int64_t>>("gpio_int", {}, d_int);
  }
  this->declare_parameter("resolution", 4);
  this->declare_parameter("frequency", 1);
  this->declare_parameter("ranging_mode", 1);

  this->configure_parameters();

  const auto n_devices = addresses_.size();

  // If connecting multiple sensors
  if (n_devices > 1) {
    if (gpios_lpn_.size() != n_devices)
      throw std::invalid_argument(
        "address and gpio_lpn must be the same size when connecting multiple sensors");
  }

  // If using INT (interrupt) pin
  if (!gpios_int_.empty()) {
    if (gpios_int_.size() != n_devices)
      throw std::invalid_argument("address and gpio_int must be the same size when using INT");
    RCLCPP_INFO(this->get_logger(), "INT enabled");
  }

  for (std::size_t i = 0; i < n_devices; ++i) {
    const auto id = ID::get();

    // Construct sensors
    VL53L5CXBuilder builder{id, static_cast<uint8_t>(addresses_[i])};
    if (!gpios_lpn_.empty()) builder.LPn = std::make_shared<GPIO>(gpios_lpn_[i]);
    if (!gpios_int_.empty()) builder.INT = std::make_shared<GPIO>(gpios_int_[i]);
    sensors_.emplace_back(builder.build());

    // Create publishers
    pubs_distance_[id] = this->create_publisher<Image>("~/" + id.get_name() + "/image", 10);
  }
  this->initialize();
  this->apply_parameters();

  // Register callback for when parameters are set
  on_parameters_callback_handle_ =
    this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &) {
      rcl_interfaces::msg::SetParametersResult result;
      have_parameters_changed_ = true;
      result.successful = true;
      return result;
    });

  // Create services
  services_.emplace_back(this->create_service<Empty>(
    "~/start_ranging",
    [this](Empty::Request::SharedPtr, Empty::Response::SharedPtr) { this->start_ranging(); }));
  services_.emplace_back(this->create_service<Empty>(
    "~/stop_ranging",
    [this](Empty::Request::SharedPtr, Empty::Response::SharedPtr) { this->stop_ranging(); }));
}

VL53L5CXNode::~VL53L5CXNode() {}

void VL53L5CXNode::initialize()
{
  if (sensors_.size() == 1) {
    sensors_.at(0)->initialize();
  } else {
    for (auto & e : sensors_) {
      e->disable_comms();
    }

    for (auto & e : sensors_) {
      e->enable_comms();

      std::stringstream ss;
      ss << "0x" << std::hex << static_cast<int>(e->get_address());
      const auto hex_address = ss.str();
      RCLCPP_INFO(this->get_logger(), "Initializing " + e->id.get_name() + " at " + hex_address);

      e->initialize();

      e->disable_comms();
    }

    for (auto & e : sensors_) {
      e->enable_comms();
      assert(e->is_alive());
    }
  }

  RCLCPP_INFO(this->get_logger(), "Successfully initialized");
}

void VL53L5CXNode::apply_parameters()
{
  // Queue this function if in ranging
  if (sensors_.at(0)->is_ranging()) {
    RCLCPP_INFO(
      this->get_logger(), "Ranging now. Parameters will be applied when ranging is stopped.");
    have_parameters_changed_ = true;
    return;
  }

  this->configure_parameters();

  for (auto & e : sensors_) {
    e->set_resolution(resolution_);
    e->set_frequency(frequency_);
    e->set_ranging_mode(ranging_mode_);
    e->apply_parameters();
  }

  RCLCPP_INFO(this->get_logger(), "Applied parameters");
}

void VL53L5CXNode::configure_parameters()
{
  this->get_parameter("address", addresses_);
  this->get_parameter("gpio_lpn", gpios_lpn_);
  this->get_parameter("gpio_int", gpios_int_);
  this->get_parameter("resolution", resolution_);
  this->get_parameter("frequency", frequency_);
  this->get_parameter("ranging_mode", ranging_mode_);
}

void VL53L5CXNode::start_ranging()
{
  if (have_parameters_changed_) {
    this->apply_parameters();
    have_parameters_changed_ = false;
  }

  const std::chrono::milliseconds delay(static_cast<int64_t>(1e3 / frequency_ / addresses_.size()));

  for (auto & e : sensors_) {
    if (e->is_ranging()) {
      assert(e->get_address() == addresses_.at(0));
      RCLCPP_INFO(this->get_logger(), "Already ranging");
      return;
    }

    // Delay the start of ranging for each sensor
    rclcpp::sleep_for(delay);
    e->start_ranging();
  }

  // Set polling
  timer_ = create_wall_timer(3ms, [this] {
    for (auto & e : sensors_) {
      if (e->check_data_ready()) {
        const auto msg = this->convert_to_image_msg(e->get_distance());
        pubs_distance_.at(e->id)->publish(msg);
      };
    }
  });

  RCLCPP_INFO(this->get_logger(), "Start ranging");
}

void VL53L5CXNode::stop_ranging()
{
  if (timer_) timer_->cancel();

  for (auto & e : sensors_) {
    if (!e->is_ranging()) {
      assert(e->get_address() == addresses_.at(0));
      RCLCPP_INFO(this->get_logger(), "Not in ranging");
      return;
    }

    e->stop_ranging();
  }

  RCLCPP_INFO(this->get_logger(), "Stopped ranging");

  if (have_parameters_changed_) {
    this->apply_parameters();
    have_parameters_changed_ = false;
  }
}

template <class T>
Image VL53L5CXNode::convert_to_image_msg(const std::vector<T> & src) const
{
  using std::is_same;
  static_assert(
    is_same<T, float>::value || is_same<T, uint8_t>::value || is_same<T, uint16_t>::value,
    "Only float, uint8_t or uint16_t is available for image encoding");

  Image msg;

  msg.width = resolution_;
  msg.height = resolution_;
  assert(msg.width * msg.height == src.size());

  if (is_same<T, float>::value)
    msg.encoding = encodings::TYPE_32FC1;
  else if (is_same<T, uint8_t>::value)
    msg.encoding = encodings::TYPE_8UC1;
  else
    msg.encoding = encodings::TYPE_16UC1;

  msg.step = msg.width * sizeof(T);
  auto img_ptr = reinterpret_cast<const uint8_t *>(src.data());
  msg.data = std::vector<uint8_t>(img_ptr, img_ptr + msg.step * msg.height);

  return msg;
}

}  // namespace vl53l5cx
