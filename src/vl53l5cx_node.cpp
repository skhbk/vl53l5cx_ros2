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

#include "vl53l5cx/vl53l5cx_node.hpp"

#define DEFAULT_ADDRESS 0x29
#define DEFAULT_RESOLUTION 4
#define DEFAULT_FREQUENCY 1
#define DEFAULT_RANGING_MODE 0

using std_srvs::srv::Empty;

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
    this->declare_parameter<std::vector<int64_t>>("address", {DEFAULT_ADDRESS}, d);
  }
  {  // GPIO
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;
    d.type = rclcpp::PARAMETER_INTEGER_ARRAY;
    d.integer_range.emplace_back(get_integer_range(0, 254));
    auto d_lpn = d, d_int = d;
    d_lpn.description = "LPn GPIO pins";
    d_int.description = "INT GPIO pins";
    this->declare_parameter<std::vector<int64_t>>("lpn_pin", {}, d_lpn);
    this->declare_parameter<std::vector<int64_t>>("int_pin", {}, d_int);
  }
  this->declare_parameter("resolution", DEFAULT_RESOLUTION);
  this->declare_parameter("frequency", DEFAULT_FREQUENCY);
  this->declare_parameter("ranging_mode", DEFAULT_RANGING_MODE);

  const auto configs = this->parse_parameters();

  // Construct sensors
  for (const auto & config : configs) sensors_.emplace_back(std::make_shared<VL53L5CX>(config));

  this->initialize();

  if (configs[0].int_pin != PinNaN) {
    RCLCPP_INFO(this->get_logger(), "INT enabled");
    throw std::runtime_error("Interrupt detection is not implemented");
  } else {
    ranging_helper_ = std::make_unique<PollI2C>(*this, sensors_);
  }

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
      RCLCPP_INFO(
        this->get_logger(), "Initializing device at 0x" + get_hex(e->get_config().address));
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

  const auto configs = this->parse_parameters();
  for (std::size_t i = 0; i < sensors_.size(); ++i) {
    sensors_[i]->set_config(configs[i]);
  }

  RCLCPP_INFO(this->get_logger(), "Applied parameters");
}

std::vector<VL53L5CX::Config> VL53L5CXNode::parse_parameters() const
{
  std::vector<int64_t> addresses, lpn_pins, int_pins;
  uint8_t resolution, frequency, ranging_mode;
  this->get_parameter("address", addresses);
  this->get_parameter("lpn_pin", lpn_pins);
  this->get_parameter("int_pin", int_pins);
  this->get_parameter("resolution", resolution);
  this->get_parameter("ranging_mode", ranging_mode);
  this->get_parameter("frequency", frequency);

  const auto n_devices = addresses.size();

  // If connecting multiple sensors
  if (n_devices > 1) {
    if (lpn_pins.size() != n_devices)
      throw std::invalid_argument(
        "address and lpn_pin must be the same size when connecting multiple sensors");
  }

  // If using INT (interrupt) pin
  if (!int_pins.empty()) {
    if (int_pins.size() != n_devices)
      throw std::invalid_argument("address and int_pin must be the same size when using INT");
  }

  Resolution resolution_parsed;
  if (resolution == 4)
    resolution_parsed = Resolution::X4;
  else if (resolution == 8)
    resolution_parsed = Resolution::X8;
  else
    throw std::invalid_argument("resolution must be 4 or 8");

  RangingMode ranging_mode_parsed;
  if (ranging_mode == 0)
    ranging_mode_parsed = RangingMode::CONTINUOUS;
  else
    ranging_mode_parsed = RangingMode::AUTONOMOUS;

  std::vector<VL53L5CX::Config> configs(n_devices);
  for (std::size_t i = 0; i < n_devices; ++i) {
    configs[i].address = static_cast<uint8_t>(addresses[i]);
    if (!lpn_pins.empty()) configs[i].lpn_pin = lpn_pins[i];
    if (!int_pins.empty()) configs[i].int_pin = int_pins[i];
    configs[i].resolution = resolution_parsed;
    configs[i].ranging_mode = ranging_mode_parsed;
    configs[i].frequency = static_cast<uint8_t>(frequency);
  }

  return configs;
}

void VL53L5CXNode::start_ranging()
{
  if (sensors_[0]->is_ranging()) {
    RCLCPP_INFO(this->get_logger(), "Already ranging");
    return;
  }

  if (have_parameters_changed_) {
    this->apply_parameters();
    have_parameters_changed_ = false;
  }

  const int frequency = sensors_[0]->get_config().frequency;
  const std::chrono::milliseconds delay(static_cast<int64_t>(1e3 / frequency / sensors_.size()));

  // Delay the start of ranging for each sensor
  for (auto & e : sensors_) {
    rclcpp::sleep_for(delay);
    e->start_ranging();
  }

  // Start publishing ranging data
  ranging_helper_->start();

  RCLCPP_INFO(this->get_logger(), "Start ranging");
}

void VL53L5CXNode::stop_ranging()
{
  if (!sensors_[0]->is_ranging()) {
    RCLCPP_INFO(this->get_logger(), "Not in ranging");
    return;
  }

  // Stop publishing ranging data
  ranging_helper_->stop();

  for (auto & e : sensors_) e->stop_ranging();

  RCLCPP_INFO(this->get_logger(), "Stopped ranging");

  if (have_parameters_changed_) {
    this->apply_parameters();
    have_parameters_changed_ = false;
  }
}

std::string VL53L5CXNode::get_sensor_name(Address address) const
{
  return std::string(this->get_fully_qualified_name()) + "_" + get_hex(address);
}

}  // namespace vl53l5cx
