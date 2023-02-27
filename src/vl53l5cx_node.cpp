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

namespace vl53l5cx
{

using std_srvs::srv::Empty;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

VL53L5CXNode::~VL53L5CXNode() {}

CallbackReturn VL53L5CXNode::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();
  const auto configs = parse_parameters(params_);

  // Construct sensors
  for (const auto & config : configs) {
    sensors_.emplace_back(std::make_shared<VL53L5CX>(config));
  }

  if (!params_.int_pin.empty()) {
    RCLCPP_INFO(this->get_logger(), "INT enabled");
    ranging_helper_ = std::make_unique<DetectInterrupt>(*this, sensors_);
  } else {
    ranging_helper_ = std::make_unique<PollI2C>(*this, sensors_);
  }

  this->initialize();

  // Create services
  services_.emplace_back(this->create_service<Empty>(
    "~/calibrate_xtalk",
    [this](Empty::Request::SharedPtr, Empty::Response::SharedPtr) { this->calibrate_xtalk(); }));

  return CallbackReturn::SUCCESS;
}

CallbackReturn VL53L5CXNode::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  param_listener_.reset();
  sensors_.clear();
  ranging_helper_.reset();
  services_.clear();

  return CallbackReturn::SUCCESS;
}

CallbackReturn VL53L5CXNode::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
  param_listener_.reset();
  sensors_.clear();
  ranging_helper_.reset();
  services_.clear();

  return CallbackReturn::SUCCESS;
}

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
        this->get_logger(),
        ("Initializing device at 0x" + get_hex(e->get_config().address)).c_str());
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
  assert(!sensors_.at(0)->is_ranging());

  if (!param_listener_->is_old(params_)) {
    return;
  }

  params_ = param_listener_->get_params();
  const auto configs = parse_parameters(params_);

  for (std::size_t i = 0; i < sensors_.size(); ++i) {
    sensors_[i]->set_config(configs[i]);
  }

  RCLCPP_INFO(this->get_logger(), "Applied parameters");
}

std::vector<VL53L5CX::Config> VL53L5CXNode::parse_parameters(const Params & params)
{
  const auto n_devices = params.address.size();

  if (params.frame_id.size() != n_devices) {
    throw std::invalid_argument("'address' and 'frame_id' must be the same size");
  }

  // If connecting multiple sensors
  if (n_devices > 1) {
    if (params.rst_pin.size() != n_devices && params.lpn_pin.size() != n_devices) {
      throw std::invalid_argument(
        "Either 'rst_pin' or 'lpn_pin' must be the same size as 'address' when connecting "
        "multiple sensors");
    }
  }

  // If using INT (interrupt) pin
  if (!params.int_pin.empty()) {
    if (params.int_pin.size() != n_devices) {
      throw std::invalid_argument("'address' and 'int_pin' must be the same size when using INT");
    }
  }

  Resolution resolution_parsed;
  if (params.resolution == 4) {
    resolution_parsed = Resolution::X4;
  } else {
    resolution_parsed = Resolution::X8;
  }

  RangingMode ranging_mode;
  if (params.integration_time == 1) {
    ranging_mode = RangingMode::CONTINUOUS;
  } else {
    ranging_mode = RangingMode::AUTONOMOUS;
  }

  PartNumber part_number;
  if (params.part_number == "vl53l7cx") {
    part_number = PartNumber::VL53L7CX;
  } else {
    part_number = PartNumber::VL53L5CX;
  }

  std::vector<VL53L5CX::Config> configs(n_devices);
  for (size_t i = 0; i < n_devices; ++i) {
    auto & config = configs[i];
    config.part_number = part_number;
    config.frame_id = params.frame_id[i];
    config.address = static_cast<Address>(params.address[i]);
    config.gpiochip = params.gpiochip;
    if (!params.rst_pin.empty()) {
      config.rst_pin = params.rst_pin[i];
    }
    if (!params.lpn_pin.empty()) {
      config.rst_pin = params.lpn_pin[i];
    }
    if (!params.int_pin.empty()) {
      config.rst_pin = params.int_pin[i];
    }
    config.resolution = resolution_parsed;
    config.frequency = static_cast<Frequency>(params.frequency);
    config.ranging_mode = ranging_mode;
    config.integration_time = static_cast<IntegrationTime>(params.integration_time);
    config.filter_outputs = params.filter_outputs;
    config.xtalk_data = params.xtalk_data;
  }

  return configs;
}

CallbackReturn VL53L5CXNode::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  this->apply_parameters();

  const int frequency = params_.frequency;
  const std::chrono::milliseconds delay(static_cast<int64_t>(1e3 / frequency / sensors_.size()));

  // Delay the start of ranging for each sensor
  for (auto & e : sensors_) {
    rclcpp::sleep_for(delay);
    e->start_ranging();
  }

  RCLCPP_INFO(this->get_logger(), "Start ranging");

  // Start publishing ranging data
  ranging_helper_->start();

  return CallbackReturn::SUCCESS;
}

CallbackReturn VL53L5CXNode::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop publishing ranging data
  ranging_helper_->stop();

  for (auto & e : sensors_) e->stop_ranging();

  RCLCPP_INFO(this->get_logger(), "Stopped ranging");

  this->apply_parameters();

  return CallbackReturn::SUCCESS;
}

void VL53L5CXNode::calibrate_xtalk()
{
  RCLCPP_INFO(this->get_logger(), "Start calibration");
  auto xtalk_data = sensors_.at(0)->calibrate_xtalk(
    params_.calibration.reflectance, params_.calibration.n_samples, params_.calibration.distance);
  this->set_parameter({"xtalk_data", xtalk_data});
  RCLCPP_INFO(
    this->get_logger(),
    "Finished calibration. The xtalk data is set to the parameter 'xtalk_data'");
}

}  // namespace vl53l5cx
