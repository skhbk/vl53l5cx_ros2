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

#include <algorithm>  // std::any_of
#include <cassert>
#include <chrono>    // std::chrono_literals
#include <cmath>     // NAN
#include <iostream>  // std::cerr
#include <thread>    // std::this_thread

#include "vl53l5cx/vl53l5cx.hpp"

extern "C" {
#include "vl53l5cx_api.h"           //NOLINT
#include "vl53l5cx_plugin_xtalk.h"  //NOLINT
}

#define VL53L5CX_USE_RAW_FORMAT

using namespace std::chrono_literals;

namespace vl53l5cx
{
class VL53L5CX::DeviceData
{
  VL53L5CX_Configuration config_;
  VL53L5CX_ResultsData results_;

public:
  DeviceData() = default;
  VL53L5CX_Configuration * config() { return &config_; }
  VL53L5CX_Platform * platform() { return &config_.platform; }
  VL53L5CX_ResultsData * results() { return &results_; }
};

bool VL53L5CX::RangingResults::is_available(RangingOutput output)
{
  switch (output) {
    case RangingOutput::DISTANCE:
#ifdef VL53L5CX_DISABLE_DISTANCE_MM
      return false;
#endif
      break;
    case RangingOutput::TARGET_STATUS:
#ifdef VL53L5CX_DISABLE_TARGET_STATUS
      return false;
#endif
      break;

    default:
      assert(false);
  }
  return true;
}

void VL53L5CX::RangingResults::resize(Resolution resolution)
{
  const auto resolution_casted = static_cast<uint8_t>(resolution);
  const auto size = resolution_casted * resolution_casted;

  if (this->is_available(RangingOutput::DISTANCE)) {
    distance.resize(size);
  }
  if (this->is_available(RangingOutput::TARGET_STATUS)) {
    target_status.resize(size);
  }
}

void VL53L5CX::RangingResults::replace_with_nan(const std::vector<TargetStatus> & valid_status)
{
  if (!this->is_available(RangingOutput::TARGET_STATUS)) {
    throw std::invalid_argument("Target status output is required");
  }

  if (!this->is_available(RangingOutput::DISTANCE)) {
    return;
  }

  for (std::size_t i = 0; i < target_status.size(); ++i) {
    const bool is_valid = std::any_of(
      valid_status.begin(), valid_status.end(),
      [&](TargetStatus x) { return x == target_status.at(i); });

    if (!is_valid) {
      distance.at(i) = NAN;
    }
  }
}

VL53L5CX::VL53L5CX(Config config) : device_(std::make_unique<DeviceData>()), config_(config)
{
  // Initialize GPIO
  if (config_.rst_pin) {
    RST = std::make_unique<GPIO>(config_.rst_pin.value(), config_.gpiochip);
    RST->set_request_type(GPIO::RequestType::OUT);
    RST->set_value(GPIO::Value::LOW);
  }
  if (config_.lpn_pin) {
    LPn = std::make_unique<GPIO>(config_.lpn_pin.value(), config_.gpiochip);
    LPn->set_request_type(GPIO::RequestType::OUT);
    LPn->set_value(GPIO::Value::HIGH);
  }
  if (config_.int_pin) {
    INT = std::make_unique<GPIO>(config_.int_pin.value(), config_.gpiochip);
    INT->set_request_type(GPIO::RequestType::FALLING_EDGE);
  }
}

VL53L5CX::~VL53L5CX()
{
  if (this->is_ranging()) {
    try {
      this->stop_ranging();
    } catch (std::runtime_error & e) {
      std::cerr << e.what() << std::endl;
    }
  }

  vl53l5cx_comms_close(device_->platform());
}

void VL53L5CX::initialize()
{
  if (this->is_ranging()) throw std::runtime_error("Stop ranging before initialization");

  int status = 0;

  if (RST || LPn) this->reset();

  device_->platform()->i2c_device = ("/dev/" + config_.i2c_bus).c_str();
  status |= vl53l5cx_comms_init(device_->platform());
  if (status) throw CommsError("Failed to initialize I2C");

  if (!this->is_alive()) {
    // Try with the given address
    device_->platform()->address = config_.address << 1;
    if (!this->is_alive()) throw DeviceError("Device not detected", config_.address);
  }

  status |= vl53l5cx_set_i2c_address(device_->config(), config_.address << 1);
  if (status) throw DeviceError("Failed to set I2C address", config_.address);

  status |= vl53l5cx_init(device_->config());
  if (status) {
    throw DeviceError(
      "Failed to initialize the device: " + std::to_string(status), config_.address);
  }

  device_status_ |= INITIALIZED;

  this->set_config(config_);
}

void VL53L5CX::set_config(const Config & config)
{
  if (!this->is_initialized()) throw std::runtime_error("Initialize first");
  if (this->is_ranging()) throw std::runtime_error("Stop ranging before applying parameters");

  config_ = config;

  // Resolution
  const auto resolution = static_cast<uint8_t>(config_.resolution);
  const uint8_t image_size = resolution * resolution;
  if (vl53l5cx_set_resolution(device_->config(), image_size)) {
    throw DeviceError("Failed to set resolution", config_.address);
  }
  results_.resize(config_.resolution);

  // Frequency
  if (vl53l5cx_set_ranging_frequency_hz(device_->config(), config_.frequency)) {
    throw DeviceError("Failed to set frequency", config_.address);
  }

  // Ranging mode
  switch (config_.ranging_mode) {
    case RangingMode::CONTINUOUS:
      if (vl53l5cx_set_ranging_mode(device_->config(), VL53L5CX_RANGING_MODE_CONTINUOUS)) {
        throw DeviceError("Failed to set ranging mode", config_.address);
      }
      break;
    case RangingMode::AUTONOMOUS:
      if (vl53l5cx_set_ranging_mode(device_->config(), VL53L5CX_RANGING_MODE_AUTONOMOUS)) {
        throw DeviceError("Failed to set ranging mode", config_.address);
      }
      if (vl53l5cx_set_integration_time_ms(device_->config(), config_.integration_time)) {
        throw DeviceError("Failed to set integration time", config_.address);
      }
      break;
    default:
      assert(false);
  }

  // Sharpener
  if (vl53l5cx_set_sharpener_percent(device_->config(), config_.sharpener)) {
    throw DeviceError("Failed to set sharpener", config_.address);
  }

  if (!config_.xtalk_data.empty()) {
    this->set_xtalk_data(config_.xtalk_data);
  }
}

void VL53L5CX::start_ranging()
{
  if (!this->is_initialized()) throw std::runtime_error("Initialize first");
  if (this->is_ranging()) throw std::runtime_error("Already in ranging");

  auto status = vl53l5cx_start_ranging(device_->config());
  if (status) throw DeviceError("Starting ranging failed", config_.address);

  device_status_ |= RANGING;
}

void VL53L5CX::stop_ranging()
{
  if (!this->is_initialized()) throw std::runtime_error("Initialize first");
  if (!this->is_ranging()) throw std::runtime_error("Not in ranging");

  auto status = vl53l5cx_stop_ranging(device_->config());
  if (status) throw DeviceError("Failed to stop ranging", config_.address);

  device_status_ &= ~RANGING;
}

bool VL53L5CX::check_data_ready()
{
  if (!this->is_initialized()) throw std::runtime_error("Initialize first");
  if (!this->is_ranging()) throw std::runtime_error("Not in ranging");

  uint8_t ready = 0;
  auto status = vl53l5cx_check_data_ready(device_->config(), &ready);
  if (status) throw DeviceError("Failed to check data-ready", config_.address);
  bool is_ready = ready ? true : false;

  if (is_ready) this->get_ranging_data();

  return is_ready;
}

bool VL53L5CX::check_interrupt()
{
  const auto is_ready = INT->check_event();
  if (is_ready) this->get_ranging_data();
  return is_ready;
}

void VL53L5CX::disable_comms() const
{
  if (LPn) {
    LPn->set_value(GPIO::Value::LOW);
  } else {
    RST->set_value(GPIO::Value::HIGH);
  }

  std::this_thread::sleep_for(10ms);
}

void VL53L5CX::enable_comms() const
{
  if (LPn) {
    LPn->set_value(GPIO::Value::HIGH);
  } else {
    RST->set_value(GPIO::Value::LOW);
  }

  std::this_thread::sleep_for(10ms);
}

void VL53L5CX::reset()
{
  // I don't know what actually happens with this operation
  // At least the address is not reset to 0x29
  if (RST) {
    RST->set_value(GPIO::Value::HIGH);
    std::this_thread::sleep_for(10ms);
    RST->set_value(GPIO::Value::LOW);
    std::this_thread::sleep_for(10ms);
  } else {
    LPn->set_value(GPIO::Value::LOW);
    std::this_thread::sleep_for(10ms);
    LPn->set_value(GPIO::Value::HIGH);
    std::this_thread::sleep_for(10ms);
  }
}

void VL53L5CX::get_ranging_data()
{
  auto status = vl53l5cx_get_ranging_data(device_->config(), device_->results());
  if (status) {
    throw DeviceError("Failed to get ranging data: " + std::to_string(status), config_.address);
  }

  const auto resolution = static_cast<uint8_t>(config_.resolution);
  for (int i = 0; i < resolution * resolution; ++i) {
    // Distance
    if (results_.is_available(RangingOutput::DISTANCE)) {
      constexpr float gain = 1 / (4 * 1000.f);  // Convert to metric
      results_.distance.at(i) = device_->results()->distance_mm[i] * gain;
    }

    // Target status
    if (results_.is_available(RangingOutput::TARGET_STATUS)) {
      results_.target_status.at(i) = device_->results()->target_status[i];
    }

    if (!config_.valid_status.empty()) {
      results_.replace_with_nan(config_.valid_status);
    }
  }
}

void VL53L5CX::set_xtalk_data(std::vector<int64_t> xtalk_data)
{
  int status = 0;

  auto xtalk_data_ptr = reinterpret_cast<uint8_t *>(xtalk_data.data());

  // 'xtalk_data' cannot be const-reference because this function takes non-const pointer
  status |= vl53l5cx_set_caldata_xtalk(device_->config(), xtalk_data_ptr);
  if (status) throw DeviceError("Failed to set calibration data", config_.address);
}

bool VL53L5CX::is_alive()
{
  uint8_t is_alive;
  auto status = vl53l5cx_is_alive(device_->config(), &is_alive);
  return (!status && is_alive) ? true : false;
}

std::vector<int64_t> VL53L5CX::calibrate_xtalk(
  uint8_t reflectance, uint8_t n_samples, uint16_t distance)
{
  int status = 0;

  status |= vl53l5cx_calibrate_xtalk(device_->config(), reflectance, n_samples, distance);
  if (status) throw DeviceError("Failed to calibrate xtalk", config_.address);

  std::array<uint8_t, VL53L5CX_XTALK_BUFFER_SIZE> xtalk_data_raw;
  status |= vl53l5cx_get_caldata_xtalk(device_->config(), xtalk_data_raw.data());
  if (status) throw DeviceError("Failed to get calibration data", config_.address);

  const auto xtalk_data_ptr = reinterpret_cast<int64_t *>(xtalk_data_raw.data());
  const std::vector<int64_t> xtalk_data(
    xtalk_data_ptr, xtalk_data_ptr + sizeof(xtalk_data_raw) / sizeof(int64_t));
  this->set_xtalk_data(xtalk_data);

  return xtalk_data;
}
}  // namespace vl53l5cx
