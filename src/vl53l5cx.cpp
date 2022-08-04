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

#include <cassert>
#include <chrono>    // std::chrono_literals
#include <iostream>  // std::cerr
#include <thread>    // std::this_thread

#include "vl53l5cx/vl53l5cx.hpp"

extern "C" {
#include "vl53l5cx_api.h"
#include "vl53l5cx_plugin_xtalk.h"
}

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

VL53L5CX::VL53L5CX(Config config) : device_(std::make_unique<DeviceData>()), config_(config)
{
  // Initialize GPIO
  if (config_.lpn_pin != PinNaN) {
    LPn = std::make_unique<GPIO>(config_.lpn_pin);
    LPn->set_request_type(GPIO::RequestType::OUT);
    this->enable_comms();
  }
  if (config_.int_pin != PinNaN) {
    INT = std::make_unique<GPIO>(config_.int_pin);
    INT->set_request_type(GPIO::RequestType::FALLING_EDGE);
  }
}

VL53L5CX::~VL53L5CX()
{
  if (this->is_ranging()) try {
      this->stop_ranging();
    } catch (std::runtime_error & e) {
      std::cerr << e.what() << std::endl;
    }

  vl53l5cx_comms_close(device_->platform());
}

void VL53L5CX::initialize()
{
  if (this->is_ranging()) throw std::runtime_error("Stop ranging before initialization");

  int status = 0;

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
  if (status)
    throw DeviceError(
      "Failed to initialize the device: " + std::to_string(status), config_.address);

  device_status_ |= INITIALIZED;

  this->set_config(config_);
}

void VL53L5CX::set_config(const Config & config)
{
  if (!this->is_initialized()) throw std::runtime_error("Initialize first");
  if (this->is_ranging()) throw std::runtime_error("Stop ranging before applying parameters");

  config_.resolution = config.resolution;
  config_.frequency = config.frequency;
  config_.ranging_mode = config.ranging_mode;

  const auto resolution = static_cast<uint8_t>(config_.resolution);
  const uint8_t image_size = resolution * resolution;
  if (vl53l5cx_set_resolution(device_->config(), image_size))
    throw std::runtime_error("Failed to set resolution");
  distances_.resize(image_size);

  if (vl53l5cx_set_ranging_frequency_hz(device_->config(), config_.frequency))
    throw std::runtime_error("Failed to set frequency");

  uint8_t ranging_mode;
  switch (config_.ranging_mode) {
    case RangingMode::CONTINUOUS:
      ranging_mode = VL53L5CX_RANGING_MODE_CONTINUOUS;
      break;
    case RangingMode::AUTONOMOUS:
      ranging_mode = VL53L5CX_RANGING_MODE_AUTONOMOUS;
      break;
    default:
      assert(false);
  }
  if (vl53l5cx_set_ranging_mode(device_->config(), ranging_mode))
    throw std::runtime_error("Failed to set ranging mode");
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

void VL53L5CX::disable_comms() const
{
  LPn->set_value(GPIO::Value::LOW);
  std::this_thread::sleep_for(10ms);
}

void VL53L5CX::enable_comms() const
{
  LPn->set_value(GPIO::Value::HIGH);
  std::this_thread::sleep_for(10ms);
}

void VL53L5CX::get_ranging_data()
{
  auto status = vl53l5cx_get_ranging_data(device_->config(), device_->results());
  if (status)
    throw DeviceError("Failed to get ranging data: " + std::to_string(status), config_.address);

  const auto resolution = static_cast<uint8_t>(config_.resolution);
  for (int i = 0; i < resolution * resolution; ++i) {
#ifdef VL53L5CX_ENABLE_DISTANCE_MM
#ifdef VL53L5CX_USE_RAW_FORMAT
    constexpr float gain = 1 / (4 * 1000.f);
#else
    constexpr float gain = 1 / 1000.f;
#endif
    // Convert to metric
    distances_.at(i) = device_->results()->distance_mm[i] * gain;
#endif
  }
}

bool VL53L5CX::is_alive()
{
  uint8_t is_alive;
  auto status = vl53l5cx_is_alive(device_->config(), &is_alive);
  return (!status && is_alive) ? true : false;
}

}  // namespace vl53l5cx
