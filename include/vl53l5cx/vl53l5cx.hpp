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
#include <string>
#include <vector>

#include "vl53l5cx/gpio.hpp"
#include "vl53l5cx/types.hpp"

namespace vl53l5cx
{
class VL53L5CXBuilder;

class VL53L5CX
{
public:
  struct Config
  {
    Address address;
    Resolution resolution;
    Frequency frequency;
    RangingMode ranging_mode;
    Pin lpn_pin = PinNaN, int_pin = PinNaN;
  };

private:
  class DeviceData;
  std::unique_ptr<DeviceData> device_;
  Config config_;
  std::unique_ptr<GPIO> LPn;
  std::unique_ptr<GPIO> INT;
  enum DeviceStatus { RANGING = 1 << 0, INITIALIZED = 1 << 1 };
  uint8_t device_status_ = 0;

  std::vector<float> distances_;

public:
  VL53L5CX(Config config);
  ~VL53L5CX();
  void initialize();
  void set_config(const Config & config);
  void start_ranging();
  void stop_ranging();
  bool check_data_ready();
  void disable_comms() const;
  void enable_comms() const;

  bool is_alive();
  bool is_initialized() const { return device_status_ & INITIALIZED; }
  bool is_ranging() const { return device_status_ & RANGING; }
  const std::vector<float> & get_distance() const { return distances_; }
  const Config & get_config() const { return config_; }

private:
  void get_ranging_data();
};

}  // namespace vl53l5cx
