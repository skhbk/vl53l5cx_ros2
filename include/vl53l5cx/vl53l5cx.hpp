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
#include <optional>
#include <string>
#include <vector>

#include "vl53l5cx/gpio.hpp"
#include "vl53l5cx/types.hpp"

namespace vl53l5cx
{
class VL53L5CX
{
public:
  struct Config
  {
    PartNumber part_number;
    std::string frame_id;
    Address address;
    Resolution resolution;
    Frequency frequency;
    RangingMode ranging_mode;
    IntegrationTime integration_time;
    bool filter_outputs;
    std::string i2c_bus;
    std::string gpiochip;
    std::optional<Pin> rst_pin;
    std::optional<Pin> lpn_pin;
    std::optional<Pin> int_pin;
    std::vector<int64_t> xtalk_data;
  };

  struct RangingResults
  {
    std::vector<float> distance;
    std::vector<uint8_t> target_status;

    static bool is_available(RangingOutput output);
    void resize(Resolution resolution);
    void filter_outputs();
  };

private:
  class DeviceData;
  std::unique_ptr<DeviceData> device_;
  Config config_;
  RangingResults results_;
  std::unique_ptr<GPIO> RST;
  std::unique_ptr<GPIO> LPn;
  std::unique_ptr<GPIO> INT;
  enum DeviceStatus { RANGING = 1 << 0, INITIALIZED = 1 << 1 };
  uint8_t device_status_ = 0;

public:
  explicit VL53L5CX(Config config);
  ~VL53L5CX();
  void initialize();
  void set_config(const Config & config);
  void start_ranging();
  void stop_ranging();
  bool check_data_ready();
  bool check_interrupt();
  void disable_comms() const;
  void enable_comms() const;
  void reset();
  std::vector<int64_t> calibrate_xtalk(uint8_t reflectance, uint8_t n_samples, uint16_t distance);

  bool is_alive();
  bool is_initialized() const { return device_status_ & INITIALIZED; }
  bool is_ranging() const { return device_status_ & RANGING; }
  const Config & get_config() const { return config_; }
  const RangingResults & get_results() const { return results_; }

private:
  void get_ranging_data();
  void set_xtalk_data(std::vector<int64_t> xtalk_data);
};

}  // namespace vl53l5cx
