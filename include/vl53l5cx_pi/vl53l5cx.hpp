#pragma once

#include <memory>
#include <string>
#include <vector>

#include "vl53l5cx_pi/builder.hpp"
#include "vl53l5cx_pi/gpio.hpp"
#include "vl53l5cx_pi/types.hpp"

namespace vl53l5cx
{
class VL53L5CXBuilder;

class VL53L5CX
{
  class DeviceData;
  std::unique_ptr<DeviceData> device_;

  enum DeviceStatus { RANGING = 1 << 0, INITIALIZED = 1 << 1 };
  uint8_t device_status_ = 0;

  uint8_t address_;
  std::shared_ptr<GPIO> LPn;
  std::shared_ptr<GPIO> INT;
  std::vector<float> distances_;

  uint8_t resolution_, frequency_, ranging_mode_;
  uint8_t width = 8, height = 8;

public:
  const ID id;

  VL53L5CX(const VL53L5CXBuilder & builder);
  ~VL53L5CX();
  void initialize();
  void apply_parameters();
  void start_ranging();
  void stop_ranging();
  bool check_data_ready();
  bool wait_for_interrupt(const std::chrono::milliseconds & timeout);
  void disable_comms() const;
  void enable_comms() const;

  bool is_alive();
  bool is_initialized() const { return device_status_ & INITIALIZED; }
  bool is_ranging() const { return device_status_ & RANGING; }
  uint8_t get_address() const { return address_; }
  const std::vector<float> & get_distance() const { return distances_; }

  void set_resolution(uint8_t resolution) { resolution_ = resolution; }
  void set_frequency(uint8_t frequency) { frequency_ = frequency; }
  void set_ranging_mode(uint8_t ranging_mode) { ranging_mode_ = ranging_mode; };

private:
  void get_ranging_data();
};

}  // namespace vl53l5cx
