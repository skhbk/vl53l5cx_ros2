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

VL53L5CX::VL53L5CX(const VL53L5CXBuilder & builder)
: device_(std::make_unique<DeviceData>()),
  address_(builder.address),
  LPn(builder.LPn),
  INT(builder.INT),
  id(builder.id)
{
  // Initialize GPIO
  if (LPn) {
    LPn->set_request_type(GPIO::RequestType::OUT);
    this->enable_comms();
  }
  if (INT) {
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
    device_->platform()->address = address_ << 1;
    if (!this->is_alive()) throw DeviceError("Device not detected", id);
  }

  status |= vl53l5cx_set_i2c_address(device_->config(), address_ << 1);
  if (status) throw DeviceError("Failed to set I2C address", id);

  status |= vl53l5cx_init(device_->config());
  if (status) throw DeviceError("Failed to initialize the device: " + std::to_string(status), id);

  device_status_ |= INITIALIZED;
}

void VL53L5CX::apply_parameters()
{
  if (!this->is_initialized()) throw std::runtime_error("Initialize first");
  if (this->is_ranging()) throw std::runtime_error("Stop ranging before applying parameters");

  const uint8_t image_size = resolution_ * resolution_;
  if (vl53l5cx_set_ranging_frequency_hz(device_->config(), frequency_))
    throw std::runtime_error("Failed to set frequency");
  if (vl53l5cx_set_resolution(device_->config(), image_size))
    throw std::runtime_error("Failed to set resolution");
  if (vl53l5cx_set_ranging_mode(device_->config(), ranging_mode_))
    throw std::runtime_error("Failed to set ranging mode");

  distances_.resize(image_size);
}

void VL53L5CX::start_ranging()
{
  if (!this->is_initialized()) throw std::runtime_error("Initialize first");
  if (this->is_ranging()) throw std::runtime_error("Already in ranging");

  auto status = vl53l5cx_start_ranging(device_->config());
  if (status) throw DeviceError("Starting ranging failed", id);

  device_status_ |= RANGING;
}

void VL53L5CX::stop_ranging()
{
  if (!this->is_initialized()) throw std::runtime_error("Initialize first");
  if (!this->is_ranging()) throw std::runtime_error("Not in ranging");

  auto status = vl53l5cx_stop_ranging(device_->config());
  if (status) throw DeviceError("Failed to stop ranging", id);

  device_status_ &= ~RANGING;
}

bool VL53L5CX::check_data_ready()
{
  if (!this->is_initialized()) throw std::runtime_error("Initialize first");
  if (!this->is_ranging()) throw std::runtime_error("Not in ranging");

  uint8_t ready = 0;
  auto status = vl53l5cx_check_data_ready(device_->config(), &ready);
  if (status) throw DeviceError("Failed to check data-ready", id);
  bool is_ready = ready ? true : false;

  if (is_ready) this->get_ranging_data();

  return is_ready;
}

bool VL53L5CX::wait_for_interrupt(const std::chrono::milliseconds & timeout)
{
  const auto is_ready = INT->wait_for_event(timeout);
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
  if (status) throw DeviceError("Failed to get ranging data: " + std::to_string(status), id);

  for (int i = 0; i < resolution_ * resolution_; ++i) {
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
