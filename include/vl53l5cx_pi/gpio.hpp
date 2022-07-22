#pragma once

#include <cstdint>  // uint8_t
#include <memory>

namespace vl53l5cx
{
class GPIO
{
  uint8_t pin_;
  bool is_empty_;

public:
  GPIO() : is_empty_(true) {}
  GPIO & operator=(uint8_t pin)
  {
    pin_ = pin;
    is_empty_ = false;
    return *this;
  }
  explicit operator bool() const { return !is_empty_; }
  uint8_t pin() const { return pin_; }
};

class GPIOHandler : public std::enable_shared_from_this<GPIOHandler>
{
public:
  GPIOHandler() = default;
  virtual ~GPIOHandler() = default;

  virtual void set_input(GPIO gpio) = 0;
  virtual void set_output(GPIO gpio) = 0;
  virtual void pullup(GPIO gpio) = 0;
  virtual void write(GPIO gpio, bool on) = 0;
  virtual void enable_falling_edge_detection(GPIO gpio) = 0;
  virtual void disable_falling_edge_detection(GPIO gpio) = 0;
  virtual bool check_event(GPIO gpio) = 0;
};
}  // namespace vl53l5cx
