#pragma once

#include <gpiod.hpp>

namespace vl53l5cx
{
class GPIO
{
public:
  enum class RequestType { IN, OUT, FALLING_EDGE };
  enum class Value { LOW, HIGH };

private:
  uint8_t pin_;
  gpiod::line line_;

public:
  explicit GPIO(uint8_t pin);
  ~GPIO() noexcept;

  uint8_t pin() const noexcept { return pin_; }

  void set_request_type(RequestType request_type) const;
  void set_value(Value value) const;
  bool wait_for_event(const std::chrono::milliseconds & timeout) const;

private:
  GPIO() = delete;
  GPIO(const GPIO &) = delete;
  GPIO & operator=(const GPIO &) = delete;
};
}  // namespace vl53l5cx
