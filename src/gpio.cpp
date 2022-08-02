#include <cassert>

#include "vl53l5cx/gpio.hpp"

namespace vl53l5cx
{
GPIO::GPIO(uint8_t pin) : pin_(pin)
{
  gpiod::chip chip("gpiochip0");
  assert(chip);
  line_ = chip.get_line(pin_);
  if (!line_) throw std::runtime_error("Cannot find GPIO line " + std::to_string(pin));
  if (line_.is_used()) throw std::runtime_error(line_.name() + " is already used");
}

GPIO::~GPIO() noexcept {}

void GPIO::set_request_type(RequestType request_type) const
{
  gpiod::line_request config;
  config.flags = 0;
  switch (request_type) {
    case RequestType::IN:
      config.request_type = gpiod::line_request::DIRECTION_INPUT;
      break;
    case RequestType::OUT:
      config.request_type = gpiod::line_request::DIRECTION_OUTPUT;
      break;
    case RequestType::FALLING_EDGE:
      config.request_type = gpiod::line_request::EVENT_FALLING_EDGE;
      break;
    default:
      assert(false);
  }

  line_.request(config);
}

void GPIO::set_value(Value value) const
{
  if (line_.direction() != gpiod::line::DIRECTION_OUTPUT)
    throw std::runtime_error(line_.name() + " is not output");

  switch (value) {
    case Value::LOW:
      line_.set_value(0);
      break;
    case Value::HIGH:
      line_.set_value(1);
      break;
    default:
      assert(false);
  }
}

bool GPIO::wait_for_event(const std::chrono::milliseconds & timeout) const
{
  return line_.event_wait(timeout);
}

}  // namespace vl53l5cx