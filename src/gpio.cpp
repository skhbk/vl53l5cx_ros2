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

#include <gpiod.h>

#include <cassert>
#include <stdexcept>

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

bool GPIO::check_event()
{
  const auto fd = line_.event_get_fd();
  gpiod_line_event event;

  if (gpiod_line_event_read_fd(fd, &event))
    return false;
  else
    return true;
}

}  // namespace vl53l5cx