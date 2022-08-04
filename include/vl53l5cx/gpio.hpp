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

private:
  GPIO() = delete;
  GPIO(const GPIO &) = delete;
  GPIO & operator=(const GPIO &) = delete;
};
}  // namespace vl53l5cx
