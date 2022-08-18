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

#include <sstream>
#include <stdexcept>
#include <string>

namespace vl53l5cx
{
using Address = uint8_t;
using Frequency = uint8_t;
using IntegrationTime = uint16_t;
using Pin = uint8_t;
static constexpr Pin PinNaN = 255;
enum class Resolution { X4 = 4, X8 = 8 };
enum class RangingMode { CONTINUOUS, AUTONOMOUS };

inline std::string get_hex(Address address)
{
  std::stringstream ss;
  ss << std::hex << static_cast<int>(address);
  const auto hex = ss.str();

  return hex;
}

class CommsError : public std::runtime_error
{
public:
  explicit CommsError(const std::string & message) : runtime_error{message} {}
};

class DeviceError : public std::runtime_error
{
public:
  explicit DeviceError(const std::string & message, const Address & address)
  : runtime_error{message + " [0x" + get_hex(address) + "]"}
  {
  }
};
}  // namespace vl53l5cx
