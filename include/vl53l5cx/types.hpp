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

#include <stdexcept>
#include <string>

namespace vl53l5cx
{
class ID
{
  uint8_t id_;

private:
  explicit ID(uint8_t id) : id_(id) {}

public:
  static ID get()
  {
    static uint8_t next = 0;
    return ID{next++};
  }
  std::string get_name() const { return "sensor_" + std::to_string(id_); }

  // For std::map
  bool operator<(const ID & right) const { return id_ < right.id_; }
};

class CommsError : public std::runtime_error
{
public:
  explicit CommsError(const std::string & message) : runtime_error{message} {}
};

class DeviceError : public std::runtime_error
{
public:
  explicit DeviceError(const std::string & message, const ID & id)
  : runtime_error{message + " [" + id.get_name() + "]"}
  {
  }
};
}  // namespace vl53l5cx
