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

#include "vl53l5cx/gpio.hpp"
#include "vl53l5cx/types.hpp"
#include "vl53l5cx/vl53l5cx.hpp"

namespace vl53l5cx
{
class VL53L5CX;

class VL53L5CXBuilder
{
public:
  ID id;
  uint8_t address;
  std::shared_ptr<GPIO> LPn;
  std::shared_ptr<GPIO> INT;

  VL53L5CXBuilder(const ID & id, uint8_t address) : id(id), address(address) {}
  std::unique_ptr<VL53L5CX> build() { return std::make_unique<VL53L5CX>(*this); }
};
}  // namespace vl53l5cx
