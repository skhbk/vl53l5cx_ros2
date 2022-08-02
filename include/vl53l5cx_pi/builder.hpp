#pragma once

#include "vl53l5cx_pi/gpio.hpp"
#include "vl53l5cx_pi/types.hpp"
#include "vl53l5cx_pi/vl53l5cx.hpp"

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
