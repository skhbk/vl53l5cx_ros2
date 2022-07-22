#pragma once

#include "vl53l5cx_pi/gpio.hpp"

namespace vl53l5cx
{
class BCM2835 : public GPIOHandler
{
public:
  BCM2835();
  ~BCM2835() override;

  void set_input(GPIO gpio) override;
  void set_output(GPIO gpio) override;
  void pullup(GPIO gpio) override;
  void write(GPIO gpio, bool on) override;
  void enable_falling_edge_detection(GPIO gpio) override;
  void disable_falling_edge_detection(GPIO gpio) override;
  bool check_event(GPIO gpio) override;
};
}  // namespace vl53l5cx
