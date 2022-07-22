#include <bcm2835.h>

#include <cassert>
#include <stdexcept>

#include "vl53l5cx_pi/bcm2835.hpp"

namespace vl53l5cx
{
BCM2835::BCM2835()
{
  if (!bcm2835_init()) throw std::runtime_error("Failed to initialize GPIO handler");
}
BCM2835::~BCM2835() { bcm2835_close(); }

void BCM2835::set_input(GPIO gpio) { bcm2835_gpio_fsel(gpio.pin(), BCM2835_GPIO_FSEL_INPT); }

void BCM2835::set_output(GPIO gpio) { bcm2835_gpio_fsel(gpio.pin(), BCM2835_GPIO_FSEL_OUTP); }

void BCM2835::pullup(GPIO gpio) { bcm2835_gpio_set_pud(gpio.pin(), BCM2835_GPIO_PUD_UP); }

void BCM2835::write(GPIO gpio, bool on)
{
  const uint8_t level = on ? HIGH : LOW;
  bcm2835_gpio_write(gpio.pin(), level);
  assert(bcm2835_gpio_lev(gpio.pin()) == level);
}

void BCM2835::enable_falling_edge_detection(GPIO gpio) { bcm2835_gpio_fen(gpio.pin()); }

void BCM2835::disable_falling_edge_detection(GPIO gpio) { bcm2835_gpio_clr_fen(gpio.pin()); }

bool BCM2835::check_event(GPIO gpio)
{
  if (bcm2835_gpio_eds(gpio.pin())) {
    bcm2835_gpio_set_eds(gpio.pin());  // Clear Event Detect Status
    return true;
  }
  return false;
}
}  // namespace vl53l5cx
