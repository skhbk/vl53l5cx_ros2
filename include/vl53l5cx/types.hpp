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
