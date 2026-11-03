#pragma once

#include <string>

#include "crobot/kdl/transmission.hpp"

namespace crobot::kdl {

class Actuator;
using ActuatorPtr      = std::shared_ptr<Actuator>;
using ConstActuatorPtr = std::shared_ptr<const Actuator>;
class RigidBodyTree;

class Actuator {
public:
  Actuator(const std::string& name, TransmissionPtr transmission);

  virtual ~Actuator() = default;

protected:
  // Name of the actuator.
  std::string m_name;

  TransmissionPtr m_transmission;

  friend class RigidBodyTree;
};

} // namespace crobot::kdl

