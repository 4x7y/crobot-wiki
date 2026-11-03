#include "crobot/kdl/actuator.hpp"

using namespace crobot;
using namespace kdl;

//==============================================================================
Actuator::Actuator(const std::string& name, TransmissionPtr transmission)
  : m_name(name), m_transmission(std::move(transmission)) {
}
