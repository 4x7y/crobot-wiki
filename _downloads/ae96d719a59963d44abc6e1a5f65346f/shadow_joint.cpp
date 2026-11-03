#include "crobot/hardware/shadow_joint.hpp"

using namespace crobot;
using namespace hardware;

//==============================================================================
ShadowJoint::ShadowJoint(const std::string& name, int id)
  : m_name(name), m_id(id) {
}

//==============================================================================
const std::string& ShadowJoint::get_name() const {
  return m_name;
}

//==============================================================================
int ShadowJoint::get_qpos_addr() const {
  return m_id;
}
