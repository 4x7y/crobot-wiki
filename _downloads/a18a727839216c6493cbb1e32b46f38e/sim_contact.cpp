#include "crobot/common/logging.hpp"
#include "crobot/simulation/sim_contact.hpp"

using namespace crobot;
using namespace simulation;

//==============================================================================
ContactPtr SimContact::MakeShared(
    int geom1, int geom2, kdl::ContactModel contact_model, double mu) {
  return std::make_shared<SimContact>(geom1, geom2, contact_model, mu);
}

//==============================================================================
SimContact::SimContact(
    int geom1, int geom2, kdl::ContactModel contact_model, double mu)
  : kdl::CollisionContact(contact_model, mu),
    m_geom1_id(geom1),
    m_geom2_id(geom2),
    m_is_active(false) {
}

//==============================================================================
bool SimContact::is_active() const {
  return m_is_active;
}

//==============================================================================
math::Vector3d SimContact::get_xpos() const {
  if (m_is_active) {
    return m_xpos;
  }
  else {
    CROBOT_LOG_ERROR(
        "Contact (geom1={}, geom2={}) is not active. Return [0;0;0] instead.",
        m_geom1_id,
        m_geom2_id);
    return math::Vector3d::Zero();
  }
}

//==============================================================================
math::Matrix3d SimContact::get_xmat() const {
  if (m_is_active) {
    return m_frame;
  }
  else {
    CROBOT_LOG_ERROR(
        "Contact (geom1={}, geom2={}) is not active. Return identity matrix "
        "instead.",
        m_geom1_id,
        m_geom2_id);
    return math::Matrix3d::Identity();
  }
}

//==============================================================================
math::Vector3d SimContact::get_xfrc() const {
  if (m_is_active) {
    return m_force;
  }
  else {
    return math::Vector3d::Zero();
  }
}

//==============================================================================
void SimContact::reset() {
  m_is_active = false;
}

//==============================================================================
void SimContact::update(
    const math::Vector3d& xpos,
    const math::Matrix3d& frame,
    const math::Vector3d& force,
    const math::Vector3d& torque) {
  m_is_active = true;
  m_xpos      = xpos;
  m_frame     = frame;
  m_force     = force;
  m_torque    = torque;
}
