#include "crobot/kdl/collision_contact.hpp"

using namespace crobot;
using namespace kdl;

//==============================================================================
CollisionContact::CollisionContact(
    ContactModel contact_model, double mu, bool is_active)
  : Contact(contact_model, mu),
    m_xfrc(math::Vector3d::Zero()),
    m_xform(math::Matrix4d::Identity()),
    m_is_active(is_active) {
}

//==============================================================================
math::Vector3d CollisionContact::get_xpos() const {
  return m_xform.tform2trvec();
}

//==============================================================================
math::Matrix3d CollisionContact::get_xmat() const {
  return m_xform.tform2rotm();
}

//==============================================================================
bool CollisionContact::is_active() const {
  return m_is_active;
}

//==============================================================================
math::Vector3d CollisionContact::get_xfrc() const {
  return m_xfrc;
}

//==============================================================================
void CollisionContact::update(
    const math::Vector3d& xpos,
    const math::Matrix3d& xmat,
    const math::Vector3d& xfrc) {
  m_xform.block<3, 3>(0, 0) = xmat;
  m_xform.block<3, 1>(0, 3) = xpos;
  m_xfrc                    = xfrc;
  m_is_active               = true;
}

//==============================================================================
void CollisionContact::update(
    const math::Matrix4d& xform, const math::Vector3d& xfrc) {
  m_xform     = xform;
  m_xfrc      = xfrc;
  m_is_active = true;
}

//==============================================================================
void CollisionContact::update_status(bool is_active) {
  m_is_active = is_active;
}
