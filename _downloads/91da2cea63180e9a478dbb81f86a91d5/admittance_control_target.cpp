#include "crobot/control/admittance_control_target.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
AdmittanceControlTarget::AdmittanceControlTarget(
    kdl::ConstFramePtr desired_frame,
    kdl::BodyContact::ConstPtr contact,
    bool is_contact_active,
    const math::Matrix3d& kp,
    const math::Matrix3d& ki)
  : kdl::WorldFrame(),
    m_desired_frame(desired_frame),
    m_contact(contact),
    m_is_contact_active(is_contact_active),
    m_kp(kp),
    m_ki(ki),
    m_bfrc_desired(math::Vector3d::Zero()) {
}

//==============================================================================
void AdmittanceControlTarget::set_desired_force(const math::Vector3d& bfrc) {
  m_bfrc_desired = bfrc;
}

//==============================================================================
void AdmittanceControlTarget::set_kp(const math::Matrix3d& kp) {
  m_kp = kp;
}

//==============================================================================
void AdmittanceControlTarget::set_ki(const math::Matrix3d& ki) {
  m_ki = ki;
}

//==============================================================================
math::Matrix4d AdmittanceControlTarget::get_xform() const {
  math::Matrix4d xform_desired = m_desired_frame->get_xform();
  math::Matrix3d xmat_desired  = xform_desired.tform2rotm();
  math::Vector3d xfrc_actual   = (m_contact && m_is_contact_active)
                                   ? m_contact->get_xfrc()
                                   : math::Vector3d::Zero();
  math::Vector3d bfrc_actual   = xmat_desired.transpose() * xfrc_actual;
  math::Vector3d bfrc_error    = m_bfrc_desired - bfrc_actual;
  math::Vector3d bpos_error    = m_kp * bfrc_error;
  math::Matrix4d xform_control = xform_desired * bpos_error.trvec2tform();

  return xform_control;
}

//==============================================================================
const std::string& AdmittanceControlTarget::get_name(size_t body_id) const {
  CROBOT_ASSERT_ERROR(
      (body_id == 1) || (body_id == 2), "Index must be 1 or 2!");
  if (body_id == 1) {
    return m_contact->get_body1()->get_name();
  }
  return m_contact->get_body2()->get_name();
}
