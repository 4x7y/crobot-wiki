
#include "crobot/ekf/obj_dynamics.hpp"

using namespace crobot;
using namespace ekf;

//==============================================================================
ObjDynamics::ObjDynamics(
    double mass,
    const math::Vector3d& pcm,
    const math::Matrix3d& inertia_cm,
    const std::vector<math::SurfacePatchSwitch::ConstPtr>& patches,
    const math::Vector3d& gravity)
  : m_mass(mass),
    m_pcm(pcm),
    m_Icm(inertia_cm),
    m_patches(patches),
    m_gravity(gravity) {
  // initialize the number of contacts
  m_num_contacts = patches.size();

  // get mass matrix in the tool frame
  m_M                   = math::Matrix6d::Zero();
  m_M.block<3, 3>(0, 0) = m_mass * math::Matrix3d::Identity();
  m_M.block<3, 3>(0, 3) = -m_mass * pcm.skew();
  m_M.block<3, 3>(3, 0) = m_mass * pcm.skew();
  m_M.block<3, 3>(3, 3) = m_Icm - m_mass * pcm.skew() * pcm.skew();

  // check if the mass matrix is invertible
  CROBOT_ASSERT_ERROR(m_M.determinant() != 0, "Mass matrix is singular");
  m_M_inv = m_M.inverse();
}

//==============================================================================
int ObjDynamics::get_num_contacts() const {
  return m_num_contacts;
}

//==============================================================================
// Get the acceleration of the object through dynamics
math::Vector6d ObjDynamics::get_acc(
    const math::VectorXd& pos,
    const math::Vector6d& bvel,
    std::vector<math::Vector2d> ctc_pos,
    std::vector<math::Vector3d> ctc_frc) const {

  CROBOT_ASSERT_ERROR(
      ctc_pos.size() == m_num_contacts, "ctc_pos size mismatch");
  CROBOT_ASSERT_ERROR(
      ctc_frc.size() == m_num_contacts, "ctc_frc size mismatch");

  math::Quaternion quat(pos(3), pos(4), pos(5), pos(6));
  math::Matrix3d rotm = quat.normalized().as_rotm();

  math::Vector3d velp = bvel.head<3>();
  math::Vector3d velr = bvel.tail<3>();

  // initialize the acceleration (linear, angular)
  math::Vector6d acc = math::Vector6d::Zero();

  // compute the acceleration due to external forces
  for (size_t i = 0; i < m_num_contacts; ++i) {
    math::SurfacePatch::ConstPtr patch = m_patches[i];
    math::Vector3d ctc                 = patch->evaluate(ctc_pos[i]);
    math::Vector3d frc                 = ctc_frc[i];

    // accp += R^-1 * f
    acc.head<3>() += rotm.transpose() * frc;
    // accr += R^-1 * ((R * ctc) x f)
    acc.tail<3>() += -rotm.transpose() * frc.skew() * rotm * ctc;
  }

  // compute the effects of gravity and coriolis/centrifugal forces
  math::Matrix3d velr_cross = velr.skew();
  math::Vector3d tmp = velr_cross * velp + velr_cross * velr_cross * m_pcm
                       - rotm.transpose() * m_gravity;

  math::Vector6d C = math::Vector6d::Zero();
  C.head<3>()      = m_mass * tmp;
  C.tail<3>()      = m_mass * m_pcm.skew() * tmp + velr_cross * m_Icm * velr;

  acc = m_M_inv * (acc - C);

  return acc;
}

//==============================================================================
// Get the Jacobian of the acceleration w.r.t. the position
math::Matrix6X ObjDynamics::get_dadp(
    const math::VectorXd& pos,
    const math::Vector6d& bvel,
    std::vector<math::Vector2d> ctc_pos,
    std::vector<math::Vector3d> ctc_frc) const {

  CROBOT_ASSERT_ERROR(
      ctc_pos.size() == m_num_contacts, "ctc_pos size mismatch");
  CROBOT_ASSERT_ERROR(
      ctc_frc.size() == m_num_contacts, "ctc_frc size mismatch");

  math::Quaternion quat(pos(3), pos(4), pos(5), pos(6));
  math::Matrix3d rotm = quat.normalized().as_rotm();

  (void)bvel; // Unused in this function

  // partial acc / partial rot
  math::Matrix6X dacc_dr = math::Matrix6X::Zero(6, 4);

  // compute the derivative part due to external forces
  for (size_t i = 0; i < m_num_contacts; ++i) {
    math::SurfacePatch::ConstPtr patch = m_patches[i];
    math::Vector3d ctc                 = patch->evaluate(ctc_pos[i]);
    math::Vector3d frc                 = ctc_frc[i];

    math::Matrix34 daccp_dr_i = quat.nonunit_diff_conjugate_rotvec_coeffs(frc);
    math::Matrix34 daccr_dr_i
        = -quat.nonunit_diff_conjugate_rotvec_coeffs(frc.skew() * rotm * ctc)
          - rotm.transpose() * frc.skew()
                * quat.nonunit_diff_rotvec_coeffs(
                    ctc); // need to add another term due to partial derivative
                          // of ctc forces (TBD)

    dacc_dr.block<3, 4>(0, 0) += daccp_dr_i;
    dacc_dr.block<3, 4>(3, 0) += daccr_dr_i;
  }

  // compute the derivative due to the effects of gravity and
  // coriolis/centrifugal forces
  dacc_dr.block<3, 4>(0, 0)
      += m_mass * quat.nonunit_diff_conjugate_rotvec_coeffs(m_gravity);
  dacc_dr.block<3, 4>(3, 0)
      += m_mass * m_pcm.skew()
         * quat.nonunit_diff_conjugate_rotvec_coeffs(m_gravity);

  dacc_dr = m_M_inv * dacc_dr;

  math::Matrix6X dadp    = math::Matrix6X::Zero(6, 7);
  dadp.block<6, 4>(0, 3) = dacc_dr;

  return dadp;
}

//==============================================================================
// Get the Jacobian of the acceleration w.r.t. the velocity (velp, velr)
math::Matrix6d ObjDynamics::get_dadv(
    const math::VectorXd& pos,
    const math::Vector6d& bvel,
    std::vector<math::Vector2d> ctc_pos,
    std::vector<math::Vector3d> ctc_frc) const {

  (void)ctc_pos;
  (void)ctc_frc;
  (void)pos; // Not used in this function

  math::Vector3d velp       = bvel.head<3>();
  math::Matrix3d velp_cross = velp.skew();
  math::Vector3d velr       = bvel.tail<3>();
  math::Matrix3d velr_cross = velr.skew();

  math::Matrix3d dwwp_dw
      = -(velr_cross * m_pcm).skew() - velr_cross * m_pcm.skew();
  math::Matrix3d dwIw_dw = -(m_Icm * velr).skew() + velr_cross * m_Icm;

  math::Matrix6d dCdv = math::Matrix6d::Zero();

  // d(accp) / d(velp)
  dCdv.block<3, 3>(0, 0) = m_mass * velr_cross;

  // d(accr) / d(velp)
  dCdv.block<3, 3>(3, 0) = m_mass * m_pcm.skew() * velr_cross;

  // d(accp) / d(velr)
  dCdv.block<3, 3>(0, 3) = m_mass * (-velp_cross + dwwp_dw);

  // d(accr) / d(velr)
  dCdv.block<3, 3>(3, 3)
      = m_mass * m_pcm.skew() * (-velp_cross + dwwp_dw) + dwIw_dw;

  // dadv.block<3, 3>(0, 3) = -velp.skew() / m_mass;
  // dadv.block<3, 3>(3, 3)
  //     = velr.skew() * inertia_rotated_inv -
  //     (inertia_rotated_inv * velr).skew();

  math::Matrix6d dadv = -m_M_inv * dCdv;

  return dadv;
}

//==============================================================================
// Get the Jacobian of the acceleration w.r.t. the contact positions
math::Matrix6X ObjDynamics::get_dadc(
    const math::VectorXd& pos,
    const math::Vector6d& bvel,
    std::vector<math::Vector2d> ctc_pos,
    std::vector<math::Vector3d> ctc_frc) const {

  (void)bvel;

  CROBOT_ASSERT_ERROR(
      ctc_pos.size() == m_num_contacts, "ctc_pos size mismatch");

  math::Quaternion quat(pos(3), pos(4), pos(5), pos(6));
  math::Matrix3d rotm = quat.normalized().as_rotm();

  (void)bvel; // Unused in this function

  math::Matrix6X dadc = math::Matrix6X::Zero(6, 2 * m_num_contacts);

  for (size_t i = 0; i < m_num_contacts; ++i) {
    math::SurfacePatch::ConstPtr patch = m_patches[i];
    (void)patch->evaluate(ctc_pos[i]); // Result not used, only side effects
    math::Vector3d frc = ctc_frc[i];

    dadc.block<3, 2>(3, 2 * i)
        = -rotm.transpose() * frc.skew() * rotm * patch->jacb(ctc_pos[i]);
  }

  dadc = m_M_inv * dadc;

  return dadc;
}
