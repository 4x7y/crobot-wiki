#pragma once

#include "crobot/common/common.hpp"
#include "crobot/math/math.hpp"

namespace crobot::ekf {

class ObjDynamics {
public:
  using Ptr      = std::shared_ptr<ObjDynamics>;
  using ConstPtr = std::shared_ptr<const ObjDynamics>;

  /**
   * @brief Construct a new Obj Dynamics object
   * @param mass The mass of the object
   * @param pcm The position of the center of mass
   * @param inertia_cm The inertia of the object at the center of mass w.r.t.
   * the tool frame's orientation
   * @param patches The contact patches on the object
   * @param gravity The gravity vector (0, 0, -9.8)
   */
  ObjDynamics(
      double mass,
      const math::Vector3d& pcm,
      const math::Matrix3d& inertia_cm,
      const std::vector<math::SurfacePatchSwitch::ConstPtr>& patches,
      const math::Vector3d& gravity);

  int get_num_contacts() const;

  /**
   * @brief Get the acceleration of the object through dynamics
   * @param pos The position of the object, position + quaternion (unit
   * parameterization (theta, phi, psi))
   * @param vel The velocity of the object
   * @param ctc_pos The contact positions parameterized by (u, v)
   * @param ctc_frc The contact forces
   */
  math::Vector6d get_acc(
      const math::VectorXd& pos,
      const math::Vector6d& bvel,
      std::vector<math::Vector2d> ctc_pos,
      std::vector<math::Vector3d> ctc_frc) const;

  /**
   * @brief Get the Jacobian of the acceleration w.r.t. the position
   * @param pos The position of the object, position + quaternion (unit
   * parameterization (theta, phi, psi))
   * @param vel The velocity of the object
   * @param ctc_pos The contact positions parameterized by (u, v)
   * @param ctc_frc The contact forces
   */
  math::Matrix6X get_dadp(
      const math::VectorXd& pos,
      const math::Vector6d& bvel,
      std::vector<math::Vector2d> ctc_pos,
      std::vector<math::Vector3d> ctc_frc) const;

  /**
   * @brief Get the Jacobian of the acceleration w.r.t. the velocity
   * @param pos The position of the object, position + quaternion (unit
   * parameterization (theta, phi, psi))
   * @param vel The velocity of the object
   * @param ctc_pos The contact positions parameterized by (u, v)
   * @param ctc_frc The contact forces
   */

  math::Matrix6d get_dadv(
      const math::VectorXd& pos,
      const math::Vector6d& bvel,
      std::vector<math::Vector2d> ctc_pos,
      std::vector<math::Vector3d> ctc_frc) const;

  /**
   * @brief Get the Jacobian of the acceleration w.r.t. the contact positions
   * @param pos The position of the object, position + quaternion (unit
   * parameterization (theta, phi, psi))
   * @param vel The velocity of the object
   * @param ctc_pos The contact positions parameterized by (u, v)
   * @param ctc_frc The contact forces
   */
  math::Matrix6X get_dadc(
      const math::VectorXd& pos,
      const math::Vector6d& bvel,
      std::vector<math::Vector2d> ctc_pos,
      std::vector<math::Vector3d> ctc_frc) const;

private:
  double m_mass;        // mass of the object (kg)
  math::Vector3d m_pcm; // position of the center of mass (m)
  math::Matrix3d m_Icm; // inertia tensor at the center of mass in tool frame's
                        // orientation (kg m^2)
  std::vector<math::SurfacePatchSwitch::ConstPtr> m_patches; // contact patches
  math::Vector3d m_gravity; // gravity (0,0,-9.8) m/s^2

  size_t m_num_contacts;
  math::Matrix6d m_M;
  math::Matrix6d m_M_inv;
};
} // namespace crobot::ekf

