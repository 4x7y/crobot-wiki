#pragma once

#include <memory>

#include "crobot/kdl/rigid_body.hpp"
#include "crobot/kdl/rolling_contact.hpp"

/**
 * this class is used to represent a grasp with rolling contacts
 * it is no longer used in the real experiment (crobot/control/actions) but
 * still used in simulation (crobot/control/screwdriving_ctrl)
 * used together with RollingContact
 * Fix later
 */

namespace crobot::kdl {

class RollingGrasp {
public:
  using Ptr      = std::shared_ptr<RollingGrasp>;
  using ConstPtr = std::shared_ptr<const RollingGrasp>;

  /**
   * @brief Construct a grasp with a body.
   *
   * @param body The RigidBody to grasp.
   */
  explicit RollingGrasp(ConstRigidBodyPtr body);

  /**
   * @brief Add a contact point to the grasp.
   *
   * @param contact_model The contact model
   * @param tform_manip The pose of the contact frame with respect to the
   * object frame
   * @param manipulator The manipulator that is grasping the object
   * @param tform_manip The pose of the contact frame with respect to the
   * manipulator's body frame
   * @param mu The friction coefficient
   */
  void add_contact(
      ContactModel contact_model,
      ConstRigidBodyPtr body1,
      math::SurfacePatch::ConstPtr surface1,
      math::Vector2d& contact_point1,
      ConstRigidBodyPtr body2,
      math::SurfacePatch::ConstPtr surface2,
      math::Vector2d& contact_point2,
      double mu = 1.0);

  const std::vector<RollingContact::Ptr>& get_contacts() const;

  /**
   * @brief Get the contact on the object.
   *
   * @param body_name
   * @return ConstBodyContactPtr
   */
  RollingContact::Ptr get_contact(const std::string& body_name) const;

  /**
   * @brief Get the number of contacts in the grasp.
   *
   * @return int Number of contacts in the grasp
   */
  int get_num_contacts() const;

  /**
   * @brief Get the number of wrench basis
   *
   * @return int
   */
  int get_num_wrench_basis() const;

  /**
   * @brief Get the grasp matrix
   *
   * The grasp matrix G defines how forces exerted at the contact points are
   * mapped into wrenches.
   *
   */
  math::Matrix6X get_grasp_matrix() const;

  /**
   * @brief Get the grasp matrix G dot
   *
   * The grasp matrix G defines how forces exerted at the contact points are
   * mapped into wrenches.
   *
   */
  math::Matrix6X get_grasp_matrix_deriv() const;

  /**
   * @brief Get the grasp matrix's pseudo inverse
   *
   * The grasp matrix Ginv defines how bwrench exerted at the body are
   * mapped into cwrenches.
   *
   */
  math::MatrixX6 get_grasp_matrix_pinv() const;

  // get contact force stack in the world frame
  math::VectorXd get_ctc_xfrc_stack() const;

  // get contact force stack in the contact frame
  math::VectorXd get_ctc_cfrc_stack(
      const std::vector<math::Matrix4d>& tforms) const;

private:
  ConstRigidBodyPtr m_body;
  std::vector<RollingContact::Ptr> m_contacts;
};

} // namespace crobot::kdl

