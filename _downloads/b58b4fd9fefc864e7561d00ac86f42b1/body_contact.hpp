#pragma once

#include "crobot/kdl/body_frame.hpp"
#include "crobot/kdl/collision_contact.hpp"
#include "crobot/kdl/rigid_body.hpp"
#include "crobot/optimization/cost_term.hpp"
#include "crobot/optimization/ipopt_solver.hpp"
#include "crobot/optimization/variable_set.hpp"

namespace crobot::kdl {

class BodyContact : public CollisionContact {
public:
  using Ptr      = std::shared_ptr<BodyContact>;
  using ConstPtr = std::shared_ptr<const BodyContact>;

  BodyContact() = default;

  /**
   * @brief Construct a new Body Contact object.
   * WARNING: This constructor is for old code only. It will be removed in the
   * future.
   *
   * @param contact_model The ContactModel of this contact.
   * @param body1
   * @param frame1
   * @param body2
   * @param frame2
   * @param mu The Columb's friction coefficient at the contact.
   */
  BodyContact(
      ContactModel contact_model,
      ConstRigidBodyPtr body1,
      BodyFrame::Ptr frame1,
      ConstRigidBodyPtr body2,
      BodyFrame::Ptr frame2,
      double mu = 1.0);

  virtual ~BodyContact() = default;

  virtual ConstRigidBodyPtr get_body1() const;

  virtual ConstRigidBodyPtr get_body2() const;

  virtual BodyFrame::Ptr get_frame1() const;

  virtual BodyFrame::Ptr get_frame2() const;

  /**
   * @brief Get the xpos of the contact
   *
   * @return math::Vector3d xpos
   */
  math::Vector3d get_xpos(const std::vector<math::Matrix4d>& tforms) const;

  /**
   * @brief get the xpos of the frame 1
   *
   */
  math::Vector3d get_xpos1(const std::vector<math::Matrix4d>& tforms) const;

  /**
   * @brief get the xpos of the frame 2
   *
   */

  math::Vector3d get_xpos2(const std::vector<math::Matrix4d>& tforms) const;

  /**
   * @brief Get the rotational matrix of the contact frame w.r.t. the world
   * frame located at the object with z axis pointing into the interior of the
   * object.
   *
   * @return math::Matrix3d rotational matrix
   */
  math::Matrix3d get_xmat(const std::vector<math::Matrix4d>& tforms) const;

  /**
   * @brief Get the xmat of frame 1
   *
   */
  math::Matrix3d get_xmat1(const std::vector<math::Matrix4d>& tforms) const;

  /**
   * @brief Get the xmat of frame 2
   *
   */
  math::Matrix3d get_xmat2(const std::vector<math::Matrix4d>& tforms) const;

  /**
   * @brief Get the grasp map portion corresponding to this frame.
   *
   * The frame map expresses the relationship between the force and wrench
   * applied by the finger contacting the object and their effect at the center
   * of mass of the object.
   *
   * The frame map (R^{6xm}) is defined as
   *
   *                 G = [        rotm          0  ] * S.T
   *                     [bpos.cross() * rotm  rotm]
   *
   * where S is the frame selection matrix.
   */
  math::Matrix6X get_contact_map() const;

  /**
   * @brief Get the inverse of contact map
   *
   * @return math::Matrix6X
   */
  math::MatrixX6 get_contact_map_inverse() const;

  /**
   * @brief Map a external contact frame wrench to the body frame through the
   * contact map.
   *
   * The input frame wrench w in the contact frame is mapped to the body frame
   * via
   *
   *                          f = G * S * w
   *
   * where G is the frame map and S is the selection matrix defined according
   * the frame model.
   *
   * @param cw The extenal contact frame wrench (R^6) represented in the contact
   * frame (force, torque).
   * @return The actual wrench applied to the object in its body frame.
   */
  math::Vector6d map_cwrench_to_bwrench(const math::Vector6d& cwrench) const;

  // /**
  //  * @brief Map a body frame wrench to the contact frame through the inverse
  //  of
  //  * the contact map.
  //  *
  //  * @param bwrench
  //  * @return math::Vector6d
  //  */
  // math::Vector6d map_bwrench_to_cwrench(const math::Vector6d& bwrench) const;

  using CollisionContact::is_active;

  using CollisionContact::update_status;

protected:
  ConstRigidBodyPtr m_body1;
  ConstRigidBodyPtr m_body2;
  BodyFrame::Ptr m_frame1;
  BodyFrame::Ptr m_frame2;
};

} // namespace crobot::kdl

