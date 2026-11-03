#pragma once

#include "crobot/kdl/body_contact.hpp"
#include "crobot/kdl/body_frame.hpp"
#include "crobot/kdl/rigid_body.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/optimization/cost_term.hpp"
#include "crobot/optimization/ipopt_solver.hpp"
#include "crobot/optimization/variable_set.hpp"

#include "yaml-cpp/yaml.h"

namespace crobot::kdl {

class BodyRollingContact : public BodyContact {
public:
  using Ptr      = std::shared_ptr<BodyRollingContact>;
  using ConstPtr = std::shared_ptr<const BodyRollingContact>;

  BodyRollingContact(
      ContactModel contact_model,
      ConstRigidBodyPtr body1,
      math::SurfacePatchSwitch::Ptr surface1,
      math::Vector2d contact_point1,
      ConstRigidBodyPtr body2,
      math::SurfacePatchSwitch::Ptr surface2,
      math::Vector2d contact_point2,
      double mu = 1.0);

  static Ptr from_yaml_node(
      const YAML::Node& node, ConstRigidBodyTreePtr rbtree);

  virtual ~BodyRollingContact() = default;

  /**
   * @brief Get the force applied at the contact point
   *
   * @return math::Vector3d force in the world frame
   */
  math::Vector3d get_xfrc() const;

  /**
   * @brief Get the torque applied at the contact point
   *
   * @return math::Vector3d torque in the world frame
   */
  math::Vector3d get_xtrq() const;

  /**
   * @brief update the contact force in the world frame
   *
   * @param xfrc
   */
  void update_xfrc(math::Vector3d xfrc);

  /**
   * @brief update the contact torque in the world frame
   *
   * @param xtrq
   */
  void update_xtrq(math::Vector3d xtrq);

  /**
   * @brief Check if the contact is valid rolling by checking the origin and
   * normal vector of the two frames (same xpos, opposite xnorm)
   */

  bool is_valid_rolling(const std::vector<math::Matrix4d>& tforms) const;

  /**
   * @brief update the contact points in surface parameterization (u, v)
   *
   * @param contact_point1 The contact point on the first body (object)
   * @param contact_point2 The contact point on the second body (finger)
   */
  void set_contact_points(
      math::Vector2d& contact_point1, math::Vector2d& contact_point2);

  void move_contact_points(
      const math::Vector2d& vec1, const math::Vector2d& vec2);

  /**
   * @brief update the contact points in moving dir uv_dot
   *
   * @param uv_dot1
   * @param uv_dot2
   */
  void update_contact_points_in_dir(
      const math::Vector2d& uv_dot1, const math::Vector2d& uv_dot2, double dt);

  /** @brief update the contact points based on the xpos of the contact point
   * provided by collision detection
   *
   * @param xpos the xpos of the contact point
   */

  void update_contact_points_by_xpos(
      const math::Vector3d& xpos, const std::vector<math::Matrix4d>& tforms);

  // compute the uv coordinates of the closest point on the surface to the given
  // point by optimization
  math::Vector2d evaluate_uv(
      const math::Vector3d& bpos,
      double surface_id,
      const math::Vector2d& uv_guess) const;

  /**
   * @brief Get the contact points in surface parameterization (u, v)
   *
   * @return std::pair<math::Vector2d, math::Vector2d>
   */
  std::pair<math::Vector2d, math::Vector2d> get_contact_points() const;

  /**
   * @brief Get the grasp map portion corresponding to this frame.
   *
   * The frame map expresses the relationship between the force and wrench
   * applied by the finger contacting the object and their effect at the center
   * of mass of the object.
   *
   * The frame map (R^{6xm}) is defined as
   *
   *    G = [ bpos.cross() * rotm                                     0 ] * S.T
   *         [bvel.cross() * rotm + bpos.cross() * bvelr_c.cross() * rotm
   *                                                 bpos.cross() * rotm]
   *
   * where S is the frame selection matrix, bvelr_c is the angular velocity of
   * the contact frame 1 in body frame.
   */
  math::Matrix6X get_contact_map_deriv() const;

  // /**
  //  * @brief Get the inverse of contact map
  //  *
  //  * @return math::Matrix6X
  //  */
  // math::MatrixX6 get_contact_map_inverse() const;

  // /**
  //  * @brief Map a body frame wrench to the contact frame through the inverse
  //  of
  //  * the contact map.
  //  *
  //  * @param bwrench
  //  * @return math::Vector6d
  //  */
  // math::Vector6d map_bwrench_to_cwrench(const math::Vector6d& bwrench) const;

  std::pair<math::Vector2d, math::Vector2d> get_moving_dir(
      const std::vector<math::Matrix4d>& tforms,
      const math::VectorXd& qvel) const;

  /**
   * @brief Get the contact point moving direction in surface
   * parameterization
   *
   * @param tforms
   * @param xvelr_o angular velocity of the object in the world frame
   * @param xvelr_m angular velocity of the manipulator/finger in the world
   * frame
   * @param xvelp_o linear velocity of the object in the world frame
   * @param xvelp_m linear velocity of the manipulator/finger in the world
   * @return moving direction of the contact point in surface
   * parameterization (u,v) on the object and manipulator surfaces
   */
  std::pair<math::Vector2d, math::Vector2d> get_moving_dir(
      const std::vector<math::Matrix4d>& tforms,
      const math::Vector3d& xvelr_o,
      const math::Vector3d& xvelr_m,
      const math::Vector3d& xvelp_diff = math::Vector3d::Zero()) const;

  std::pair<math::Vector2d, math::Vector2d> get_rolling_moving_dir(
      const std::vector<math::Matrix4d>& tforms,
      const math::VectorXd& qvel) const;

  /**
   * @brief Update the contact point moving direction in the surface
   * parameterization
   *
   * @param dudv1 <udot, vdot> at surface 1
   * @param dudv2 <udot, vdot> at surface 2
   */
  void set_moving_dir(const math::Vector2d& dudv1, const math::Vector2d& dudv2);

  /**
   * @brief Get the contact point moving direction in the world frame
   *
   * @param tforms
   * @param xvelr_o angular velocity of the object in the world frame
   * @param xvelr_m angular velocity of the manipulator/finger in the world
   * frame
   * @return moving direction of the contact point in the world frame
   */
  // math::Vector3d get_moving_xdir(
  //     const std::vector<math::Matrix4d>& tforms,
  //     const math::Vector3d& xvelr_o,
  //     const math::Vector3d& xvelr_m) const;

  using BodyContact::is_active;

  using BodyContact::update_status;

  math::Vector3d get_bvelr_ctc1() const;

  math::Vector3d get_xvelr_ctc1(
      const std::vector<math::Matrix4d>& tforms) const;

  math::Matrix32 get_jacb1() const;

  math::Matrix32 get_jacb2() const;

  BodyRollingContact::Ptr clone() const;

  math::SurfacePatchSwitch::Ptr get_surface1() const;

  math::SurfacePatchSwitch::Ptr get_surface2() const;

  bool is_boundary_contact() const;

  YAML::Node to_yaml_node() const;

private:
  /**
   * @brief Update the frames of the contact based on the current contact
   * points
   */
  void update_frames();

  // 1 is the contact surface in the object frame
  math::SurfacePatchSwitch::Ptr m_surface1;

  // 2 is the contact surface in the finger frame
  math::SurfacePatchSwitch::Ptr m_surface2;
  // u, v
  math::Vector2d m_contact_point1;
  math::Vector2d m_contact_point2;

  math::Vector3d m_xfrc;
  math::Vector3d m_xtrq;

  math::Vector2d m_uv_dot1;
  math::Vector2d m_uv_dot2;

  std::vector<int> m_qvel_addr1;
  std::vector<int> m_qvel_addr2;

  std::vector<std::shared_ptr<optimization::Problem>> m_problems;
  std::shared_ptr<optimization::IpoptSolver> m_solver;
};

} // namespace crobot::kdl

