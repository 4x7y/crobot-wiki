#pragma once

#include <yaml-cpp/yaml.h>

#include "crobot/ik/cons_body_contact.hpp"
#include "crobot/ik/cons_plane_contact.hpp"
#include "crobot/ik/cons_point_contact.hpp"
#include "crobot/ik/cons_surface_params.hpp"
#include "crobot/ik/cost_body_contact_potential.hpp"
#include "crobot/ik/cost_body_pose_potential.hpp"
#include "crobot/ik/vars_obj_pose.hpp"
#include "crobot/ik/vars_surface_patch.hpp"
#include "crobot/kdl/kdl.hpp"
#include "crobot/math/math.hpp"

namespace crobot::ik {

class PoseRefiner {
  /**
   * refine the pose of an object with
   * QR code estimation and robot forward kinematics
   */

public:
  /**
   * @brief Construct and initialize the pose refinement problem
   * from a RigidBodyTree.
   *
   * @param rbtree A RigidBodyTree instance.
   */
  explicit PoseRefiner(kdl::ConstRigidBodyTreePtr rbtree);

  /**
   * @brief Solve for the problem.
   *
   * @param max_iter The maximum number of iterations.
   * @param max_time The maximum time in seconds.
   * @param verbose If the solver should print out the progress.
   * @return true If the solver converges.
   * @return false If the solver does not converge.
   */
  bool solve(
      const math::VectorXd& initial_pose,
      int max_iter    = 1,
      double max_time = 0.1,
      bool verbose    = false);

  /**
   * @brief Print the details of the IK problem.
   */
  void show_details() const;

  /**
   * @brief Add a constraint to enforce that two surface patches are in
   * contact with each other.
   *
   * @param name Name of the constraint
   * @param body1 Name of the first body
   * @param patch1 The first surface patch
   * @param body2 Name of the second body
   * @param patch2 The second surface patch
   * @param tolerance Maximum allowed contact tolerance
   * @return std::pair<optimization::VariableSet::Ptr,
   * optimization::VariableSet::Ptr> The variables that represent the
   * parameter of two surface patches.
   */
  std::pair<optimization::VariableSet::Ptr, optimization::VariableSet::Ptr>
  add_body_contact_constraint(
      const std::string& name,
      const std::string& body_obj,
      const std::shared_ptr<const math::SurfacePatch>& patch_obj,
      const std::string& body_rob,
      const std::shared_ptr<const math::SurfacePatch>& patch_rob,
      double tolerance_p = 0.0,
      double tolerance_n = 0.0);

  void add_plane_contact_constraint(
      const std::string& name,
      const math::Vector3d& bpos,
      const math::Vector3d& point,
      const math::Vector3d& normal,
      double tolerance = 0.0);

  void add_surface_params_constraint(
      const std::string& name,
      const std::string& varname,
      const math::Vector2d& uv_est,
      double tolerance);

  void add_point_contact_constraint(
      const std::string& name,
      const math::Vector3d& bpos,
      const math::Vector3d& xpos,
      double tolerance = 0.0);

  void add_potential_body_pose_cost(const math::VectorXd& pose_est);

  void add_potential_body_contact_cost(
      const std::string& name,
      const std::string& body_obj,
      const std::shared_ptr<const math::SurfacePatch>& patch_obj,
      const std::string& body_rob,
      const std::shared_ptr<const math::SurfacePatch>& patch_rob,
      const math::Vector3d& xpos_screw);

  /**
   * @brief Validate the IK constraints and costs by comparing the exact
   * Jacobian with the numerical finite difference Jacobian.
   */
  void validate_all() const;

  /**
   * @brief Set the initial guess for the IK problem.
   *
   * @param pose xpos(3) + xquat(4)
   */
  void set_init_pose(const math::VectorXd& pose);

  void set_qpos(const math::VectorXd& qpos);

  void set_init_uv(const std::string& var_name, const math::Vector2d& uv);

  void set_plane_point(const math::Vector3d& point);

  void set_plane_normal(const math::Vector3d& normal);

  /**
   * @brief Get the solution of the IK problem.
   *
   * @return math::VectorXd Joint positions.
   */
  math::VectorXd get_pose();

  void set_cost_pose_est(const math::VectorXd& xpose_est);

  void set_point_ctc(const math::Vector3d& xpos);

  void set_surf_params(const std::vector<math::Vector2d>& uv);

private:
  /// The non-linear optimization problem.
  optimization::Problem m_nlp;

  /// The RigidBodyTree instance.
  kdl::ConstRigidBodyTreePtr m_rbtree;

  /// object pose variable
  ObjectPoseVariables::Ptr m_pose_varset;
  std::vector<ik::SurfacePatchVariables::Ptr> m_patch1_varsets;
  std::vector<ik::SurfacePatchVariables::Ptr> m_patch2_varsets;

  // body contact constraints
  std::vector<ik::BodyContactConstraint::Ptr> m_cons_body_ctc;

  // plane contact constraints
  ik::PlaneContactConstraint::Ptr m_cons_plane_ctc;

  // point contact constraint
  ik::PointContactConstraint::Ptr m_cons_point_ctc;

  std::vector<ik::SurfaceParamsConstraint::Ptr> m_surf_params_cons;

  // cost
  BodyPosePotentialCost::Ptr m_cost_body_pose_potential;

  std::vector<ik::BodyContactPotentialCost::Ptr> m_cost_ctc;
};

} // namespace crobot::ik

