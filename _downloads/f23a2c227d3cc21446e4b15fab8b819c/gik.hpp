#pragma once

#include <yaml-cpp/yaml.h>

#include "crobot/ik/constraint.hpp"
#include "crobot/ik/vars_qpos.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/surface_patch.hpp"
#include "crobot/optimization/problem.hpp"

namespace crobot::ik {

class GeneralizedIK {
public:
  /**
   * @brief Load an IK problem from a file.
   *
   * @param rbtree The rigid body tree.
   * @param filename The file to load.
   * @return GeneralizedIK The loaded IK problem.
   */
  static std::shared_ptr<GeneralizedIK> LoadFromFile(
      kdl::ConstRigidBodyTreePtr rbtree, const std::string& filename);

  /**
   * @brief Load an IK problem from a YAML node.
   *
   * @param rbtree The rigid body tree.
   * @param node The YAML node.
   * @return std::shared_ptr<GeneralizedIK>
   */
  static std::shared_ptr<GeneralizedIK> LoadFromYamlNode(
      kdl::ConstRigidBodyTreePtr rbtree, const YAML::Node& node);

  /**
   * @brief Load an IK problem from a YAML node and necessary geometries.
   *
   * @param rbtree The rigid body tree.
   * @param node The YAML node.
   * @param geoms The set of geometries to be used for constructing surface and
   * curve contact constraints.
   * @return std::shared_ptr<GeneralizedIK>
   */
  static std::shared_ptr<GeneralizedIK> LoadFromYamlNode(
      kdl::ConstRigidBodyTreePtr rbtree,
      const YAML::Node& node,
      const std::unordered_map<std::string, math::Geometry::ConstPtr>& geoms);

  /**
   * @brief Construct and initialize the generalized inverse kinematics problem
   * from a RigidBodyTree.
   *
   * @param rbtree A RigidBodyTree instance.
   * @param limited If the joint range limits are enforced.
   */
  explicit GeneralizedIK(
      kdl::ConstRigidBodyTreePtr rbtree, bool limited = true);

  /**
   * @brief Solve for the IK problem.
   *
   * @param max_iter The maximum number of iterations.
   * @param max_time The maximum time in seconds.
   * @param verbose If the solver should print out the progress.
   * @return true If the solver converges.
   * @return false If the solver does not converge.
   */
  bool solve(int max_iter = 1, double max_time = 0.1, bool verbose = false);

  /**
   * @brief Print the details of the IK problem.
   */
  void show_details() const;

  /**
   * @brief Add a constraint on the position of a body's local point.
   *
   * @param name Name of this IK constraint.
   * @param body Name of the body.
   * @param bpos Position of the body point in the body frame.
   * @param xpos Position of the target in the world frame.
   * @param tolerance Maximum allowed distance tolerance.
   */
  void add_position_target(
      const std::string& name,
      const std::string& body,
      const math::Vector3d& bpos,
      const math::Vector3d& xpos,
      double tolerance = 0.0);

  /**
   * @brief Add a constraint on the position of a body frame origin.
   *
   * @param name Name of this IK constraint.
   * @param body Name of the body.
   * @param bpos Position of the body point in the body frame.
   * @param xpos Position of the target in the world frame.
   * @param tolerance Maximum allowed distance tolerance.
   */
  void add_position_target(
      const std::string& name,
      const std::string& body,
      const math::Vector3d& xpos,
      double tolerance = 0.0);

  /**
   * @brief Add a bounding box constraint on the position of one body
   * (end-effector) relative to a target bounding box fixed on another body (the
   * reference body). This constraint is satisfied if the position of the
   * end-effector bpos relative to the target bounding box center remains within
   * the Bounds specified.
   *
   * @param name Name of this constraint
   * @param body Name of the end-effector body
   * @param bpos Position of the end-effector point in the body frame
   * @param body_bbox Name of the body that the target bounding box is fixed on
   * @param bpos_bbox Position of the target bounding box center in the
   * reference body
   * @param rotm_bbox Rotation matrix of the target bounding box in the
   * reference body
   * @param size_bbox Half-sizes of the target bounding box in the reference
   * @param tolerance Maximum allowed distance tolerance
   */
  void add_bounding_box_constraint(
      const std::string& name,
      const std::string& body,
      const math::Vector3d& bpos,
      const std::string& body_bbox,
      const math::Vector3d& bpos_bbox,
      const math::Matrix3d& rotm_bbox,
      const math::Vector3d& size_bbox,
      double tolerance = 0.0);

  /**
   * @brief Add a constraint on the relative distance between a body local point
   * and a fixed point in the world frame.
   *
   * @param name Name of this IK constraint.
   * @param body Name of the body.
   * @param bpos Position of the body point in the body frame.
   * @param xpos Position of the target in the world frame.
   * @param distance Distance between the two points.
   * @param tolerance Maximum allowed distance tolerance.
   */
  void add_distance_target(
      const std::string& name,
      const std::string& body,
      const math::Vector3d& bpos,
      const math::Vector3d& xpos,
      double distance,
      double tolerance = 0.0);

  /**
   * @brief Add a constraint on the relative distance between a body frame
   * origin and a fixed point in the world frame.
   *
   * @param name Name of this IK constraint.
   * @param body Name of the body.
   * @param xpos Position of the target in the world frame.
   * @param distance Distance between the two points.
   * @param tolerance Maximum allowed distance tolerance.
   */
  void add_distance_target(
      const std::string& name,
      const std::string& body,
      const math::Vector3d& xpos,
      double distance,
      double tolerance = 0.0);

  void add_distance_target(
      const std::string& name,
      const std::string& body1,
      const math::Vector3d& bpos1,
      const std::string& body2,
      const math::Vector3d& bpos2,
      double distance,
      double tolerance = 0.0);

  void add_opposite_orientation_target(
      const std::string& name,
      const std::string& body1,
      const math::Vector3d& bpos1,
      const math::Vector3d& bvec1,
      const std::string& body2,
      const math::Vector3d& bpos2,
      const math::Vector3d& bvec2,
      double tolerance = 0.0);

  void add_cartesian_target(
      const std::string& name,
      const std::string& body,
      const math::Vector3d& bpos,
      const math::Vector3d& xpos_lb,
      const math::Vector3d& xpos_ub);

  void add_cartesian_target(
      const std::string& name,
      const std::string& body,
      const math::Vector3d& xpos_lb,
      const math::Vector3d& xpos_ub);

  void add_orientation_target(
      const std::string& name,
      const std::string& body,
      const math::Quaternion& xquat,
      double tolerance = 0.0);

  /**
   * @brief Constrains the angle difference between a world frame vector and a
   * vector fixed to a given body frame.
   *
   * @param name name of the constraint
   * @param body name of the body
   * @param bvec local vector relative to the body frame
   * @param xvec target vector relative to the world frame
   * @param tolerance angle tolerance in radian
   */
  void add_orientation_vector_target(
      const std::string& name,
      const std::string& body,
      const math::Vector3d& bvec,
      const math::Vector3d& xvec,
      double tolerance = 0.0);

  /**
   * @brief Add a constraint that requires the z-axis of one body (the end
   * effector) to aim at a world frame target point. This constraint is
   * satisfied if the z-axis of the end-effector frame is within an angular
   * tolerance in any direction of the line connecting the end-effector origin
   * and the target point.
   *
   * @param name Name of the constraint
   * @param body Name of the body
   * @param xpos Position of the target point in the world frame
   * @param tolerance Angle tolerance in radian
   */
  void add_aiming_target(
      const std::string& name,
      const std::string& body,
      const math::Vector3d& xpos,
      double tolerance = 0.0);

  /**
   * @brief Add a constraint that requires a body vector (located at body frame
   * origin) of one body to aim at a world frame target point. This constraint
   * is satisfied if the z-axis of the end-effector frame is within an angular
   * tolerance in any direction of the line connecting the end-effector origin
   * and the target point.
   *
   * @param name Name of the constraint
   * @param body Name of the body
   * @param bvec Local vector relative to the body frame
   * @param xpos Position of the target point in the world frame
   * @param tolerance Angle tolerance in radian
   */
  void add_aiming_target(
      const std::string& name,
      const std::string& body,
      const math::Vector3d& bvec,
      const math::Vector3d& xpos,
      double tolerance = 0.0);

  /**
   * @brief Add a constraint that requires a body vector (located at a given
   * body frame point) of one body to aim at a world frame target point. This
   * constraint is satisfied if the z-axis of the end-effector frame is within
   * an angular tolerance in any direction of the line connecting the
   * end-effector origin and the target point.
   *
   * @param name Name of the constraint
   * @param body Name of the body
   * @param bpos Origin of the body frame vector
   * @param bvec Direction of the body frame vector
   * @param xpos Position of the target point in the world frame
   * @param tolerance Angle tolerance in radian
   */
  void add_aiming_target(
      const std::string& name,
      const std::string& body,
      const math::Vector3d& bpos,
      const math::Vector3d& bvec,
      const math::Vector3d& xpos,
      double tolerance = 0.0);

  /**
   * @brief Add a joint coupling constraint to the IK problem.
   *
   * The constraint enforces that
   *
   *   c1 * qpos[body1.joint_id] + c2 * qpos[body2.joint_id] == 0
   *
   * within the given tolerance.
   */
  void add_joint_coupling_constraint(
      const std::string& name,
      const std::string& joint1_name,
      double coeff1,
      const std::string& joint2_name,
      double coeff2,
      double tolerance = 0.0);

  /**
   * @brief Add a joint position constriant to the IK problem.
   *
   * @param name Name of the constraint.
   * @param joint_name Name of the joint.
   * @param qpos Joint position.
   * @param tolerance Maximum allowed joint position tolerance.
   */
  void add_joint_position_constraint(
      const std::string& name,
      const std::string& joint_name,
      double qpos,
      double tolerance = 0.0);

  /**
   * @brief Add a constraint to enforce that two surface patches are in contact
   * with each other.
   *
   * @param name Name of the constraint
   * @param body1 Name of the first body
   * @param patch1 The first surface patch
   * @param body2 Name of the second body
   * @param patch2 The second surface patch
   * @param tolerance Maximum allowed contact tolerance
   * @return std::pair<optimization::VariableSet::Ptr,
   * optimization::VariableSet::Ptr> The variables that represent the parameter
   * of two surface patches.
   */
  std::pair<optimization::VariableSet::Ptr, optimization::VariableSet::Ptr>
  add_surface_contact_constraint(
      const std::string& name,
      const std::string& body1,
      const std::shared_ptr<const math::SurfacePatch>& patch1,
      const std::string& body2,
      const std::shared_ptr<const math::SurfacePatch>& patch2,
      double tolerance = 0.0);

  std::pair<optimization::VariableSet::Ptr, optimization::VariableSet::Ptr>
  add_surface_curve_contact_constraint(
      const std::string& name,
      const std::string& body1,
      const std::shared_ptr<const math::SurfacePatch>& patch,
      const std::string& body2,
      const std::shared_ptr<const math::CurveSegment>& segment,
      double tolerance = 0.0);

  /**
   * @brief Add a constraint to enforce a ball joint configuration
   * (represented by a quaternion) to be valid, thus norm equals 1.
   *
   * @param name Name of the constraint.
   * @param joint_name Name of the joint.
   * @param tolerance Maximum allowed squared norm tolerance.
   */
  void add_unit_quaternion_constraint(
      const std::string& name,
      const std::string& joint_name,
      double tolerance = 0.0);

  /**
   * @brief Add a unit norm constraint for each of the ball joints in the given
   * RigidBodyTree model.
   *
   * @param tolerance Maximum allowed squared norm tolerance.
   */
  void add_unit_quaternion_constraint(double tolerance = 0.0);

  /**
   * @brief Potential cost type for the IK problem.
   */
  enum PotentialCost
  {
    /// @brief Squared sum cost on the joint distance to the center of joint
    /// limits.
    QPOS_QUADRATIC_POTENTIAL,

    /// @brief \f$ \frac{1}{(q - q_{lb}) (q - q_{ub})} \f$ cost on the joint
    /// distance to the center of joint limits.
    QPOS_USHAPE_POTENTIAL,

    BODY_POSE_POTENTIAL,
  };

  /**
   * @brief Add a potential cost to the IK problem.
   */
  void add_potential_cost(PotentialCost cost = QPOS_QUADRATIC_POTENTIAL);

  void fix_joint_range(
      const std::pair<int, int>& range, const math::VectorXd& qpos) const;

  /**
   * @brief Fix a joint to a given position by taking the joint's configuration
   * variable out of the optimization.
   *
   * @param joint_name Name of the joint.
   * @param qpos Joint position.
   */
  void fix_joint(
      const std::string& joint_name, const math::VectorXd& qpos) const;

  /**
   * @brief Fix a joint to a given position by taking the joint's configuration
   * variable out of the optimization. The joint must be a prisimatic joint or a
   * revolute joint.
   *
   * @param joint_name Name of the joint.
   * @param qpos Joint position.
   */
  void fix_joint(const std::string& joint_name, double qpos) const;

  /**
   * @brief Fix a joint to a given position by taking the joint's configuration
   * variable out of the optimization. The joint must be a ball joint.
   *
   * @param joint_name Name of the joint.
   * @param qpos Joint position.
   */
  void fix_joint(
      const std::string& joint_name, const math::Quaternion& qpos) const;

  /**
   * @brief Validate the IK constraints by comparing the exact Jacobian with the
   * numerical finite difference Jacobian.
   */
  void validate_constraints() const;

  /**
   * @brief Set the initial guess for the IK problem.
   *
   * @param qpos Joint positions.
   */
  void set_init_qpos(const math::VectorXd& qpos);

  void set_init_guess_limit(
      const math::VectorXd& qpos_ref,
      const math::VectorXd& qpos_offset_limit,
      int solve_count = 5);

  /**
   * @brief Get the solution of the IK problem.
   *
   * @return math::VectorXd Joint positions.
   */
  math::VectorXd get_qpos();

  /**
   * @brief Get the constriant with the given name.
   *
   * @param name Name of the constraint.
   * @return ik::Constraint::Ptr Constraint.
   */
  ik::Constraint::Ptr get_constriant(const std::string& name);

  void add_contact_constraint(
      const std::string& name,
      const std::string& body1_name,
      const math::Matrix4d& body1_tform,
      const math::Matrix4d& body2_xform,
      double pos_tol = 0.0,
      double ori_tol = 0.0);

  optimization::VariableSet::Ptr add_surface_point_contact_constraint(
      const std::string& name,
      const std::string& body1,
      const std::shared_ptr<const math::SurfacePatch>& patch1,
      const math::Vector3d& xpos_body2,
      const math::Vector3d& xvec_body2,
      double tolerance = 0.0);

  optimization::VariableSet::Ptr add_object_contact_constraint(
      const std::string& name,
      const std::string& body_obj,
      const math::Vector3d& bpos_obj,
      const math::Vector3d& bvec_obj,
      const std::string& body_rob,
      std::shared_ptr<const math::SurfacePatch> patch_rob,
      double tolerance_p = 0.0,
      double tolerance_n = 0.0);

private:
  /// The non-linear optimization problem.
  optimization::Problem m_nlp;

  /// @brief The RigidBodyTree instance.
  kdl::ConstRigidBodyTreePtr m_rbtree;

  /// Joint configuration variable
  std::shared_ptr<QposVariables> m_qpos_varset;

  math::VectorXd m_qpos_ref;
  math::VectorXd m_qpos_offset_limit;
  int m_solve_count;
};

} // namespace crobot::ik

