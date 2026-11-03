#pragma once

#include <optional>
#include <set>
#include <tuple>
#include <unordered_map>

#include "crobot/ik/jacobian_ik_target.hpp"
#include "crobot/ik/null_space_objective.hpp"
#include "crobot/kdl/body_contact.hpp"
#include "crobot/kdl/frame.hpp"
#include "crobot/kdl/rigid_body.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

namespace crobot::ik {

enum class JacobianIKAlgorithm
{
  TRANSPOSE,
  SVD,
  DAMPED_LEAST_SQUARE,
  LEVENBERG_MARQUARDT,
};

class JacobianIK {
public:
  using ConstPtr = std::shared_ptr<const JacobianIK>;
  using Ptr      = std::shared_ptr<JacobianIK>;

  /**
   * @brief Construct an empty JacobianIK problem.
   *
   * @param rbtree The RigidBodyTree to use
   * @param alg The Jacobian IK algorithm to use. Valid values are: TRANSPOSE,
   * SVD, DAMPED_LEAST_SQUARE, LEVENBERG_MARQUARDT
   * @param num_iters Maximum number of iterations
   * @param tol Tolerance
   */
  JacobianIK(
      kdl::ConstRigidBodyTreePtr rbtree,
      JacobianIKAlgorithm alg = JacobianIKAlgorithm::DAMPED_LEAST_SQUARE,
      int num_iters           = 100,
      double tol              = 1e-5);

  /**
   * @brief Add a 6 DoF constraint target to the IK problem. The orientation
   * error is reprensented in terms of angle-axis for a better convergence rate.
   *
   * @param desired_frame Desired frame
   * @param body_name Name of the body to constrain
   * @param bpos Position of the reference frame in the body frame
   * @param rotm Orientation of the reference frame in the body frame
   */
  void add_target(
      kdl::ConstFramePtr desired_frame,
      const std::string& body_name,
      const math::Vector3d& bpos = math::Vector3d::Zero(),
      const math::Matrix3d& rotm = math::Matrix3d::Identity(),
      double weight              = 1.0);

  /**
   * @brief Add a set of pose constraints to the IK problem with pose errors
   * represented in the specified directions. The orientation error is
   * reprensented as Euler angles with the specified sequence.
   *
   * @param desired_frame Desired frame
   * @param body_name Name of the body to constrain
   * @param dofs Indices of the degrees of freedom to constrain
   * @param euler_seq Euler angle sequence to use for the orientation error
   * @param bpos Position of the reference frame in the body frame
   * @param rotm Orientation of the reference frame in the body frame
   */
  void add_rolling_target_selected_dofs(
      kdl::ConstFramePtr desired_frame,
      const std::string& body_name,
      const std::vector<int>& dofs,
      kdl::BodyContact::ConstPtr contact,
      math::EulerAngleSeq euler_seq = math::EulerAngleSeq::ZYX,
      double weight                 = 1.0);

  /**
   * @brief Add a set of pose constraints to the IK problem with pose errors
   * represented in the specified directions. The orientation error is
   * reprensented as Euler angles with the specified sequence.
   *
   * @param desired_frame Desired frame
   * @param body_name Name of the body to constrain
   * @param dofs Indices of the degrees of freedom to constrain
   * @param euler_seq Euler angle sequence to use for the orientation error
   * @param bpos Position of the reference frame in the body frame
   * @param rotm Orientation of the reference frame in the body frame
   */
  void add_target_selected_dofs(
      kdl::ConstFramePtr desired_frame,
      const std::string& body_name,
      const std::vector<int>& dofs,
      math::EulerAngleSeq euler_seq = math::EulerAngleSeq::ZYX,
      const math::Vector3d& bpos    = math::Vector3d::Zero(),
      const math::Matrix3d& rotm    = math::Matrix3d::Identity(),
      double weight                 = 1.0);

  /**
   * @brief Fix the joint angles in the IK problem.
   *
   * @param joint_name Name of the joint to fix
   */
  void fix_joint(const std::string& joint_name, const math::VectorXd& value);

  /**
   * @brief Fix the joint angles in the IK problem.
   *
   * @param joint_name Name of the joint to fix
   */
  void fix_joint(const std::string& joint_name, double value);

  /**
   * @brief Fix the joint angles in the IK problem.
   *
   * @param joint_name Name of the joint to fix
   */
  void fix_joint(const std::string& joint_name);

  void remove_fix_joint(const std::string& joint_name);

  /**
   * @brief Fix the body pose in the IK problem.
   *
   * @param body_name Name of the body to fix
   */
  void fix_body(const std::string& body_name);

  /**
   * @brief Solve the IK problem.
   *
   * @param qpos Initial guess for the joint angles
   * @return true if the IK problem was solved successfully
   * @return false otherwise
   */
  bool solve(const math::VectorXd& qpos, const math::VectorXd& qvel);

  /**
   * @brief Get the joint angles solved by the IK problem.
   *
   * @return math::VectorXd Joint angles
   */
  const math::VectorXd& get_qpos() const;

  /**
   * @brief Set the initial guess of the joint angles.
   *
   * @return math::VectorXd Joint angles for initial guess
   */
  void set_qpos(const math::VectorXd& qpos);

  void set_null_space_objective(NullSpaceObj::Ptr obj);

  /**
   * @brief Set the bias damping value for Levenberg-Marquardt algorithm.
   *
   * @param bias_damping Damping bias value (suggested range [0.001*l^2,
   * 0.1*l^2] where l is typical link length in meters). Default is 0.01.
   */
  void set_lm_bias_damping(double bias_damping);

  /**
   * @brief Set the LM step tolerance used for stationary-point detection.
   *
   * @param step_tol Step norm tolerance. Default matches the solver tolerance
   * passed to the constructor.
   */
  void set_lm_step_tolerance(double step_tol);

private:
  void add_target(std::shared_ptr<JacobianIKTarget> target, double weight);

  kdl::ConstRigidBodyTreePtr m_rbtree;
  JacobianIKAlgorithm m_alg;
  math::VectorXd m_qpos;
  std::vector<std::tuple<std::shared_ptr<JacobianIKTarget>, double>> m_targets;
  int m_num_iters;
  double m_tol_squared;
  int m_ncons;
  NullSpaceObj::Ptr m_null_space_obj;

  std::set<int> m_qvel_addr;
  std::unordered_map<int, std::optional<double>> m_qvel_addr_fixed;

  // Levenberg-Marquardt specific parameters
  double m_lm_bias_damping; // Bias damping value W_bar_N
  double m_lm_step_tol;     // Step norm convergence tolerance
};

} // namespace crobot::ik
