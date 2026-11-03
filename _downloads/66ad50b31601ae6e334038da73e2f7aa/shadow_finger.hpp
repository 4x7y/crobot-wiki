#pragma once

#include <string>
#include <vector>

#include "crobot/hardware/shadow_joint.hpp"
#include "crobot/hardware/shadow_motor.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/optimization/cost_term.hpp"
#include "crobot/optimization/ipopt_solver.hpp"

namespace crobot::hardware {

class ShadowConfig;
class ShadowHand;

class ShadowFinger {
  friend ShadowConfig;
  friend ShadowHand;

public:
  using Ptr      = std::shared_ptr<ShadowFinger>;
  using ConstPtr = std::shared_ptr<const ShadowFinger>;

  /**
   * @brief Get the name of the finger.
   *
   * @return const std::string& Name of the finger
   */
  const std::string& get_name() const;

  /**
   * @brief Get the motor ids in the ShadowHand ROS topic order.
   *
   * @return std::vector<int> The motor ids
   */
  std::vector<int> get_motor_ids() const;

  /**
   * @brief Get the motor names in the ShadowHand ROS topic order.
   *
   * @return std::vector<std::string> The motor names
   */
  std::vector<std::string> get_motor_names() const;

  /**
   * @brief Get the qpos addrs in the RigidBodyTree order.
   *
   * @return std::vector<int> The qpos addrs
   */
  std::vector<int> get_qpos_addrs() const;

  /**
   * @brief Get the joint names on this finger.
   *
   * @return std::vector<std::string> Names of the joints
   */
  std::vector<std::string> get_joint_names() const;

  enum class EstimateAlgorithm
  {
    DIRECT,
    FRICTION_CONE_OPTIMIZATION
  };

  /**
   * @brief Estimate the rh_ff fingertip contact force.
   *
   * coefficients in @c joint_names_active order convert strain gauge readings
   * (pwm) to joint torques (Nm)
   *
   *   qfrc = k_s * strain_gauge - k_s * (k_q * qpos + b_q)
   *
   * @param bpos The contact position at fingertip in the body frame.
   * @param xvec The axis of the contact friction cone, (fingertip to object),
   * represented in the world frame. (unit vector)
   * @param qpos_hand Robot configuration.
   * @param sg_pwm_hand The strain gauge values of all motors (40
   * dimensional).
   * @param alg The algorithm to use for the estimation.
   * @return math::Vector3d The estimated contact force (fingertip to object) in
   * the world frame.
   */
  math::Vector3d estimate_contact_force(
      const math::Vector3d& bpos,
      const math::Vector3d& xvec,
      const math::VectorXd& qpos_hand,
      const math::VectorXd& sg_pwm_hand,
      EstimateAlgorithm alg
      = EstimateAlgorithm::FRICTION_CONE_OPTIMIZATION) const;

  /**
   * @brief Whether the strain gauge values of this joint are being used in
   * the estimation of contact force.
   */
  bool is_motor_active(const std::string& motor_name) const;

  std::pair<math::VectorXd, math::VectorXd> get_sg_pwm_finger(
      const math::VectorXd& sg_pwm_hand) const;

private:
  ShadowFinger(
      kdl::ConstRigidBodyTreePtr hand,
      const std::string& name,
      int id,
      const std::string& tip_bodyname,
      const std::string& tip_sitename);

  math::VectorXd get_sg_pwm_active(
      const math::VectorXd& sg_pwm_hand, const math::VectorXd& qfrc_est) const;

  math::VectorXd get_qpos_active(const math::VectorXd& qpos) const;

  std::vector<int> get_qpos_addrs_active(const math::VectorXd& qpos) const;

  math::VectorXd get_qfrc_guess_active(
      const math::VectorXd& qfrc_guess_hand, const math::VectorXd& qpos) const;

  /**
   * @brief Convert strain gauge raw readings (in PWM unit) to joint torques (in
   * Nm).
   *
   * @param strain_gauges The strain gauge readings.
   * @return math::VectorXd The joint torques.
   */
  math::VectorXd convert_pwm2nm(
      const math::VectorXd& strain_gauges,
      const math::VectorXd& qpos_crobot,
      const math::VectorXd& qfrc_est) const;

  kdl::ConstRigidBodyTreePtr m_hand;

  std::string m_name;

  int m_id;

  std::string m_tip_bodyname;

  std::string m_tip_sitename;

  std::vector<ShadowJoint::Ptr> m_joints;

  std::vector<ShadowMotor::Ptr> m_motors;

  std::vector<ShadowMotor::Ptr> m_motors_active;

  math::Vector3d m_bpos_btsp_ctc;

  std::shared_ptr<optimization::Problem> m_problem;

  std::shared_ptr<optimization::IpoptSolver> m_solver;
};

} // namespace crobot::hardware

