#pragma once

#include <memory>

#include "crobot/hardware/shadow_joint.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::hardware {

class ShadowConfig;
class ShadowFinger;
class ShadowHand;

class ShadowMotor {
  friend ShadowConfig;
  friend ShadowFinger;
  friend ShadowHand;

public:
  using Ptr      = std::shared_ptr<ShadowMotor>;
  using ConstPtr = std::shared_ptr<const ShadowMotor>;

  /**
   * @brief Get the name of the motor.
   *
   * @return const std::string& Name of the motor
   */
  const std::string& get_name() const;

  /**
   * @brief Get the motor id.
   *
   * @return int id of the motor
   */
  int get_id() const;

  /**
   * @brief Get the qpos of the motor. If the motor is associtated with two
   * joints, it will return the total of qpos of the two joints.
   *
   * For example, motor rh_FFJ0 will return the sum of the qpos corrsponding to
   * rh_FFJ1 and rh_FFJ2.
   *
   * @param qpos_hand The qpos of the hand
   * @return double The qpos of the motor
   */
  double get_qpos(const math::VectorXd& qpos_hand) const;

  /**
   * @brief Get the active joint qpos addr object.
   *
   * Each motor is associated with one or more joints. Only one of the joints is
   * active at a time. This function returns the qpos addr of the active joint.
   *
   * @param qpos_hand The qpos of the hand
   * @return int The qpos addr of the active joint
   */
  int get_active_joint_qpos_addr(const math::VectorXd& qpos_hand) const;

  /**
   * @brief Convert motor strain gauge raw value (PWM) to motor torque (Nm).
   *
   * @param pwm The motor strain gauge raw value (PWM)
   * @param qfrc_guess The guess of the motor torque (Nm)
   * @param qpos_hand The qpos of the hand
   * @return double The motor torque (Nm)
   */
  double get_torque_pwm2nm(
      double pwm, double qfrc_guess, const math::VectorXd& qpos_hand) const;

  int get_ind_pos_torque() const {
    return m_strain_gauge.index_pos_torque;
  }

  int get_ind_neg_torque() const {
    return m_strain_gauge.index_neg_torque;
  }

private:
  ShadowMotor();

  struct Factor
  {
    ShadowJoint::ConstPtr joint;
    double k_s_pos;
    double k_s_neg;
  };

  struct StrainGauge
  {
    int index_pos_torque;
    int index_neg_torque;
    std::vector<Factor> factors;

    /**
     * @brief Convert the strain gauge values from strain gauge readings (PWM)
     * to joint torque (Newton Meter).
     *
     * @param pwm
     * @param qfrc_guess
     * @param qpos
     * @return double
     */
    double convert_pwm2nm(
        double pwm, double qfrc_guess, const math::VectorXd& qpos) const;
  };

  struct Control
  {
    double k;
    double b;
  };

  struct Target
  {
    ShadowJoint::ConstPtr joint;
    std::pair<double, double> range;
  };

private:
  std::string m_name;
  int m_id;
  StrainGauge m_strain_gauge;
  Control m_control;

  std::vector<Target> m_targets;
};

} // namespace crobot::hardware

