#pragma once

#include <memory>

#include "crobot/control/pid_target.hpp"
#include "crobot/control/single_target.hpp"

namespace crobot::control {

class AdmittanceTarget;
using AdmittanceTargetPtr      = std::shared_ptr<AdmittanceTarget>;
using ConstAdmittanceTargetPtr = std::shared_ptr<const AdmittanceTarget>;

class AdmittanceTarget : public SingleTarget {
public:
  /**
   * @brief Construct a new Admittance Target object
   *
   * @param m  mass
   * @param kd damping
   * @param ks stiffness
   * @param kp proportional gain for position ctrl
   * @param kv velocity gain for position ctrl
   * @param ki integral gain for position ctrl
   * @param dt time step
   * @return AdmittanceTargetPtr
   */
  static AdmittanceTargetPtr MakeShared(
      double m,
      double kd,
      double ks,
      double kp,
      double kv,
      double ki,
      double dt,
      TargetType type);
  /**
   * @brief Construct a new Admittance Target object
   *
   * @param m  mass
   * @param kd damping
   * @param ks stiffness
   * @param kp proportional gain for position ctrl
   * @param kv velocity gain for position ctrl
   * @param ki integral gain for position ctrl
   * @param dt time step
   */
  AdmittanceTarget(
      double m,
      double kd,
      double ks,
      double kp,
      double kv,
      double ki,
      double dt,
      TargetType type);

  virtual ~AdmittanceTarget() = default;

  /**
   * @brief generate admittance control for a single value
   *
   * @param f_ext external force it is trying to accomodate
   * @param x current position
   * @param x_dot current velocity
   * @param x_equilibrium equilibrium position
   * @param x_dot_equilibrium equilibrium velocity
   * @param x_ddot_equilibrium equilibrium acceleration
   * @return double admittance control value x_ddot
   */
  double generate(
      double x,
      double x_dot,
      double x_equilibrium,
      double x_dot_equilibrium,
      double x_ddot_equilibrium,
      double f_ext) override;

private:
  double m;  // mass
  double kd; // damping
  double ks; // stiffness

  double m_x_d;     // desired position
  double m_x_dot_d; // desired velocity

  PIDTargetPtr m_pid_target;

  double m_dt; // time step
};
} // namespace crobot::control

