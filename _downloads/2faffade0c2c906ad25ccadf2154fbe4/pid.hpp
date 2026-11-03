#pragma once

#include <functional>

#include "crobot/math/math.hpp"

namespace crobot::control {

/**
 * @brief Implements the PID controller.
 *
 * Given estimated state `x_in = (q_in, v_in)`, the controlled state x_c = (q_c,
 * v_c) is computed by x_c = P_x * x_in, where P_x is a state projection matrix.
 * The desired state x_d = (q_d, v_d), is in the same space as x_c. The output
 * of this controller is:
 *
 * y = P_y * (kp * (q_d - q_c) + kd * (v_d - v_c) + ki * integral(q_d - q_c)),
 * where P_y is the output projection matrix.
 */
class PIDController {
public:
  /**
   * @brief Constructs a PID controller.
   **/
  PIDController(
      const math::VectorXd& kp,
      const math::VectorXd& ki,
      const math::VectorXd& kd,
      std::function<math::VectorXd()> qpos_callback,
      std::function<math::VectorXd()> qvel_callback);

  ~PIDController() = default;

  /**
   * @brief Update controller state.
   *
   * @param time current time.
   * @param duration duration since last calling of update.
   */
  void update(double time, double duration);

  /**
   * @brief Set the desired qpos and qvel.
   *
   * @param qpos desired qpos.
   * @param qvel desired qvel.
   */
  void set_desired(const math::VectorXd& qpos, const math::VectorXd& qvel);

  /**
   * @brief Constructs a PID controller.
   **/
  math::VectorXd generate();

private:
  math::VectorXd m_kp;

  math::VectorXd m_ki;

  math::VectorXd m_kd;

  std::function<math::VectorXd()> m_qpos_callback;

  std::function<math::VectorXd()> m_qvel_callback;

  math::VectorXd m_qpos_desired;

  math::VectorXd m_qvel_desired;

  math::VectorXd m_qpos_err_int;

  math::VectorXd m_gain;
};

} // namespace crobot::control

