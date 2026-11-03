#include "crobot/control/admittance_target.hpp"

#include "iostream"

using namespace crobot;
using namespace control;

//==============================================================================
AdmittanceTargetPtr AdmittanceTarget::MakeShared(
    double m,
    double kd,
    double ks,
    double kp,
    double kv,
    double ki,
    double dt,
    TargetType type) {
  return std::make_shared<AdmittanceTarget>(m, kd, ks, kp, kv, ki, dt, type);
}

//==============================================================================
AdmittanceTarget::AdmittanceTarget(
    double m,
    double kd,
    double ks,
    double kp,
    double kv,
    double ki,
    double dt,
    TargetType type)
  : SingleTarget(type), m(m), kd(kd), ks(ks) {
  m_pid_target = PIDTarget::MakeShared(kp, kv, ki, dt, TargetType::POSITION);
  m_dt         = dt;
  m_x_d        = 0.0;
  m_x_dot_d    = 0.0;
}

//==============================================================================
double AdmittanceTarget::generate(
    double x,
    double x_dot,
    double x_equilibrium,
    double x_dot_equilibrium,
    double x_ddot_equilibrium,
    double f_ext) {

  double x_ddot_d = (f_ext - kd * (m_x_dot_d - x_dot_equilibrium)
                     - ks * (m_x_d - x_equilibrium))
                        / m
                    + x_ddot_equilibrium;

  // print all inputs
  std::cout << "----" << std::endl;
  std::cout << "x: " << x << std::endl;
  std::cout << "x_dot: " << x_dot << std::endl;
  std::cout << "x_equilibrium: " << x_equilibrium << std::endl;
  std::cout << "x_dot_equilibrium: " << x_dot_equilibrium << std::endl;
  std::cout << "x_ddot_equilibrium: " << x_ddot_equilibrium << std::endl;
  std::cout << "f_ext: " << f_ext << std::endl;

  m_x_dot_d += x_ddot_d * m_dt;
  m_x_d += m_x_dot_d * m_dt;

  return m_pid_target->generate(x, x_dot, m_x_d, m_x_dot_d, x_ddot_d);
}
