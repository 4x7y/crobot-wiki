#include "crobot/control/pid_target.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
PIDTargetPtr PIDTarget::MakeShared(
    double kp, double kv, double ki, double dt, TargetType type) {
  return std::make_shared<PIDTarget>(kp, kv, ki, dt, type);
}

//==============================================================================
PIDTarget::PIDTarget(
    double kp, double kv, double ki, double dt, TargetType type)
  : SingleTarget(type), kp(kp), kv(kv), ki(ki), dt(dt), integral_error(0.0) {
}

//==============================================================================
double PIDTarget::generate(
    double x,
    double x_dot,
    double x_desired,
    double x_dot_desired,
    double x_ddot_desired,
    double f_ext) {
  (void)f_ext;
  double error     = x_desired - x;
  double error_dot = x_dot_desired - x_dot;
  double u = kp * error + kv * error_dot + ki * integral_error + x_ddot_desired;
  integral_error += error * dt;
  return u;
}
