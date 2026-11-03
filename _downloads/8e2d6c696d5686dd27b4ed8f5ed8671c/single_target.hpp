#pragma once

namespace crobot::control {
enum class TargetType
{
  POSITION,
  ADMITTANCE,
  FORCE,
  COUPLED,
  NULLTARGET
};
class SingleTarget {

public:
  SingleTarget(TargetType type) {
    m_type = type;
  }

  SingleTarget() {
    m_type = TargetType::NULLTARGET;
  }

  virtual double generate(
      double x,
      double x_dot,
      double x_desired,
      double x_dot_desired,
      double x_ddot_desired,
      double f_ext)
      = 0;

  TargetType m_type;
};
} // namespace crobot::control

