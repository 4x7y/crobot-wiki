#pragma once

#include <memory>

#include "crobot/control/single_target.hpp"

namespace crobot::control {

class PIDTarget;
using PIDTargetPtr      = std::shared_ptr<PIDTarget>;
using ConstPIDTargetPtr = std::shared_ptr<const PIDTarget>;

class PIDTarget : public SingleTarget {
public:
  static PIDTargetPtr MakeShared(
      double kp, double kv, double ki, double dt, TargetType type);

  PIDTarget(double kp, double kv, double ki, double dt, TargetType type);

  virtual ~PIDTarget() = default;

  double generate(
      double x,
      double x_dot,
      double x_desired,
      double x_dot_desired,
      double x_ddot_desired,
      double f_ext = 0.0) override;

private:
  double kp; // proportional gain
  double kv; // velocity gain
  double ki; // integral gain

  double dt;             // time step
  double integral_error; // integral of error
};
} // namespace crobot::control

