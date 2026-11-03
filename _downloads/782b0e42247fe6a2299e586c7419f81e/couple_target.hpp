#pragma once

#include <memory>

#include "crobot/control/single_target.hpp"

namespace crobot::control {

class CoupleTarget;
using CoupleTargetPtr      = std::shared_ptr<CoupleTarget>;
using ConstCoupleTargetPtr = std::shared_ptr<const CoupleTarget>;

class CoupleTarget : public SingleTarget {
public:
  static CoupleTargetPtr MakeShared(
      std::size_t active, std::size_t leader, double coef, TargetType type);

  CoupleTarget(
      std::size_t active, std::size_t leader, double coef, TargetType type);

  // generate the coupled control signal
  double generate(
      double x,
      double x_dot          = 0.0,
      double x_desired      = 0.0,
      double x_dot_desired  = 0.0,
      double x_ddot_desired = 0.0,
      double f_ext          = 0.0) override;

  std::size_t get_active_id() const;

  std::size_t get_leader_id() const;

private:
  std::size_t m_active; //
  std::size_t m_leader; //
  double m_coef;        //
};
} // namespace crobot::control

