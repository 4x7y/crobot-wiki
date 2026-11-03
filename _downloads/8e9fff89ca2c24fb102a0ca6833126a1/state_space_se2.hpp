#pragma once

#include "crobot/math/state_space.hpp"

namespace crobot::planning {

class StateSpaceSE2 : public math::StateSpace<math::VectorXd> {
public:
  StateSpaceSE2(
      const math::Vector3d& lb,
      const math::Vector3d& ub,
      double weight_xy,
      double weight_theta);

  // Documentation inherited.
  math::VectorXd sample() const;

  // Documentation inherited.
  math::VectorXd interpolate(
      const math::VectorXd& source,
      const math::VectorXd& target,
      double ratio) const override;

  // Documentation inherited.
  double distance(
      const math::VectorXd& from, const math::VectorXd& to) const override;

  // Documentation inherited.
  bool is_state_valid(const math::VectorXd& state) const override;

  // Documentation inherited.
  bool is_transition_valid(
      const math::VectorXd& from, const math::VectorXd& to) const override;

private:
  math::Vector3d m_lb;
  math::Vector3d m_ub;

  double m_weight_xy;
  double m_weight_theta;
};

} // namespace crobot::planning

