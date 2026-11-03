#pragma once

#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/math/state_space.hpp"

namespace crobot::planning {

class RobotStateSpace : public math::StateSpace<math::VectorXd> {
public:
  RobotStateSpace(kdl::ConstRigidBodyTreePtr rbtree);

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
  kdl::ConstRigidBodyTreePtr m_rbtree;
};

} // namespace crobot::planning

