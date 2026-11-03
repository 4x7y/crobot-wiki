#pragma once

#include "crobot/ik/cost.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

namespace crobot::ik {

class QposQuadraticPotentialCost : public ik::Cost {
public:
  QposQuadraticPotentialCost(
      const std::string& name, kdl::ConstRigidBodyTreePtr rbtree);

  // Documentation inherited
  double get_cost(const math::VectorXd& x) const override;

  // Documentation inherited
  void fill_jacobian_block(
      Variable::ConstPtr varsets,
      math::SparseJacobian& jac_block) const override;

private:
  kdl::ConstRigidBodyTreePtr m_rbtree;
  math::VectorXd m_qpos_center;
};

} // namespace crobot::ik

