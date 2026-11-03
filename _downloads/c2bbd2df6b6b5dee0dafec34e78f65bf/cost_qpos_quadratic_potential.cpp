#include "crobot/ik/cost_qpos_quadratic_potential.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
QposQuadraticPotentialCost::QposQuadraticPotentialCost(
    const std::string& name, kdl::ConstRigidBodyTreePtr rbtree)
  : ik::Cost(name, {"qpos"}), m_rbtree(std::move(rbtree)) {
  auto qpos_limits = m_rbtree->get_qpos_limits();
  m_qpos_center    = (qpos_limits.first + qpos_limits.second) / 2.0;
}

//==============================================================================
double QposQuadraticPotentialCost::get_cost(const math::VectorXd& qpos) const {
  math::VectorXd dist = qpos - m_qpos_center;
  return dist.squaredNorm();
}

//==============================================================================
void QposQuadraticPotentialCost::fill_jacobian_block(
    Variable::ConstPtr varsets, math::SparseJacobian& jac_block) const {
  if (varsets->get_name() == "qpos") {
    math::VectorXd qpos = get_dependent_variable_values();
    math::VectorXd jacb = 2.0 * (qpos - m_qpos_center);
    for (int i = 0; i < jacb.size(); ++i) {
      jac_block.coeffRef(0, i) = jacb(i);
    }
  }
}
