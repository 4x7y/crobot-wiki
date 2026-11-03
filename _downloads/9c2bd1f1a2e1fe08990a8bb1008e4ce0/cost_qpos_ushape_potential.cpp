#include "crobot/ik/cost_qpos_ushape_potential.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
QposUshapePotentialCost::QposUshapePotentialCost(
    const std::string& name, kdl::ConstRigidBodyTreePtr rbtree)
  : ik::Cost(name, {"qpos"}), m_rbtree(std::move(rbtree)) {
  auto qpos_limits = m_rbtree->get_qpos_limits();

  math::VectorXd qpos_range = qpos_limits.second - qpos_limits.first;

  m_qpos_lb = qpos_limits.first - 0.2 * qpos_range;
  m_qpos_ub = qpos_limits.second + 0.2 * qpos_range;
}

//==============================================================================
double QposUshapePotentialCost::get_cost(const math::VectorXd& qpos) const {
  math::VectorXd dist
      = -1.0 / ((qpos - m_qpos_lb).array() * (qpos - m_qpos_ub).array());
  return dist.squaredNorm();
}

//==============================================================================
void QposUshapePotentialCost::fill_jacobian_block(
    Variable::ConstPtr varsets, math::SparseJacobian& jac_block) const {
  if (varsets->get_name() == "qpos") {
    math::VectorXd qpos = get_dependent_variable_values();
    math::VectorXd dist
        = -1.0 / ((qpos - m_qpos_lb).array() * (qpos - m_qpos_ub).array());

    const math::VectorXd& a = m_qpos_lb;
    const math::VectorXd& b = m_qpos_ub;
    math::VectorXd c1 = -1. / ((b - qpos).array() * (a - qpos).array().pow(2));
    math::VectorXd c2 = -1. / ((a - qpos).array() * (b - qpos).array().pow(2));
    math::VectorXd jacb = 2.0 * dist.array() * (c1 + c2).array();
    for (int i = 0; i < jacb.size(); ++i) {
      jac_block.coeffRef(0, i) = jacb(i);
    }
  }
}
