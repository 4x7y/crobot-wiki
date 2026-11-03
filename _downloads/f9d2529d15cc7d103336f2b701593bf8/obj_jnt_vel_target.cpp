#include "crobot/ik/obj_jnt_vel_target.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
ObjJntVelTarget::ObjJntVelTarget(
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::vector<std::string>& joint_names,
    double weight)
  : ObjTarget(rbtree, weight) {
  m_qvel_addrs = math::VectorXd(m_rbtree->get_qvel_dim());
  for (size_t i = 0; i < joint_names.size(); ++i) {
    int id = m_rbtree->get_joint(joint_names[i])->get_qvel_addr()[0].first;
    m_qvel_addrs[id] = 1;
  }
}

//==============================================================================
math::VectorXd ObjJntVelTarget::get_partial_deriv(
    const math::VectorXd& qpos, const math::VectorXd& qvel) const {
  (void)qpos;
  // go in negative direction to decrease the velocity
  return -get_weight() * (qvel.cwiseProduct(m_qvel_addrs));
}
