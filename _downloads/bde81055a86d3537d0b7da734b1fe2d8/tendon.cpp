#include "crobot/kdl/tendon.hpp"

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace kdl;

//==============================================================================
Tendon::Tendon(ConstRigidBodyTreePtr rbtree, const std::string& name)
  : m_rbtree(std::move(rbtree)), m_name(name) {
}

//==============================================================================
void Tendon::add_joint(ConstJointPtr joint, double coeff) {
  m_joints.push_back(joint);
  m_coeffs.push_back(coeff);
}

//==============================================================================
std::vector<std::pair<int, int>> Tendon::get_qpos_addr() const {
  std::vector<std::pair<int, int>> res;
  res.reserve(m_joints.size());
  for (size_t i = 0; i < m_joints.size(); i++) {
    const ConstJointPtr& joint = m_joints[i];
    CROBOT_ASSERT_ERROR(joint->m_in_tree);
    res.push_back(joint->m_qpos_addr);
  }
  return res;
}

//==============================================================================
std::vector<std::pair<int, int>> Tendon::get_qvel_addr() const {
  std::vector<std::pair<int, int>> res;
  res.reserve(m_joints.size());
  for (size_t i = 0; i < m_joints.size(); i++) {
    const ConstJointPtr& joint = m_joints[i];
    CROBOT_ASSERT_ERROR(joint->m_in_tree);
    res.push_back(joint->m_qvel_addr);
  }
  return res;
}

//==============================================================================
std::vector<double> Tendon::get_transmission_coeffs() const {
  return m_coeffs;
}
