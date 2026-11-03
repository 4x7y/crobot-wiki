#include "crobot/planning/pivot_target.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
PivotTarget::PivotTarget(
    kdl::ConstRigidBodyTreePtr rbtree,
    kdl::ConstRigidBodyPtr body,
    const math::Vector3d& bpos_pivot,
    const math::Vector3d& xpos_pivot)
  : m_rbtree(rbtree),
    m_body(body),
    m_bpos_pivot(bpos_pivot),
    m_xpos_pivot(xpos_pivot) {
}

//==============================================================================
const math::Vector3d& PivotTarget::get_pivot_bpos() const {
  return m_bpos_pivot;
}

//==============================================================================
const math::Vector3d& PivotTarget::get_pivot_xpos() const {
  return m_xpos_pivot;
}

//==============================================================================
const kdl::ConstRigidBodyPtr& PivotTarget::get_body() const {
  return m_body;
}

//==============================================================================
const kdl::ConstRigidBodyTreePtr& PivotTarget::get_rbtree() const {
  return m_rbtree;
}

//==============================================================================
math::Matrix4d PivotTarget::get_xform(const math::Quaternion& xquat) const {
  math::Matrix4d xform    = xquat.as_tform();                    // xmat
  xform.block<3, 1>(0, 3) = m_xpos_pivot - xquat * m_bpos_pivot; // xpos
  return xform;
}

//==============================================================================
math::VectorXd PivotTarget::get_qpos(const math::Quaternion& xquat) const {
  math::VectorXd qpos(7);
  qpos.segment(0, 3) = m_xpos_pivot - xquat * m_bpos_pivot;
  qpos.segment(3, 4) = xquat.as_vector();
  return qpos;
}
