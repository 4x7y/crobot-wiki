#pragma once

#include "crobot/kdl/rigid_body.hpp"

namespace crobot::planning {

class PivotTarget {
public:
  using Ptr      = std::shared_ptr<PivotTarget>;
  using ConstPtr = std::shared_ptr<const PivotTarget>;

  PivotTarget(
      kdl::ConstRigidBodyTreePtr rbtree,
      kdl::ConstRigidBodyPtr body,
      const math::Vector3d& bpos_pivot,
      const math::Vector3d& xpos_pivot);

  const math::Vector3d& get_pivot_bpos() const;

  const math::Vector3d& get_pivot_xpos() const;

  const kdl::ConstRigidBodyPtr& get_body() const;

  const kdl::ConstRigidBodyTreePtr& get_rbtree() const;

  math::Matrix4d get_xform(const math::Quaternion& xquat) const;

  math::VectorXd get_qpos(const math::Quaternion& xquat) const;

private:
  const kdl::ConstRigidBodyTreePtr m_rbtree;
  const kdl::ConstRigidBodyPtr m_body;
  const math::Vector3d m_bpos_pivot;
  const math::Vector3d m_xpos_pivot;
};

} // namespace crobot::planning

