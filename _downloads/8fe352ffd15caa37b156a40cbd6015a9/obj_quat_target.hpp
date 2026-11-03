#pragma once

#include <functional>

#include "crobot/ik/obj_target.hpp"
#include "crobot/kdl/kdl.hpp"

namespace crobot::ik {

class ObjQuatTarget : public ObjTarget {
public:
  using Ptr      = std::shared_ptr<ObjQuatTarget>;
  using ConstPtr = std::shared_ptr<const ObjQuatTarget>;
  ObjQuatTarget(
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::string& body_name,
      const math::Quaternion& quat_target,
      double weight = 0.0);

  ~ObjQuatTarget() = default;

  math::VectorXd get_partial_deriv(
      const math::VectorXd& qpos, const math::VectorXd& qvel) const override;

private:
  kdl::ConstRigidBodyPtr m_body;
  math::Quaternion m_quat_target;
};

} // namespace crobot::ik

