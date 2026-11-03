#pragma once

#include <functional>

#include "crobot/ik/obj_target.hpp"

namespace crobot::ik {

class ObjJntVelTarget : public ObjTarget {
public:
  using Ptr      = std::shared_ptr<ObjJntVelTarget>;
  using ConstPtr = std::shared_ptr<const ObjJntVelTarget>;
  ObjJntVelTarget(
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::vector<std::string>& joint_names,
      double weight = 0.0);

  ~ObjJntVelTarget() = default;

  math::VectorXd get_partial_deriv(
      const math::VectorXd& qpos, const math::VectorXd& qvel) const override;

private:
  math::VectorXd m_qvel_addrs;
};

} // namespace crobot::ik

