#pragma once

#include <functional>

#include "crobot/ik/obj_target.hpp"

namespace crobot::ik {

class ObjJntLimitTarget : public ObjTarget {
public:
  using Ptr      = std::shared_ptr<ObjJntLimitTarget>;
  using ConstPtr = std::shared_ptr<const ObjJntLimitTarget>;
  ObjJntLimitTarget(
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::vector<std::string>& joint_names,
      double weight = 0.0);

  ~ObjJntLimitTarget() = default;

  math::VectorXd get_partial_deriv(
      const math::VectorXd& qpos, const math::VectorXd& qvel) const override;

private:
  math::VectorXd m_qpos_lb;
  math::VectorXd m_qpos_ub;
  math::VectorXd m_qpos_range2;
  math::VectorXd m_qpos_center;

  math::VectorXd m_qpos_addrs;
};

} // namespace crobot::ik

