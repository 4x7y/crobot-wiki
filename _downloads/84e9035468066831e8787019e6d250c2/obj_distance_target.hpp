#pragma once

#include <functional>

#include "crobot/ik/obj_target.hpp"
#include "crobot/kdl/kdl.hpp"

namespace crobot::ik {

class ObjDistanceTarget : public ObjTarget {
public:
  using Ptr      = std::shared_ptr<ObjDistanceTarget>;
  using ConstPtr = std::shared_ptr<const ObjDistanceTarget>;
  ObjDistanceTarget(
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::string& body_name1,
      const math::Vector3d& bpos1,
      const std::string& body_name2,
      const math::Vector3d& bpos2,
      double checking_threshold,
      double weight = 0.0);

  ~ObjDistanceTarget() = default;

  math::VectorXd get_partial_deriv(
      const math::VectorXd& qpos, const math::VectorXd& qvel) const override;

private:
  kdl::ConstRigidBodyPtr m_body1;
  kdl::ConstRigidBodyPtr m_body2;
  math::Vector3d m_bpos1;
  math::Vector3d m_bpos2;
  double m_threshold;
};

} // namespace crobot::ik

