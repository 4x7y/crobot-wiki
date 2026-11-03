#pragma once

#include "crobot/ik/jacobian_ik_target.hpp"
#include "crobot/kdl/frame.hpp"
#include "crobot/kdl/rigid_body.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::ik {

class JacobianIKPoseTarget : public JacobianIKTarget {
public:
  JacobianIKPoseTarget(
      kdl::ConstFramePtr desired_frame,
      kdl::ConstRigidBodyPtr body,
      const math::Vector3d& bpos,
      const math::Matrix3d& rotm);

  // Documentation inherited
  std::pair<math::VectorXd, math::MatrixXd> compute_value_and_jacobian(
      const std::vector<math::Matrix4d>& tforms) const override;

  // Documentation inherited
  int get_ndofs() const override;

  // Documentation inherited
  std::vector<int> get_qvel_addrs() const override;

private:
  kdl::ConstFramePtr m_desired_frame;
  kdl::ConstRigidBodyPtr m_body;
  math::Vector3d m_bpos;
  math::Matrix3d m_rotm;
  std::vector<int> m_qvel_addrs;
};

} // namespace crobot::ik

