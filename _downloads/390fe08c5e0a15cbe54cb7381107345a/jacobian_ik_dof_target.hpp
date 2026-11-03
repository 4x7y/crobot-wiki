#pragma once

#include "crobot/ik/jacobian_ik_target.hpp"
#include "crobot/kdl/body_contact.hpp"
#include "crobot/kdl/frame.hpp"
#include "crobot/kdl/rigid_body.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::ik {

class JacobianIKDofTarget : public JacobianIKTarget {
public:
  JacobianIKDofTarget(
      kdl::ConstFramePtr desired_frame,
      kdl::ConstRigidBodyPtr body,
      const std::vector<int>& dofs,
      kdl::BodyContact::ConstPtr contact,
      math::EulerAngleSeq euler_seq);

  // old version
  JacobianIKDofTarget(
      kdl::ConstFramePtr desired_frame,
      kdl::ConstRigidBodyPtr body,
      const std::vector<int>& dofs,
      math::EulerAngleSeq euler_seq,
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
  std::vector<int> m_dofs;
  kdl::BodyContact::ConstPtr m_contact;

  math::EulerAngleSeq m_euler_seq;
  std::vector<int> m_qvel_addrs;

  math::Vector3d m_bpos;
  math::Matrix3d m_rotm;
};

} // namespace crobot::ik

