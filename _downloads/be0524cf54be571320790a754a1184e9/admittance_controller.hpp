#pragma once

#include <memory>

#include "crobot/control/admittance_control_target.hpp"
#include "crobot/ik/jacobian_ik.hpp"
#include "crobot/kdl/body_contact.hpp"
#include "crobot/kdl/body_rolling_contact.hpp"
#include "crobot/kdl/frame.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/matrix.hpp"
namespace crobot::control {

class AdmittanceController {
public:
  using Ptr      = std::shared_ptr<AdmittanceController>;
  using ConstPtr = std::shared_ptr<const AdmittanceController>;

  AdmittanceController(kdl::ConstRigidBodyTreePtr rbtree);

  bool compute(
      const math::VectorXd& qpos_actual,
      const math::VectorXd& qpos_cmd,
      const math::VectorXd& qvel);

  std::pair<math::VectorXd, math::VectorXd> get_result();

  AdmittanceControlTarget::Ptr add_target(
      kdl::ConstFramePtr desired_frame,
      const std::string& body_name,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      math::EulerAngleSeq euler_seq,
      const std::vector<int>& dofs,
      kdl::BodyContact::ConstPtr contact = nullptr);

  AdmittanceControlTarget::Ptr add_rolling_target(
      kdl::ConstFramePtr desired_frame,
      const std::string& body_name,
      math::EulerAngleSeq euler_seq,
      const std::vector<int>& dofs,
      kdl::BodyRollingContact::ConstPtr contact);

  void fix_joint(const std::string& joint_name, double value);

  void fix_joint(const std::string& joint_name);

  void fix_body(const std::string& body_name);

  void set_null_space_objective(ik::NullSpaceObj::Ptr obj);

private:
  kdl::ConstRigidBodyTreePtr m_rbtree;
  ik::JacobianIK::Ptr m_ik_actual;
  ik::JacobianIK::Ptr m_ik_command;
};

} // namespace crobot::control

